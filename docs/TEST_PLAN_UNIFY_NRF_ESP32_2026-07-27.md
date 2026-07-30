# Bench test plan — `feat/unify-nrf-esp-phase3`

**Status: NOT RUN.** Written 2026-07-27. Expands the eight-item bench matrix in
`PLAN_BLE_TRANSPORT_ABSTRACTION_2026-07-27.md` §7 into executable procedures and adds
coverage for the work that landed after that plan was written.

Verification is **hardware only**. CI builds all 13 environments and executes nothing,
so a green CI says the code compiles and nothing else. Nothing on this branch has been
run on hardware.

## 1. What is under test

`main` (`772e9f8`) → `feat/unify-nrf-esp-phase3` (`6cc4e13`), 27 commits.

The change with real behavioural risk is **Phase 3**: nRF command dispatch moved off
the SoftDevice callback task onto `loop()`. Everything else is either an abstraction
that preserved behaviour, logging, or a fix whose ESP32 blast radius is bounded by an
unchanged binary (§2).

## 2. Scoping: what does NOT need retesting on ESP32

Three commits produced a **byte-identical** `esp32-N4` binary (998,507 B before and
after each):

| Commit | Change |
|---|---|
| `b3b23dc` | power latch made portable |
| `7b94aa4` | ADC ladder + touch parity (B3, B4) |
| `6cc4e13` | `od_log_flush` delay on both targets |

For those three the ESP32 regression surface is nil and testing effort belongs on nRF.
This does **not** extend to the logging commits (`1998691`, `f76693e`, `8f08ed5`,
`14b89b6`) or the race fix (`c4bd4bc`), all of which changed the ESP32 binary.

## 3. Builds required

```bash
pio run -e nrf52840custom-debug   -t upload   # nRF, DEBUG logging, USB CDC 115200
pio run -e nrf52840custom         -t upload   # nRF, shipping log level
pio run -e esp32-s3-N16R8-extuart-debug -t upload   # ESP32, DEBUG, CH343P UART
pio run -e esp32-N4               -t upload   # ESP32, shipping, PIPE_SMALL_DRAM_WINDOW
```

Run functional tests on the `-debug` envs (the ERX/URX and ETX/UTX lines are
`od_log_debug` and are absent otherwise), then confirm the headline cases on the
shipping envs — the debug builds add ~13 KB and real serial time, so timing-sensitive
results must be confirmed without them.

**Reference build for regressions: `feat/unify-nrf-esp` (`1050517`).** That is the last
commit where nRF still dispatched on the callback task, so it is the only build that
can supply "before" numbers for T7. Its nRF RAM figure is *not* comparable
(`--gc-sections` drops the unused rings there); only current and throughput are.

## 4. Test groups

Ordered by risk. T1 and T2 gate everything else.

### T1 — Smoke, both targets

| # | Step | Pass |
|---|---|---|
| T1.1 | Power on | Boot screen renders; `=== FIRMWARE INFO ===` and the git SHA log |
| T1.2 | Scan | Device advertises as `OD<chipid>`, manufacturer id 9286 |
| T1.3 | Connect | `=== BLE CLIENT CONNECTED ===` then `[LINK negotiated] PHY=2M … DLE=251` at INFO |
| T1.4 | Authenticate | `Authentication successful, session established` |
| T1.5 | Full-frame push | Image renders; `RESP_DIRECT_WRITE_REFRESH_SUCCESS` reaches the client |

T1.3 is worth its own line: `requestFastLink()` is new on ESP32 (`6891956`) and the
INFO-level link log is new on both. On ESP32 the negotiated **DLE is not reported** —
NimBLE exposes no accessor — so confirm PHY and MTU only there.

### T2 — Core BLE, both targets (the §7 matrix)

| # | Test | Pass |
|---|---|---|
| T2.1 | PIPE_WRITE full image | Completes; record throughput + retry count for T7 |
| T2.2 | Config read-back > 864 B | Multi-chunk read returns intact; no response-ring drops |
| T2.3 | Disconnect mid-transfer | Panel powers down, no zombie session; reconnect works |
| T2.4 | Reconnect + re-subscribe | CCCD re-enabled; notifications resume |
| T2.5 | Button + touch during transfer | See T5.2 — behaviour **changed on nRF** |
| T2.6 | Buzzer command during transfer | Plays; transfer completes |
| T2.7 | Partial write (0x0076) | Rect updates; ETAG committed |
| T2.8 | ESP32 deep-sleep / wake cycle | Wakes, reconnects, `Deep sleep count` increments |

### T3 — Phase 3 execution model (nRF only, highest risk)

The threading contract: stack callbacks may only copy bytes into the RX ring and set a
flag; everything else runs on `loop()`.

| # | Test | Pass |
|---|---|---|
| T3.1 | Sustained PIPE_WRITE at full window | No `Command queue full` at any point |
| T3.2 | Command latency | Response follows command within ~100 ms (the `idleDelay` chunk); no multi-second stalls |
| T3.3 | Command during `idleDelay` | `idleDelay()` returns early on pending RX — no 500 ms floor |
| T3.4 | Ring depth under load | RX `[Q:n]` stays ≪ `PIPE_MAX_W + 2`; TX `[Q:n]` returns to 0 between commands |
| T3.5 | Rapid connect/disconnect ×20 | No hang, no leaked session, advertising always resumes |

T3.4 is the direct readout of whether the derived ring depth is right. A TX `Q` that
climbs monotonically means the drain is behind the producer; an RX `Q` that climbs
means arrivals are outrunning `loop()`.

### T4 — The reconnect race (`c4bd4bc`) — both targets

This reproduces a defect actually observed on nRF on 2026-07-27, so it is a regression
test with a known-failing predecessor, not a hypothetical.

**T4.1 — stale disconnect must not eat the next client's frames**
1. Start a full-frame push to a Spectra 6-colour panel (~16 s refresh).
2. While `Refresh took …` has not yet printed, **disconnect** client A.
3. Still inside the refresh, **connect** client B and send one command (e.g. `0x0080`).
4. Wait for the refresh to finish.

**Pass:** B's command dispatches. `Dropped N queued command(s)` either does not appear
or reports only A's frames. **Fail (pre-fix behaviour):** `Disconnect reason: 19` and
`Dropped 1 queued command(s)` print *after* `=== BLE CLIENT CONNECTED ===`, and B's
command never dispatches.

**T4.2 — cleanup must not tear down the new client's session.** Same setup, but B
starts a PIPE_WRITE before the refresh ends. **Pass:** `Disconnect cleanup skipped:
transfer still owned by a live BLE session`, and B's transfer completes. This path was
unguarded on nRF before `c4bd4bc` (the guard sat inside `#ifdef OPENDISPLAY_HAS_WIFI`).

**T4.3 — the ordinary case still flushes.** Disconnect mid-transfer with no reconnect.
**Pass:** `Dropped N queued command(s)` reports the frames A actually left.

### T5 — Parity fixes (nRF only; ESP32 binaries unchanged)

**T5.1 — power latch (`b3b23dc`).** No nRF board has the hardware. Confirm only that a
config *without* `DEVICE_FLAG_BATTERY_LATCH` / `DEVICE_FLAG_PWR_LATCH_DFF` behaves
exactly as before: no spurious power-off, `0x0052` still NACKs. On ESP32 latch
hardware, re-run press-and-hold power-off and the D-FF rail cut — the refactor touched
those call sequences even though the binary did not change.

**T5.2 — touch suspension during transfers (B4, `7b94aa4`).** *Behaviour change on
nRF.* During a transfer or refresh, touch must **stop** responding and resume after.
Compare pipe-write throughput with and without continuous touch input; it should no
longer degrade. Confirm touch is not left permanently disabled after a failed or
aborted transfer.

**T5.3 — ADC ladder (B3, `7b94aa4`).** No nRF ladder hardware exists. Confirm the
negative case: a config declaring `BINARY_INPUT_TYPE_ADC_LADDER` must log
`ADC ladder: pin …` and must **not** attach a digital-button interrupt to that pin
(pre-fix, the `continue` was compiled out and it did). On ESP32, re-run ladder button
detection to confirm `adcLadderConfigurePin()` is equivalent to the old inline
attenuation call.

### T6 — Logging correctness (`-debug` envs)

| # | Test | Pass |
|---|---|---|
| T6.1 | Every command | Exactly one `ERX`/`URX` line, then the banner, then one `ETX`/`UTX` line |
| T6.2 | nRF parity | nRF emits RX lines at all (it emitted none before `8f08ed5`) and carries `[Q:n]` |
| T6.3 | Encryption token | Authenticated traffic reads `ERX`/`ETX`; handshake (0x0050/0x000A) reads `URX`/`UTX` |
| T6.4 | Mid-stream quiet | 0x0071/0x0081 frames log chunk 1 then go silent; no per-frame spam |
| T6.5 | Oversize frame | Logs `Command too large for queue`, **not** `queue full` (the nRF misreport fixed in `8f08ed5`) |
| T6.6 | Decrypt failure | `Decryption failed (0x…, N B payload, nonce …)` — nonce present on the failure path only |

T6.5 is the specific pre-fix misdiagnosis: nRF reported all three push failures as
"queue full", pointing at ring depth for a malformed frame.

### T7 — Power and throughput regressions (nRF)

The two baselines still uncaptured. Measure on `feat/unify-nrf-esp` (`1050517`) first,
then on this branch, on the same hardware and battery.

| # | Metric | Threshold |
|---|---|---|
| T7.1 | Battery idle current, advertising, no client | No material rise. `loop()` spins only while `workInFlight`; a persistent rise means a term is stuck true |
| T7.2 | PIPE_WRITE throughput | Within noise of the reference. Each ACK now costs a loop pass |
| T7.3 | Retry / NACK count | No increase |

T7.1 is the one Phase 3 most plausibly regresses, and the one no amount of code reading
settles.

## 5. Not covered, and why

| Item | Reason |
|---|---|
| nRF power latch on real hardware | No such board exists |
| nRF ADC ladder classification | No such board exists; thresholds need per-board calibration and reference voltages differ from ESP32 |
| The 16 s refresh stall itself | Out of scope — see `DESIGN_COOPERATIVE_REFRESH_WAIT_2026-07-27.md`. T4 tests that deferred work is *correct when late*, not that it stops being late |
| Audit findings M2, L1, L2, L3, L5 | Unreviewed; `AUDIT_FIRMWARE_2026-07-13.md` is demonstrably stale (M1 and L4 were already fixed) |
| `esp32-s3-E1004` / FastEPD paths | Needs the reTerminal E1004 panel |

## 6. Exit criteria

Ship when:

1. T1–T4 pass on **both** targets.
2. T5 passes on nRF, and T5.1/T5.3's ESP32 halves pass on latch/ladder hardware.
3. T6 passes on both `-debug` envs.
4. T7 shows no material regression against `feat/unify-nrf-esp`.
5. Anything that fails is either fixed or recorded here with a decision.

**Rollback:** phases are independent commits. Phase 3 is the only one that changes nRF
runtime behaviour and can be reverted alone, leaving the abstraction (Phases 1–2) in
place. `c4bd4bc` depends on Phase 3 and must revert with it.
