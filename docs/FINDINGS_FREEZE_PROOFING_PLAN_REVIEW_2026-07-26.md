# Adversarial Review — PLAN_FREEZE_PROOFING_2026-07-26

**Date:** 2026-07-26
**Reviewing:** `docs/PLAN_FREEZE_PROOFING_2026-07-26.md` (branch `debug/ble-hardening`)
**Scope:** `Firmware` repo — nRF52840 (Bluefruit) + ESP32-S3/C6/C3/classic (NimBLE-Arduino 2.5.0)

> All Critical/High/Medium corrections below have been folded back into
> `PLAN_FREEZE_PROOFING_2026-07-26.md` (revised 2026-07-26), tagged `[C1]`…`[X7]`.
> This document is retained as the rationale record — read it before re-litigating any
> plan decision, especially the phase ordering and the progress-stamp definition.

---

## Verdict

**Sound in shape, wrong in several load-bearing details — ship only with the Critical
corrections below.** The diagnosis (§Context items 1–5) is accurate and I verified four of
the five wedge mechanisms against source. The phase decomposition is sensible and most of
the platform facts the author claims to have "verified this session" hold up.

But the plan has **four Critical defects**, two of which mean the headline deliverable
does not work:

- **The 10-minute supervisor cannot fire in the exact wedge it was designed for**, because
  `markSessionProgress()` is stamped on *command dispatch* and *successful notify* — both
  of which keep ticking while the device answers every command `RESP_AUTH_REQUIRED`.
- **Phase 4's second-central refusal is a no-op on the wire**: `BLE_ERR_CONN_LIMIT` (0x09)
  is not a legal `HCI_Disconnect` reason, so `ble_gap_terminate` is rejected by the
  controller.
- **The `pwrmgmLock` 10 s steal is shorter than a legitimate lock hold** and destroys the
  lock's mutual exclusion when it fires.
- **Phase 4's refusal path re-enters the shared disconnect-cleanup flag**, which on the two
  non-WiFi envs has no owner guard at all.

None of these require restructuring the plan. All four are surgical.

---

## CRITICAL

### C1 — Progress stamps on "command dispatch" and "successful notify" make the supervisor blind to the primary wedge

**Plan element:** Phase 6 — *"Progress stamps (`markSessionProgress()`): command dispatch,
pipe frame accept, successful notify, LAN frame dispatch, refresh completion."*

**Why it breaks:** The wedge the plan opens with (§Context item 1) is: `integrity_failures
>= 3` → `clearEncryptionSession()` mid-transfer, after which
`src/communication.cpp:664-670` answers **every** command with
`{RESP_ACK, cmd, RESP_AUTH_REQUIRED}` via `sendResponseUnencrypted()`. That is a *command
dispatch* and a *successful notify* on every single client retry.

**Concrete sequence** (ESP32, W=16 pipe transfer, HA push):

| t | event | state |
|---|---|---|
| 0 s | `0x0080` START accepted | `directWriteActive=true`, `pipeState.active=true` |
| 41 s | window loss burst; 3 nonce rejections | `encryption.cpp:691-696` → `clearEncryptionSession()` |
| 41 s+ | client retransmits `0x0081` frames | each reaches `imageDataWritten` → **stamp**; each is answered `RESP_AUTH_REQUIRED` → notify → **stamp** |
| 56 s | py-opendisplay hits `MAX_PTO=3 × TIMEOUT_PIPE_DATA_COMPRESSED=5.0 s` (`device.py:465`, `commands.py:93`) and raises `ProtocolError` | client stops sending; **but does not necessarily drop the link** |
| 56 s → ∞ | HA keeps the `Device` object; any subsequent poll/telemetry command re-stamps | `now - g_lastProgressMs` never exceeds 600 000 |

The wedge predicate `(transferActive() || chunkedWriteState.active || directWriteActive)`
is **true** the whole time (`directWriteActive` is only cleared by
`cleanupDirectWriteState`, `display_service.cpp:2011`). The supervisor therefore *sees* the
wedge and *never times it out*. Phase 6 delivers nothing for the case it was written for.

The same defect hits Phase 5: `g_lastLinkRxMs` is stamped in `onWrite`
(`esp32_ble_callbacks.h:81`), so a client that keeps writing doomed commands defeats the
5-minute idle disconnect too. With Phase 6 subsuming the 15-min watchdogs (below), the
device ends up with **strictly less** recovery than today.

**Correction:** `markSessionProgress()` must mean *the state machine advanced*, not *bytes
moved*. Stamp only at:
- `pipeState.expected_seq` actually advancing (inside the in-order accept in
  `handlePipeWriteData`, not at frame receipt),
- `directWriteBytesWritten` increasing (`display_service.cpp:2005`),
- `chunkedWriteState.receivedChunks` incrementing (`communication.cpp:565`),
- `partialCtx` byte counter advancing,
- refresh completion (`display_service.cpp:2436`).

Do **not** stamp on command dispatch, on notify, or on LAN frame dispatch. Add one extra
stamp on `handleAuthenticate` success so a legitimate re-auth resets the clock.

---

### C2 — `pwrmgmLock` 10 s steal is shorter than legitimate holds, and stealing corrupts the lock

**Plan element:** Phase 0 — *"`pwrmgmLockTake`: 10 s deadline
(`OD_PWRMGM_LOCK_TIMEOUT_MS = 10000`); on expiry log ERROR and force-take (steal)."*

**Legitimate holds already exceed 10 s.** The lock is held across panel I/O in three
places (`display_service.cpp:439`, `:496`, `:513`), and the bb_epaper busy wait inside
those paths is bounded at **30 000 ms for 3-/4-/7-colour panels**:

```
.pio/libdeps/esp32-s3-N16R8/bb_epaper/src/bb_ep.inl:3959-3975
    int iMaxTime = 5000;                       // B/W
    if (pBBEP->iFlags & (BBEP_3COLOR|BBEP_4COLOR|BBEP_7COLOR)) iMaxTime = 30000;
```

- `epdSessionForceOffLocked()` (`display_service.cpp:417-433`) calls `bbepSleep(&bbep,1)`,
  which for UC81xx 3-/4-colour panels calls `bbepWaitBusy()` (`bb_ep.inl:4122`, `:4129`) →
  **up to 30 s under the lock**, plus `delay(50)`.
- `epdSessionAcquire()` (`:437-487`) holds the lock across `bbepWakeUp` +
  `bbepSendCMDSequence` + `epdAlignCustomPartialRamMode`; `bbepSetAddrWindow` alone ends in
  a `bbepWaitBusy` (`bb_ep.inl:4104`).

So on any Spectra/ACeP tag, a **normal** `cleanupDirectWriteState(true)` → `epdSessionForceOff()`
can hold `pwrmgmLock` for ~30 s. On nRF the transfer runs on the Bluefruit Callback task
while `epdSessionTick()` runs on loop — the plan's own rationale for the lock existing.
A 10 s steal fires on healthy hardware.

**What the steal does when it fires** (`display_service.cpp:401-413`):

```c
static void pwrmgmLockTake(void)  { while (__atomic_exchange_n(&pwrmgmLock,1,ACQUIRE)) delay(1); }
static void pwrmgmLockGive(void)  { __atomic_store_n(&pwrmgmLock, 0, RELEASE); }
```

`pwrmgmLock` is a plain 0/1 flag with no owner field. After a steal there are two threads
that each believe they hold it:

1. Stealer runs `epdSessionForceOffLocked()` → `bbepSleep` + rail cut while the true holder
   is mid-`bbepSendCMDSequence` → **two tasks driving the same SPI bus and CS line**.
2. True holder finishes and calls `pwrmgmLockGive()` → flag = 0 **while the stealer still
   holds it**. Mutual exclusion is now gone for every subsequent operation, permanently.
   `epdSessionTick()`'s `pwrmgmLockTryTake()` (`:520`) will now succeed mid-transfer and
   rail-cut a live push — the precise failure the lock was introduced to prevent.
3. `pwrmgmState` ends up whatever the *loser* wrote last: the holder's `pwrmgmState =
   PWR_ACTIVE` (`:467`) lands after the stealer's `PWR_OFF`, leaving the firmware convinced
   the panel is powered when the rail is down. `epdSessionTick()` then never re-arms
   (`:520` early-returns unless `PWR_WARM`), so the panel stays permanently mis-tracked.

**Correction:** do not steal. Make `pwrmgmLockTake` return `bool` with a **60 s** deadline
(≥ 2× the worst-case `bbepWaitBusy`), and on expiry **fail the operation** — log ERROR,
return false, let the caller skip its panel work and set a "panel state unknown" flag that
`abortToKnownState` reports. If a forced recovery is genuinely required, add an owner field
(`volatile TaskHandle_t pwrmgmOwner`) and have the stealer set `pwrmgmOwner = self` so the
original holder's `Give` becomes a detectable no-op instead of a silent unlock.

---

### C3 — `BLE_ERR_CONN_LIMIT` is not a legal `HCI_Disconnect` reason; the exclusivity refusal silently no-ops

**Plan element:** §Key platform facts and Phase 4 — *"refuse via `pServer->disconnect(connInfo,
BLE_ERR_CONN_LIMIT)`"*.

`NimBLEServer::disconnect(connInfo, reason)` forwards straight to `ble_gap_terminate`:

```
.pio/libdeps/esp32-s3-N16R8/NimBLE-Arduino/src/NimBLEServer.cpp:321-332
bool NimBLEServer::disconnect(uint16_t connHandle, uint8_t reason) const {
    int rc = ble_gap_terminate(connHandle, reason);
    ... NIMBLE_LOGE("ble_gap_terminate failed: rc=%d ..."); return false;
```

The Core Spec restricts the `HCI_Disconnect` Reason parameter to a small allowlist —
`0x05` Authentication Failure, `0x13`/`0x14`/`0x15` (remote/local user or low-resources
termination), `0x1A` Unsupported Remote Feature, `0x29` Pairing with Unit Key Not Supported,
`0x3B` Unacceptable Connection Parameters. **`0x09` (Connection Limit Exceeded) is not in
that set**; the controller answers `Invalid HCI Command Parameters (0x12)`,
`ble_gap_terminate` returns non-zero, and `disconnect()` returns `false` after logging.

Result: the gatecrasher stays connected. Phase 4's central promise — single-owner
exclusivity, and the "completes the earlier BLE-max-connections task" claim — does not hold,
while the code *looks* like it worked. This one is easy to miss on the bench because the
second central often disconnects on its own.

**Correction:** use `BLE_ERR_REM_USER_CONN_TERM` (0x13). Check the `bool` return and log
WARN on failure. Add the same fix to the LAN side (`incoming.stop()` needs no reason code,
so LAN is unaffected).

---

### C4 — Refusing the second central re-enters the shared disconnect-cleanup flag, which has no owner guard on `esp32-N4`

**Plan element:** Phase 4 — *"ESP32 `onConnect`: refuse second central … `disconnect(connInfo, …)`"*.

Terminating the refused link fires `MyBLEServerCallbacks::onDisconnect`
(`esp32_ble_callbacks.h:57-70`), which is a blind flag-setter with **no conn-handle
discrimination**:

```c
bleDisconnectCleanupPending = true;
bleRestartAdvertisingPending = true;
```

`serviceBleDisconnectCleanup()` (`main.cpp:321-348`) then decides whether to tear down. Its
only protection is the `ownerStillUp` guard — and that guard is inside
`#ifdef OPENDISPLAY_HAS_WIFI` (`main.cpp:328-338`).

`OPENDISPLAY_HAS_WIFI` is `TARGET_ESP32 && OPENDISPLAY_ENABLE_WIFI` (`wifi_service.h:13-15`).
Expanding `extends` in `platformio.ini`, exactly two of the eleven envs lack it:
`nrf52840custom` (no NimBLE at all) and **`esp32-N4`** (`platformio.ini:284-299`).

**Concrete failure on `esp32-N4`:** client A is mid-pipe-transfer at chunk 90/300. A phone
running the opendisplay.org web client scans and connects. `onConnect` sees
`getConnectedCount() == 2`, refuses B. B's `onDisconnect` sets the flag. Next loop pass,
`serviceBleDisconnectCleanup` runs with the guard compiled out:

```
main.cpp:343  if (directWriteActive) cleanupDirectWriteState(true);   // panel torn down
main.cpp:346  cleanupPartialWriteOnDisconnect();
main.cpp:347  resetPipeWriteState();                                   // A's transfer destroyed
```

Client A's transfer is killed by a stranger walking past. **This is a new remote-DoS
introduced by the plan**, on the one env that already has the least headroom.

**Correction:** two changes, both required.
1. Give the refusal its own path: set a `bleRefusedGatecrasherPending` flag in `onConnect`
   and have `onDisconnect` skip raising `bleDisconnectCleanupPending` when the departing
   handle is the refused one (capture the conn handle in `onConnect` and compare
   `connInfo.getConnHandle()` in `onDisconnect`).
2. Move the `ownerStillUp` early-return **out** of `#ifdef OPENDISPLAY_HAS_WIFI` — the
   `pServer->getConnectedCount() > 0` half of it is unconditionally correct and costs
   nothing on a no-WiFi build.

---

## HIGH

### H1 — Phase 5 escalates a recoverable command drop into a forced disconnect, and the ring holds 32, not 33

**Plan element:** Phase 5 — *"Command ring full: set `commandQueueOverflowAbort` (host task);
loop services → `abortToKnownState("command queue overflow", dropLink=true)`."*

Today a full ring drops the newest frame (`esp32_ble_callbacks.h:127-129`). The pipe
protocol is *designed* for that: the missing seq shows up as a zero bit in the next SACK
mask and the client retransmits exactly that chunk (`docs/pipe-write-protocol.md` §5.2;
`device.py:2771-2772`). Loss of one frame costs one round trip. Phase 5 turns it into a
link drop plus a full re-auth plus a restarted transfer.

Worse, the ring's usable capacity is `COMMAND_QUEUE_SIZE - 1 = 32`, not 33 — the producer
refuses at `nextHead == tail` (`esp32_ble_callbacks.h:122`). The comment at `main.h:365-370`
("33 slots hold a full W=32 in-flight window + END") is off by one. With W=32 negotiated
(`PIPE_MAX_W = 32`, `structs.h:51`) and the loop task stalled in a Spectra SPI write, a full
window fills the ring exactly; **any** interleaved non-pipe command (a `0x0060` telemetry
poll, a CCCD write) overflows it and, under Phase 5, kills the link.

**Correction:** keep the drop, do not drop the link. Set the flag, log one WARN with the
count, and gate the *abort* on the overflow recurring while `transferActive()` and
`g_lastProgressMs` is already stale — i.e. let the supervisor own the decision. If you want
belt-and-braces, bump `COMMAND_QUEUE_SIZE` to 34 on the envs with DRAM to spare (not
`esp32-N4`) so the documented W=32+END claim is actually true.

### H2 — Phase 2, read literally, makes the cross-transport clobber *worse*, and it ships two phases before the fix

**Plan element:** Phase 2 — *"Queue flush + session clear must run even when the
transfer-owner guard early-returns for the other transport."*

`disconnectWiFiServer()` sets `bleDisconnectCleanupPending = true` (`wifi_service.cpp:812`).
If Phase 2 puts `clearEncryptionSession()` **before** the `ownerStillUp` guard at
`main.cpp:333`, then on every WiFi-lost tick (`main.cpp:452-457`) or LAN client close, a
live, authenticated, mid-transfer BLE session is destroyed — which is §Context item 4, the
bug Phase 4 exists to fix, now reachable from a *second* code path and shipping first.

The guard as written happens to save you today: `transferSessionOrigin()` returns
`sessionOrigin`, which defaults to `0` (`ORIGIN_BLE`, `display_service.cpp:2114`) and is
only written at transfer START, so `lanOwnsSession` is false and `ownerStillUp` reads
`getConnectedCount() > 0`. Putting the clear before it throws that away.

**Correction:** place `flushCommandQueue(); flushResponseQueue(); clearEncryptionSession();`
**after** the `ownerStillUp` early-return, and make the guard unconditional (see C4.2).
Better: land the Phase 4 owner token *first* and scope the clear to
`linkOwner() == OWNER_BLE`. The plan's own note ("until then clear when the departing
transport owned it") is the right instinct — make it a hard requirement, not a parenthetical.

### H3 — Phase 6 deletes the only timeouts that currently work, before the supervisor is trustworthy

**Plan element:** Phase 6 — *"Subsume the 15-min watchdogs: delete main.cpp:436-442 direct-write
block; retire `checkPartialWriteTimeout`'s 15-min path."*

Given C1, the supervisor's `g_lastProgressMs` is refreshed by ordinary command traffic.
Deleting `main.cpp:436-442` (which keys on `directWriteStartTime`, a *wall-clock start*
stamp that nothing refreshes) replaces an unconditional 15-minute bound with a conditional
one that a chatty client defeats. Same for `checkPartialWriteTimeout` (`display_service.cpp:578-587`),
which keys on `partialCtx.start_time`.

**Correction:** keep both wall-clock bounds as a backstop and simply raise them past the
supervisor (e.g. 20 min), or add a second supervisor arm that is *not* progress-based:
`transferActive() && now - g_transferStartMs > OD_TRANSFER_HARD_CAP_MS`. Delete the old
blocks only after hardware soak proves the progress arm fires.

### H4 — nRF: the "single task, no race" premise has a documented hole, and `abortToKnownState` is reachable from the Bluefruit task

**Plan element:** Phase 2 — *"nRF `disconnect_callback`: add `clearEncryptionSession()` (same
task as all crypto — no race)"*; Phase 1 — *"nRF variant … callable from Bluefruit task"*;
Phase 6 — *"nRF: `volatile bool g_commandInFlight` around `imageDataWritten`"*.

The premise is *usually* right and I verified it: both `connect/disconnect_callback`
(`bluefruit.cpp:829`, `:849`) and the characteristic write callback
(`BLECharacteristic.cpp:538-542`) are dispatched through `ada_callback()`, i.e. serialized
on the single "Callback" task (`AdaCallback.c:45`, `:147`).

The hole: `ada_callback_invoke()` allocates its item with `rtos_malloc` and
`VERIFY(cb_data)` returns **false** on failure — at which point `BLECharacteristic.cpp:541`
invokes `_wr_cb` **inline on the BLE event task**:

```c
if ( !(_use_ada_cb.write &&
       ada_callback(request->data, request->len, _wr_cb, ...)) )
{
    _wr_cb(conn_hdl, this, request->data, request->len);   // BLE task, not Callback task
}
```

Heap exhaustion during a large nRF52840 transfer is exactly when this triggers. Then
`imageDataWritten` runs concurrently on two tasks, `clearEncryptionSession()`'s
`memset(session_key, 0, 16)` (`encryption.cpp:205`) can land mid-`aes_ccm_decrypt`, and
`g_commandInFlight` — a plain bool, not a counter — is cleared by whichever nests out
first, letting the supervisor abort under a live handler.

**Correction:** make `g_commandInFlight` a `volatile uint8_t` **depth counter**
(increment/decrement, abort only at 0). On nRF, make `clearEncryptionSession()` from
`disconnect_callback` deferred: set `nrfSessionClearPending` and service it from `loop()`
when the depth counter is 0. Note the inline-fallback path explicitly in the plan so nobody
later relies on "single task" as an invariant.

---

## MEDIUM

### M1 — Asymmetric nonce window (+128 / −32): not a replay hole, but a 4× DoS amplifier

**Plan element:** Phase 3 — *"backward stays −32; forward becomes `OD_NONCE_FORWARD_WINDOW = 4*PIPE_MAX_W = 128`"*.

I worked the exact math on the 64-entry ring and the widening is **not** exploitable as a
replay:

- `encryption.cpp:136` only consults `replay_window[]` when `counter <= last_seen_counter`.
  A forward-accepted counter immediately becomes `last_seen_counter` (`:149-151`), so a
  second copy of it takes the `<=` branch and is caught by the ring.
- Backward window (32) < ring depth (64), so every counter reachable via the backward branch
  is still in the ring. Sound.

What *is* real: the window is one-sided. An attacker who captures a single valid frame at
counter `N` and replays it can jam `last_seen_counter` forward. With `+32` today the damage
is 32 counters; with `+128` it is 128. The client's next 96 legitimate frames
(`N+1 … N+96`) then fall outside the `−32` backward window and are rejected. Under Phase 3
those rejections no longer count as `integrity_failures` (correct), so the session survives
— and stalls, which is exactly the wedge the supervisor must catch. The plan makes the
recovery path load-bearing while widening the trigger.

Ranking this Medium, not Critical: it requires a local attacker with a captured frame, and
the same class of attack exists today at ¼ the magnitude.

**Correction:** widen **both** sides to `OD_NONCE_WINDOW = 128` and grow
`replay_window[]` from 64 to 256 entries (`encryption_state.h:21`; +1.5 KB `.bss`, check it
on `esp32-N4`). If the RAM is not there, keep backward at −32 but cap forward at +64 so a
single replay cannot open a gap wider than the ring can police.

Also: the plan correctly moves `replay_window_index` out of the function-static
(`encryption.cpp:152`) into `encryptionSession`. That is a real bug today —
`clearEncryptionSession()` memsets the ring (`:217`) but leaves the index, so the new
session's first 64 accepts overwrite an arbitrary rotation. **Verified correct; keep it.**

### M2 — Phase 4 exclusivity breaks reconnect after an abrupt client loss

If the central's host crashes or its adapter is reset, the device does not learn the link is
gone until its **supervision timeout** expires (typically 4 s, up to 32 s; negotiated by the
central — the repo only requests PHY/DLE, `ble_init.cpp:` `ble_nrf_request_fast_link`, and
sets nothing on ESP32). During that window, `getConnectedCount()` still reads 1, so the
returning client's reconnect is refused. Today, with `CONFIG_BT_NIMBLE_MAX_CONNECTIONS = 3`
(`sdkconfig.h:613`), it just works.

HA delivery is `async with Device(...)` per push (`device.py:655-697`), so a supervision-timeout
window that eats one delivery attempt will be retried next wake (`delivery.py:452`) — not
fatal, but a visible reliability regression on flaky links.

**Correction:** on refusal, if the *existing* link has no transfer in flight
(`!transferActive()`) **and** its last RX is older than ~10 s, terminate the **old** link
with `BLE_ERR_REM_USER_CONN_TERM` and accept the new one. Refuse only when the incumbent is
actively transferring.

### M3 — `touchForceResume()` and `directWriteTouchSuspended` drift out of sync

**Plan element:** Phase 1 — *"`touchForceResume()` (zero the `s_epd_refresh_suspend` counter)"*.

The counter has a paired bool guard: `handleDirectWriteStart` and the pipe full-frame
setup set `directWriteTouchSuspended = true` (`display_service.cpp:2137`, `:2800`), and
`cleanupDirectWriteState` consumes it (`:2035-2038`). If `abortToKnownState` zeroes the
counter *before* calling `cleanupDirectWriteState`, the subsequent
`touchResumeAfterEpdRefresh()` early-returns at `touch_input.cpp:418` — harmless — but
`directWriteTouchSuspended` is left `true` only if the abort ordering is different from the
plan's. As written (`cleanupDirectWriteState(true)` first, `touchForceResume()` later) the
ordering is fine.

The genuine hazard the prompt flags — touch polling I2C concurrently with SPI streaming —
does **not** materialise on ESP32: `processTouchInput()` also gates on `transferActive()`
(`touch_input.cpp:584`), and `abortToKnownState` clears all three transfer flags before
`touchForceResume()`. On nRF there is no such gate at all (the block is `#ifdef TARGET_ESP32`),
but nRF also runs the transfer on a different task from `processTouchInput()`, so nothing
changes.

**Correction:** have `touchForceResume()` also clear `directWriteTouchSuspended` (expose a
setter or move the bool next to the counter), and assert the counter is 0 afterwards. Low
effort, removes a future footgun. Keep the plan's ordering.

### M4 — `esp32_set_ble_connectable()` has no failure path; a failed `start()` leaves the radio permanently dark

**Plan element:** Phase 4 — *"stop → `setConnectableMode(NON/UND)` → re-push
`setAdvertisementData` → start"*.

The manufacturer-data concern is **handled correctly** — see V4 below. The gap is failure
handling. `NimBLEAdvertising::start()` returns `bool`; `setAdvertisementData()` returns
`bool` and fails if the payload exceeds 31 bytes. If either fails after the `stop()`, the
device is not advertising, is not connected, and nothing retries: `bleRestartAdvertisingPending`
is only raised by `onDisconnect`. That is "unresponsive but not frozen" — a support ticket
that looks like a dead device.

**Correction:** check both return values; on any failure, force
`setConnectableMode(BLE_GAP_CONN_MODE_UND)`, re-push, `start()` again, and set
`bleRestartAdvertisingPending = true` so the loop keeps retrying.

### M5 — Drain-abort `break` placement relative to `commandQueue[tail].pending = false`

**Plan element:** Phase 1 — *"after `imageDataWritten` returns, `if (commandDrainAbortPending)
{ clear; break; }` — break WITHOUT the tail store."*

The index walk is correct as specified. `flushCommandQueue()` sets
`commandQueueTail := head`, the drain then breaks without executing `main.cpp:417`, so tail
stays at the flushed position and nothing is re-dispatched or leaked. **Verified sound.**

The one hazard is placement: the check must go **between** `main.cpp:415` and `main.cpp:416`.
If it goes after `:416`, the `commandQueue[tail].pending = false` writes into a slot the
producer may already have re-filled post-flush. `pending` is write-only today (its only
readers are `main.cpp:291/309/416` — all writes), so the consequence is nil, but it is a
landmine for anyone who later gives `pending` meaning.

**Correction:** state the exact insertion point in the plan, and delete the vestigial
`pending` field while you are in there — it has no readers in either ring.

---

## LOW

### L1 — Plan factual error: the response ring *is* flushed today

§Context item 3 asserts *"neither ring is EVER flushed (heads/tails never reset anywhere)"*.
The response ring is drained to empty on every loop pass with no central connected:

```
main.cpp:307-312
} else {
    while (responseQueueTail != responseQueueHead) {
        responseQueue[responseQueueTail].pending = false;
        responseQueueTail = (responseQueueTail + 1) % RESPONSE_QUEUE_SIZE;
    }
}
```

So Phase 2's `flushResponseQueue()` on BLE disconnect is redundant on ESP32 (it happens one
pass later anyway). Only the **command** ring genuinely survives a disconnect. Harmless to
add, but the plan should not claim it as a fix for a bug that does not exist — and the
finding matters because it means stale *responses* were never the wedge.

### L2 — The 60 s pipe-error reset is safe but far outside client patience, and the doc needs a line

The prompt's worry that Phase 2 breaks `docs/pipe-write-protocol.md` §5.1 does **not** hold.
§5.1 (lines 363-365) promises only that later `0x0081` frames are *silently discarded* until
the next `0x0080` or a disconnect. After `resetPipeWriteState()`, `handlePipeWriteData`'s
first line still discards them (`display_service.cpp:2811`: `if (!pipeState.active ||
pipeState.error) return;`). Client-observable behaviour is identical.

What is off is the plan's rationale: *"preserves the 60 s stable-ACK-position window for
client retries."* py-opendisplay treats every `0x81` NACK as fatal and raises immediately
(`device.py`, `ProtocolError` on fatal NACK); nothing re-reads the ACK position. 60 s is
~4× the client's entire `MAX_PTO` budget. The number is harmless but the justification is
fiction — pick 10 s and say it is a hardware-release deadline, not a client-retry window.

Also: `PipeWriteState` (`structs.h:106-126`) has no timestamp field at all, so
`error_since_ms` is a genuine struct addition, not a rename. Fine on RAM (4 bytes).

### L3 — "No hardware WDT" is true only because every long wait yields

The plan's premise is defensible but the surrounding facts are not quite as stated:

- The FreeRTOS Task WDT **is** enabled with panic on the pinned platform:
  `CONFIG_ESP_TASK_WDT_INIT 1`, `CONFIG_ESP_TASK_WDT_PANIC 1`,
  `CONFIG_ESP_TASK_WDT_TIMEOUT_S 5`, watching `IDLE_TASK_CPU0`
  (`framework-arduinoespressif32-libs/esp32s3/qio_qspi/include/sdkconfig.h:906-910`). Plus
  `CONFIG_ESP_INT_WDT` at 300 ms (`:903-904`).
- Every ESP env passes `-DCONFIG_FREERTOS_WATCHDOG_TIMEOUT_S=120` (e.g. `platformio.ini:189`).
  That symbol does not exist in IDF 5.x (it is `CONFIG_ESP_TASK_WDT_TIMEOUT_S`), and the
  precompiled `sdkconfig.h` wins anyway — the same trap the repo already documented for
  `CONFIG_BT_NIMBLE_MAX_CONNECTIONS`. **The flag is inert.** Either delete it or fix the
  name; leaving a dead knob that reads like a 120 s guarantee is how the next reader gets
  the sizing wrong.
- On single-core C3/C6 the Arduino loop task shares CPU0 with IDLE0, so a genuinely
  non-yielding `loop()` *would* panic-reboot in 5 s. It does not today because every long
  wait yields (`waitforrefresh` `delay(10)` at `display_service.cpp:759`; `bbepWaitBusy`
  `bbepLightSleep(20)` at `bb_ep.inl:3973`). Worth one sentence in the plan so a future
  busy-spin does not get added casually.

The plan's RTC rationale is **correct and already hardware-proven** — see V5.

### L4 — Minor line-reference drift in the plan

Not worth churn, but for the record: `communication.cpp:113-116` is actually `:117-121`
(the ring-full check); `sdkconfig.h:471` is `:613` on the S3 variant; `display_fastepd.cpp:222-231`
lands on `fastepd_full_update` at `:227-231`. Everything else I spot-checked
(`encryption.cpp:149-155`, `display_service.cpp:401-408`, `power_latch.cpp:87-90`,
`main.cpp:406-423`/`:321-348`/`:436-442`, `device_control.cpp:227-240`,
`esp32_ble_callbacks.h:128`, `touch_input.cpp:584`, `wifi_service.cpp:804`/`:879`) is exact.

---

## What the plan MISSES — freeze vectors not addressed

### X1 — I2C bus hang: `Wire.setTimeOut()` is never called anywhere

`grep -n "setTimeOut" src/*.cpp` returns nothing. The firmware drives four I2C peripherals
(AXP2101 PMIC in `display_service.cpp`, GT911 touch in `touch_input.cpp`, SHT40, BQ27220)
through `Wire`, whose Arduino-ESP32 default timeout is 50 ms per transaction — survivable —
**but** `wireBeginForOpenDisplay()` (`display_service.cpp:785-800`) and
`invalidateOpenDisplayWire()` never re-assert it after a re-`begin()`. More importantly a
slave holding SDA low (classic GT911 wedge after a rail cut) is not recovered by a timeout:
it needs the nine-clock bus-recovery pulse train, which the firmware never issues.

This is a real, common e-paper-tag freeze mode (touch controller and panel share the rail
that `epdSessionForceOff` cuts), and the supervisor cannot see it — `transferActive()` is
false while `processTouchInput()` spins. **Recommend:** add explicit `Wire.setTimeOut(25)`
after every `Wire.begin()`, and a nine-clock SDA-recovery routine invoked when
`rt->i2c_fail_streak` crosses a threshold (`touch_input.cpp:400` already tracks it).

### X2 — The boot refresh is invisible to every `epdRefreshInProgress` gate

`refreshBootScreenFull()` (`display_service.cpp:533-542`) calls `touchSuspendForEpdRefresh()`
+ `bbepRefresh` + `waitforrefresh(60)` and **never sets `epdRefreshInProgress`**. Neither
does the FastEPD boot path (`:1588-1594`). Consequences: `serviceBleDisconnectCleanup`
(`main.cpp:322`), `esp32_restart_ble_advertising` (`ble_init.cpp:236`), the `workInFlight`
gate (`main.cpp:478`) and the plan's new `idleLinkTick()` / supervisor "never interrupt a
refresh" rule all mis-read a boot refresh as idle. On a Spectra panel that is a 30–60 s
blind spot on every cold boot, and the retry path (`:1624-1633`) can double it.

**Recommend:** set/clear `epdRefreshInProgress` around both boot-refresh paths as a Phase 0
one-liner. It is free and it makes the supervisor's refresh exemption honest.

### X3 — `fastepd_wait_refresh()` is a stub — the IT8951 path has *zero* firmware-side bound

```
src/display_fastepd.cpp:228-231
bool fastepd_wait_refresh(int timeout_sec) {
    (void)timeout_sec;
    return !s_init_failed;
}
```

`waitforrefresh(60)` short-circuits to this on FastEPD builds (`display_service.cpp:749-751`),
so the "60 s cap" the plan cites does not exist on IT8951/E1004. The plan's Phase 0 mitigation
(log if `fullUpdate` exceeds 120 s) wraps `fastepd_full_update` (`:227-231`) but **not**
`fastepd_direct_refresh`, which is the path a real transfer takes
(`display_service.cpp:2422-2423`). Post-hoc logging on the wrong function is not the
"documented residual risk" the plan claims to have accepted.

**Recommend:** wrap `fastepd_direct_refresh` too, and implement `fastepd_wait_refresh` as a
real busy poll against the IT8951 LUT-busy register with the passed `timeout_sec`.

### X4 — Config chunked-write has no timer of its own

`chunkedWriteState.active` is set at `communication.cpp:496` and cleared only on completion
(`:574`), on a malformed chunk (`:558`), or on an auth failure (`:550`). A client that sends
`0x0040` START and vanishes leaves it latched forever — and, unlike the transfer flags, it
is not covered by any existing watchdog. The plan adds `resetChunkedWriteState()` and puts
`chunkedWriteState.active` in the supervisor predicate, which is correct — but per C1 the
supervisor never fires. Once C1 is fixed this is covered; flagging it so the dependency is
explicit.

### X5 — Deep-sleep entry with a live transfer

`enterDeepSleep()` (`main.cpp:577`) is reachable from the advertising-window branch
(`main.cpp:388`) and the idle branch (`:498`). The idle branch is gated by `workInFlight`,
which includes `epdRefreshInProgress` and `getConnectedCount() > 0` — but **not**
`transferActive()`. A pipe transfer whose client link has already dropped (so
`getConnectedCount() == 0`) but whose `pipeState.active` is still latched, with
`bleDisconnectCleanupPending` deferred behind `epdRefreshInProgress`, can reach
`enterDeepSleep()` with the panel rail up. Rare, but the plan's `abortToKnownState` is the
natural place to close it.

**Recommend:** add `transferActive()` to the `workInFlight` disjunction at `main.cpp:474-479`.
One term, no behaviour change in the normal case.

### X6 — Buzzer and LED timers are outside every bound

`buzzerService()` / `processLedFlash()` run unbounded from `loop()`. The plan's
`abortToKnownState` lists "buzzer/LED stop" — good — but nothing bounds a stuck
`buzzer_control` sequence in the first place. Low probability; noting for completeness.

### X7 — Nothing detects "advertising stopped and never restarted"

`esp32_restart_ble_advertising()` early-returns without starting when
`epdRefreshInProgress` (`ble_init.cpp:236-239`, re-arming the flag) or when
`getConnectedCount() > 0` (`:232-235`, **clearing** the flag). The second is the hole: if
the count is stale-nonzero (refused gatecrasher, C3/C4) the flag is cleared and advertising
never resumes. **Recommend:** a cheap `advertisingHealthTick()` — if no peer, not advertising
(`BLEDevice::getAdvertising()->isAdvertising()`), and no pending flag for > 30 s, force a
restart and log WARN.

---

## Phase-ordering hazards

| Phase shipped alone | Verdict |
|---|---|
| **0** | Net **regression** as written, because of the `pwrmgmLockTake` steal (C2). The `powerOff` and drain-cap bounds are fine. Ship Phase 0 only after C2's correction (fail-closed instead of steal). |
| **1** | Safe alone. The drain-trap fix (M5) and the queue flushes are self-contained and correct. `abortToKnownState` has no callers until Phase 5/6, so it is dead code — which is fine, but means Phase 1 provides no field benefit on its own. |
| **2** | **Must not ship before Phase 4** if implemented literally (H2). With the clear placed *after* an unconditional owner guard, it is safe and beneficial alone. The nRF half needs H4's deferral. |
| **3** | Safe alone and the highest value-per-risk phase — it removes the actual root cause of the field failures. Ship it **first**. |
| **4** | Blocked on C3 (wrong reason code) and C4 (cleanup re-entry). Once fixed, safe alone. |
| **5** | Net **regression** alone: the command-ring abort (H1) and the RX/TX-keyed idle timer (C1's twin) each make things worse without the corrected progress accounting from Phase 6. |
| **6** | Net **regression** alone as written: it deletes two working wall-clock watchdogs (H3) and replaces them with a predicate that command traffic defeats (C1). |

**Recommended order:** 3 → 0(corrected) → 1 → 4(corrected) → 2 → 6(corrected, keeping the
old watchdogs) → 5(corrected). That front-loads the phase that fixes the reported field
failure and puts the owner token in place before anything depends on it.

---

## Verified CORRECT — do not churn on these

1. **`verifyNonceReplay` commits before tag verification.** `encryption.cpp:149-155` writes
   `last_seen_counter` and the ring *before* `aes_ccm_decrypt` runs at `:714`. Real bug,
   correctly diagnosed, and the `nonceCheck`/`nonceCommit` split is the right shape.
2. **Nonce failures should not touch `integrity_failures`.** `encryption.cpp:691-697`
   currently increments on *any* `verifyNonceReplay` failure — including plain packet loss.
   Confirmed as the mechanism behind §Context item 1. Fix is correct.
3. **The pipe fatal-NACK latch has no timeout.** `sendPipeNack` (`display_service.cpp:2562-2578`)
   sets `pipeState.error = true` and calls `cleanupDirectWriteState(true)`, which clears
   `directWriteActive` (`:2011`) — so `main.cpp:436-442`'s watchdog no longer applies, and
   `checkPartialWriteTimeout` (`:578`) only covers `partialCtx`. For a non-partial pipe
   transfer there is genuinely **no** bound. Diagnosis exact.
4. **`setConnectableMode(NON)` calls `setFlags(0)` and the re-push is required.** Confirmed
   at `NimBLEAdvertising.cpp:82-84`; `setFlags(0)` is one-way (UND does not restore it), and
   `setAdvertisementData(*advertisementData)` copies the app object which carries
   `setFlags(0x06)` from `ble_init.cpp:302`. The plan's mitigation, and the
   "`setAdvertisementData` must be LAST" trap at `ble_init.cpp:307-312`, are both correct.
5. **RTC memory does not survive a non-deep-sleep reset.** `main.cpp:96-100` documents it
   from hardware (`FINDINGS_DEEP_SLEEP_WAKE_BOOT_SCREEN_2026-07-07.md`), and
   `displayed_etag` is `RTC_DATA_ATTR` (`main.h:294`). The "reset state, never reboot"
   decision is well-founded.
6. **NimBLE adds the peer before `onConnect`.** `NimBLEServer.cpp:464-471` fills
   `m_connectedPeers` then calls `onConnect`, so `getConnectedCount() > 1` is a valid
   gatecrasher test. (The refusal *mechanism* is still wrong — C3.)
7. **`-DCONFIG_BT_NIMBLE_MAX_CONNECTIONS=1` is inert.** `sdkconfig.h:613` defines it to 3.
   Correct to enforce in code.
8. **nRF `Bluefruit.begin(1, 0)` already caps at one link.** `ble_init.cpp:152`. Correct.
9. **Clearing the session on disconnect is client-compatible.** py-opendisplay authenticates
   inside `__aenter__` on every connection (`device.py:670`) and calls `_clear_session()` in
   `__aexit__` (`:697`) and on any setup failure (`:680-683`). HA's delivery path is one
   `async with Device(...)` per push. No client assumes session persistence across a link
   drop. **No compatibility risk.**
10. **The 10-minute supervisor timeout is not too short for legitimate work.** Worst-case
    legitimate blocks: `waitforrefresh(60)` = 60 s (`display_service.cpp:2423`),
    `bbepWaitBusy` 30 s, and the E1004 ~960 KB upload — but the client itself gives up far
    sooner (`TIMEOUT_PIPE_DATA_COMPRESSED = 5.0` × `MAX_PTO = 3` ≈ 15 s;
    `TIMEOUT_REFRESH = 90.0`, `device.py:457-473`). Provided the stamps are moved per C1, no
    legitimate transfer comes near 600 s of no progress. The number is fine.
11. **The 2 s drain cap is safe.** The drain already caps at `COMMAND_QUEUE_SIZE` iterations
    and flushes responses between commands (`main.cpp:419-421`); a wall-clock cap on top
    changes nothing in the normal case.
12. **The queue-flush SPSC reasoning is right.** `commandQueueTail` is written only by the
    consumer, and every call path the plan names (`serviceBleDisconnectCleanup`, the drain
    loop, LAN dispatch, `abortToKnownState`) is on the loop task. `tail := head` snapshot is
    the correct flush. `responseQueue` head/tail are both loop-task-only, so its flush is
    trivially safe.
13. **The OTA exception is genuinely satisfied.** ESP32 `0x0051` is `esp_restart`; nRF DFU
    jumps to the bootloader. Nothing to special-case.
14. **Phase 2's 60 s pipe-error reset does not break `pipe-write-protocol.md` §5.1** — see
    L2. The doc's contract is about client-observable discard behaviour, which is unchanged.
    Update §5.1 to mention the reset, but no protocol change is needed.

---

## Summary of required corrections

| # | Phase | Correction |
|---|-------|-----------|
| C1 | 6, 5 | Stamp progress only on state-machine advancement; never on command dispatch or notify. Key the idle timer the same way. |
| C2 | 0 | Replace the 10 s lock steal with a 60 s fail-closed take; add an owner field if a steal is ever genuinely needed. |
| C3 | 4 | Use `BLE_ERR_REM_USER_CONN_TERM` (0x13), not `BLE_ERR_CONN_LIMIT` (0x09). Check the return value. |
| C4 | 4 | Discriminate the refused conn handle in `onDisconnect`; move the `ownerStillUp` guard out of `#ifdef OPENDISPLAY_HAS_WIFI`. |
| H1 | 5 | Command-ring overflow logs and drops; it does not drop the link. Fix the off-by-one in the `main.h:365` comment. |
| H2 | 2 | Place `clearEncryptionSession()` after the owner guard, or land Phase 4's token first. |
| H3 | 6 | Keep the wall-clock watchdogs as a backstop until the progress arm is soak-proven. |
| H4 | 1, 2, 6 | `g_commandInFlight` becomes a depth counter; defer the nRF disconnect-time session clear to `loop()`. |
| M1 | 3 | Make the nonce window symmetric (±128 with a 256-entry ring, or ±64). |
| M2 | 4 | Prefer evicting an idle incumbent over refusing a reconnect. |
| M4 | 4 | Check `setAdvertisementData`/`start()` returns; fall back to connectable. |
| X1 | new | `Wire.setTimeOut()` + a nine-clock I2C recovery routine. |
| X2 | 0 | Set `epdRefreshInProgress` around both boot-refresh paths. |
| X3 | 0 | Wrap `fastepd_direct_refresh`, and implement `fastepd_wait_refresh` for real. |
| X5 | 6 | Add `transferActive()` to the `workInFlight` disjunction. |
| X7 | new | `advertisingHealthTick()` — detect a permanently dark radio. |

## Build/portability check

New `src/session_guard.cpp` compiles into **all eleven** envs. Guards needed:
`pServer` / `responseQueue` / `commandQueue` under `#ifdef TARGET_ESP32`; LAN calls under
`#ifdef OPENDISPLAY_HAS_WIFI` (**not** `TARGET_ESP32` — `esp32-N4` is ESP32 without WiFi,
`platformio.ini:284`). `lib_ignore = NimBLE-Arduino` on nRF (`platformio.ini:36`) means
`session_guard.cpp` must not include `ble_init.h`'s NimBLE surface unguarded. RAM cost is a
handful of scalars plus `PipeWriteState::error_since_ms` (+4 B) — fine even on `esp32-N4`,
which is DRAM-tight enough to need `PIPE_SMALL_DRAM_WINDOW` (`structs.h:45-48`). If M1's
256-entry `replay_window` is adopted (+1.5 KB `.bss`), verify `esp32-N4` links before
committing to it.
