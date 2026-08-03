# Phase 2 Implementation Plan — Bound the Refresh Waits

**Branch:** `debug/freeze-fix-phase2` (cut from Phase 1 as-built) · **Date:** 2026-07-26
**Parent plan:** [`PLAN_FREEZE_PROOFING_2026-07-26.md`](PLAN_FREEZE_PROOFING_2026-07-26.md) § "Phase 2"
**Supersedes:** [`PLAN_PHASE2_BOUND_WAITS_2026-07-26.md`](PLAN_PHASE2_BOUND_WAITS_2026-07-26.md)
— **obsolete**, retained for its analysis of the five cut items and the eight decisions they carried.

---

## What Phase 2 is now

**Three edits, two files, one subsystem: the e-paper refresh wait.**

| # | Item | File | Targets |
|---|---|---|---|
| **P2-3** | `epdRefreshInProgress` around both boot-refresh paths | `display_service.cpp` | both (inert on nRF today) |
| **P2-8** | `waitforrefresh` → wall-clock deadline instead of an iteration count | `display_service.cpp` | both — **nRF-critical** |
| **P2-4** | `fastepd_wait_refresh` → a real bounded wait instead of a stub | `display_fastepd.cpp` | ESP32 + FastEPD only |
| *(P2-7)* | *optional, recommend defer* — `Wire.setTimeOut(25)` | `display_service.cpp` | ESP32 |

Landing order **P2-3 → P2-8 → P2-4**. Independent of each other; each is separately revertable.

**Not in scope, and deliberately so.** The earlier plan also proposed a `pwrmgmLockTake` deadline, a
`powerOff` stuck-button bound, a loop-drain cap, deleting the inert TWDT build flag, and a
loop-liveness monitor task. All five were cut. Their specifications and the reasoning behind them
live in the superseded document; the residual each leaves is recorded in the parent plan and
restated under "What Phase 2 does not do" below.

**Constraints.** No new file. No new task. No change to `src/main.cpp`, `platformio.ini`, or
anything under `include/`. `src/session_guard.*` belongs to Phase 3 and is not referenced here.

---

## The problem, stated precisely

A refresh is the longest thing this firmware does — 30–60 s on a colour panel. Three defects mean
that wait is either unmeasured, unbounded, or invisible:

### 1. `waitforrefresh` counts iterations, not time

[display_service.cpp:747-776](../src/display_service.cpp):

```c
for (size_t i = 0; i < (size_t)(timeout * 100); i++){
    delay(10);
    ...
    if(!bbepIsBusy(&bbep)){ ... return true; }
}
od_log_warn("Refresh timed out");
```

`timeout * 100` iterations of `delay(10)` equals `timeout` seconds **only if each iteration really
takes 10 ms**. `delay()` is `vTaskDelay` on both targets — it yields, and it guarantees only a
*minimum*. Under contention each pass can take arbitrarily longer, so the "60 s" bound is really
"60 s of scheduled time for this task", with no ceiling in wall-clock terms.

**This is worst exactly where it matters most.** Phase 1 established on hardware that nRF's `loop()`
task (priority 1, `rtos.h:58`) is starved mid-transfer by the callback task (2) and the Bluefruit
task (3) — its deferred link-drop never ran, forcing the inline disconnect in `23ecaed`. A refresh
wait that measures scheduled iterations is precisely the wrong instrument on a task that can be
descheduled for long stretches.

### 2. On FastEPD panels there is no wait at all

[display_fastepd.cpp:228-231](../src/display_fastepd.cpp) — the whole function:

```c
bool fastepd_wait_refresh(int timeout_sec) {
    (void)timeout_sec;
    return !s_init_failed;
}
```

It ignores its timeout and returns immediately. And [display_service.cpp:749](../src/display_service.cpp)
short-circuits to it before any of the polling loop above:

```c
if (fastepd_driver_used()) return fastepd_wait_refresh(timeout);
```

So on IT8951/E1004, `waitforrefresh(60)` returns `true` in microseconds. **The documented 60 s cap
does not exist on those panels, and neither does the wait** — callers proceed as though the panel
had finished. This is `[X3]` from the original review, confirmed against the current tree.

### 3. Boot refreshes are invisible to every gate

`epdRefreshInProgress` ([display_service.cpp:85](../src/display_service.cpp)) is set/cleared around
the two *transfer* refresh paths — [:2415](../src/display_service.cpp)/[:2436](../src/display_service.cpp)
and [:3284](../src/display_service.cpp)/[:3294](../src/display_service.cpp) — but **not** around
either boot path:

| Boot path | Site | Sets the flag? |
|---|---|---|
| `refreshBootScreenFull()` | [:532-542](../src/display_service.cpp) — `bbepRefresh` then `waitforrefresh(60)` | ❌ |
| FastEPD boot | [:1587-1595](../src/display_service.cpp) — `fastepd_full_update()` then `waitforrefresh(60)` | ❌ |

Its consumers all treat the flag as "a refresh is in flight, do not disturb":
[ble_init.cpp:236](../src/ble_init.cpp) (advertising restart),
[main.cpp:322](../src/main.cpp) (defer disconnect cleanup),
[main.cpp:482](../src/main.cpp) (`workInFlight` / deep-sleep gate), and Phase 6's supervisor rule
"never interrupt a refresh". During a 30–60 s boot refresh every one of them believes the device is
idle.

---

## P2-3 — Set `epdRefreshInProgress` around both boot paths

Wrap each boot refresh exactly as the transfer paths already do: set before the refresh call, clear
after the wait returns, on **every** exit path including the failure returns.

- `refreshBootScreenFull()` [:532-542](../src/display_service.cpp) — note the early `return false`
  when `writeBootScreenWithQr()` fails; the flag must not be left set on that path.
- FastEPD boot [:1587-1595](../src/display_service.cpp) — the flag must be cleared before
  `epdSessionForceOff()`.

**Scoped-guard pattern preferred** over paired assignments, so a future early return cannot leak the
flag. If a plain pair is used instead, say why in the commit message.

**On nRF this is inert today, and that is fine.** All three current consumers are inside ESP32-only
code, so setting the flag on nRF changes nothing at runtime. It is still correct, it costs nothing,
and Phase 6 adds the nRF consumers. **Say so in the commit message** — otherwise the next reader
sees a flag set and never read, and "fixes" it.

---

## P2-8 — Give `waitforrefresh` a real deadline

Replace the iteration count with a `millis()` deadline. Keep everything else: the 10 ms poll, the
`i == 0` "never went busy" error, the progress dots, the completion log.

```c
const uint32_t deadline = millis() + (uint32_t)timeout * 1000u;
bool first = true;
while ((int32_t)(millis() - deadline) < 0) {
    delay(10);
    ...
    if (!bbepIsBusy(&bbep)) { /* first-pass error check, elapsed log, return true */ }
    first = false;
}
od_log_warn("Refresh timed out after %u ms (deadline %d s)", elapsed, timeout);
return false;
```

Three details that matter:

1. **Signed-difference comparison**, `(int32_t)(millis() - deadline) < 0`, not `millis() < deadline`
   — correct across the 49.7-day `millis()` wrap. The codebase already uses this idiom
   ([display_service.cpp:522](../src/display_service.cpp)).
2. **Keep the `i == 0` semantics** as a *first-iteration* check, not an index test. It catches "the
   panel never asserted BUSY", i.e. the refresh never started, which is a different failure from a
   timeout and is worth keeping distinct in the log.
3. **Report elapsed wall-clock on both exits.** The current success path logs `i / 100` — an
   iteration count presented as seconds, which is exactly the confusion this item removes.

**Timebase caveat, accepted (was D-K).** On nRF `millis()` is `tick2ms(xTaskGetTickCount())` — the
FreeRTOS tick at 1024 Hz with `configUSE_TICKLESS_IDLE 1` — not a hardware timer as on ESP32. It
advances while the task is descheduled, which is what this item needs, but it is not a
high-integrity clock: a fault that stops the scheduler also stops the deadline. That fault class is
already accepted as unrecoverable software-side. **Do not chase it here.**

---

## P2-4 — Make `fastepd_wait_refresh` real

Implement it as a bounded poll of the IT8951 LUT-busy state, honouring `timeout_sec` with the same
`millis()` deadline idiom as P2-8, returning `false` on expiry.

**Wrap the path a real transfer takes.** [display_service.cpp:2422-2423](../src/display_service.cpp):

```c
fastepd_direct_refresh(refreshMode);
refreshSuccess = waitforrefresh(60);
```

`fastepd_direct_refresh` is what a transfer calls — not only `fastepd_full_update`. Both must end up
covered; the original `[X3]` finding specifically called out wrapping the wrong one.

**Blast radius is one env family.** Guarded by `TARGET_ESP32 && OPENDISPLAY_FASTEPD` and reached
only when `fastepd_driver_used()`, so `esp32-s3-E1004` is the build and behaviour gate. Every other
env keeps today's `bbepIsBusy` loop unchanged.

**This changes observable timing.** Today the call returns immediately; afterwards it blocks until
the panel is genuinely idle. That is the point — callers currently believe a refresh finished when
it had not — but it means the E1004 upload regression test is not optional (see Verification).

---

## P2-7 — *(optional, recommend defer)*

`Wire.setTimeOut(25)` after each `Wire.begin()`, halving the ~50 ms Arduino default that a failing
GT911 transaction blocks for. Worst case today is ~250 ms of blocked `loop()` before the driver
disables the controller after 5 consecutive failures — bounded and acceptable, which is why `[X1]`
was downgraded. **ESP32 only: the API does not exist on nRF**, whose TWIM driver busy-spins with no
timeout at all (`Wire_nRF52.cpp:166-181`). That nRF gap needs a physical fault to trigger, is
outside this effort's freeze class, and is *not* addressed here — but note that `[X1]`'s
"the driver gives up after 5 failures" reasoning is ESP32-specific and does **not** transfer to nRF.

---

## What Phase 2 does not do

Recorded so nobody assumes Phase 2 covered it:

- **`pwrmgmLockTake` stays unbounded** on both targets ([display_service.cpp:401-408](../src/display_service.cpp)).
  A panel-lock holder that never releases blocks its waiter forever. No `panelStateUnknown` flag is
  produced — **Phase 3 must not expect one**.
- **A stalled `loop()` is undetected on both targets.** ESP32's TWDT (5 s, panic on IDLE0) will not
  fire, because every long wait here yields and IDLE0 is never starved. nRF has no watchdog at all.
  Detection is entirely Phase 6's.
- **`powerOff`'s stuck-button wait stays unbounded** (ESP32-only, needs a hardware fault).
- **The inert `-DCONFIG_FREERTOS_WATCHDOG_TIMEOUT_S=120` stays** in 9 ESP envs, still implying a
  120 s watchdog that does not exist.

**So Phase 2 bounds the refresh waits. It does not detect stalls, and it is not the "defensive
floor" the earlier plan described** — that framing depended on the monitor task and no longer holds.

---

## Decisions

None blocking. Four carried over, all with a default:

| | Question | Default |
|---|---|---|
| **D-D** | Accept the `[X3]` downgrade (implement the real wait rather than a larger redesign)? | **yes** — this plan assumes it |
| **D-G** | Include P2-7? | **no**, defer |
| **D-K** | Accept nRF's tick-derived `millis()`? | **yes** — see P2-8 |
| **D-L** | nRF I2C busy-spins with no timeout: bound it or accept it? | **accept**, out of scope |

---

## Verification

### Build

```bash
/home/davelee/.platformio/penv/bin/pio run          # all 12 envs; pio is NOT on PATH
```

`esp32-s3-E1004` is the P2-4 gate; `nrf52840custom` is the P2-8 gate. CI also runs the `host-tests`
job added by Phase 1 — Phase 2 adds nothing to it.

### Static

- `git diff --stat` touches **only** `src/display_service.cpp` and `src/display_fastepd.cpp`.
  Any new file, or any change to `src/main.cpp`, `platformio.ini` or `include/`, means scope crept.
- `pwrmgmLockTake` is **unchanged**, signature included.
- No `epdRefreshInProgress = true` without a matching clear on every path out, including failures.
- No remaining `timeout * 100` iteration arithmetic in `waitforrefresh`.

### Hardware — both targets required

1. **Boot refresh (P2-3)** — cold boot with a colour panel; confirm a BLE connect during the boot
   refresh is deferred rather than acted on mid-refresh, and that deep sleep is not entered during it.
2. **Normal refresh (P2-8)** — full transfer + refresh completes; the completion log now reports
   **elapsed milliseconds**, and it should be close to the real refresh duration.
3. **Timeout path (P2-8)** — force a panel that never clears BUSY; confirm the warning fires at
   ~`timeout` seconds of *wall clock*, not later.
4. **Never-started path (P2-8)** — confirm the "not busy after refresh command" error still fires,
   distinct from a timeout.
5. **E1004 regression (P2-4)** — a ~960 KB upload + refresh completes, and the wait now takes real
   time instead of returning instantly. **This is the test most likely to surface a surprise**, since
   callers have never previously waited on this path.
6. **nRF under load (P2-8)** — refresh during a BLE transfer, where the loop task is contended. The
   deadline should hold in wall-clock terms; an iteration-counted wait would have overrun.

---

## Residual risk after Phase 2

- A hard hang inside a library call below our waits (wedged SPI, stuck DMA) is still invisible —
  `bbepWaitBusy`'s own 30 s cap and `it8951WaitForLUTReady`'s 30 s cap are the library's, not ours.
  Consistent with the software-only decision.
- A single long refresh still owns `loop()` for its duration. By design: interrupting a refresh is
  worse than waiting for it. P2-8 bounds it; it does not shorten it.
- On nRF, a scheduler-stopping fault stops `millis()` and therefore the deadline (D-K).
- Everything under "What Phase 2 does not do" — most consequentially, **nothing here detects a
  stalled `loop()`**. That now rests entirely on Phase 6, on both targets.
