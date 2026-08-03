# Phase 2 Implementation Plan — Bound Every Unbounded Wait

> # ⛔ OBSOLETE — SUPERSEDED
>
> **This plan is no longer the Phase 2 specification.** It was written for a seven-item Phase 2;
> five of those items (P2-1, P2-2, P2-5, P2-6, P2-9) were subsequently cut, leaving a document that
> is mostly struck-through material.
>
> **The current plan is
> [`PLAN_PHASE2_REFRESH_BOUNDS_2026-07-26.md`](PLAN_PHASE2_REFRESH_BOUNDS_2026-07-26.md)** — three
> items (P2-3, P2-8, P2-4), two files, written from scratch against the current tree.
>
> **Do not implement from this document.** It is retained for one reason: the analysis behind the
> cut items is expensive to reproduce and is what stops each being re-proposed —
> `[C2]`'s do-not-steal argument for the panel lock, P2-5's arithmetic showing a drain cap is
> powerless, P2-9's rejected alternatives (nRF hardware WDT, idle-hook heartbeat), and the eight
> decisions those items carried. Read it as an appendix to the current plan, not as a plan.


**Branch:** `debug/freeze-fix-phase2` (branched from Phase 1 as-built) · **Date:** 2026-07-26
**Parent plan:** [`PLAN_FREEZE_PROOFING_2026-07-26.md`](PLAN_FREEZE_PROOFING_2026-07-26.md) § "Phase 2"
**Review that shaped it:** [`FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md`](FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md) `[C2] [X1] [X2] [X3] [L3]`

Phase 2 bounds the **refresh waits**. It adds no new subsystem, no new state machine, and after the
2026-07-26 scope cut no new file and no new task: it puts a wall-clock bound on each place where `loop()` can block indefinitely, and
it makes the one gate the supervisor depends on (`epdRefreshInProgress`) actually cover every
refresh.

> **Scope, in one line:** three items — P2-3, P2-8, P2-4 (+ optional P2-7) — touching
> `src/display_service.cpp` and `src/display_fastepd.cpp` and nothing else. Everything cut is in the
> appendices, struck through, with its residual recorded. Nothing blocks implementation.
>
> **Adjusted 2026-07-26 for Phase 1 as-built.** Phase 1 has shipped (`02bdd5c`..`d62cb29`) and this
> branch is cut from it, so the "independent, can land before or after" framing is now moot in
> practice: Phase 2 lands *on top of* Phase 1. Three concrete consequences, folded in below.
>
> 1. **The `esp32-N4` RAM gate moved in Phase 2's favour, not against it.** Phase 1 did *not* ship
>    `replay_window[256]` (+1,536 B) as this plan assumed. Decision B changed to a 32 B sliding
>    bitmap replacing the 512 B ring, so Phase 1 **gave back 480 B**: `esp32-N4` measured
>    81,940 → **81,468 B**. P2-9's ~2 KB task stack has *more* headroom than budgeted here.
> 2. **P2-9's core premise is now confirmed by shipped code, not just by reading the core.** Phase 1's
>    `23ecaed` was forced to drop the BLE link *inline from the nRF callback task* precisely because
>    `loop()` is starved mid-transfer — the same starvation P2-9 exists to observe. The priority
>    facts in the table below were independently re-verified during that work.
> 3. **Phase 1 pulled Phase 5's link-drop forward** (an auth-gate guard that disconnects after 10
>    consecutive unauthenticated commands). Phase 2 does not interact with it, but it means an
>    out-of-`loop()` actor that drops the link **already exists** on nRF — relevant context for D-H
>    and D-I, which assumed P2-9 would be the first such actor. Neither decision is reopened here.
>
> Line references to `src/main.cpp` and `src/communication.cpp` in this plan were taken against
> `02bdd5c`; Phase 1 modified both, so re-anchor before editing rather than trusting a line number.

**Both targets, or it does not count.** Every bound here must be *binding* on `nrf52840custom` as
well as the ESP32 envs — executing, measured in a clock that keeps running, and observable when
violated. Most of the original findings were ESP32-shaped and four of them compile out on nRF; §
"Making the bounds binding on both targets" works through the coverage per item and adds the two
items (**P2-8**, **P2-9**) needed to close the nRF side.

**Scope discipline.** Phase 2 must remain self-contained — `src/session_guard.*` does not exist
yet (Phase 3) and `abortToKnownState()` cannot be called from here. Where Phase 2 produces a
signal that Phase 3 will consume, it exports a flag and a getter and nothing else. Those
hand-off points are marked **→ Phase 3** below.

---

## Verification done before writing this plan

Two of the parent plan's five Phase-2 findings did not survive re-verification against the actual
sources. `[X1]` was already downgraded by the parent plan. `[X3]` is downgraded here for the same
reason: the claim was made from the shape of the firmware-side stub without reading the library
underneath it.

| Parent claim | Verified status |
|---|---|
| `[C2]` `pwrmgmLockTake` spins forever; a steal is unsafe | **CONFIRMED.** [display_service.cpp:401-408](../src/display_service.cpp) is `while (exchange(&pwrmgmLock,1)) delay(1);` with no deadline. Legit holds really do reach 30 s: `bbepWaitBusy` uses `iMaxTime = 30000` for `BBEP_3COLOR\|4COLOR\|7COLOR` (`bb_ep.inl:3966-3968`), and `epdSessionForceOffLocked` holds the lock across that. The steal is unsafe exactly as described — `pwrmgmLock` is a bare `volatile uint8_t` ([main.h:185](../src/main.h)) with no owner field. |
| `powerOff` stuck-button loop | **CONFIRMED.** [power_latch.cpp:85-90](../src/power_latch.cpp) — `while (digitalRead(buttonPin()) == LOW) delay(20);` with no bound. ESP32-only file (`#if defined(TARGET_ESP32)`, [:3](../src/power_latch.cpp)). |
| `[X2]` boot refreshes bypass `epdRefreshInProgress` | **CONFIRMED.** The flag is set in exactly two places — [display_service.cpp:2415/2436](../src/display_service.cpp) (direct-write END refresh) and [:3284/:3294](../src/display_service.cpp) (partial refresh). Neither boot path sets it: `refreshBootScreenFull` [:533-542](../src/display_service.cpp) and the FastEPD boot path [:1586-1594](../src/display_service.cpp). |
| `[X3]` FastEPD refresh is unbounded | **WRONG — DOWNGRADED.** Every FastEPD/IT8951 wait in the vendored library is already bounded: `it8951WaitForLUTReady` breaks at **30 000 ms** (`FastEPD.inl:2027-2037`) and `it8951WaitForReady` breaks at **3 000 ms** (`:1908-1919`). `bbepFullUpdate` on a parallel panel is a fixed-trip-count DMA loop with no busy-wait at all (`:2732-3040`), and on `BB_PANEL_IT8951` it short-circuits to `it8951WriteFramebuffer*Bit` (`:2745-2754`), which waits LUT-ready at entry. **There is a real defect here, but it is the opposite one** — see D-D. |
| `[L3]` `CONFIG_FREERTOS_WATCHDOG_TIMEOUT_S=120` is inert | **CONFIRMED.** The symbol appears in 9 ESP envs (`platformio.ini:53, 83, 112, 140, 189, 209, 229, 253, 295`) and in **no** IDF 5.x sdkconfig. The real setting, from the precompiled `sdkconfig.h`, is `CONFIG_ESP_TASK_WDT_TIMEOUT_S 5` with `CONFIG_ESP_TASK_WDT_PANIC 1` and `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 1`. So the true TWDT is **5 s / panic-reboot on IDLE0 starvation**, and today's 30–60 s waits survive only because every one of them yields (`delay`/`vTaskDelay`/`bbepLightSleep`). |

---

## What Phase 2 changes

Eight independent work items (P2-5 dropped — see D-F). Each is separately revertable; none depends
on another.

| # | Item | Files | Targets | Risk |
|---|---|---|---|---|
| ~~P2-1~~ | ~~Bound `pwrmgmLockTake` (60 s deadline, no steal)~~ — ❌ **DROPPED** (owner decision) | — | — | — |
| ~~P2-2~~ | ~~`powerOff` stuck-button wait bounded at 10 s~~ — ❌ **DROPPED** (owner decision) | — | — | — |
| **P2-3** | `epdRefreshInProgress` around both boot-refresh paths | `display_service.cpp` | both | Low |
| **P2-4** | Make `fastepd_wait_refresh` a real bounded LUT wait | `display_fastepd.cpp` | ESP32/E1004 | Medium |
| ~~P2-5~~ | ~~Wall-clock cap on the loop command drain~~ — ❌ **DROPPED** (rationale did not survive checking; see D-F) | — | — | — |
| ~~P2-6~~ | ~~Delete the inert TWDT flag; document the real one~~ — ❌ **DROPPED** (owner decision) | — | — | — |
| **P2-7** | *(optional)* `Wire.setTimeOut(25)` — **ESP32 only, the API does not exist on nRF** | `display_service.cpp` | ESP32 | Low |
| **P2-8** | `waitforrefresh` → wall-clock deadline, not an iteration count | `display_service.cpp` | **both (nRF-critical)** | Low |
| ~~P2-9~~ | ~~Loop-liveness heartbeat + monitor task~~ — ❌ **DROPPED** (owner decision) | — | — | — |

> **Scope cut 2026-07-26 (owner decision): P2-1, P2-2, P2-6 and P2-9 are DROPPED**, joining P2-5.
> Phase 2 is now **P2-3, P2-4, P2-8**, with P2-7 still optional. Each dropped item keeps its
> full specification below, struck through, with its residual cost stated.
>
> What survives is exactly the refresh path: the real `waitforrefresh` deadline (P2-8), the real
> FastEPD wait (P2-4), and `epdRefreshInProgress` covering the boot paths (P2-3). Everything that
> added a new mechanism is gone (the lock deadline, the button bound, the monitor task), and so is
> the one documentation-only item (P2-6). **Three items, two files, one subsystem.**
>
> **Net effect on the phase's own thesis.** Phase 2 opened by defining a bound as binding only if it
> (1) executes, (2) has a live timebase, and (3) is observable by a third party. With P2-9 dropped,
> **condition 3 is satisfied by nothing on either target**, and with P2-1 dropped the panel lock
> fails condition 1 as well. Phase 2 is now a *narrower* claim than it set out to make: it bounds
> the refresh paths, and defers detection of a stalled `loop()` to Phase 6.

Suggested landing order: **P2-3 → P2-8 → P2-4** (+ P2-7 if D-G flips). All three are low-risk
and independent. Phase 2 no longer touches `src/main.cpp` or `platformio.ini` at all, so there is no conflict with
Phase 3's `[M5]` drain-trap fix.

---

## Making the bounds binding on both targets

The first draft of this plan was implicitly ESP32-shaped: four of the seven original items compile
out on nRF, and the one enforcement mechanism it leaned on (the IDF task watchdog) does not exist
there. This section states what "binding" has to mean, checks each item against it per target, and
added the two items (**P2-8**, **P2-9**) needed to close the nRF side.

> **After the scope cut, only P2-8 of those two survives.** P2-9 was the answer to condition 3, so
> the analysis below still stands as *diagnosis* but Phase 2 no longer *treats* the third condition.
> Read the coverage table as "what Phase 2 bounds" (P2-3, P2-4, P2-6, P2-8) plus a record of what
> was knowingly left unbounded (P2-1, P2-2) and unobserved (P2-9).

### A bound is binding only if all three hold

1. **It executes on that target** — the deadline code is not `#ifdef`'d away, and its consumers exist.
2. **Its timebase advances while the thing it bounds is stuck** — a deadline measured in a clock that
   stops when the fault occurs is not a deadline.
3. **A violation is observable by something other than the blocked party** — otherwise the bound
   only protects against faults that were already going to resolve.

(1) is a coverage question, (2) is an nRF timebase question, (3) is the "watchdog" question proper.

### Verified platform facts

ESP32 facts are from the precompiled `sdkconfig.h` (see `[L3]` above). nRF facts are from
`~/.platformio/packages/framework-arduinoadafruitnrf52-seeed`, the core the `nrf52840custom` env
actually builds against (`platformio.ini:29-32`).

**The two nRF priority rows are no longer theory.** Phase 1's `23ecaed` had to move the BLE
link-drop *inline into the callback task* because deferring it to `loop()` did not execute during a
transfer — the loop task (priority 1) is starved by the callback task (2) and the Bluefruit task
(3), and on nRF `loop()` reaches its service calls only after `idleDelay(sleep_timeout_ms)`. That is
the same starvation P2-9 exists to detect, now observed on hardware rather than inferred. It
strengthens P2-9's case and is the strongest single argument that a loop-serviced bound is **not**
binding on nRF mid-transfer — the premise behind condition 2 below.

The same work established that `Bluefruit.disconnect()` is safe from callback context (it defers to
`sd_ble_gap_disconnect()`, and the disconnect callback is serialized behind the write callback on
the one `ada_callback` queue). Useful precedent if any Phase 2 item is ever tempted to act from
outside `loop()` — though nothing in Phase 2 currently needs to.

| | ESP32 (Arduino / IDF 5.5.4) | nRF52840 (Adafruit/Seeed core) |
|---|---|---|
| RTOS | FreeRTOS, dual-core (S3/classic), single (C3/C6) | FreeRTOS, single core, `configMAX_PRIORITIES 5`, tick **1024 Hz** (`FreeRTOSConfig.h:55-56`) |
| `loop()` task priority | 1 | 1 — `TASK_PRIO_LOW` (`cores/nRF5/main.cpp:88`, `rtos.h:58`) ✅ re-verified in Phase 1 |
| Higher-priority tasks | NimBLE host task | Callback task = 2, **Bluefruit task = 3** (`rtos.h:59-61`) ✅ re-verified in Phase 1 |
| `delay()` | `vTaskDelay` — yields | `vTaskDelay` — yields (`cores/nRF5/delay.c:33-49`) |
| **`millis()` source** | `esp_timer_get_time()/1000` — **hardware timer** | **`tick2ms(xTaskGetTickCount())` — FreeRTOS tick** (`delay.c:29-31`, `rtos.h:65`) |
| Tickless idle | n/a for the timebase | `configUSE_TICKLESS_IDLE 1` (`FreeRTOSConfig.h:52`) |
| Task watchdog | `CONFIG_ESP_TASK_WDT_TIMEOUT_S 5`, `_PANIC 1`, `_CHECK_IDLE_TASK_CPU0 1` | **none** |
| Hardware WDT in use | no | **no** — `NRF_WDT` never started; no `wdt` symbol anywhere in `src/` |
| Idle hook | via `esp_register_freertos_idle_hook()` | `configUSE_IDLE_HOOK 1`, `vApplicationIdleHook` is a **weak alias to `__empty`** (`cores/nRF5/hooks.c:33`) — free to override |
| Reset-cause reporting | `esp_reset_reason()`, decoded at [main.cpp:26-37](../src/main.cpp) | nothing equivalent wired up |

### Item-by-item coverage

| Item | ESP32 | nRF | Action |
|---|---|---|---|
| ~~**P2-1**~~ lock deadline ❌ dropped | — | — | **left unbounded**; `pwrmgmLockTake` keeps its infinite spin on both targets |
| ~~**P2-2**~~ `powerOff` ❌ dropped | — | **n/a** — whole file is `#if defined(TARGET_ESP32)` ([power_latch.cpp:3](../src/power_latch.cpp)); nRF has no latch path and therefore no stuck-button loop | none |
| **P2-3** `epdRefreshInProgress` | ✅ 4 consumers | ⚠️ **flag is set but has ZERO consumers on nRF** — all four live in ESP32-only code ([ble_init.cpp:236](../src/ble_init.cpp), [main.cpp:322](../src/main.cpp), [:478](../src/main.cpp), + Phase 6) | set it anyway (correct, cheap, and Phase 6 adds the nRF consumers); **note the inertness in the commit message** |
| **P2-4** FastEPD | ✅ | **n/a** — `OPENDISPLAY_FASTEPD` is ESP32-only (`platformio.ini:54, 84, 113, 254`) | nRF's only refresh bound is `waitforrefresh` → **P2-8** |
| ~~**P2-5**~~ drain cap ❌ dropped | — | **n/a by design** — nRF has no command queue; `imageDataWritten` runs inline on the Bluefruit **callback task (prio 2)**, which *preempts* `loop()` (prio 1) | see "different failure mode" below |
| ~~**P2-6**~~ TWDT flag ❌ dropped | — | — | inert flag left in place on ESP32; nRF's watchdog absence left undocumented |
| **P2-7** `Wire.setTimeOut` | ✅ default 50 ms, settable | ❌ **API absent**, and the TWIM driver busy-spins with *no* timeout (`Wire_nRF52.cpp:166-181`) | ESP32-only; the nRF gap is **D-L** |

Two structural observations fall out of that table:

**The nRF `loop()` body is four function calls.** Lines [519-529](../src/main.cpp): `idleDelay`,
`ble_nrf_advertising_tick`, `processButtonEvents`, `processTouchInput`, `buzzerService`. Everything
else in `loop()` — the drain, `serviceBleDisconnectCleanup`, the 900 s direct-write watchdog at
[:436-442](../src/main.cpp), `checkPartialWriteTimeout()` at [:443](../src/main.cpp), the
`workInFlight`/deep-sleep gate — is inside the `#ifdef TARGET_ESP32` arm. This is the same finding
the parent plan records for Phase 6 ("nRF has NO transfer watchdog today"); it applies equally to
Phase 2's coverage and is why P2-9 must not be written as an ESP32 addition with an nRF port bolted on.

**nRF's blocking failure mode is inverted, not absent.** On ESP32 a long command blocks `loop()`,
which is the only drainer, so everything stops. On nRF a long command blocks the **callback task
(prio 2)**, while `loop()` (prio 1) keeps running — so buttons, touch, buzzer and `epdSessionTick`
stay alive, but no further BLE writes are serviced. Worse, per the parent plan's `[H4]`, Bluefruit
falls back to invoking the write callback **inline on the BLE task (prio 3)** when `rtos_malloc`
fails, and *that* blocks the stack itself. nRF therefore needs no drain cap, but it does need an
observer that is not the loop task — which is exactly P2-9.

### The nRF timebase hazard (condition 2)

**Every deadline in this plan is `millis()`-based, and on nRF `millis()` is derived from the
FreeRTOS tick, not from hardware.** `millis()` is `tick2ms(xTaskGetTickCount())` (`delay.c:29-31`).
If the tick stops, every Phase 2 bound silently stops counting — precisely in the pathological case
it was written for.

How much does this actually cost? Checked case by case:

- **Cooperative blocking (what Phase 2 targets): SAFE.** Every long wait yields via `vTaskDelay`, so
  the tick keeps running and `millis()` advances normally. `pwrmgmLockTake`'s `delay(1)`,
  `waitforrefresh`'s `delay(10)`, and `bbepWaitBusy`'s `bbepLightSleep(20, …)` all qualify — and
  note that `bbepLightSleep` is a **plain `delay()` off ESP32** (`bb_ep.inl:3947-3950`), so the
  30 s `bbepWaitBusy` cap that P2-1's 60 s is derived from is sound on nRF too.
- **Tickless idle: SAFE.** `configUSE_TICKLESS_IDLE 1` suppresses ticks during idle, but
  `vPortSuppressTicksAndSleep` compensates the count on wake, so `millis()` stays monotonic.
- **Interrupts disabled / critical section / SoftDevice storm: NOT COVERED.** Ticks are lost and
  never compensated; `millis()` stalls and the 60 s deadline never expires.

The third case is the parent plan's already-accepted "true CPU/peripheral hard hang" residual, and
no software-only mechanism recovers from it. **The correct action is to document it, not to
engineer around it** — a DWT-cycle-counter timebase (`dwt_enable()` / `DWT->CYCCNT` are already
exposed in `cores/nRF5/delay.c:52-58`) would work but wraps every ~67 s at 64 MHz, needs wrap
accumulation, and buys nothing for a fault class we have already accepted. Recorded so it is not
rediscovered.

---

## P2-3 — `epdRefreshInProgress` around both boot-refresh paths

Two edits, both mechanical. The flag is `volatile bool` at
[display_service.cpp:85](../src/display_service.cpp), declared in
[display_service.h:77](../src/display_service.h).

**bb_epaper boot path** — [display_service.cpp:533-542](../src/display_service.cpp). Wrapping it
here covers both call sites ([:1624](../src/display_service.cpp) and the retry at
[:1632](../src/display_service.cpp)):

```c
static bool refreshBootScreenFull() {
    if (!writeBootScreenWithQr()) { od_log_warn("Boot screen render failed"); return false; }
    od_log_info("EPD refresh: FULL (boot)");
    touchSuspendForEpdRefresh();
    epdRefreshInProgress = true;
    bbepRefresh(&bbep, REFRESH_FULL);
    bool ok = waitforrefresh(60);
    epdRefreshInProgress = false;
    return ok;
}
```

**FastEPD boot path** — [display_service.cpp:1586-1594](../src/display_service.cpp). Set before
`fastepd_full_update()`, clear after `waitforrefresh(60)` and **before** `epdSessionForceOff()`
(the force-off is teardown, not refresh, and holding the flag across it would block the very
Phase-6 supervisor pass that might need to act).

### Why this matters more than it looks

Three consumers gate on this flag and all three are wrong today during a boot refresh:

- [ble_init.cpp:236](../src/ble_init.cpp) — advertising restart deferral.
- [main.cpp:322](../src/main.cpp) — `serviceBleDisconnectCleanup` deferral.
- [main.cpp:478](../src/main.cpp) — the `workInFlight` deep-sleep gate.

A 30–60 s Spectra boot refresh is currently invisible to all of them, and **→ Phase 6** adds a
fourth consumer (the supervisor's "never interrupt a refresh" rule) that would inherit the same
hole. P2-3 is a prerequisite for Phase 6 being correct, not just a tidy-up.

---

## P2-8 — `waitforrefresh`: wall-clock deadline, not an iteration count

[display_service.cpp:747-776](../src/display_service.cpp). The bb_epaper wait — **the only refresh
bound nRF has** — counts iterations, not time:

```c
for (size_t i = 0; i < (size_t)(timeout * 100); i++){
    delay(10);
    ...
}
```

Each iteration is `delay(10)` = *at least* 10 ms. Under preemption it is more, and on nRF the loop
task is the **lowest** non-idle priority (1), sitting under the callback task (2) and the Bluefruit
task (3). So `waitforrefresh(60)` is a bound of "60 s of loop-task scheduling", not 60 s of wall
clock — it can overrun arbitrarily while a BLE transfer keeps the higher-priority tasks busy. It
also mis-reports: the `"Refresh took %.2f seconds"` line divides `i` by 100 and is wrong by the same
factor.

Same defect on ESP32, less visible there because the loop task is less contended.

```c
bool waitforrefresh(int timeout){
    ...
    // Wall-clock deadline, not an iteration count. delay(10) is a *minimum*: on
    // nRF the loop task is the lowest non-idle priority (1) under the Bluefruit
    // callback task (2) and BLE task (3), so counting iterations bounds
    // loop-task scheduling rather than elapsed time and can overrun arbitrarily
    // during a transfer.
    const uint32_t deadline = millis() + (uint32_t)timeout * 1000u;
    uint32_t polls = 0;
    const uint32_t t0 = millis();
    while ((int32_t)(millis() - deadline) < 0) {
        delay(10);
        if (polls % 50 == 0) od_log_raw(".");
        if (!bbepIsBusy(&bbep)) {
            if (polls == 0) {
                od_log_error("ERROR: Epaper not busy after refresh command - refresh may not have started");
                return false;
            }
            od_log_raw(".\n");
            od_log_info("Refresh took %u ms", (unsigned)(millis() - t0));
            return true;
        }
        polls++;
    }
    od_log_warn("Refresh timed out after %u ms", (unsigned)(millis() - t0));
    return false;
}
```

Preserve the `polls == 0` check exactly — the existing comment at
[:754-757](../src/display_service.cpp) explains that BUSY asserts within µs of `MASTER_ACTIVATE`, so
a first poll at 10 ms is what makes "never went busy" a valid error. Do not reorder the increment.

This is the cheapest item in the plan with the largest nRF-side effect: it converts nRF's single
refresh bound from advisory to real.

---

## P2-4 — Make `fastepd_wait_refresh` a real, bounded wait

### Correcting `[X3]`

The parent plan says FastEPD is unbounded and asks for a busy poll "honouring `timeout_sec`". That
is not the defect. Every wait in the library is already capped (30 s LUT, 3 s HRDY, fixed-trip DMA
loops). The actual defect is the mirror image:

> `fastepd_wait_refresh` returns **immediately** ([display_fastepd.cpp:228-231](../src/display_fastepd.cpp)),
> and the refresh functions it is paired with return **before the panel has finished refreshing**.
> So the caller believes the refresh is complete and proceeds to cut power.

Trace it: `fastepd_direct_refresh(1)` ([:276-284](../src/display_fastepd.cpp)) →
`it8951_fullscreen_du()` ([:132-180](../src/display_fastepd.cpp)) → issues
`it8951DisplayArea1Bit(...)` then `it8951WaitForReady(st)`, which waits **HRDY** (host interface
ready, 3 s cap) and *not* **LUTAFSR** (waveform complete). The panel is still painting when the
function returns. `waitforrefresh(60)` then routes to the stub
([display_service.cpp:749](../src/display_service.cpp)) and returns instantly, and
`cleanupDirectWriteState(false)` → `epdSessionRelease()` can call `einkPower(0)` / `deInit()`
mid-waveform.

The library gets away with this internally because `it8951WriteFramebuffer1Bit` waits LUT-ready at
**entry** (`FastEPD.inl:2125`) — the next operation absorbs the previous one's tail. Our code path
does not have a next operation; it has a power-down.

### The fix

```c
// display_fastepd.cpp — replace the stub at :228-231
// Real wait: block until the IT8951 waveform LUT is idle. The refresh entry
// points (it8951_fullscreen_du / bbepFullUpdate->it8951WriteFramebuffer*Bit)
// return once the image is LOADED and HRDY is back, NOT once the panel has
// finished painting -- the library relies on the *next* operation's entry-side
// LUT wait to absorb that. Our next operation is a power-down, so we must do
// the wait here or we cut the rail mid-waveform.
//
// The bound is the library's own: it8951WaitForLUTReady breaks at 30 000 ms
// (FastEPD.inl:2027-2037). timeout_sec is therefore advisory and is logged when
// it is tighter than the library's cap; we do not reimplement the register poll.
bool fastepd_wait_refresh(int timeout_sec) {
    if (s_init_failed) return false;
    FASTEPDSTATE* st = g_epd.state();
    if (!st) return false;
    const uint32_t start = millis();
    it8951WaitForLUTReady(st);           // hard-bounded at 30 s inside the library
    const uint32_t waited = millis() - start;
    if (waited >= 30000u) {
        od_log_error("[FastEPD] LUT-ready timeout (%u ms) - refresh incomplete", (unsigned)waited);
        return false;
    }
    if (timeout_sec > 0 && waited > (uint32_t)timeout_sec * 1000u) {
        od_log_warn("[FastEPD] refresh took %u ms (caller budget %ds)", (unsigned)waited, timeout_sec);
    }
    return true;
}
```

`it8951WaitForLUTReady` is already declared at
[display_fastepd.cpp:15](../src/display_fastepd.cpp), so no new extern is needed.

### Coverage

This one change covers **every** FastEPD refresh, because they all funnel through
`waitforrefresh()` → `fastepd_wait_refresh()`:

- `fastepd_direct_refresh` + `waitforrefresh(60)` — [display_service.cpp:2422-2423](../src/display_service.cpp) *(the path a real transfer takes)*
- `fastepd_full_update` + `waitforrefresh(60)` — [display_service.cpp:1591-1592](../src/display_service.cpp) *(boot)*
- `fastepd_partial_refresh` — [display_service.cpp:3287](../src/display_service.cpp)

The parent plan asked for `fastepd_direct_refresh` specifically; wrapping the *wait* instead of
each *refresh* gets all three for free and cannot be bypassed by a future fourth caller.

### Blast radius

`OPENDISPLAY_FASTEPD` is defined in 4 envs (`platformio.ini:54, 84, 113, 254`) and the code path is
live only when `fastepd_driver_used()` — i.e. IT8951/E1004 panels. **This change makes refreshes
take measurably longer on those boards** (they were previously returning early). That is the point,
but it means the E1004 ~960 KB upload regression test is mandatory, not optional.

---

## P2-7 — *(optional)* `Wire.setTimeOut(25)` — **ESP32 only**

### What the call actually does

`TwoWire::setTimeOut(uint16_t timeOutMillis)` sets the per-transaction I2C timeout that Arduino-ESP32
passes down to `i2cRead()` / `i2cWrite()` (`framework-arduinoespressif32/libraries/Wire/src/Wire.cpp:458,
:518, :525`). **The default is 50 ms** (`Wire.cpp:44`, `_timeOutMillis(50)`), and the firmware never
calls the setter, so every I2C transaction that fails to complete blocks the calling task for 50 ms.

Per the parent plan's downgraded `[X1]`: the GT911 driver already self-disables after 5 consecutive
read failures ([touch_input.cpp:39](../src/touch_input.cpp), [:642](../src/touch_input.cpp)), so the
worst case is **~5 × 50 ms = ~250 ms of blocked `loop()`** before the controller is dropped for good.
`setTimeOut(25)` halves that to ~125 ms. That is the entire benefit — a latency trim, not a new bound.

Call sites, all after a successful `Wire.begin()`: [display_service.cpp:786](../src/display_service.cpp)
and [:794](../src/display_service.cpp) (in `wireBeginForOpenDisplay`), [:872](../src/display_service.cpp)
and [:885](../src/display_service.cpp) (in `initOrRestoreWireForOpenDisplay`), and
[:920](../src/display_service.cpp).

### ⚠️ Correction: ESP32-only, and nRF's situation is worse

An earlier revision of this plan listed P2-7 as applying to both targets. That was wrong twice over:

**1. The API does not exist on nRF.** `setTimeOut` (capital O) is an Arduino-ESP32 extension. The
Adafruit/Seeed core's `TwoWire` (`libraries/Wire/Wire.h:33`) does not declare it; it only inherits
`Stream::setTimeout` (lowercase o), which is a *stream read* timeout with nothing to do with I2C
transactions.

**2. nRF's I2C driver has no timeout at all — it spins forever.** `Wire_nRF52.cpp` waits on TWIM
events with bare, non-yielding busy-loops:

```c
while(!_p_twim->EVENTS_RXSTARTED && !_p_twim->EVENTS_ERROR);   // :166
while(!_p_twim->EVENTS_LASTRX    && !_p_twim->EVENTS_ERROR);   // :169
while(!_p_twim->EVENTS_STOPPED);                               // :175  <- no ERROR check
while(!_p_twim->EVENTS_SUSPENDED);                             // :181  <- no ERROR check
```

(Same shape on the write path at `:230-247`.) No bound, no yield, and the last two do not even
inspect `EVENTS_ERROR`.

Scope of the exposure, stated honestly: a **disconnected or NACKing** device is fine — TWIM raises
`EVENTS_ERROR` and the first two loops exit. The unbounded case is a genuine **bus lockup** (a
peripheral holding SDA low, a missing or weak pull-up), where neither the completion event nor the
error event ever arrives and the calling task hangs permanently. Rarer than a NACK, but it is exactly
the GT911-wedge scenario `[X1]` was originally written about — and on nRF there is no
`TOUCH_I2C_FAIL_DISABLE_THRESHOLD` escape, because the driver never returns to be counted.

I2C is genuinely used on nRF: `sensor_sht40.cpp` includes `<Wire.h>` and calls
`Wire.beginTransmission()` with no target guard.

### Recommendation

**Still defer P2-7 itself.** 250 ms is not a freeze; the setting is global to the bus, so it also
applies to SHT40, BQ27220 and AXP2101, and a 25 ms ceiling is not obviously safe for a
clock-stretching sensor. If deferred, record it as accepted-as-is alongside `[X1]`.

**The nRF finding is a separate, new question — see D-L.** It is a real unbounded, non-yielding
wait, which is precisely Phase 2's subject matter, and `setTimeOut` cannot fix it because the API is
absent. It deserves its own decision rather than being folded into an optional latency trim.

---

## Decisions needed

> **After the scope cut, nothing blocks implementation.** D-A, D-A2, D-B, D-C and D-E existed only
> to shape P2-1; D-H, D-I and D-J only to shape P2-9. All eight are moot and struck through below,
> retained because their analysis is the useful part if either item is ever revived.
>
> **Still live:** **D-D** (accept the `[X3]` downgrade — recommend yes), **D-G** (include P2-7 —
> recommend no, defer), **D-K** (accept the nRF tick-derived `millis()` limitation — recommend yes),
> and **D-L** (nRF I2C busy-spins with no timeout: bound it or accept it). All four have a
> recommendation that can be taken as the default, so P2-3/P2-4/P2-6/P2-8 can start immediately.

### D-D — Accept the `[X3]` downgrade? *(recommendation: yes)*

The parent plan's "implement a real busy poll against the LUT-busy register honouring
`timeout_sec`" is superseded by P2-4, which calls the library's existing bounded poll instead of
reimplementing it. This is a **correction to the parent plan** and should be folded back into
`PLAN_FREEZE_PROOFING_2026-07-26.md` § Phase 2 the same way `[X1]` was, so review does not
re-litigate it. It also means the parent plan's Verification line "FastEPD refresh has no
firmware-side bound" needs rewording.

### D-F — Keep P2-5 at all? — ✅ **RESOLVED: dropped**

*Confirmed 2026-07-26.* **P2-5 is out of Phase 2 entirely** — no cap, no constant, no saturation
WARN. The drain loop is untouched by this phase. Phase 3's `[M5]` becomes the sole edit to that
region; the ring-saturation diagnostic, if wanted, belongs to Phase 7's `[H1]`.

Reframed from "what value for `COMMAND_DRAIN_BUDGET_MS`" — the value question was moot once the
item went. Per the P2-5 section: a 2 s cap never fires (32 DATA frames ≈ 0.1–1 s), cannot interrupt the
one genuinely long command (a 30–60 s refresh — the check is between commands), and the stacked-
refresh case it *would* catch is unreachable
([display_service.cpp:2366](../src/display_service.cpp)). The drain is already bounded by
33 × (per-command time), with per-command time bounded by P2-1 / P2-4 / P2-8.

Positions considered:

1. **Drop entirely** — ✅ **chosen.** Also removes the `[M5]` merge conflict with Phase 3.
2. **Replace with a 2-line saturation WARN** on `drained == COMMAND_QUEUE_SIZE` — rejected for
   Phase 2; handed to Phase 7's `[H1]` queue-full work, where the ring is already the subject.
3. **Keep the 2 s cap** — rejected. Only defensible as TWDT margin against a hypothetical future
   where commands get 5× slower. Today's margin is already 5×.

This is a **correction to the parent plan**, like `[X1]` and `[X3]`, and should be folded back into
`PLAN_FREEZE_PROOFING_2026-07-26.md` § Phase 2 ("Loop command drain: 2 s wall-clock cap alongside
the count cap") so it is not re-litigated.

### D-G — Include P2-7? *(recommendation: no, defer)*

See P2-7. If deferred, add it to the parent plan's *Deliberately NOT changed* section with the
reasoning, so it does not get rediscovered. Note the scope correction: P2-7 is **ESP32-only** — the
`setTimeOut` API does not exist on the nRF core.

### D-L — nRF I2C has no timeout and busy-spins forever. Bound it, or accept it? *(new)*

Discovered while checking D-G's scope. `Wire_nRF52.cpp:166-181` / `:230-247` spin on TWIM events with
no deadline and no yield; a bus lockup hangs the calling task permanently. This is a genuine unbounded
wait on nRF — Phase 2's exact subject matter — and unreachable via `setTimeOut`, which the core does
not implement.

| Option | Assessment |
|---|---|
| **(a) Accept and document** | Consistent with `[X1]`'s downgrade and with the accepted hard-hang residual. Cheapest. But `[X1]`'s "the driver already gives up after 5 failures" reasoning **does not hold on nRF** — the driver never returns, so nothing counts failures |
| **(b) Wrap our own call sites** with a pre-flight bus check (sample SDA/SCL; skip the transaction if SDA is held low) | Bounded, ~15 lines, no library fork. Does not help if the bus locks *mid*-transaction |
| **(c) Fork/patch the core's Wire** to add a deadline to the four loops | Correct fix, but it is a vendored-core patch that every toolchain update must re-apply. High maintenance for a rare fault |
| **(d) Bound it at the caller** — run I2C only from a context where a hang is survivable | Not achievable; `loop()` is the caller |

**Recommend (a) for Phase 2, with (b) noted as the cheap follow-up if a lockup is ever observed in
the field.** Rationale: the failure needs a physical bus fault (stuck SDA, bad pull-up), not a
software condition — no client, protocol state, or packet loss can trigger it — so it is outside the
freeze class this effort targets. But it must be recorded honestly rather than inherited from
`[X1]`, whose "the driver gives up" argument is ESP32-specific and does not transfer.

If (a) is chosen, add it to the parent plan's residual-risk list, **not** to *Deliberately NOT
changed* — it is an accepted gap, not a considered-and-rejected change.

### D-K — nRF timebase: accept the tick-derived `millis()` limitation? *(recommendation: yes)*

Per the timebase analysis: every cooperative wait yields, so ticks keep running and all Phase 2
bounds hold on nRF. Only a scheduler-starving fault (interrupts disabled / SoftDevice storm) stalls
`millis()` — which is the parent plan's already-accepted hard-hang residual. **Recommend accepting
and documenting**, rather than adding a DWT-cycle-counter timebase with 67 s wrap handling for a
fault class we cannot recover from anyway. Add it to the parent plan's residual-risk list.

---

## Files touched

| File | Items | Targets |
|---|---|---|
| `src/display_service.cpp` | P2-3, P2-8, (P2-7) | both |
| ~~`src/display_service.h`~~ | ~~P2-1 (`panelStateUnknown` extern)~~ — dropped | — |
| ~~`src/session_monitor.cpp/.h`~~ *(new)* | ~~P2-9~~ — dropped, **no new file in Phase 2** | — |
| `src/display_fastepd.cpp` | P2-4 | ESP32 |
| ~~`src/power_latch.cpp`~~ | ~~P2-2~~ — dropped | — |
| ~~`src/main.cpp`~~ | ~~P2-6 (comment)~~ — dropped; **Phase 2 does not touch `main.cpp` at all** | — |
| ~~`platformio.ini`~~ | ~~P2-6~~ — dropped; **no build-config change in Phase 2** | — |
| ~~`docs/TIMER_AND_WATCHDOG_INVENTORY_2026-07-26.md`~~ | ~~P2-6~~ — dropped | — |
| `docs/PLAN_FREEZE_PROOFING_2026-07-26.md` | D-D (`[X3]` downgrade), **the scope cut: § Phase 2 must drop P2-1/P2-2/P2-5/P2-9 and move their residuals to Phase 6's remit**, D-K (residual) | — |

~~`src/session_monitor.*` is deliberately not `src/session_guard.*`…~~ — moot with P2-9 dropped.
**Phase 2 now creates no new file at all**, so Phase 3 owns `src/session_guard.*` with nothing to
coordinate around and nothing to `#include`.

**No protocol surface is touched.** No `CMD_*`/`RESP_*`, no frame layout, no config-packet layout,
no client-observable behaviour change other than "a FastEPD refresh now completes before the device
reports it complete" — which is a bug fix in the client's favour. Run the parent plan's constraint
check before calling the phase done:

```bash
cd ../opendisplay-protocol && tools/sync_protocol_header.py --check --only Firmware
cd ../Firmware && git diff main --stat -- include/opendisplay_protocol.h include/opendisplay_structs.h   # must be empty
```

---

## Verification

### Build

```bash
pio run -e nrf52840custom -e esp32-s3-N16R8 -e esp32-c3-N16 -e esp32-c6-N4 -e esp32-N4 -e esp32-s3-E1004
```

`esp32-s3-E1004` is added to the parent plan's set because it is the FastEPD/IT8951 gate for P2-4.
CI builds all **12** envs on push (`esp32-s3-N16R8-extuart-debug` is easy to miss when counting
from `platformio.ini`). Items P2-1…P2-8 add no meaningful `.bss` (one `bool`, a few `uint32_t`
locals). **P2-9 adds a ~2 KB task stack, and `esp32-N4` is the gate.** Phase 1 as-built *freed*
480 B there (81,940 → 81,468 B), so the headroom is better than this plan originally assumed — see
"Cost" under P2-9. Compare the new figure against **81,468 B**, not the pre-Phase-1 baseline.

CI now also runs a `host-tests` job (added by Phase 1) alongside the 12-env matrix; Phase 2 adds
nothing to it unless P2-8/P2-9 grow host-testable pure logic, which is worth considering for the
`waitforrefresh` deadline arithmetic.

### Static

- ~~`grep -rn "CONFIG_FREERTOS_WATCHDOG_TIMEOUT_S" platformio.ini` → no hits.~~ — moot, P2-6
  dropped; the flag stays and `platformio.ini` must be **unchanged**.
- ~~`pwrmgmLockTake` call sites / lost-`Give` audit~~ — moot, P2-1 dropped; `pwrmgmLockTake()` keeps
  its `void` signature and every existing call site is unchanged. **Confirm the diff does not touch
  it**, which is now the check that matters.
- ~~`sessionMonitorHeartbeat` placement, P2-9 stack-size units~~ — moot, P2-9 dropped.
- `git diff --stat` should show **no new files**, and changes confined to
  `src/display_service.cpp` and `src/display_fastepd.cpp`. Any touch to `src/main.cpp` or
  `platformio.ini` means scope has crept back in.

### Hardware — both targets

**Every test below must be run on `nrf52840custom` as well as an ESP32 env.** The parent plan's
Phase 6 note applies here too: do not assume nRF parity from an ESP32 pass. nRF-specific
expectations are called out where the platforms legitimately differ.

| Test | Expect |
|---|---|
| **P2-1** Force a lock timeout (temporary `pwrmgmLock = 1` from a debug command) | ERROR logged once, panel op skipped, **`loop()` continues**, BLE stays responsive, no reboot. ESP32: proves the yielding wait survives the 5 s TWDT. **nRF: proves `millis()` keeps advancing under the `delay(1)` spin** — the timebase check |
| **P2-2** Jumper the button pin low, issue power-off | ESP32 only. Latch drops at ~10 s with a WARN; note whether the held button re-latches (expected on a latching board) |
| **P2-3** Boot with a Spectra panel, `nRF Connect` scanning throughout | ESP32: advertising restart deferred, no disconnect-cleanup during the boot refresh. **nRF: expect no behaviour change** — the flag has no consumers there yet; confirming the *absence* of a regression is the test |
| **P2-4** E1004: full ~960 KB upload → refresh → immediate power-down | ESP32/E1004 only. Image fully painted; no truncated/ghosted waveform. Compare against a pre-change capture — this is the regression that proves the early return was real |
| **P2-4** E1004 boot refresh | Same, on the boot path |
| ~~P2-5~~ | Dropped — no test. *(If you want the evidence on record, instrument a saturated drain once and log its wall-clock; expect ≪ 2 s, which is why the cap went.)* |
| **P2-8** Refresh during a concurrent BLE transfer, both targets | Reported duration matches a stopwatch. **nRF is the real test**: with the callback task (prio 2) busy, the old iteration count would overrun 60 s while the new deadline holds |
| **P2-8** Disconnect the BUSY line to force a timeout | WARN at ~60 s wall clock on both targets, not later |
| **P2-9** Block `loop()` deliberately (debug command doing a non-yielding busy-wait > 2.5 min) | One ERROR at ~150 s naming the phase breadcrumb, rate-limited to one line per 30 s, one INFO on recovery with the total. **nRF: confirm the prio-2 task actually preempts prio-1 `loop()`** — this is the whole premise |
| **P2-9** Normal operation, 1 h idle + several full transfers | **Zero** stall lines. A false positive here means `LOOP_STALL_WARN_MS` (D-J) is too tight |
| **Regression** Full Spectra transfer (60 s+ refresh), both targets; E1004 ~960 KB upload | Complete untouched, no stall lines |

Every new bound logs exactly one ERROR/WARN with the reason and the elapsed time — per the parent
plan's blanket requirement.

---

## Residual risk after Phase 2

- **A hard peripheral hang inside a single library call is still unbounded from our side.** We bound
  the *waits we own*; `bbepWaitBusy`'s own 30 s cap and `it8951WaitForLUTReady`'s 30 s cap are the
  library's, and a hang below those (a wedged SPI transaction, a stuck DMA) is invisible to us.
  Consistent with the software-only decision.
- **`pwrmgmLockTake` is still unbounded (P2-1 dropped).** A panel-lock holder that never releases
  blocks its waiter forever, on both targets. This is the largest residual Phase 2 knowingly leaves;
  it was previously the item's whole reason for existing.
- **A stalled `loop()` is undetected on both targets (P2-9 dropped).** ESP32's TWDT will not fire —
  every long wait yields, so IDLE0 is never starved — and nRF has no watchdog at all. Phase 2 bounds
  the refresh waits but reports nothing when a bound is exceeded by something it does not own.
  Detection now waits for Phase 6.
- **`powerOff`'s stuck-button wait is still unbounded (P2-2 dropped).** ESP32-only, needs a hardware
  fault, and it removes a recovery path rather than creating a freeze.
- **A single long command still owns `loop()` for its duration** — one 60 s refresh blocks the loop
  task for 60 s, and no drain cap can change that (the check would be between commands). This is by
  design: interrupting a refresh is worse than waiting for it. What bounds it is P2-4/P2-8, not P2-5.
- **A saturated drain costs ~1 s of unserviced touch/buttons.** Measured-order estimate, not a
  freeze, and accepted — see D-F.
- **The inert `CONFIG_FREERTOS_WATCHDOG_TIMEOUT_S=120` flag stays in 9 ESP envs (P2-6 dropped)**,
  still reading like a 120 s watchdog guarantee that does not exist. Harmless to execution, but a
  standing trap for the next reader of `platformio.ini`. nRF's total absence of a watchdog also goes
  unrecorded.
- **Phase 2's own thesis is only partly delivered.** Of the three conditions for a "binding" bound,
  condition 3 (observable by a third party) is now met by nothing, and condition 1 fails for the
  panel lock. Phase 2 delivers *bounded refreshes*; it does not deliver *detected stalls*.
- ~~**P2-2 may re-latch.**~~ Moot — P2-2 is dropped, so the latch behaviour is unchanged from today.
- **nRF I2C can hang forever on a bus lockup** (`Wire_nRF52.cpp:166-181`, `:230-247` — bare
  non-yielding `while(!EVENTS_x);`). Needs a physical fault (stuck SDA, bad pull-up), not a software
  condition, so it is outside the freeze class this effort targets — but `[X1]`'s "the driver gives
  up after 5 failures" reasoning is ESP32-specific and does **not** transfer. See D-L.
- **On nRF, `millis()` stops if the scheduler stops** (tick-derived, `delay.c:29-31`). Every Phase 2
  deadline goes with it. Only reachable via interrupts-disabled / critical-section / SoftDevice-storm
  faults, which are the already-accepted hard-hang class; all cooperative waits yield and keep the
  tick running. Documented rather than engineered around — see D-K.
- **P2-9 detects, it does not recover.** Under the no-reboot decision an observer task has no safe
  action against a blocked `loop()`. It converts an invisible field freeze into a timestamped ERROR
  naming the phase; the recovery story remains the per-wait deadlines in P2-1/P2-4/P2-8. **A hard
  hang is therefore still unrecovered after Phase 2** — deliberately, per D-I, with a rate-limited
  flash-persisted reset held as the documented follow-up if soak shows stalls actually occur.
- **`epdRefreshInProgress` remains inert on nRF until Phase 6** adds the consumers. P2-3 sets it
  correctly on both targets, but on nRF nothing reads it yet.
- **nRF still has no transfer watchdog after Phase 2** — the 900 s direct-write bound and
  `checkPartialWriteTimeout()` live in the `#ifdef TARGET_ESP32` arm of `loop()`
  ([main.cpp:436-443](../src/main.cpp)). This is Phase 6's `[H3]` "ADD on nRF" item, restated here
  so Phase 2 is not mistaken for having closed it.
## Appendix — decisions made moot by the scope cut

Eight decisions existed only to shape P2-1 (D-A, D-A2, D-B, D-C, D-E) or P2-9 (D-H, D-I, D-J).
Both items are dropped, so none of them needs an answer. Retained because the analysis is the
part worth keeping if either item is ever revived.

### ~~D-A~~ — `epdSessionAcquire` signature — ⛔ **MOOT: P2-1 dropped**

#### The problem

`epdSessionAcquire` currently returns `bool cold` — a *result* (was the rail off?), not a *status*.
Once `pwrmgmLockTake()` can fail there is nowhere to report "I did not acquire anything." The two
values are not merge-able: `false` already means "warm", which is a success.

#### Proposed signature

```c
// display_service.cpp:438
// Bring the panel up for a transfer/refresh.
//
// Returns true iff the session was acquired. On FALSE the pwrmgm lock timed out:
// nothing was powered, nothing was initialised, panelStateUnknown is set, and the
// caller MUST skip all panel work -- driving SPI/CS without the lock is exactly
// the two-tasks-one-bus hazard the lock exists to prevent.
//
// *outCold is written ONLY when the function returns true, and is the old return
// value: true = the rail was off and we did a COLD bring-up (callers may need to
// reopen the address window). May be NULL if the caller does not care.
static bool epdSessionAcquire(bool partialInit, bool* outCold);
```

#### The full cascade — bounded, 6 functions, 4 leaf sites

This is the real cost of D-A and it is worth stating exactly, because "the compiler finds them all"
is only reassuring if the count is small. It is:

```
epdSessionAcquire()                          2 call sites
├─ :2083  directWriteActivatePanel()         void -> must become bool
│           ├─ :2154  handleDirectWriteStart()      ← leaf (protocol handler)
│           └─ :2807  handlePipeWriteStart()        ← leaf (protocol handler)
└─ :3253  partial_prepare_panel_ram()        void -> must become bool
            ├─ :2252  handlePartialWriteStart()     ← leaf (protocol handler)
            └─ :2792  handlePipeWriteStart()        ← leaf (same handler, other arm)
```

So: two `void`→`bool` changes on file-static helpers, and **three protocol handlers** that need a
failure branch. Every one of the three is a `*Start` handler, which is the best possible place for
this — the failure happens before any state is committed, so the branch is "NACK and return", not
a teardown.

The `directWriteActivatePanel` site at [:2083](../src/display_service.cpp) is the one that matters
most: it currently **ignores the return value entirely** and sets `directWriteActive = true` at
[:2075](../src/display_service.cpp) *before* acquiring. If the acquire fails there and nothing
checks, the device latches `directWriteActive` with the panel un-acquired — a wedge of exactly the
kind this whole effort exists to remove. Order the new code so `directWriteActive` is set only
after a successful acquire.

#### Which driver paths does this affect? — **all three, and both targets**

`epdSessionAcquire` is **driver-agnostic**. It always takes the lock and always actuates the rail
via `pwrmgm(true)` ([:443](../src/display_service.cpp)); only the controller-init step branches, on
`epdSessionUsesFastepd()` ([:371-377](../src/display_service.cpp)):

| Path | What `epdSessionAcquire` does inside the lock | Affected by D-A? |
|---|---|---|
| **bb_epaper generic** | `pwrmgm(true)` + `bbepInitIO` + `bbepWakeUp` + `bbepSendCMDSequence` + `epdAlignCustomPartialRamMode` ([:453-456](../src/display_service.cpp), warm re-acquire [:478-481](../src/display_service.cpp)) | **Yes** |
| **E1004 dual-CS** (`#ifdef BBEP_T133A01`) | `pwrmgm(true)` + `e1004InitPanel()` ([:448](../src/display_service.cpp)) | **Yes** |
| **FastEPD / IT8951** | `pwrmgm(true)` **only** — the TCON init happens outside, in `fastepd_direct_write_reset()` / `fastepd_epaper_begin()` | **Yes** (the rail is still the lock's business) |

So this is emphatically **not** a FastEPD-only change. Note also that `epdSessionUsesFastepd()`
returns `false` unless `TARGET_ESP32 && OPENDISPLAY_FASTEPD`, so **on nRF the bb_epaper branch is
the only one that ever runs** — D-A is fully live on `nrf52840custom`, and both leaf call sites
(`directWriteActivatePanel`, `partial_prepare_panel_ram`) are outside every `#ifdef TARGET_ESP32`.

#### What the lock does and does not guard — why only `epdSessionAcquire` changes

Two separate questions hide behind "why only this function":

**(a) Why no other lock-taker needs a signature change.** `epdSessionRelease` and
`epdSessionForceOff` return `void`, so an early return is a complete response;
`epdSessionTick` already uses `pwrmgmLockTryTake()`; `epdSessionForceOffLocked` is caller-holds.
`epdSessionAcquire` is the only one whose *caller must behave differently* on failure.

**(b) Why `epdSessionAcquire` is not the only path that touches the panel.** It isn't — and that is
by design, not an oversight. Several paths drive the panel without ever taking the lock:

| Bypass | Why it is safe today |
|---|---|
| `prepareEpdRailForBoot()` [:173-185](../src/display_service.cpp) — raw `pwrmgm(true/false/true)` | Boot only; BLE is not up, no transfer can exist |
| `initBbepPanelSession()` [:341-355](../src/display_service.cpp) — full `bbepInitIO`+`bbepWakeUp`+init-seq | Boot only, same reason |
| `initDisplay()` [:1567-1600](../src/display_service.cpp) — raw `pwrmgm(true)`, `fastepd_epaper_begin()`, `pwrmgm(false)` | Boot only, same reason |
| `fastepd_prepare_hardware()` at [:2140](../src/display_service.cpp) / [:2803](../src/display_service.cpp) | Runs in the START handler *before* the acquire; loop/callback task, serialized by transfer ownership |
| **All data streaming** — `bbepSetAddrWindow`/`bbepStartWrite` [:2098-2099](../src/display_service.cpp), `e1004_begin_plane()`, `fastepd_direct_write_reset()`, every subsequent `bbepWriteData` | The lock is `Give`n at [:488](../src/display_service.cpp) *before* `epdSessionAcquire` returns |

**The lock is a rail-and-init guard, not a bus mutex.** Streaming is protected by `pwrmgmState`
instead: `epdSessionTick` acts only in `PWR_WARM`, and a live transfer holds `PWR_ACTIVE`, so the
keep-alive tick cannot rail-cut mid-stream. The lock exists for the narrower race the comment at
[:396-400](../src/display_service.cpp) names — the tick's `ForceOff` landing mid-*init*. This is
coherent and Phase 2 should not widen it; noted here so a reviewer does not read the bypass list as
a new defect.

**Consequence for D-A's failure branch:** at both leaf sites, `fastepd_prepare_hardware()` has
*already run* by the time the acquire fails ([:2140](../src/display_service.cpp),
[:2803](../src/display_service.cpp)). On the FastEPD path the failure branch should therefore call
`fastepd_mark_hw_deinitialized()` so the next attempt does a full TCON re-init rather than a
`wake()` on a controller whose rail state we no longer know — the same reasoning the existing
comment at [:424-426](../src/display_service.cpp) gives for the rail-drop case.

#### Call-site shape

```c
static bool directWriteActivatePanel(void) {
    ...
    bool cold = false;
    if (!epdSessionAcquire(false, &cold)) {
        od_log_error("ERROR: panel session acquire failed (lock timeout) - not starting direct write");
        return false;               // directWriteActive NOT set; nothing to tear down
    }
    directWriteActive = true;
    directWriteStartTime = millis();
    ...
    return true;
}
```

and at each leaf, the handler's existing NACK idiom:

```c
    if (!directWriteActivatePanel()) { /* NACK, see D-A2 */ return; }
```

#### Why not the alternative

Keeping `bool cold` and having callers poll `panelStateUnknown` is a smaller diff, but it inverts
the default: every happy path proceeds to drive an un-acquired panel unless someone remembered to
add a check. That is the same failure shape as the `[C4]` finding in Phase 4 (a blind flag-setter
with the guard somewhere else). **Recommend the signature change.**

`epdSessionRelease` / `epdSessionForceOff` do **not** need signature changes — both return `void`
today and an early return on lock-timeout is a complete response for them (P2-1 table). Only
`epdSessionAcquire` has a caller that must change behaviour.

### ~~D-A2~~ — What does the START handler NACK with? — ⛔ **MOOT: P2-1 dropped**

The three leaf handlers must tell the client "not started". Checked against
`include/opendisplay_protocol.h:784-804`: **neither error namespace has a "device busy" or
"internal error" code.** `OD_ERR_PIPE_START_*` 0x04 is unused, but assigning it a meaning is a
protocol change and explicitly out of bounds.

| Option | Assessment |
|---|---|
| Reuse `OD_ERR_PIPE_START_PARTIAL_UNSUPPORTED` / `OD_ERR_PARTIAL_UNSUPPORTED` | Semantically wrong — tells the client the *mode* is unsupported, so it may permanently stop trying partials |
| Bare 2-byte `[RESP_NACK][opcode]` | **Precedent exists** — `handlePartialWriteStart` already sends `{RESP_NACK, 0x76}` at [display_service.cpp:2232](../src/display_service.cpp). But the header specifies the 4-byte form for 0x80 (`:600-608`), so this needs checking against py-opendisplay's NACK parser before use on the pipe path |
| Claim 0x04 | Out of bounds — protocol change |

**Recommend the 2-byte form, contingent on a read of py-opendisplay's NACK parsing.** If the client
rejects a short NACK on 0x80, fall back to `OD_ERR_PIPE_START_BAD_HEADER` (0x01) — wrong, but
non-poisoning: the client retries rather than disabling a feature. This is the one point where D-A
touches the no-protocol-change constraint, so settle it before writing the handlers.

### ~~D-B~~ — Who owns `panelStateUnknown`? — ⛔ **MOOT: P2-1 dropped** (no such flag is produced)

Proposed: `display_service.cpp` owns it, set only by `pwrmgmLockTake`, cleared only by a successful
`epdSessionForceOff`. Alternative: defer the whole flag to Phase 3 and have Phase 2 only log the
timeout. **Recommend keeping it in Phase 2** — the flag is where the knowledge is, and Phase 3
just reads it. Confirm that a dead-until-Phase-3 flag is acceptable in review.

### ~~D-C~~ — Does the panel become unusable after a lock timeout? — ⛔ **MOOT: P2-1 dropped** (there is no lock timeout)

*Confirmed 2026-07-26.* `pwrmgmLockTake` always attempts the full 60 s take; `panelStateUnknown`
is reported but never suppresses a retry. Aggregate cost is bounded by P2-5's drain budget and
(→ Phase 6) the supervisor. Revisit only if hardware soak shows 60 s retries stacking. Options
considered, retained for the record:

1. **Keep trying** — every subsequent panel op re-attempts the 60 s take. Simple; a genuinely
   wedged lock costs 60 s per operation forever.
2. **Fail fast while set** — `pwrmgmLockTake` returns false immediately when `panelStateUnknown`
   is already set, until a successful force-off clears it. Bounds the damage; risks latching the
   panel off after one transient.
3. **Fail fast with a retry window** — as (2), but allow one full attempt every N seconds.


### ~~D-E~~ — `PWRMGM_LOCK_TIMEOUT_MS` value — ⛔ **MOOT: P2-1 dropped**

Derived above from `bbepWaitBusy`'s `iMaxTime = 30000`. If someone wants it tighter, the derivation
— not the intuition — has to change. Note that a `pwrmgmLockTake` timeout is *itself* a 60 s block
of `loop()`, which is fine under the 5 s TWDT only because of the yielding `delay(1)`.

### ~~D-H~~ — Does P2-9 belong in Phase 2 at all? — ⛔ **ANSWERED BY THE DROP: no**

It is arguably Phase 6's job — it is a supervisor. The case for Phase 2: Phase 6's supervisor runs
*inside* `loop()` and therefore cannot observe a blocked `loop()` on either target, so it is not
the same mechanism and it cannot subsume this one. The case against: Phase 2 is otherwise a set of
local edits with no new files or tasks, and P2-9 is neither.

**Recommend keeping it in Phase 2** but landing it **last**, so it can be dropped without
disturbing the other eight items. If it moves to Phase 6, the parent plan's Phase 6 section must be
amended to say the supervisor has two arms — in-loop (state stalls) and out-of-loop (loop
blockage) — because as written it only has the first.

### ~~D-I~~ — detect-and-log vs reboot for P2-9 — ⛔ **MOOT: P2-9 dropped**

*Confirmed 2026-07-26.* **P2-9 is detect-and-log only.** It logs an ERROR with elapsed time and
phase breadcrumb (rate-limited to one line per 30 s), sets `g_loopStalled` for Phase 6 to report on
resume, and takes **no recovery action**. No reboot, no hardware WDT, on either target.

Accepted consequence, stated plainly so review does not reopen it: **a genuinely blocked `loop()`
is not recovered by Phase 2.** The recovery story for cooperative blocking remains the per-wait
deadlines in P2-1 / P2-4 / P2-8; for a true hard hang there is none, and the device stays frozen —
still displaying the correct image, since e-paper holds it without power — until a power cycle.
What P2-9 buys is that the event stops being invisible.

Option (3) below (rate-limited, flash-persisted reset) is **deferred, not rejected**. Revisit only
if hardware soak produces stall lines; if it does, the phase breadcrumb tells us where, which is
the input that proposal needs. Do not implement it speculatively.

The full trade is retained below because the "no reboot" decision rests on a narrower argument than
it first appears — it rules out *unconditional* reboot, and a future reader will need to know that.

---

Under "software-only, never reboot", an observer task has no safe recovery action: it cannot touch
panel/BLE/session state from another task context. So for the one fault class P2-9 exists to catch
— a genuinely blocked `loop()` — **log-only does nothing**. That makes the reboot option worth
arguing properly rather than dismissing by reference to the earlier decision.

#### What a reboot actually costs (verified)

The parent plan's justification is "a reboot wipes RTC incl. `displayed_etag` and forces a
boot-screen redraw." Both halves check out, and the mechanism is worse than the summary suggests:

- **RTC does not survive a reset at all.** The bootloader reloads RTC memory segments from the app
  image on every reset *except* a deep-sleep wake ([main.cpp:95-99](../src/main.cpp), captured on
  hardware in `docs/FINDINGS_DEEP_SLEEP_WAKE_BOOT_SCREEN_2026-07-07.md`). Lost: `displayed_etag`
  ([main.h:294](../src/main.h)), `deep_sleep_count`, `mloopcounter`, `rebootFlag`, and the cached
  WiFi BSSID/channel ([wifi_service.cpp:511-513](../src/wifi_service.cpp)).
- **The redraw is automatic and unconditional.** `is_deep_sleep_wake` false → `rebootFlag = 1` →
  `initDisplay()` → full boot-screen refresh ([main.cpp:120-129](../src/main.cpp)). On a Spectra
  panel that is 30–60 s of powered refresh.
- **`displayed_etag = 0` forces a full re-upload.** The next partial update gets an ETAG mismatch
  and falls back to a full push — so HA re-sends the whole image.
- **It is indistinguishable from a first boot.** The code comment says so explicitly: a hidden
  mid-cycle reset "lands here with count 0, indistinguishable from a true first boot."

#### The case FOR rebooting — stronger than the parent plan allows

1. **It is the only thing that recovers a hard hang.** Every other bound in Phase 2 assumes the
   fault is cooperative (the blocked task yields). If it does not, a reset is the sole remaining
   mechanism. "Log and hope" is not a recovery story.
2. **The nRF hardware is already there and unused**, and nRF has *no* recovery mechanism of any
   kind today.
3. **A frozen tag self-heals within one push cycle if it reboots.** HA pushes on a schedule; a
   device that reboots, redraws a boot screen, and accepts the next push is functional again in
   minutes. A frozen one is dead until someone physically intervenes.
4. **The physical fallback is weaker than it sounds.** The parent plan's own residual-risk section
   establishes that a long-press cannot power off while `loop()` is blocked — the hold evaluation
   runs in `processButtonEvents()`, called only from `loop()`/`idleDelay()`. On a
   `DEVICE_FLAG_BATTERY_LATCH` unit the user's fallback is "remove the battery." Against that
   baseline, an automatic reboot looks generous.

#### The case AGAINST — and why it wins for *this* device

The decisive argument is specific to e-paper, and it is not the etag:

**A frozen tag still displays the correct image.** E-paper holds its image with no power. A wedge
costs you *updates*, not the display. So the trade is not "broken vs working" — it is:

> **freeze** = correct image, no updates, silent · **reboot** = *wrong* image (boot screen) for a
> push cycle, updates work, silent

And then the tail risk, which is what settles it: **the wedge this plan targets is loss-driven and
recurrent.** A reboot-on-stall device that hits it repeatedly gives you a tag that reboots every
few minutes, performs a full-panel refresh each time — the single largest energy cost on a battery
unit — and, because RTC is wiped, reports every one of those boots as a first boot. You get a
battery-destroying reboot loop that is *invisible in telemetry*. That is a worse failure than the
freeze, and it is a realistic one, not a hypothetical.

The secondary argument: a WDT firing on a **false positive** during a legitimate 60 s refresh
interrupts the refresh mid-waveform, which violates the parent plan's "never interrupt a refresh"
rule and can leave the panel in a bad state.

**So the original decision holds — but for a sharper reason than "reboots are bad": an unbounded,
unlogged reboot loop on a battery e-paper device is worse than the freeze it fixes.** Note that
this reasoning attacks *unconditional* reboot, not reboot as such.

#### The positions

1. **Log only** — ✅ **chosen.** ERROR with elapsed time and phase breadcrumb, rate-limited,
   `g_loopStalled` for Phase 6 to report on resume. Closes the *diagnosis* gap; leaves the
   *recovery* gap open for hard hangs.
2. **Unconditional reboot on stall** — rejected on the reboot-loop argument above.
3. **Rate-limited, persisted reboot** — the middle the plan does not currently offer, and the only
   version of (2) that survives the objection. Reboot on stall **at most once per N hours**, with
   the reason and a counter written to **flash, not RTC** (LittleFS is configured on the ESP32 envs
   via `board_build.filesystem = littlefs`; the nRF equivalent needs checking) so the reboot is
   visible afterwards and the loop is bounded. On exceeding the rate limit, stop rebooting and fall
   back to (1).
4. **Log + monitor sets abort flags** for `loop()` to service on resume — this is (1) plus a Phase 3
   dependency, and does nothing a *stalled* `loop()` can act on. Not a distinct position.

**Resolution: (1), with (3) as the documented follow-up.** Soak tells us the thing we actually need
and do not have: whether a blocked `loop()` ever happens once P2-1, P2-4 and P2-8 are in. If soak
produces zero stall lines, (3) is unnecessary complexity; if it produces them, we will have a phase
breadcrumb saying where, which is worth more than a blind reset. Choosing (3) now would be building
a recovery mechanism for a fault we have not yet observed.

### ~~D-J~~ — `LOOP_STALL_WARN_MS` value — ⛔ **MOOT: P2-9 dropped**

Derived: longest legitimate pass = `waitforrefresh(60)` = 60 s (real wall clock after P2-8) plus a
worst-case `pwrmgmLockTake` timeout = 60 s (D-E), so 150 s clears both with margin. Note the
dependency — **if D-E lowers the lock timeout, this can come down with it.** Must stay well under
Phase 6's 600 s supervisor so the two signals are distinguishable in a log.

## Appendix — dropped items, retained for the record

Everything below was specified for Phase 2 and then cut. It is kept because the *analysis* is the
expensive part and is what stops each item being re-proposed from scratch — `[C2]`'s do-not-steal
argument, P2-5's arithmetic showing the drain cap is powerless, and P2-9's rejected alternatives
(nRF hardware WDT, idle-hook heartbeat) in particular.

**None of it is in scope.** Each section opens with what was dropped and the residual it leaves;
the residuals are collected in "Residual risk after Phase 2" above.

---

## ~~P2-9~~ — Loop-liveness heartbeat + monitor task — ❌ **DROPPED**

*Dropped 2026-07-26 by owner decision.* **Not implemented in Phase 2.** No `src/session_monitor.*`,
no monitor task, no `loop()` heartbeat, no `LOOP_STALL_WARN_MS`. `src/main.cpp` is therefore
**untouched by Phase 2 except for P2-6's comment**, which also removes the last point of contact
with Phase 3's drain-loop edit.

**This is the most consequential of the three drops, so be explicit about what it costs.** P2-9 was
the only item satisfying **condition 3** — *a violation is observable by something other than the
blocked party*. Without it:

- **A stalled `loop()` is silent on both targets.** ESP32's TWDT watches IDLE0 starvation at 5 s /
  panic, but every long wait here yields, so IDLE0 is never starved and the TWDT will not fire on
  any fault Phase 2 was about. nRF has no watchdog at all (`NRF_WDT` is never started).
- **nRF keeps zero out-of-`loop()` observers.** This matters more than it did when the plan was
  written: Phase 1 demonstrated on hardware that nRF's `loop()` *is* starved mid-transfer (its
  deferred link-drop never executed, forcing the inline disconnect in `23ecaed`). Phase 2 now
  bounds several waits it cannot report on for that target.
- **Combined with the P2-1 drop**, a panel-lock holder that never releases is both unbounded and
  undetected until Phase 6.

The `~2 KB` stack and the `esp32-N4` headroom question go away with it; so do D-H, D-I and D-J.

The original design is retained below for the record — including the rejected alternatives (nRF
hardware WDT, idle-hook heartbeat), which are the useful part if this is ever revived.

---

### The gap *(retained for the record — not implemented)*

Condition (3) — *a violation is observable by someone other than the blocked party* — is unmet on
**both** targets:

- **nRF has no observer at all.** No task watchdog, no hardware WDT started, `vApplicationIdleHook`
  empty.
- **ESP32's observer is the wrong one.** The TWDT watches IDLE0 starvation with a 5 s timeout and
  `_PANIC 1` — it **reboots**, which the user's decision explicitly forbids ("reset state, never
  reboot; a reboot wipes RTC incl. `displayed_etag`"). And because every Phase 2 wait yields, IDLE0
  is never starved, so the TWDT will not fire on any fault Phase 2 is about.
- **→ Phase 6's supervisor cannot fill this gap** on either target: it is specified to run *inside*
  `loop()`, so a blocked `loop()` means the supervisor never executes. Phase 6 detects *state-machine
  stalls*; nobody detects *loop blockage*. Worth flagging back into the parent plan.

### Design: a heartbeat stamped by `loop()`, read by a higher-priority task

```c
// session_monitor.h  (new; deliberately NOT session_guard.* — that is Phase 3's file)
void   sessionMonitorBegin(void);        // create the task; call at the end of setup()
void   sessionMonitorHeartbeat(const char* phase);   // stamp from loop()
bool   sessionMonitorLoopStalled(void);  // -> Phase 3 / Phase 6
```

- `loop()` calls `sessionMonitorHeartbeat("top")` as its first statement — **outside** every
  `#ifdef TARGET_ESP32`, so it is one call on both targets. Optional extra stamps with a phase
  breadcrumb (`"drain"`, `"refresh"`, `"wifi"`) make the stall log say *where*.
- The monitor task runs at **priority 2** and `vTaskDelay(1000)`s. Priority 2 is above the loop task
  on both targets (ESP32 `loopTask` = 1; nRF `TASK_PRIO_LOW` = 1) and at or below the BLE tasks, so
  it preempts a spinning or compute-bound `loop()` but never delays the radio.
- On `millis() - g_loopHeartbeatMs > LOOP_STALL_WARN_MS`: log one ERROR with the elapsed time and
  the last phase breadcrumb, then **rate-limit to one line per 30 s** so a long stall does not flood
  the log. Set `g_loopStalled`.
- On recovery: log one INFO with the total stall duration, clear `g_loopStalled`.

Creation is the only platform-specific line:

```c
#if defined(TARGET_ESP32)
    xTaskCreatePinnedToCore(monitorTask, "odmon", 2048, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
#else   // TARGET_NRF — Adafruit core, cores/nRF5/rtos.h:59 TASK_PRIO_NORMAL == 2
    xTaskCreate(monitorTask, "odmon", 512 /*words = 2 KB*/, NULL, TASK_PRIO_NORMAL, NULL);
#endif
```

Note the stack-size unit differs: ESP32's `xTaskCreate` takes **bytes**, vanilla FreeRTOS (nRF)
takes **words**. Getting this wrong is a silent stack overflow — though nRF has
`configCHECK_FOR_STACK_OVERFLOW 1` (`FreeRTOSConfig.h:78`) to catch it in test.

### What it can and cannot do

**Deliberately detect-and-log only.** It must not touch panel, BLE, or session state — it runs on a
different task from every one of those subsystems, and `pwrmgmLock` is the only cross-task guard in
the codebase. Under the no-reboot decision there is no safe recovery action available to an
observer task; its value is that a field freeze stops being invisible and starts producing a
timestamped ERROR naming the phase it died in. `g_loopStalled` is a **→ Phase 6** input: the
supervisor, once `loop()` resumes, can report that a stall occurred and how long it lasted.

This is an honest limit, not a hedge: it closes the *diagnosis* gap, not the *recovery* gap. The
recovery story for a blocked `loop()` remains P2-1/P2-4/P2-8's per-wait deadlines — P2-9 is what
tells you when one of them failed to hold.

### `LOOP_STALL_WARN_MS`

Must exceed the longest legitimate single `loop()` pass. That is a full refresh: `waitforrefresh(60)`
after P2-8 is a true 60 s wall-clock bound, and a `pwrmgmLockTake` timeout adds another 60 s.
**`LOOP_STALL_WARN_MS = 150000` (2.5 min)** clears 60 + 60 with margin and still fires long before
Phase 6's 10-minute supervisor. See D-J.

### Cost

One task, ~2 KB stack + TCB, plus two `uint32_t` and a `const char*`. nRF52840 has 256 KB RAM and is
not a concern. **`esp32-N4` is still the gate** — it needs `PIPE_SMALL_DRAM_WINDOW` to fit at all —
but the combined-headroom warning that stood here is obsolete in Phase 2's favour: Phase 1 shipped a
**32 B bitmap in place of the 512 B ring**, not the `replay_window[256]` this plan was written
against, so it *returned* 480 B rather than consuming 1,536. Measured on the as-built branch:
`esp32-N4` **81,468 B** vs the 81,940 B pre-Phase-1 baseline.

So P2-9 starts with ~480 B more room than budgeted. Still measure the link rather than assume — the
figure that matters is the successful link, not the percentage — but the fallback (compile P2-9 out
on that env alone via `-DOPENDISPLAY_NO_LOOP_MONITOR`, rather than shrinking the stack) is now less
likely to be needed.

### Rejected: the nRF hardware WDT

`NRF_WDT` is available and unused, and would be the obvious "make it binding" answer. Rejected:

- It **resets the chip**, which is the one outcome the user's decision rules out.
- Its `EVENTS_TIMEOUT` fires only ~2 LFCLK cycles (~61 µs) before the reset — not remotely enough to
  run a state teardown, so it cannot be repurposed into a soft supervisor.
- Once started it **cannot be stopped** except by reset, which complicates DFU and any future
  debugging session.

Recorded here so the option is not rediscovered and re-argued. The same reasoning is why P2-6
deletes the ESP32 TWDT flag rather than trying to make it work.

**But note the limit of that rejection.** It rules out an *unconditional* WDT reset. A rate-limited
reset with the reason persisted to flash is a materially different proposal and is not covered by
the arguments above — see **D-I** for the full trade, including why an unlogged reboot loop on a
battery e-paper tag is worse than the freeze it fixes, and why the recommendation is nonetheless to
defer it until soak data exists.

### Rejected: an idle-hook heartbeat

`vApplicationIdleHook` is overridable on nRF (weak alias, `hooks.c:33`) and `esp_register_freertos_idle_hook()`
exists on ESP32, so a symmetric idle-hook stamp is cheap and needs no task. But the idle hook
answers *"did anything run?"*, not *"did `loop()` advance?"* — and every fault in scope leaves the
system busily yielding, so idle keeps running throughout. It would detect only total CPU
starvation, which is the fault class already accepted as unrecoverable. The extra task is what buys
the actual signal.

---

## ~~P2-1~~ — Bound `pwrmgmLockTake`, do not steal — ❌ **DROPPED**

*Dropped 2026-07-26 by owner decision.* **Not implemented in Phase 2.** `pwrmgmLockTake()`
([display_service.cpp:401-408](../src/display_service.cpp)) keeps its unbounded
`while (__atomic_exchange_n(...)) { delay(1); }` spin, unchanged. No deadline, no `bool` return, no
`panelStateUnknown` flag, and no change to any `epdSession*` signature.

**What this leaves open — state it plainly.** The parent plan lists this spin as one of its five
unbounded waits. A holder that never releases still blocks its waiter forever. The two known
long-but-legitimate holds remain the reason a naive bound was risky in the first place
(`bbepWaitBusy` caps at 30 000 ms on 3/4/7-colour panels; `epdSessionForceOffLocked` holds across
`bbepSleep` → `bbepWaitBusy`), so the residual is specifically "a hold that never *ends*", not "a
hold that runs long". With P2-9 also dropped, **nothing in Phase 2 detects or reports that stall on
either target** — recovery rests entirely on Phase 6's supervisor, and on nRF (where the ESP32
wall-clock watchdogs do not run) on nothing at all until Phase 6 lands. The `[C2]` reasoning stands
and is worth preserving:
if anyone revisits this, **do not steal the lock** — it is a bare 0/1 flag with no owner, so a steal
makes the true holder's later `Give` unlock it underneath the stealer, permanently destroying mutual
exclusion on the panel's SPI/CS lines.

**Knock-on: five decisions become moot** — D-A, D-A2, D-B, D-C and D-E all existed only to shape
this item. See the Decisions section.

The original specification is retained below for the record.

---

### The rule *(retained for the record — not implemented)*

> `pwrmgmLockTake()` gets a deadline and a `bool` return. On expiry it **does not acquire**. The
> caller skips its panel work, reports failure upward, and sets a sticky `panelStateUnknown` flag.
> Nothing is ever stolen.

Stealing a bare 0/1 flag is unrecoverable: two tasks would drive the same SPI/CS lines, and the
original holder's eventual `pwrmgmLockGive()` ([:412](../src/display_service.cpp)) unlocks the
lock *out from under the stealer*, permanently killing mutual exclusion. There is no owner field
to detect it with. This is the whole point of `[C2]`.

### Deadline: 60 000 ms — justification

The bound must exceed the longest **legitimate** hold, or it converts a slow panel into a
spurious failure. Longest legitimate hold, measured from the sources:

- `epdSessionAcquire` ([:437-489](../src/display_service.cpp)) holds across `bbepWakeUp` +
  `bbepSendCMDSequence`, each of which can call `bbepWaitBusy` → up to **30 000 ms** on a
  3/4/7-colour panel (`bb_ep.inl:3966-3968`).
- `epdSessionForceOffLocked` ([:416-433](../src/display_service.cpp)) holds across `bbepSleep` →
  `bbepWaitBusy` → another **30 000 ms**, plus a `delay(50)` loop.

So a single hold can legitimately approach 30 s, and a queued acquire behind a force-off can
legitimately wait ~30 s more. **60 s = 2× worst-case single busy wait** is the smallest number
that cannot fire on healthy hardware. Do not tune it down without re-deriving from
`bbepWaitBusy`'s `iMaxTime`.

### Code shape

```c
// display_service.cpp — replace :401-408
// Bounded acquire. Returns false on timeout WITHOUT acquiring; the caller must
// then skip all panel work. We deliberately do NOT steal: pwrmgmLock is a bare
// flag with no owner, so a steal lets two tasks drive the same SPI/CS and the
// true holder's later Give unlocks it under the stealer -- mutual exclusion
// permanently dead. Deadline is 2x bbepWaitBusy's 30 s multi-colour cap.
#define PWRMGM_LOCK_TIMEOUT_MS 60000u

static bool pwrmgmLockTake(void) {
    const uint32_t start = millis();
    while (__atomic_exchange_n(&pwrmgmLock, 1, __ATOMIC_ACQUIRE)) {
        if ((uint32_t)(millis() - start) > PWRMGM_LOCK_TIMEOUT_MS) {
            od_log_error("[EPD session] pwrmgm lock TIMEOUT after %u ms - panel state UNKNOWN",
                         PWRMGM_LOCK_TIMEOUT_MS);
            panelStateUnknown = true;
            return false;
        }
        delay(1);   // vTaskDelay: must yield, see priority-inversion note below
    }
    return true;
}
```

Keep the existing priority-inversion comment verbatim — the `delay(1)` is load-bearing on nRF and
someone will otherwise "optimise" it back to a busy-spin.

### The five call sites

`pwrmgmLockTake()` has five callers today. Each needs an explicit failure branch. `pwrmgmLockGive()`
must be reachable on **every** path that took the lock and on **no** path that didn't.

| Site | Function | Failure behaviour |
|---|---|---|
| [:439](../src/display_service.cpp) | `epdSessionAcquire(bool partialInit)` | **Signature change** — must report failure. See below. |
| [:496](../src/display_service.cpp) | `epdSessionRelease(bool)` | Return early. Panel is left however it is; `panelStateUnknown` is set. |
| [:513](../src/display_service.cpp) | `epdSessionForceOff(void)` | Return early. Log ERROR — this is the worst one to lose (rail stays up). |
| [:520](../src/display_service.cpp) | `epdSessionTick(void)` | Already `pwrmgmLockTryTake()` — **unchanged**. |
| — | `epdSessionForceOffLocked` | Caller-holds-lock; **unchanged**. |

`epdSessionAcquire` currently returns `bool cold` — a *result*, not a status. Two options:

- **(a)** `static bool epdSessionAcquire(bool partialInit, bool* outCold)` — returns success.
- **(b)** Keep `bool cold` and expose the failure via `panelStateUnknown`, checked by callers.

**Recommend (a).** Option (b) makes every caller's happy path silently proceed to drive an
un-acquired panel, which is exactly the class of bug this phase exists to remove. (a) is a
mechanical change across the `epdSessionAcquire` call sites and the compiler finds them all.

### `panelStateUnknown` → Phase 3

```c
// display_service.h
extern volatile bool panelStateUnknown;   // sticky: a panel op was skipped on a lock timeout
```

- Set: only by `pwrmgmLockTake()` on expiry.
- Cleared: on the next **successful** `epdSessionForceOff()` — that is the one operation that
  restores a known state (rail down).
- Consumed: **→ Phase 3.** `abortToKnownState()` reports it and skips `epdSessionForceOff()`
  when set (retrying a lock that just timed out costs another 60 s inside the abort path).
  Phase 2 only produces it; nothing reads it yet, which is intentional and should be noted in
  the commit message so review does not flag it as dead code.

### Deferred: the owner handle

`volatile TaskHandle_t pwrmgmOwner` (so a stale `Give` becomes detectable) is **not** in Phase 2.
It is only needed if a forced take ever becomes necessary, and Phase 2's position is that it never
is. Recorded here so the option is not rediscovered from scratch.

---

## ~~P2-2~~ — Bound the `powerOff` stuck-button wait — ❌ **DROPPED**

*Dropped 2026-07-26 by owner decision.* **Not implemented in Phase 2.**
[power_latch.cpp:85-90](../src/power_latch.cpp) is unchanged; a stuck-low button pin still means the
device never powers off. Narrow residual: ESP32-only (the whole file is `#if defined(TARGET_ESP32)`),
requires a hardware fault in the button itself, and it removes a *recovery* path rather than adding a
freeze — the device is not wedged by it, the user's last-resort power-off is simply unavailable. It
compounds the parent plan's residual-risk note that "hold the button" is already a weak fallback
while `loop()` is blocked.

The original specification is retained below for the record.

---

*(retained for the record — not implemented)* Today a shorted or stuck-low button pin means the
device never powers off — the user's last-resort recovery is gone (see the parent plan's residual-risk
section, which already calls this out as the reason "hold the button" is a weak fallback).

```c
void powerOff() {
    const gpio_num_t latch = latchPin();
    if (hasButton()) {
        pinMode(buttonPin(), INPUT_PULLUP);
        // Bounded: a stuck-low button must not block power-off forever. After
        // 10 s we drop the latch anyway -- worst case the rail cycles and the
        // still-held button re-latches, which is indistinguishable to the user
        // from a normal press-and-hold-too-long.
        const uint32_t start = millis();
        while (digitalRead(buttonPin()) == LOW) {
            if ((uint32_t)(millis() - start) > 10000u) {
                od_log_warn("[power] button still held after 10 s - dropping latch anyway");
                break;
            }
            delay(20);
        }
    }
    ...
```

**Why the wait exists at all:** it prevents the rail dropping while the button is still held, which
on a latching board would immediately re-latch and power the device back on. Bounding it
reintroduces exactly that possibility after 10 s — which is the correct trade (a power-cycle is
recoverable; a device that cannot be turned off is not).

`power_latch.cpp` is inside `#if defined(TARGET_ESP32)`; **nRF is unaffected** by this item.

---

## ~~P2-5~~ — Wall-clock cap on the loop command drain — ❌ **DROPPED**

*Decided 2026-07-26.* **Not implemented in Phase 2.** No change to the drain loop at
[main.cpp:406-424](../src/main.cpp) — no `COMMAND_DRAIN_BUDGET_MS`, no cap, and no saturation WARN
either. The whole item is out.

Two consequences worth carrying forward:

- **Phase 3 owns the drain loop uncontested.** `[M5]`'s drain-trap fix is now the only edit to those
  five lines; there is no `drainStart` in scope and no merge conflict to sequence around.
- **The saturation signal moves to Phase 7, where it belongs.** "The command ring ran full" is
  `[H1]`'s subject matter, not a Phase 2 bound. If a diagnostic is wanted, Phase 7 should add it
  alongside its overflow handling rather than Phase 2 bolting a WARN onto a loop it otherwise does
  not touch.

The analysis below is retained so this is not re-proposed.

### The stated rationale does not survive checking

The parent plan asks for a 2 s cap because "a full window of commands can hold `loop()` for
minutes." Walked through against the code, that cannot happen:

| Scenario | Actual cost | Does a 2 s cap help? |
|---|---|---|
| Full window of 32 pipe DATA frames | Each is a zlib inflate + SPI write — single-digit ms. 32 of them ≈ **0.1–1 s** even on a slow target | **No** — never reaches 2 s |
| One END frame triggering a full refresh | **30–60 s**, the genuinely long case | **No** — the check is *between* commands and cannot interrupt one |
| Several refreshes stacked in one drain — the only case a cap would catch | **Unreachable.** A second `0x0072` short-circuits at [display_service.cpp:2366](../src/display_service.cpp) (`if (!directWriteActive) return;`); the pipe END paths are guarded by `pipeState.active` | **N/A** |

So the cap fires in no realistic scenario, and in the one case where `loop()` really is blocked for
a minute it is powerless by construction. **The drain is already bounded** — 33 × (per-command
time), and per-command time is bounded by P2-1, P2-4 and P2-8. The real cost of a saturated drain
is ~1 s of unserviced touch/buttons, which is not the unbounded-wait class Phase 2 exists to fix.

I had also flagged this item as a **merge hazard with Phase 3's `[M5]`** — it edits the same five
lines. Dropping it removes that conflict for free.

### The one residual argument, and why it is not enough

The drain is the one place where many non-yielding operations run back-to-back, and ESP32's TWDT
panics at 5 s of IDLE0 starvation. A 2 s cap would guarantee a yield point inside the drain and so
buy TWDT margin if commands ever got much slower. But at today's ~1 s worst case the margin is
already 5×, and adding a mechanism against a hypothetical future regression is exactly the
speculative engineering this plan rejects elsewhere (see D-I option 3).

### Original proposal, retained for the record

```c
{
    uint8_t drained = 0;
    const uint32_t drainStart = millis();
    while (drained < COMMAND_QUEUE_SIZE) {
        // Wall-clock cap alongside the count cap: a single command (a pipe END
        // frame runs a full refresh inline) can take seconds, so 33 of them can
        // hold loop() for minutes and starve disconnect cleanup / epdSessionTick /
        // WiFi / buttons. Unconsumed commands stay queued and drain next pass.
        if (drained > 0 && (uint32_t)(millis() - drainStart) > COMMAND_DRAIN_BUDGET_MS) {
            od_log_warn("[drain] budget exceeded after %u commands - deferring rest", drained);
            break;
        }
        ...
```

**`COMMAND_DRAIN_BUDGET_MS = 2000`** (parent plan's number). Note the `drained > 0` guard: the cap
must never prevent the *first* command from running, or a single slow command starves the queue
forever.

Notes that would have applied: break-don't-drop (unconsumed entries stay in the ring, serviced next
pass); the check cannot fire mid-command; `flushResponseQueueToBle()` already runs between commands
([:422](../src/main.cpp)) so breaking early could not strand pipe ACKs.

**Merge hazard (now avoided by dropping the item):** Phase 3's `[M5]` edits the same five lines —
it inserts a `commandDrainAbortPending` check between [:415](../src/main.cpp) and `:416` and deletes
the vestigial `pending` field. If P2-5 is kept after all, land it before Phase 3 and note there that
`drainStart` is already in scope.

---

## ~~P2-6~~ — Delete the inert TWDT flag, document the real one — ❌ **DROPPED**

*Dropped 2026-07-26 by owner decision.* **Not implemented in Phase 2.**
`-DCONFIG_FREERTOS_WATCHDOG_TIMEOUT_S=120` stays in all 9 ESP envs
(`platformio.ini:53, 83, 112, 140, 189, 209, 229, 253, 295`), and no comment is added recording the
real setting.

**Residual: a misleading dead knob stays in the tree.** The symbol is not an IDF 5.x setting — the
real one is `CONFIG_ESP_TASK_WDT_TIMEOUT_S`, and the precompiled `sdkconfig.h` wins regardless, so
the true watchdog is **5 s / panic on IDLE0 starvation**, not the 120 s the flag implies. Today's
30–60 s waits survive only because every one of them yields. The next reader of `platformio.ini`
has no way to know that from the tree. Not a freeze risk — the flag has never done anything — but
it is a live source of wrong conclusions, and it is the cheapest item in the phase (a build-flag
deletion plus one comment). Worth revisiting whenever anything else touches `platformio.ini`.

**Consequence:** Phase 2 now touches **no build configuration and no `src/main.cpp`** — the item was
the only reason for either.

The original specification is retained below for the record.

---

### Original specification *(retained for the record — not implemented)*

Remove `-DCONFIG_FREERTOS_WATCHDOG_TIMEOUT_S=120` from all 9 ESP envs (`platformio.ini:53, 83, 112,
140, 189, 209, 229, 253, 295`). It is an IDF 4.x symbol name; IDF 5.x uses
`CONFIG_ESP_TASK_WDT_TIMEOUT_S`, which the precompiled `sdkconfig.h` fixes at 5 and which a
`build_flags` define cannot override anyway (same mechanism as the documented
`CONFIG_BT_NIMBLE_MAX_CONNECTIONS` trap). Delete rather than rename — renaming to the correct
symbol would be a no-op that *looks* effective, which is worse than the current dead knob.

Replace with a comment at the top of `loop()` in `main.cpp`:

```c
// Task watchdog reality check (do not "fix" this with a build flag):
// the precompiled IDF 5.5.4 sdkconfig.h fixes CONFIG_ESP_TASK_WDT_TIMEOUT_S=5
// with _PANIC=1 and _CHECK_IDLE_TASK_CPU0=1, and a -D in build_flags cannot
// override it. So the real bound on this task is 5 s of IDLE0 starvation ->
// panic reboot. Today's 30-60 s panel waits survive only because every one of
// them yields (delay()/vTaskDelay()/bbepLightSleep()). Any new busy-spin in a
// panel or BLE path WILL reboot the device. A reboot is also the one outcome
// this whole effort is trying to avoid -- it wipes RTC state including
// displayed_etag and forces a boot-screen redraw.
```

Optionally add one line to `docs/TIMER_AND_WATCHDOG_INVENTORY_2026-07-26.md` recording that the
120 s entry was fiction.

---

