# Can the nRF52840 and ESP32 `loop()` / `idleDelay()` paths be converged?

**Date:** 2026-07-26 · **Branch:** `debug/ble-hardening` (HEAD `2e2131b`)
**Scope:** this repo only. Three framework sources were opened for one decisive signature each — every such use is labelled **[LIB]**; nothing else was read from them. Sibling repos not consulted.

## Recommendation

**Verdict: partially feasible — and the feasible part is not the appealing part.** Full convergence (A) is not worth it: of the ~160-line ESP32 arm, ~130 lines are deep-sleep, WiFi/LAN, NimBLE advertising and queue-drain work with no nRF counterpart. Converging the *execution model* (C — nRF adopts the SPSC ring) is **feasible on RAM but counter-productive**: Bluefruit already copies each write payload and posts it to a dedicated FIFO "Callback" task with an *elastic* queue **[LIB]**, so an app ring adds a second copy and hop only to replace an unbounded-but-growing queue with a fixed 32-slot drop-on-full ring, and to move dispatch from a `TASK_PRIO_NORMAL` task onto the `TASK_PRIO_LOW` loop task that lives inside 100 ms `delay()` chunks. **Recommended: option B fused with D** — extract the target-agnostic supervisory work (`processLedFlash`/`epdSessionTick`/`buzzerService`, the two 15-min watchdogs, later the Phase 6 supervisor) into one `serviceSupervisoryTick()` called from both `loop()` arms *and* from `idleDelay()`. ~50 LOC in one file, closes the real defect (nRF has no transfer watchdog), gives the freeze-proofing plan one wiring point instead of two. Hard prerequisite: plan item **H4**'s `g_commandInFlight` depth counter must land first.

## 0. Corrections to the stated facts

Nine of twelve confirmed exactly. Confirmed: single top-level `#ifdef` ([main.cpp:352-354](../src/main.cpp) prologue, `#ifdef` at [:355](../src/main.cpp) and [:358](../src/main.cpp), `#else` [:518](../src/main.cpp), `#endif` [:530](../src/main.cpp)); 11-line nRF arm [:519-529](../src/main.cpp); `idleDelay` [:535-551](../src/main.cpp); both watchdogs inside the ESP32 arm ([:436-442](../src/main.cpp), [:443](../src/main.cpp)) while `checkPartialWriteTimeout()` itself ([display_service.cpp:578-587](../src/display_service.cpp)) is target-agnostic; no `sd_power_system_off` anywhere and `power_latch.cpp` stubs out on non-ESP32 ([power_latch.cpp:194-210](../src/power_latch.cpp)); ESP32 `onConnect` deferral ([esp32_ble_callbacks.h:52-55](../src/esp32_ble_callbacks.h)); `pwrmgmLock` comment ([display_service.cpp:397-408](../src/display_service.cpp)).

Three refinements:

- **R1 (load-bearing).** *"On nRF there is NO queue."* There **is** one — it belongs to the framework. `BLECharacteristic::_eventHandler` dispatches the write via `ada_callback(request->data, request->len, _wr_cb, …)` **[LIB]** (`Bluefruit52Lib/src/BLECharacteristic.cpp:537-541`), which `rtos_malloc`s a copy of the payload and posts it to a FreeRTOS queue drained by a dedicated `"Callback"` task **[LIB]** (`cores/nRF5/utility/AdaCallback.c:145-147`). The payload is already copied out of the SoftDevice buffer and the handler already runs off the BLE task. What is true: it does not run on the *loop* task, and this repo owns no queue of its own.
- **R2.** `disconnect_callback` is *also* dispatched through `ada_callback` **[LIB]** (`bluefruit.cpp:849`), as is `connect_callback` (`:829`). Connect/write/disconnect all run on the **same** task in **strict arrival order** — a serialization guarantee the ESP32 flag-deferral lacks. "Inline" is right relative to `loop()`, wrong relative to the BLE stack.
- **R3.** `idleDelay` services *before* it sleeps ([main.cpp:542-548](../src/main.cpp)), so anything added there inherits ≤100 ms service latency.

## 1. Side-by-side responsibility table

| # | Responsibility | ESP32 | nRF52840 | Divergence |
|---|---|---|---|---|
| 1 | `processLedFlash()` | loop, [:352](../src/main.cpp)/[:544](../src/main.cpp) | same | shared already |
| 2 | `epdSessionTick()` | loop, [:353](../src/main.cpp)/[:545](../src/main.cpp) | same | shared already |
| 3 | `buzzerService()` | loop, [:354](../src/main.cpp),[:483](../src/main.cpp),[:516](../src/main.cpp),[:546](../src/main.cpp) | loop, [:529](../src/main.cpp),[:546](../src/main.cpp) | shared already |
| 4 | buttons / touch | loop, [:481-482](../src/main.cpp),[:514-515](../src/main.cpp),[:542-543](../src/main.cpp) | loop, [:527-528](../src/main.cpp),[:542-543](../src/main.cpp) | shared already |
| 5 | BLE command dispatch | **loop task**, ring drain [:406-423](../src/main.cpp); producer on NimBLE host task [esp32_ble_callbacks.h:117-129](../src/esp32_ble_callbacks.h) | **Bluefruit Callback task**, payload copied by `ada_callback` **[LIB]**; registered [ble_init.cpp:157](../src/ble_init.cpp) | **platform** — NimBLE has no `ada_callback`; the ESP32 ring hand-rolls what Bluefruit ships |
| 6 | BLE response TX | queued → `flushResponseQueueToBle()` [:275-313](../src/main.cpp), 16/call | inline `notify()` + bounded retry [communication.cpp:334-355](../src/communication.cpp) | platform-adjacent |
| 7 | Disconnect teardown | flag → `serviceBleDisconnectCleanup()` [:321-348](../src/main.cpp),[:428](../src/main.cpp) | on Callback task [device_control.cpp:227-240](../src/device_control.cpp) | **incidental**, but nRF form is safe today via FIFO ordering (R2) |
| 8 | `updatemsdata()` on connect | flag → loop [esp32_ble_callbacks.h:55](../src/esp32_ble_callbacks.h),[main.cpp:429-432](../src/main.cpp) | **inline on Callback task** [device_control.cpp:219](../src/device_control.cpp) | **incidental — latent nRF defect (§2.1)** |
| 9 | `updatemsdata()` periodic | 60 s, idle branch only [:509-513](../src/main.cpp) | every `sleep_timeout_ms`, only if nonzero [:519-522](../src/main.cpp) | incidental |
| 10 | Advertising restart/interval | flag → `esp32_restart_ble_advertising()` [:433-435](../src/main.cpp),[ble_init.cpp:227-245](../src/ble_init.cpp) | `ble_nrf_advertising_tick()` [:526](../src/main.cpp),[:540](../src/main.cpp),[ble_init.cpp:59-76](../src/ble_init.cpp) | platform |
| 11 | Direct-write 15-min watchdog | loop [:436-442](../src/main.cpp) | **absent** | **incidental — real gap** |
| 12 | `checkPartialWriteTimeout()` | loop [:443](../src/main.cpp) | **absent** | **incidental — real gap** |
| 13 | WiFi/LAN + 10 s supervisor | loop [:444-465](../src/main.cpp) | n/a | platform |
| 14 | `pollActivity()` | loop [:218-265](../src/main.cpp), called [:356](../src/main.cpp) | n/a | platform |
| 15 | Post-wake window + `enterDeepSleep()` | [:360-398](../src/main.cpp),[:486-500](../src/main.cpp) | n/a | platform |
| 16 | `workInFlight` cadence | [:474-517](../src/main.cpp) | `idleDelay(sleep_timeout_ms)`/`(500)` [:519-525](../src/main.cpp) | platform in substance |
| 17 | Ring flush on disconnect | **absent both** | n/a | plan Phase 3 |

**Net:** rows 13–16 plus the drain account for ~130 of the ~160 ESP32 lines. Genuinely incidental divergence (rows 8, 9, 11, 12) totals **about 15 lines**. That ratio is the answer.

### 2.1 Concrete latent defect this exposes

`updatemsdata()` ([display_service.cpp:1734-1820](../src/display_service.cpp)) polls I²C sensors, reads the battery ADC, then on nRF does `clearData`→`addFlags`→`addName`→`addData`→`stop()`→`start(0)` ([display_service.cpp:1767-1783](../src/display_service.cpp)) guarded by an unlocked file-static `prev_msd_payload_nrf[16]`. Called from the **Callback task** ([device_control.cpp:219](../src/device_control.cpp)) *and* the **loop task** ([main.cpp:521](../src/main.cpp)) concurrently, where the Callback task outranks loop **[LIB]** (`AdaCallback.c:147` `TASK_PRIO_NORMAL` vs `cores/nRF5/main.cpp:88` `TASK_PRIO_LOW`). This is exactly the hazard ESP32 avoided ([esp32_ble_callbacks.h:52-55](../src/esp32_ble_callbacks.h)). Two-line fix, independent of every option.

## 3. Can nRF adopt the queue model?

### 3.1 RAM — non-issue, measured

A clean `pio run -e nrf52840custom` was performed (SUCCESS, 3.4 s):
`RAM: 18.0% (42772 / 237568 B)`, `Flash: 31.1% (251972 / 811008 B)`. `size -A`: `.text` 251 056, `.data` 908, `.bss` 41 864, **`.heap` 192 748**. Largest `.bss`: `pipeReorder` 8 316 B (nRF takes the full 33-slot window — `PIPE_SMALL_DRAM_WINDOW` is `esp32-N4`-only, [structs.h:44-56](../src/structs.h)), `chunkedWriteState` 4 116, `configScratch` 4 096, `Bluefruit` 1 932.

Ring cost: `CommandQueueItem` = 260 B padded ([esp32_ble_callbacks.h:25-29](../src/esp32_ble_callbacks.h)) × 33 ([main.h:371](../src/main.h)) = **8 580 B**; `ResponseQueueItem` 260 × 10 = **2 600 B**; total **≈11.2 KB** → `.bss` ~53 KB, heap ~181 KB. **RAM does not constrain this decision.** Reproduce with `~/.platformio/penv/bin/pio run -e nrf52840custom` (bare `pio` is not on `PATH` here).

### 3.2 The framework already does what the ring was invented for

`ada_callback()` **[LIB]** (`AdaCallback.c:106-140`) mallocs a copy and `xQueueSend`s it (100 ms bounded, `CFG_CALLBACK_TIMEOUT`); on queue-full it **doubles the queue depth and retries** (`AdaCallback.c:76-99`). Today: *SoftDevice event → copy → elastic FIFO → dispatch on a NORMAL-priority task in arrival order, with connect/write/disconnect serialized.* Under C: *…→ second copy into a 33-slot ring → second hop → dispatch on a LOW-priority task usually inside `delay()`, with **drop-on-full** ([esp32_ble_callbacks.h:127-129](../src/esp32_ble_callbacks.h)), and disconnect no longer FIFO-ordered against in-flight writes.* Every property that changes, changes for the worse except one.

### 3.3 Flow control

The characteristic is `BLEWrite | BLEWriteWithoutResponse | BLENotify`; for a stack-located value the SoftDevice generates the ATT response itself, so inline processing already gives **no** ATT-level backpressure — deferring loses nothing. *(Assumption; verify on hardware by checking whether client write-with-response latency tracks firmware dispatch time.)* What the current model does give is implicit rate limiting via queue **growth**; a fixed ring converts that into `"Command queue full, dropping command"`. The pipe protocol absorbs drops (SACK zero-bit → retransmit; `docs/pipe-write-protocol.md` §5.2, plan `[H1]`), but usable ring depth is `COMMAND_QUEUE_SIZE-1 = 32` — exactly `PIPE_MAX_W`, zero headroom for the `0x0082` END (the `[H1]` off-by-one; the [main.h:365-370](../src/main.h) comment is wrong). **A queue makes drops more likely on nRF, not less.**

### 3.4 Latency/throughput — the real cost

The nRF steady state is `idleDelay(sleep_timeout_ms)`/`(500)` ([:519-525](../src/main.cpp)), chunking at 100 ms and servicing-then-sleeping ([:538-549](../src/main.cpp)). Draining from `loop()` alone delays each frame by the whole `idleDelay` (up to 65 s expressible). Draining from `idleDelay` bounds it at ~100 ms — a ceiling of roughly **78 KB/s** (32 × 244 B per 100 ms) plus a 100 ms floor on ACK latency the pipe SACK cadence ([display_service.cpp:2854](../src/display_service.cpp)) is not tuned for. Recovering today's throughput means rebuilding `idleDelay` into a real event loop — a change to the one function *both* targets share, risking 11 envs to fix an nRF-only problem.

### 3.5 Response path

nRF `sendResponse` notifies inline with a bounded 4×5 ms retry ([communication.cpp:342-349](../src/communication.cpp)) — ≤20 ms, on backpressure only. A response ring would **hurt** latency (≤100 ms), **help** nothing (ESP32 needs one because its dispatcher *is* the loop task), and raise a new question about `notify()` from two tasks. **Do not add a response ring to nRF under any option.**

### 3.6 What breaks if dispatch leaves the Callback task

Little — which is why C is *feasible* though unwise. Nothing reads the callback args (`(void)conn_hdl; (void)chr;`, [communication.cpp:625-627](../src/communication.cpp)). DFU uses the global accessor `Bluefruit.disconnect(Bluefruit.connHandle())` ([device_control.cpp:844-848](../src/device_control.cpp)), valid from any task; the bootloader jump ([:850-866](../src/device_control.cpp)) is arguably safer off the stack's own task. **But ordering is load-bearing:** disconnect is FIFO-ordered behind queued writes **[LIB]** (`bluefruit.cpp:849`); split them and a disconnect tears down `pipeState`/`partialCtx`/`directWriteActive` ([device_control.cpp:237-239](../src/device_control.cpp)) underneath frames still in the ring. So C must *also* defer disconnect. And the `transferActive()` touch gate ([touch_input.cpp:584-586](../src/touch_input.cpp), `#ifdef TARGET_ESP32`) becomes newly *necessary* on nRF. C is "port the entire ESP32 deferral discipline", not "add a ring".

## 4. Options

**A — full convergence.** ~200 LOC changed across `main.cpp/.h` + stubs in `wifi_service`/`ble_init`/`power_latch`. The ESP32 arm has three early `return`s ([:366](../src/main.cpp),[:389](../src/main.cpp),[:397](../src/main.cpp)) that skip the rest of the pass, and a `workInFlight` branch whose arms differ in cadence *and* in which services they call ([:480-517](../src/main.cpp)). Target-neutral form means either keeping the early returns (so "shared" is fiction) or restructuring deep sleep — the least-testable-without-hardware subsystem. **Risk HIGH. Reject.**

**B — shared supervisory tick (recommended, fused with D).** New `static void serviceSupervisoryTick()` in `main.cpp`: `processLedFlash` + `epdSessionTick` + `buzzerService` + both wall-clock watchdogs (+ Phase 6 supervisor later). Called at the top of `loop()` and per chunk in `idleDelay()`. ~50 LOC net, one file, deletes ~6 duplicated lines. Closes the nRF watchdog gap *structurally*; gives Phases 6/7 **one** wiring point; makes the supervisor fire *during* a long `idleDelay` — which an nRF-arm-only addition cannot, since that arm doesn't run while blocked. **Risk LOW-MEDIUM**, entirely from H4: teardown on the loop task can race the Callback task, so gate the watchdog arm on `g_commandInFlight == 0` and take `pwrmgmLock`. Keep the tick `millis()`-only (it also runs in the ESP32 post-wake window, [:396](../src/main.cpp)).

**C — nRF adopts the queue model.** ~150 LOC across `main.cpp/.h`, `ble_init.cpp`, `device_control.cpp`, `communication.cpp`, `touch_input.cpp`, `structs.h`. Benefit: eliminates every nRF cross-task hazard — `pwrmgmLock` could become an assert, H4 unnecessary, Phase 3 single-task on both targets, §2.1 race gone. Cost: §3.2–3.6, all throughput/timing-sensitive and **unverifiable without hardware**. **Risk HIGH. Reject.**

**D — targeted additions only.** ~15 LOC in `main.cpp` + `device_control.cpp`. Same H4 caveat. Weakness: the additions sit in the nRF arm body, which doesn't run while `idleDelay` blocks (up to 65 s), and reintroduce the copy-paste that produced the gap. **Strictly dominated by B; fuse into B.**

**E — shrink the ESP32 arm instead** (extract `serviceBleQueues`/`serviceWiFiLink`/`serviceSleepPolicy`/`servicePostWakeWindow`, ~120 LOC of pure motion, no behaviour delta). Readability only, and pure motion here makes the freeze-proofing diff unreviewable. **Do it after the plan ships.**

## 5. Interaction with the freeze-proofing plan

The plan already concedes the problem: *"nRF has NO transfer watchdog today — H3 is 'keep' on ESP32 but 'ADD' on nRF … Neither the original plan nor the adversarial review caught this."* B is the cheapest structural answer.

| Plan item | Under B | Under C | Under D |
|---|---|---|---|
| **H4** depth counter | **Still required and becomes a hard prerequisite** (B moves teardown onto the nRF loop task) | Would become unnecessary — only after C's full deferral discipline, which is the risky part | Same as B |
| **Phase 3** `abortToKnownState` cross-task safety | Simpler: one call site, one rule ("only when depth == 0") | Simplest in principle, highest cost to reach | Two call sites; per-target reasoning persists |
| **Phase 5 H4** deferred nRF session clear | Unchanged; flag serviced from the tick, which now runs in both arms | Folds into the general deferral | Unchanged |
| **Phase 6** supervisor | **Clearly simpler** — the "must be wired into the nRF path" requirement is satisfied structurally, and coverage extends into `idleDelay` | Simpler still, at C's cost | Two bodies to keep in sync — the exact failure mode that produced the gap |
| **Phase 6 `[X5]`** | Unaffected | Unaffected | Unaffected |
| **Phase 7** BLE idle timeout | Slightly simpler (check belongs in the tick). Note the `[C1]` stamp stays in `imageDataWritten`, i.e. on the Callback task on nRF → must be `volatile uint32_t`, comparison must tolerate a concurrent write | Simpler (single task) | Two arms again |
| **Phases 1/2** | Orthogonal | Orthogonal | Orthogonal |

**Sequencing:** (1) Phases 1 and 2 unchanged — highest value-per-risk. (2) **H4 on its own**, before anything drives teardown from the nRF loop task. (3) **Option B here**, as a pure refactor + two watchdog additions, in its own commit. (4) Phases 3/5/6/7 wire into `serviceSupervisoryTick()` instead of into two arms. Do **not** land B before H4, and do **not** fold B into a phase commit.

## 6. Risk

| Risk | Under B | Under C |
|---|---|---|
| Transfer throughput | Nil (dispatch untouched) | **High** — ~78 KB/s ceiling unless `idleDelay` is rebuilt |
| Command latency | Nil | ≤100 ms/frame + ≤100 ms/ACK |
| Teardown racing a live transfer | **Main risk** — mitigated by H4 gate + `pwrmgmLock` | Eliminated |
| Deep sleep | nRF has none; ESP32 exposure via `idleDelay` ([:396](../src/main.cpp)) — keep tick `millis()`-only | Same ESP32 exposure |
| DFU | Untouched | Bootloader jump moves to loop task — plausible, unproven |
| Power draw | Negligible | Unknown; more task switches |

**Test coverage today: none.** `find . -name "*test*"` outside `.pio` yields only `tools/test_zlib_stream.c` (host-side zlib harness). CI (`.github/workflows/main.yaml:12-44`) is **build-only** across 11 envs. Green CI proves they link, nothing more.

**Mandatory hardware validation for B:** (a) nRF stalled direct write → watchdog fires, panel rail drops; (b) same for a stalled `0x76` partial; (c) full Spectra push with 60 s refresh → tick does *not* tear down mid-refresh (`epdRefreshInProgress`/`pwrmgmLock` gating); (d) ESP32 battery unit → post-wake window and idle-hold timings unchanged. **For C additionally:** measured throughput before/after, sniffer trace for dropped `0x0081` at W=32, heap-pressure soak exercising the inline-fallback path, DFU-from-loop-task.

**Cannot be validated without hardware:** all of the above, plus SoftDevice ATT auto-response behaviour, inline-fallback frequency under heap pressure, `notify()` from the loop task, and every panel-rail timing interaction.

## 7. Historical evidence

Both deliberate and drift, in sequence — **no evidence of a reverted convergence attempt**.

The original nRF-only firmware already used *deferral*: `git show df6d088:src/main.cpp` shows `loop()` draining a `currentImage.ready` flag set by the BLE callback. The split was born with ESP32 support in `955f2d0` *("Add ESP32-C6 and ESP32-C3 support")*, whose first `#ifdef TARGET_ESP32` in `loop()` states the rationale outright: `// Process queued commands outside of callback context`. That is a platform statement — Bluedroid/NimBLE has no `ada_callback` equivalent; nRF didn't need a ring because Bluefruit already provides one.

**Everything since is drift.** `git log -L 518,530:src/main.cpp` shows the nRF arm changed exactly **three** times ever: `7ffea91` (added `ble_nrf_advertising_tick()`), `aad0c6d` *("Drop per-iteration 'Loop end' debug log on nRF")*, `5ed21a9` *("feat: quarter-tone musical buzzer…")* (added `buzzerService()`). Over the same period the ESP32 arm absorbed deep sleep (`ee98b65`, `d974f9d`, `c377dec`), the watchdogs (`abcf95d` *"Add stuck-state watchdog for abandoned partial writes (panel rail)"*), WiFi/LAN (`2e2131b`), the NimBLE migration (`d4da951`) and the pipe drain (`a628d84`). **The nRF arm did not diverge; it stood still while the ESP32 arm grew** — which is why the missing watchdog is a gap, not a decision.

One near-miss: the unmerged branch `feat/esp32-freertos-command-queue` (single commit `43b0799`, never merged): *"Swap the volatile-index SPSC command ring for a static FreeRTOS queue (`xQueueCreateStatic`), fixing cross-core memory-ordering races and the silent drop-on-overflow. … **ESP32-only; the processing pipeline (`imageDataWritten`) and the nRF/WiFi paths are unchanged.**"* The last time anyone touched this machinery the scope was drawn deliberately at the ESP32 boundary — corroboration for keeping the execution models separate.

## 8. What would have to be true

| # | Assumption | Check | If false |
|---|---|---|---|
| 1 | Bluefruit dispatches writes on a separate FIFO task with an elastic queue | `AdaCallback.c:76-99, 145-147` **[LIB]** (done); on hardware log `uxQueueMessagesWaiting`/free heap during a full-window push | C's cost/benefit flips; re-evaluate C |
| 2 | Callback task outranks loop (`NORMAL` vs `LOW`) | `AdaCallback.c:147`, `cores/nRF5/main.cpp:88` **[LIB]** (done); corroborated by [display_service.cpp:401-407](../src/display_service.cpp) | `pwrmgmLockTake`'s inversion mitigation is over-engineered but harmless |
| 3 | ~11 KB extra `.bss` affordable on nRF | Measured: 42 772/237 568 B, 192 748 B heap | Only matters for C |
| 4 | Watchdogs safe from the nRF loop task once `g_commandInFlight == 0` | Inspection says yes; **must be confirmed on hardware** with a stalled transfer + concurrent traffic | B degrades to "check and log only"; Phase 6 supervisor owns teardown |
| 5 | `millis()`-only additions to `idleDelay` don't perturb ESP32 deep sleep | Battery soak, compare wake-window/idle-hold before/after | Call the tick from `loop()` only (loses long-block coverage) |
| 6 | SoftDevice, not the app callback, generates the ATT write response | Hardware: client write-with-response latency vs firmware dispatch time | Further argument against C; no impact on B |
| 7 | No client depends on nRF's sub-ms dispatch latency | py-opendisplay timeouts; full-panel timing run | Only matters for C |

## 9. Do not do this

1. **Do not move nRF dispatch to the loop task without also deferring `disconnect_callback`.** They are FIFO-ordered today **[LIB]** (`bluefruit.cpp:849`); split them and a disconnect tears down `pipeState`/`partialCtx`/`directWriteActive` ([device_control.cpp:237-239](../src/device_control.cpp)) underneath queued frames — use-after-teardown on a live transfer, worse than any freeze this work targets.
2. **Do not add a response ring to nRF.** Buys nothing (§3.5), adds ≤100 ms to every pipe ACK, throttling the window.
3. **Do not copy `COMMAND_QUEUE_SIZE 33` to nRF as-is.** Usable depth is 32 ([esp32_ble_callbacks.h:122](../src/esp32_ble_callbacks.h)) = `PIPE_MAX_W`, no slot for the `0x0082` END — importing the `[H1]` off-by-one onto a target that lacks it.
4. **Do not "simplify" `idleDelay` into a plain `delay()`.** It is the only servicing of buttons/touch/LED/keep-alive/buzzer during long waits on both targets ([:538-548](../src/main.cpp)) and keeps the ESP32 post-wake window responsive ([:391-396](../src/main.cpp)).
5. **Do not put I²C, SPI or advertising work in the shared tick.** It runs every 100 ms inside `idleDelay` on both targets, including the ESP32 deep-sleep advertising window. `millis()` comparisons and flag checks only.
6. **Do not add the nRF watchdogs before H4.** `cleanupDirectWriteState(true)` from the loop task while the Callback task is inside `handlePipeWriteData` is a new freeze mechanism dressed as a fix.
7. **Do not un-`#ifdef` the `transferActive()` touch gate "for symmetry" under B** ([touch_input.cpp:584-586](../src/touch_input.cpp)). Under B the transfer still runs on the Callback task, so the gate isn't needed — enabling it would kill touch on nRF for every transfer with no benefit.
8. **Do not restructure `loop()` and change behaviour in one commit.** Zero automated tests + build-only CI = unreviewable, unbisectable.
9. **Do not treat green CI as validation.** It links; it does not run.

## 10. Uncertainty

Timing, throughput and power figures are inference from source plus arithmetic — no nRF hardware was exercised; the ~78 KB/s is a computed ceiling, not a measurement. RAM numbers **are** measured but reflect section partitioning only; FreeRTOS stacks and Bluefruit's dynamic allocations come out of the 192.7 KB heap and were not runtime-profiled. Bluefruit internals were inspected at exactly three points (write dispatch, callback queue, connect/disconnect dispatch); the **frequency** of the inline-fallback path that `[H4]` depends on is unknowable from source. I did not evaluate whether Bluefruit's queue growth is bounded in practice — it doubles from the heap, trading a dropped frame for heap pressure, a different failure mode deserving its own investigation if nRF ever shows heap exhaustion during large pushes. Finally, "nRF is safe today" rests on the Callback task's FIFO ordering; §2.1 documents one place that ordering does not protect. There may be others — the BLE-adjacent call graph was checked, not every shared global.
