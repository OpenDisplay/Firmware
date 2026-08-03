# Adversarial review — `PLAN_PHASE3_SESSION_GUARD_2026-07-26.md`

Reviewed 2026-07-26 against `debug/ble-hardening` HEAD `ee43e19`. Every file:line below was read.

## Verdict

**Not safe to implement as written.** Research quality is high — citations are almost all accurate, the ESP32 single-task claim survives exhaustive checking, and D7's hazard analysis is largely sound. But six defects are load-bearing enough that following the plan literally produces a non-compiling tree or a teardown that violates the invariant D7 exists to protect.

---

## Critical

### `[C1]` `main.h` is a definitions header — every "declared in `main.h`" instruction is unbuildable  ✅ **FIXED 2026-07-26** — plan §3a added (header-placement table); every call site corrected

**Plan:** §4.1 "declared in `main.h`"; §7 "`session_guard.cpp` calls them through the `main.h` declarations" and "nRF stubs in `main.h`"; §8.4; §9.2 `nrfDisconnectCleanupPending`; D6c `linkIsUp()`.

**Source:** `src/main.h` has no include guard and **defines** globals — `main.h:91` `BBEPDISP bbep;`, `:165` `bool directWriteActive = false;`, `:283` `chunked_write_state_t chunkedWriteState = {...};`, `:284` `globalConfig`, `:289` `EncryptionSession encryptionSession`, `:374-391` `responseQueue`/`commandQueue`/`pServer`/callback objects.

```
$ grep -rn '#include "main.h"' src/
src/main.cpp:1:#include "main.h"
```

**Why wrong:** `communication.cpp`, `device_control.cpp` and a new `session_guard.cpp` cannot include it — second definition of every global ("multiple definition of `bbep`"), and on nRF it drags in `<bluefruit.h>` plus NimBLE-aliased `BLE*` types the plan itself forbids in shared headers. Concretely: `communication.cpp` does not include `main.h` (it declares `extern chunked_write_state_t chunkedWriteState;` itself at `communication.cpp:83`), so §4.1's `resetChunkedWriteState()` would be declared where neither its own TU nor its caller can see it. §7's `static inline` nRF stubs are invisible to `session_guard.cpp`. §9.2's flag needs to be visible to both `device_control.cpp` (writer) and `main.cpp` (reader).

**Correction:** `resetChunkedWriteState()` → `communication.h`. `flushCommandQueue`/`flushResponseQueue`/`linkIsUp`/`serviceLinkDrop`/`serviceDeferredPanelOff`/`nrfDisconnectCleanupPending` → `session_guard.h`, with ESP32 and nRF bodies both out-of-line in `main.cpp`. `session_guard.cpp` may include `structs.h`, `display_service.h`, `communication.h`, `encryption.h`, `od_log.h` — never `main.h`.

### `[C2]` Step 4 cuts the panel rail before step 6 can defer it — the whole `epdStreamInProgress` design is bypassed

**Plan:** §8.3 step 4 runs `cleanupDirectWriteState(true)` + `cleanupPartialWriteOnDisconnect()`; step 6 then declares *"HARD INVARIANT: … an in-flight controller stream is never cut mid-write (D7 hazard 1)"* and defers behind `!epdRefreshInProgress && !epdStreamInProgress`.

**Source:**
```c
// display_service.cpp:2028-2031 (cleanupDirectWriteState)
    if (pwrmgmState == PWR_ACTIVE) {
        if (refreshDisplay) epdSessionForceOff();
        else                epdSessionRelease(true);
    }
```
```c
// display_service.cpp, cleanup_partial_write_state()
    bool teardown = partialCtx.active && pwrmgmState == PWR_ACTIVE;
    memset(&partialCtx, 0, sizeof(partialCtx));
    if (teardown) epdSessionForceOff();
```
`epdSessionForceOff()` (`:512-516`) → `epdSessionForceOffLocked()` (`:418-434`) → `bbepSleep` + `delay(50)` + `pwrmgm(false)` → `SPI.end()` + rail LOW (`main.cpp:759-770`).

**Why wrong:** During any wedged transfer the panel is `PWR_ACTIVE` by construction (`epdSessionAcquire` sets it at `:443`/`:467`). Step 4 therefore *always* cuts the rail mid-stream, several statements before step 6 evaluates `epdStreamInProgress`. Step 6 then finds `PWR_OFF`, `epdSessionForceOffLocked` early-returns at `:419`, and `epdForceOffPending` is never needed. D7's "close it structurally, at the panel" accommodation protects a path that never runs — including the exact nRF double-handler case it was written for.

**Correction:** Move the deferral *below* `epdSessionForceOff()`: either (a) guard inside it — `if (epdRefreshInProgress || epdStreamInProgress) { epdForceOffPending = true; return; }` — one place, covers `cleanupDirectWriteState`, `cleanup_partial_write_state`, `sendPipeNack`, `epdSessionTick`; or (b) give the two cleanup functions a bookkeeping-only mode and leave all panel power to step 6. Either way §8.3's printed ordering is wrong.

### `[C3]` "Two choke points cover ALL controller writes" is false

**Plan:** D7 — set/clear around `pipeConsumePayload()` and "the legacy 0x0071 data handler", *"so one flag covers pipe, partial, compressed, legacy, FastEPD and E1004."*

**Enumerated uncovered controller writes reachable from a command handler:**

| Path | Site | Reached from |
|---|---|---|
| `partial_prepare_panel_ram()` → `bbepFill(&bbep, BBEP_WHITE, PLANE_1/PLANE_0)` — two **whole-plane** SPI writes | `display_service.cpp:3271-3272` | `handlePartialWriteStart` (0x0076) `:2252`; `handlePipeWriteStart` partial arm `:2792` |
| `partial_consume_bytes()` on the **inline initial payload** of a 0x0076 START | `:2258` | `handlePartialWriteStart` |
| `directWriteActivatePanel()` → `bbepSetAddrWindow` + `bbepStartWrite`; `e1004_begin_plane()` issues `bbepWriteData` (`:314-337`) | `:2092-2100` | `handleDirectWriteStart` (0x0070), `handlePipeWriteStart` full arm `:2807` |
| `zlib_stream_to_partial_write(nullptr,0,true)` final flush writes residual bytes | `:2339` | `handleDirectWriteEnd` (0x0072/0x0082) |
| `partial_write_stream_bytes()` → `partial_set_addr_window` + `bbepStartWrite` + `bbepWriteData` | `:3215-3223` | reachable from `:2258`, outside both choke points |

`bbepFill` on a 1200×1600 panel is hundreds of KB of SPI. This is not a corner case — it is the entire 0x0076 family and the `PIPE_FLAG_PARTIAL` bring-up.

**Correction:** Stop chasing call sites. One opcode-keyed pair in `imageDataWritten`'s switch covering `CMD_DIRECT_WRITE_{START,DATA,END}`, `CMD_PARTIAL_WRITE_START`, `CMD_PIPE_WRITE_{START,DATA,END}`. Provably exhaustive, no early-return exposure — which also answers the plan's unaddressed question about `pipeConsumePayload`'s `return true` at `:2601` and three `return false` sites (`:2607`, `:2612`) that a naive wrapper would leak the flag on.

### `[C4]` The teardown powers down a WARM panel on every disconnect — D1b is not a "strict superset"

**Plan:** §1/D1 *"a strict superset of what those sites do today"*; §9.1 *"All are either no-ops or strictly correct on a link that just went away."*

**Source — the invariant is documented twice:**
```c
// main.cpp:339-342
// ACTIVE-only-teardown invariant: a WARM (post-successful-refresh) panel
// SURVIVES disconnect and keeps its keep-alive window …
```
```c
// device_control.cpp:232-236
// … a reconnect within the window pays only a warm re-acquire.
```
`epdSessionForceOffLocked` early-returns **only** on `PWR_OFF` (`:419`); on `PWR_WARM` it runs the full teardown.

**Why wrong:** Both existing teardowns achieve ACTIVE-only semantics precisely *by never calling `epdSessionForceOff()` directly*. `abortToKnownState` step 6 calls it unconditionally. So the healthy path — push image, refresh succeeds (→ `PWR_WARM` with armed deadline, `:504-505`), disconnect — now powers the panel down, and the next push pays the ~900 ms cold rail bring-up (`main.cpp:721`) plus `bbepInitIO`/`bbepWakeUp`/init-sequence instead of a warm re-acquire. Fires on **every** disconnect.

**Correction:**
```c
    if (pwrmgmState == PWR_ACTIVE) {              // ACTIVE-only-teardown invariant
        if (!epdRefreshInProgress && !epdStreamInProgress) epdSessionForceOff();
        else epdForceOffPending = true;
    }
```
Add a §12 case: connect, push, disconnect, reconnect inside `screen_timeout_seconds` → log must show `acquire: WARM re-acquire`, not `COLD bring-up`.

### `[C5]` The §8.3 log line does not compile

`display_service.cpp:556` `static PartialStreamContext partialCtx = {};` and `:575` `static PipeWriteState pipeState = {};` are file-static; `display_service.h` exposes only `transferActive()` (`:70`), `epdRefreshInProgress` (`:77`) and the three cleanup entry points. The step-1 log line reads `pipeState.active` and `partialCtx.active`.

**Correction:** add `bool pipeWriteActive(void)` / `bool partialWriteActive(void)` next to `transferActive()`. (`directWriteActive` and `chunkedWriteState` are fine via `extern`, as `communication.cpp:83` / `esp32_ble_callbacks.h:43` already do.)

### `[C6]` `commandDrainAbortPending` latches outside a drain → one command dispatched twice  ✅ **FIXED 2026-07-26** — plan §5 now clears the flag at the top of the drain block; regression test added to §12

**Plan:** §7 sets the flag unconditionally in `flushCommandQueue()`; §5 consumes it with `flag=false; break;` without the tail store.

**Source:** D1b calls `abortToKnownState` from `serviceBleDisconnectCleanup()`, which runs at `main.cpp:370` (deep-sleep-wake branch, **before** the drain, then `return`s) and `:428` (**after** the drain). The plan itself says so in §12.

**Why wrong:** With a non-empty ring at that moment, the flag is set and nothing consumes it this pass. Next pass the drain (1) dispatches `commandQueue[tail]`, (2) sees the flag, clears it, `break`s **without storing the tail**, (3) next pass dispatches the same slot again. For `CMD_CONFIG_WRITE`, `CMD_POWER_OFF`, `CMD_DEEP_SLEEP`, `CMD_REBOOT` that is not benign. A *new* bug shipped in the same commit as the `[M5]` fix.

**Correction:** `commandDrainAbortPending = false;` at the top of the drain block before the `while`, or scope the flag to an active drain via a `commandDrainActive` flag. Secondary: when called from inside the drain, `dropped` over-counts by one (tail not yet advanced) — cosmetic, worth a comment.

---

## High

**`[H1]` The deferred scrub can zero a *fresh* session's nonce.** `handleAuthenticate` is dispatched **before** the auth gate (`communication.cpp:649-652`; gate at `:663-669`), and step 1 writes exactly what the scrub erases (`encryption.cpp:586-587`: `secure_random(pending_server_nonce,16)`). On nRF: handler A clears at depth≥1 → `sessionScrubPending`; A returns; a new `CMD_AUTHENTICATE` step 1 arrives on the callback task; `loop()` samples depth 0 and memsets `pending_server_nonce` → step 2's CMAC fails → `AUTH_STATUS_ERROR` on reconnect. D7's *"`isAuthenticated()` is already false, so no new command can use the key"* does not close this, because authenticate bypasses that gate by design. **Fix:** generation counter, or `if (sessionScrubPending && g_commandInFlight==0 && !authenticated && server_nonce_time==0)`. Also state the answer D7 only asks: `ccm_session_free` is never deferred (ESP32-only, `encryption.cpp:202`, and `sessionScrubIsSafeNow()` returns `true` there).

**`[H2]` `touchForceResumeAll()` skips the real resume work.** `touchResumeAfterEpdRefresh()` (`touch_input.cpp:417-432+`) is not just a decrement — at zero it runs `invalidateOpenDisplayWire()`, `delay(GT911_POST_RESET_SETTLE_MS)`, and per-controller re-init + INT re-attach. Zeroing the counter re-enables polling (`:584`) against a controller never re-initialised after `pwrmgm(false)`'s `Wire.end()` (`main.cpp:764-766`). §12's criterion *"touch responds again (`s_epd_refresh_suspend == 0`)"* checks the very proxy that will read zero while touch is dead. **Fix:** `if (s_epd_refresh_suspend) { s_epd_refresh_suspend = 1; touchResumeAfterEpdRefresh(); }` — note this makes the helper blocking, which must be stated; and test a real touch event.

**`[H3]` nRF's `loop()` cannot service the new deferrals promptly.** The nRF arm (`main.cpp:518-530`) opens with `idleDelay(globalConfig.power_option.sleep_timeout_ms)`, which blocks for the full duration in 100 ms chunks servicing only buttons/touch/LED/`epdSessionTick`/buzzer (`:535-551`). `serviceBleDisconnectCleanup()` is inside `#ifdef TARGET_ESP32` (`:191`–`:349`) and called only at `:370`/`:428` — there is no nRF analogue to sit "next to". Teardown latency regresses from immediate to `sleep_timeout_ms`, with the rail up mid-transfer, touch suspended, and key material + `epdForceOffPending` unserviced. **Fix:** put the service calls in `loop()`'s shared prologue (`:352-354`) **and** in `idleDelay()`'s body — the precedent `epdSessionTick()` already sets at `:545`.

**`[H4]` The `g_commandInFlight == 0` caller check is a sampled TOCTOU.** §9.2 claims it *"guarantees no `imageDataWritten` is on the stack"*. On nRF the counter is written by the Bluefruit Callback task and read by `loop()` with no mutual exclusion; a write callback firing one instruction later re-enters. D7 says the right thing three sections later (*"defence in depth rather than the only defence"*) — §9.2 contradicts it, and that sentence is what justifies deleting Phase 5's `nrfSessionClearPending`. **Fix:** reword; let Phase 5 re-decide.

**`[H5]` Phase 2 assigns `panelStateUnknown` to Phase 3; Phase 3 never wires it.** Phase 2 states: *"Consumed: → Phase 3. `abortToKnownState()` reports it and skips `epdSessionForceOff()` when set (retrying a lock that just timed out costs another 60 s inside the abort path)."* Phase 3's §8.3 log line omits it and step 6 has no test. After Phase 2, `epdSessionForceOff()` (`:513`) takes a lock with a 60 s deadline — so the recovery path can block `loop()` for 60 s on every disconnect. Also: Phase 2's **D-A and D-B are listed as blocking and unresolved**, and D-B's alternative is literally *"defer the whole flag to Phase 3"*. **Fix:** wire it into the log + step 6; settle Phase 2 D-A/D-B first.

**`[H6]` A deferred abort can fire after a reconnect.** nRF re-advertises autonomously (`ble_nrf_advertising_tick`, `main.cpp:526` and inside `idleDelay` at `:540`); on ESP32 the deep-sleep-wake branch does cleanup at `:370` then restart at `:371-373` then `return`s. §9.1 waves `esp32-N4` through on the grounds that no LAN path raises the flag spuriously — correct, but it does not address a *genuine* disconnect serviced after a reconnect, and `esp32-N4` has no `ownerStillUp` guard at all (`main.cpp:328-338` is inside `#ifdef OPENDISPLAY_HAS_WIFI`). D1b widens the blast radius to session clear, chunked state, buzzer, LED and panel. **Fix:** connection generation, or an `!linkIsUp()` test at the service point (reuses D6c's new predicate).

**`[H7]` Stopping buzzer/LED on every disconnect is a client-observable change.** "Beep/flash and disconnect" is the natural pattern for 0x0077/0x0073 — neither has a completion notification, and the parent plan records LED flash as deliberately unbounded and accepted. §12's own criterion says *"any behavioural difference visible to the client is a constraint violation, not a Phase 3 feature."* §2 lists this change in neither the in-bounds nor out-of-bounds table. **Fix:** restrict to `dropLink=true`, or get an explicit decision recorded in §2.

---

## Medium

**`[M1]`** "`cleanupDirectWriteState` no-ops when `!directWriteActive`" (§8.3 Idempotence) is false — `display_service.cpp:2010-2039` has no such guard; it zeroes eleven globals, force-offs on `PWR_ACTIVE`, calls `e1004_end_plane()`, resumes touch. That is why the ESP32 site guards the *call* (`main.cpp:343`) and nRF does not (`device_control.cpp:237`). The claim is what `[C2]` and `[C4]` both depend on being false. Re-derive step 4's ordering rationale: for a partial transfer the *first* call is what powers the panel down, and `cleanup_partial_write_state`'s own predicate then evaluates false — same end state, opposite reason to the one given.

**`[M2]`** §8.3's "1. LOG FIRST" contradicts the D7 guard snippet showing the atomic test-and-set first. Guard must be first. Separately, `__atomic_store_n(&inAbort,0,RELEASE)` appears once at the end; `[H5]` adds an early return and Phase 2 makes panel calls failable — one skipped release latches `inAbort=1` and **permanently disables recovery**. Mandate single-exit `goto done` or RAII.

**`[M3]`** §8.1's header surface omits 11 symbols used elsewhere in the plan: `odLinkDropRequest`, `g_linkDropPending`, `linkReleaseIfHeld`, `sessionScrubPending`, `sessionScrubIsSafeNow`/`sessionScrubNow`, `nrfDisconnectCleanupPending`, `serviceDeferredPanelOff`, `serviceLinkDrop`, `linkIsUp`, `epdStreamInProgress`, `panelStateUnknown`.

**`[M4]`** Phase 1 deletes `replay_window[64]` for a bitmap (net −480 B) and rewrites `clearEncryptionSession()`; Phase 3's D7 split reproduces the pre-Phase-1 field list verbatim → guaranteed conflict in the phase's most safety-critical function. §10's *"gate for Phase 1's `replay_window[256]`"* is stale. Rebase the split; state which half the bitmap reset lands in.

**`[M5]`** Nothing resets `sessionOrigin` (`display_service.cpp:2114`, set only at `:2129`/`:2170`/`:2638`). After aborting a LAN transfer it stays LAN, so the ESP32 guard's own input (`main.cpp:329`: `transferSessionOrigin() != 0`) is stale — a later BLE-only disconnect is classified LAN-owned and, with a LAN client up, the **BLE teardown is silently skipped**. Squarely in the "known state" remit.

---

## Low

- `[L1]` `pending` removal saves 2 B/slot (alignment: 260→258), so −86 B not −43 B; direction holds. §10 omits `nrfDisconnectCleanupPending` and counts `epdStreamInProgress` against `session_guard` when D7 puts it in `display_service.cpp`.
- `[L2]` `flushResponseQueue()` (discards) vs existing `flushResponseQueueToBle()` (`main.cpp:275`, *sends*) — one keystroke apart, opposite effects, same file. Rename.
- `[L3]` §12's `grep -n 'pending' src/` always matches `bleDisconnectCleanupPending`, `msdUpdatePending`, `bleRestartAdvertisingPending`, `buttonEventPending` and every new Phase 3 flag. Use `grep -rn '\.pending' src/`.
- `[L4]` §12's `grep -nE '^\+.*(RESP_|CMD_)[A-Z_]+ *='` looks for assignments; opcodes are `#define`s in the vendored header. The two `git diff --stat` header checks are the real guard and are correct.
- `[L5]` §9.1 cites `main.cpp:447` for the WiFi-lost tick calling `disconnectWiFiServer()` — `:447` is `handleWiFiServer()`; the call is at `:456` (and `wifi_service.cpp:873`).
- `[L6]` `od_log_*` carries `format(printf,2,3)` (`od_log.h:30`); codebase convention is explicit `(unsigned)` casts (`main.cpp:439`, `:497`). §7's `%u` with `uint8_t` should match.
- `[L7]` §8.3.1's "next to `serviceBleDisconnectCleanup()`" is an orphan — that function is ESP32-only. See `[H3]`.
- `[L8]` §4.1's note is right (`communication.cpp:574-576` clears 3 of 5 fields) but understated: `:550` and `:558` clear only `active`, so the consolidation *changes their behaviour*. That is a behaviour change, not a pure refactor.

---

## Gaps missed entirely

- `[X1]` `abortToKnownState()` returns `void` and cannot report failure. After Phase 2 every panel op is failable and the lock can time out; §12's *"panel rail down"* / *"all state flags false"* criteria assume success. Give it a `bool` or status flag for Phase 6.
- `[X2]` No zlib streamer reset. `od_zlib_stream_reset` runs only at START (`:2102`, `:2253`, `:2793`); after an abort the inflater keeps its state. Benign today (START always resets) but a "known state" gap.
- `[X3]` Partial transfers never suspend touch — `directWriteTouchSuspended` is set only on the full-frame paths (`:2136-2137`, `:2800-2801`), and the partial bring-up at `:2785-2794` deliberately skips it. `[M3]`'s analysis reasons only about full-frame; say this explicitly, because `[H2]`'s fix makes the helper do real I2C work.
- `[X4]` `epdPlanesPrepared`/`epdSessionInitWasPartial` (`:365`, `:369`) are not reset; `epdSessionForceOffLocked` clears the former at `:433`, so the deferred-force-off path leaves it stale until `serviceDeferredPanelOff()` completes.
- `[X5]` §2's flush justification via SACK retransmit (`pipe-write-protocol.md` §5.2 — verified, `:373-380`) applies only to `0x0081` DATA. A flushed `0x0082` END, `0x0041`, or `0x0050` has no retransmit path. Same shape as today's ring-full drop, so the constraint argument survives — but "no new behaviour" needs the qualifier now that the flush is deliberate.

---

## Verified correct — do not re-check

1. **ESP32 has no cross-task command-handler execution.** Exhaustive: `imageDataWritten` is reached from exactly three places — `main.cpp:415` (loop drain), `wifi_service.cpp:978` (LAN dispatch inside `handleWiFiServer()`, called from `loop()` at `main.cpp:447`), and `ble_init.cpp:157` (**nRF/Bluefruit only**). `onWrite` (`esp32_ble_callbacks.h:81-135`) only memcpys and stores the head; `onConnect`/`onDisconnect` (`:46-70`) are flag-only. No `xTaskCreate`/`esp_timer_create`/`xTimerCreate` in `src/` on ESP32 (`ble_init.cpp:104`'s `TimerHandle_t` is nRF). Both WiFi event handlers (`wifi_service.cpp:574`, `:643`) only set flags — the file carries an explicit "EVENT-CONTEXT RULE" comment at `:625` and everything is drained by `serviceWifiEventFollowUp()`. Button/touch ISRs (`device_control.cpp:679`, `:697`; `touch_input.cpp:175`) set bitmasks only. **D4's "document, no assert" and D7's "ESP32 callers need no depth check" are both correct.**
2. **The `[M5]` drain race is real and the fix placement is right.** `main.cpp:409-421`: `tail` cached at `:409`, dispatch at `:415`, `__atomic_store_n(...tail+1..., RELEASE)` at `:417` would overwrite a `tail := head` snapshot. Between `:415` and `:416`, breaking without the store, is correct. (`[C6]` is about the flag's lifetime, not the placement.)
3. **`pending` has no readers** — assignments only, at exactly the five cited sites: `esp32_ble_callbacks.h:125`, `communication.cpp:119`, `main.cpp:291`, `:309`, `:416`.
4. **`COMMAND_QUEUE_SIZE 33` is defined twice** — `main.h:371` and `esp32_ble_callbacks.h:19` (`#ifndef`); `main.h` defines at `:371` and includes the callbacks header at `:378`, so it wins. D5's Phase 7 warning is valid.
5. **The `main.h:365-370` capacity comment is wrong as `[H1]` says** — producer refuses at `nextHead == tail` (`esp32_ble_callbacks.h:121-122`), usable capacity 32. D5's replacement text is accurate.
6. **`EncryptionSession` has no origin field** (`encryption_state.h:11-30`). `g_commandOrigin` is per-dispatch (`communication.cpp:37`, set/restored at `wifi_service.cpp:976-979`); `sessionOrigin` tracks the transfer (`display_service.cpp:2114`). D6c's "genuinely unanswerable in Phase 3" is correct, and erring toward *not* clearing is the right direction.
7. **`wifiLanClientConnected()` behaves as claimed** — `wifi_service.cpp:355`.
8. **`isAuthenticated()` precedes `decryptCommand` on every decrypting path** — `communication.cpp:664` gates before `:698`. LAN-TLS bypasses the CCM envelope by design and never touches `session_key`, so clearing the session does not stop LAN-TLS execution — correct and pre-existing, worth one sentence in D6c.
9. **The invalidate/scrub field split is accurate** — `encryption.cpp:201-219`: racy half is `ccm_session_free` (`:202`) + five memsets (`:204-207`, `:217`); the eight scalar stores (`:208-216`) are coherent for a concurrent reader. The *premise* holds; `[H1]` is a defect in the *consequence*.
10. **`esp32-N4` is ESP32 without WiFi and without any LAN flag-setter** — `platformio.ini:284` defines only `TARGET_ESP32` + `PIPE_SMALL_DRAM_WINDOW`; `wifi_service.cpp:3` wraps the whole file in `#ifdef OPENDISPLAY_HAS_WIFI`.
11. **No self-deadlock on `pwrmgmLock`, no use-after-free in the teardown.** All four take/give pairs are function-local (`:439`/`:488`, `:496`/`:509`, `:513`/`:515`, `:520`/`:526`); the lock is never held across a return into handler code. `pipeState`, `pipeReorder`, `partialCtx`, `chunkedWriteState` are all static storage. D7's strongest ungating argument stands.
12. **All other spot-checked citations are accurate**: `display_service.cpp:2502-2504` (`transferActive`), `:2008`/`:2035-2038`, `communication.cpp:113`, `esp32_ble_callbacks.h:128`, `buzzer_control.cpp:147`, `device_control.cpp:341`, `structs.h:86`, `main.h:371`, `platformio.ini:284`. (`touch_input.cpp` counter is at `:64`; the plan cites `:115-119`, which is `touchSuspendForEpdRefresh` — close enough to be useful.)
13. **Wire-protocol constraint: Phase 3 as scoped does not violate it.** No canonical-header edits; D3 removes the only genuine candidate; both ring structs are firmware-local. `[H7]` is the one *client-observable* change and it is a behaviour question, not a header question.
14. **`pipe-write-protocol.md` §5.2 exists and says what §2 cites it for** (`:373-380`).

---

## Phase interaction risks

| Phase | Risk | Action |
|---|---|---|
| **1** | Phase 1 deletes `replay_window[64]` → bitmap and rewrites `clearEncryptionSession()`; Phase 3's D7 split targets the pre-Phase-1 field list. §10's "+1.5 KB `replay_window[256]`" is stale (Phase 1 is now −480 B). | `[M4]` — rebase before implementing. |
| **1** | D6c's gap argument (stale session bounded by out-of-window rejection + Phase 1's `counter_diff == 0` fix) is sound — verified at `encryption.cpp:136`. | Order Phase 1 → Phase 3 confirmed. |
| **2** | `pwrmgmLockTake` → bool + 60 s deadline makes `epdSessionForceOff()` failable **and blocking**; Phase 3 calls it from the loop task on every disconnect and ignores both. Phase 2 explicitly assigns `panelStateUnknown` to Phase 3; Phase 3 never wires it. | `[H5]`, `[X1]`. |
| **2** | Phase 2's **D-A** and **D-B** are blocking and unresolved; D-B's alternative is "defer the flag to Phase 3". | Settle both before Phase 3 starts; record in §1. |
| **2** | `[X2]` (boot-refresh `epdRefreshInProgress`) and `[X3]` (real `fastepd_wait_refresh`) are prerequisites for §8.3.1's "no timeout on this deferral". Boot suspends/resumes touch symmetrically (`:539`↔`:1640`, retry arm `:1627`) but sets no `epdRefreshInProgress`. | Keep the stated ordering. |
| **4** | §9.1's "keep `ownerStillUp` in front" is correct and verified (`main.cpp:328-338`, `wifi_service.cpp:812`, `main.cpp:456`) — but `[M5]`'s stale `sessionOrigin` corrupts the guard's own input, a bug Phase 4 inherits. | Reset `sessionOrigin` now. |
| **4** | D6c correctly defers owner-scoped `linkIsUp()` to Phase 4; `[H6]`'s reconnect race is also naturally fixed by the token, but not until then. | Add the interim `!linkIsUp()` service-point check. |
| **5** | §9.3 declares `nrfSessionClearPending` redundant on the strength of `[H4]`'s false guarantee. | Downgrade the claim; let Phase 5 re-decide. |
| **5** | D7 rightly puts Phase 5's in-function guard in the invalidate half; `[H1]` adds a generation-check requirement Phase 5's guard does not provide. | Land the generation check in Phase 3. |
| **5** | Phase 3 delivers the BLE-disconnect session clear one phase early with no backstop; §9.3 states the mitigation honestly and it is sound for Phase 3's own two callers. | Keep the "hand-check any new caller" warning. |
| **6** | D2's dead `g_lastProgressMs` is intentional and correctly documented; the six stamp sites in §8.2 match `[C1]`. | No action. |
| **6** | D7's "never gate" header comment is right; Phase 6's nRF supervisor must carry the (weakened, `[H4]`) depth term itself. | Already in D7 condition 2. |
| **7** | `[C6]`'s fix must not collide with Phase 7's `commandQueueOverflowAbort` policy; D5's two-files warning is accurate. | No action beyond `[C6]`. |

---

## Required corrections

| # | Sev | Correction | Section |
|---|---|---|---|
| ~~C1~~ | ~~Crit~~ | ✅ **DONE** — plan §3a header-placement table; all sites corrected. | §3a, §4.1, §7, §8.4, §9.2, §11, D6c |
| C2 | Crit | Move the refresh/stream deferral inside `epdSessionForceOff()`, or stop step 4 touching panel power. | §8.3 steps 4+6, D7 |
| C3 | Crit | Replace "two choke points" with one opcode-keyed pair in `imageDataWritten`. | D7, §11 step 9a |
| C4 | Crit | Gate step 6 on `pwrmgmState == PWR_ACTIVE`; add a WARM-survives-disconnect test. | §8.3 step 6, §9.1, D1 |
| C5 | Crit | Add `pipeWriteActive()`/`partialWriteActive()` accessors. | §8.3 step 1 |
| ~~C6~~ | ~~Crit~~ | ✅ **DONE** — cleared at top of drain block; alternative recorded; §12 test added. | §5, §7, §12 |
| H1 | High | Generation/handshake check on the deferred scrub; state that `ccm_session_free` is never deferred. | D7 hazard 2 |
| H2 | High | Route `touchForceResumeAll()` through `touchResumeAfterEpdRefresh()`; test a real touch event. | §4.2, §12 |
| H3 | High | Service new flags from `loop()`'s shared prologue **and** `idleDelay()`. | §8.3.1, §9.2 |
| H4 | High | Reword §9.2; re-check the `nrfSessionClearPending` deletion. | §9.2, §9.3 |
| H5 | High | Wire `panelStateUnknown`; settle Phase 2 D-A/D-B; give the abort a failure signal. | §8.3, §1 |
| H6 | High | Gate the deferred teardown on `!linkIsUp()` / a connection generation. | §9.1, §9.2 |
| H7 | High | Restrict buzzer/LED stop to `dropLink=true`, or record an explicit decision in §2. | §9.1, §12 |
| M1 | Med | Correct the `cleanupDirectWriteState` idempotence claim; re-derive step 4's rationale. | §8.3 |
| M2 | Med | Guard first, log second; single-exit so `inAbort` cannot latch. | §8.3, D7 |
| M3 | Med | Complete §8.1's header surface (11 symbols). | §8.1 |
| M4 | Med | Rebase the scrub split onto Phase 1's bitmap; fix the stale `replay_window[256]` ref. | D7, §10 |
| M5 | Med | Reset `sessionOrigin` in the teardown. | §8.3 step 4 |
| L1–L8 | Low | RAM arithmetic; rename `flushResponseQueue`; fix both §12 greps; `:447`→`:456`; `(unsigned)` casts; orphan cross-ref; note the `:550`/`:558` behaviour change. | §10, §7, §12, §9.1, §4.1 |
| X1–X5 | Gap | Abort failure signalling; zlib reset; partial-path touch note; `epdPlanesPrepared` window; qualify the flush's client-neutrality. | new |
