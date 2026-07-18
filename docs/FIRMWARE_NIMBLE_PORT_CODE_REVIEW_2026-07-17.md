# Firmware Code Review — Post NimBLE Port

**Repo:** `Firmware` (PlatformIO; ESP32-S3/C3/C6 + nRF52840)
**Date:** 2026-07-17
**Trigger:** ESP32 BLE stack migrated Bluedroid → NimBLE-Arduino 2.x (`5d7e705` + follow-ups `a5b49b2`, `84c322e`, `ae85de8`)
**Scope:** Whole Firmware repo, current `main`, with a deliberate bias toward **race conditions, object lifetime, and NimBLE 2.x porting hazards**, plus general error conditions.
**Method:** 5 parallel review agents, each reading the actual source (and, for the BLE core, the vendored `NimBLE-Arduino` 2.x library under `.pio/libdeps/`). Every finding cites `file:line`. Findings marked *needs verification* could not be fully confirmed from source alone.
**Nature:** Report only — no code was modified.

---

## Executive summary

The NimBLE port's **data plane is well built.** The command path uses a correct single-producer/single-consumer (SPSC) ring with acquire/release atomics; the `onWrite` callback only copies bytes into that ring and does no processing; binary payloads (including leading `0x00`) survive because the port uses `NimBLEAttValue` rather than Arduino `String`; the callback objects are statically allocated with the `deleteCallbacks=false` ownership flags set correctly; and `deinit(true)` + handle-clearing are ordered safely on a single task. The known `setCallbacks` hazard class was audited and has **no remaining siblings** (see *Verified clean*).

The port's structural weakness is concentrated in **one root cause**: heavyweight, state-mutating work still runs *inside the NimBLE server callbacks*, on the NimBLE host task. On nRF/Bluefruit the connect/disconnect/write callbacks and command processing all shared a single BLE task, so these operations were serialized by construction. The migration moved command/response processing to `loop()` but left connect-time and disconnect-time teardown/refresh on the now-genuinely-concurrent (and on the S3, other-core) host task. This produces the Critical SPI/rail-teardown race and two High races. **All three share one fix** — make every callback *flag-only* and service the flag from `loop()`, exactly as the code already does for `bleRestartAdvertisingPending`.

A second, independent cluster of findings is in the **encryption/session layer** (replay + nonce reuse). These are **pre-existing** (not introduced by the NimBLE port) but are the highest-severity correctness/security issues after the Critical race, so they are included.

### Provenance at a glance

| Cluster | Introduced by NimBLE port? |
|---|---|
| Callback-on-host-task races (#1, #2, #3, #6) | **Yes** — migration moved command processing to `loop()`, left teardown on the callback task |
| `0x0052` force-sleep deinit-while-connected (#8) | Partly — interacts with NimBLE `deinit(true)` semantics |
| MTU silent-drop, advertising restart, MSD churn (#7, #13, #14, #26) | Yes — NimBLE 2.x advertising/MTU behavior |
| Crypto replay + nonce reuse + DoS (#4, #5, #9) | **No** — pre-existing session logic |
| Display TCON/pipe/keep-alive (#10, #11, #22) | No — pre-existing display path (but #11 interacts with disconnect cleanup) |
| Buzzer/sleep, sensors, wifi (#12, #23, #25) | No — pre-existing |

### Count by severity

| Severity | Count | Numbers |
|---|---|---|
| Critical | 1 | #1 |
| High | 4 | #2, #3, #4, #5 |
| Medium | 7 | #6, #7, #8, #9, #10, #11, #12 |
| Low | 13 | #13–#25 |
| Informational | 7 | #26–#32 |

---

## CRITICAL

### #1 — `onDisconnect` (host task) tears down the EPD/direct-write session while `loop()` is streaming SPI
- **Location:** `src/esp32_ble_callbacks.h:53-74`; `src/display_service.cpp:1711-1737` (`cleanupDirectWriteState`), `:220-236` (`epdSessionForceOffLocked`); `src/main.cpp:325-341` (queue drain), `:662-673` (`pwrmgm`/`SPI.end()`); `src/main.h:381-384`
- **Category:** race (cross-task, use-after-free of the SPI bus/rail)
- **Provenance:** Introduced by the NimBLE migration.
- **Observed:** In NimBLE-Arduino 2.x, `MyBLEServerCallbacks::onDisconnect` is invoked directly from the GAP event handler on the `nimble_host` FreeRTOS task (confirmed in the vendored `NimBLEServer.cpp:507`), **not** on `loopTask`. The handler does:
  ```cpp
  if (epdRefreshInProgress) { /* defer */ } else {
      if (directWriteActive) cleanupDirectWriteState(true);   // -> epdSessionForceOff()
      cleanupPartialWriteOnDisconnect();
  }
  ```
  `cleanupDirectWriteState(true)` → `epdSessionForceOff()` runs `bbepSleep(&bbep,1); delay(50); pwrmgm(false)`, and `pwrmgm(false)` calls `SPI.end()` and cuts the panel rail. Meanwhile the **loop task** is draining queued `0x71`/`0x81` frames (`imageDataWritten` at `main.cpp:332`) and clocking them out via `bbepWriteData`. `main.h:381-383` explicitly sizes the 33-slot queue to absorb "a 60 s Spectra SPI stall (loop blocked in `bbepWriteData`)."
- **Why wrong / trigger:** The only guard, `epdRefreshInProgress`, covers the *refresh* phase, not the *streaming* phase. A client disconnect (supervision timeout / phone out of range — especially likely *during* that 60 s SPI stall) while `directWriteActive` and the loop is inside `bbepWriteData` causes the host task to issue SPI ops (`bbepSleep`) on the bus the loop is concurrently clocking, then `SPI.end()` and power the rail off under the loop's feet. On the dual-core S3 these run truly in parallel. Result: SPI driver corruption/crash, or the loop writing to a dead bus and hanging. `pwrmgmLock` does not protect this — the loop only holds it inside `epdSessionAcquire/Release`, and it is released (`display_service.cpp:273`) before any data is streamed. The `directWrite*` globals cleared at `display_service.cpp:1712-1723` are plain non-atomic bools/ints, read mid-operation by the loop, and even the `directWriteActive` check in the callback is a cross-task read of a non-volatile bool.
- **Solution:** Make `onDisconnect` **flag-only**: set a `volatile bool bleDisconnectCleanupPending` and perform `cleanupDirectWriteState` / `cleanupPartialWriteOnDisconnect` / `resetPipeWriteState` in `loop()` (where they are already single-task-safe). The advertising restart already uses exactly this deferral pattern (`bleRestartAdvertisingPending`); the cleanups must too.
- **Confidence:** High — call chains and callback task context confirmed against vendored `NimBLEServer.cpp`.

---

## HIGH

### #2 — `resetPipeWriteState()` on disconnect races the loop's pipe-frame processing (and bypasses the refresh guard)
- **Location:** `src/esp32_ble_callbacks.h:72`; `src/display_service.cpp:2137-2140`
- **Category:** race
- **Provenance:** Introduced by the NimBLE migration.
- **Observed:** `resetPipeWriteState()` (`pipeState = PipeWriteState{};` + clearing every `pipeReorder[i].occupied`) is called **unconditionally** in `onDisconnect` — outside the `if (epdRefreshInProgress)` else-branch — on the host task.
- **Why wrong / trigger:** After a disconnect, up to 32 queued `0x0081` frames may still be draining in `loop()` via `handlePipeWriteData`, which reads/mutates `pipeState` and memcpys payloads into `pipeReorder` slots. A non-atomic multi-word struct wipe racing that produces torn state: `pipeState.active` cleared mid-handler, half-zeroed window bookkeeping, or a reorder slot marked unoccupied while the loop is memcpying into it. Worst realistic outcome is corrupted/misrouted transfer state (combined with the `pipeState.partial` path, an inconsistent panel session) rather than a crash. It also defeats the refresh deferral: even when the rest of cleanup is deferred, the pipe state is still ripped out from the host task.
- **Solution:** Defer to the loop via the same pending flag as #1. If an immediate marker is needed, set only a single `volatile bool pipeAbortPending` that `handlePipeWriteData` checks.
- **Confidence:** High — code-confirmed.

### #3 — `updatemsdata()` called from `onConnect` (host task) races the loop-task `updatemsdata()`: concurrent `std::vector` mutation + cross-task I2C/ADC
- **Location:** `src/esp32_ble_callbacks.h:51` (`onConnect` → `updatemsdata()`); `src/display_service.cpp:1474-1560`; `src/main.cpp:415-419` (60 s loop caller); `src/ble_init.cpp:244` (restart-path caller)
- **Category:** race (heap corruption)
- **Provenance:** Introduced by the NimBLE migration.
- **Observed:** `updatemsdata()` (a) polls I2C sensors (`pollSht40SensorsForMsd`, `pollBq27220ForMsd`) and the battery ADC (`readBatteryVoltage`, unguarded static cache at `display_service.cpp:1449-1451`), (b) writes shared globals `msd_payload[16]`, `mloopcounter`, `rebootFlag`, and a function-static `prev_msd_payload`, and (c) on the connected branch does `*advertisementData = BLEAdvertisementData();` then `setName/setFlags/setManufacturerData`, i.e. assignment and repeated `m_payload.insert(...)` on the `std::vector` inside global `globalAdvertisementData`. The loop task calls the same function every 60 s and from `esp32_restart_ble_advertising`.
- **Why wrong / trigger:** If a central connects while the loop is inside its periodic `updatemsdata()` (which includes a `stop(); … delay(50); start()` window that widens the overlap), two tasks concurrently assign/append to the same `std::vector` — undefined behavior, realistically heap corruption. Independently, the host-task I2C polls contend with the loop's own `Wire` traffic (GT911 touch, sensors) on a non-thread-safe bus, and the `readBatteryVoltage` static cache is read-modify-written from both tasks.
- **Solution:** In `onConnect`, set flags only (`rebootFlag = 0; esp32BleNotifySubscribed = false; msdUpdatePending = true;`) and let `loop()` run `updatemsdata()`. Note the connected-branch rebuild of `*advertisementData` is dead work anyway — it is never pushed via `setAdvertisementData()`; the restart path rebuilds fresh data itself (`display_service.cpp:1540-1550`).
- **Confidence:** High — call sites, the `std::vector` member, and both task contexts verified.

### #4 — Replay protection accepts a repeat of the current-highest nonce counter
- **Location:** `src/encryption.cpp:128-155` (esp. `:136`)
- **Category:** crypto-logic / replay
- **Provenance:** Pre-existing (not the NimBLE port).
- **Observed:**
  ```c
  if (nonce_counter <= encryptionSession.last_seen_counter && counter_diff != 0) {
      bool already_seen = false;
      for (int i = 0; i < 64; i++) { if (replay_window[i] == nonce_counter) { already_seen = true; } }
      if (already_seen) return false;
  }
  if (nonce_counter > encryptionSession.last_seen_counter) last_seen_counter = nonce_counter;
  ```
  The seen-set membership test is gated by `counter_diff != 0`, so when `nonce_counter == last_seen_counter` the check is skipped and the function returns `true`.
- **Why wrong / trigger:** After any command with counter *K* (which sets `last_seen_counter = K` when it is the max), an attacker who re-sends that exact captured frame has `counter_diff == 0`, passes `verifyNonceReplay`, and — because the ciphertext/tag are identical — passes `aes_ccm_decrypt`, so the command re-executes. The most-recent authenticated command is therefore always replayable (re-trigger a config write, LED, deep-sleep, reboot…). Also hits at session start: `last_seen_counter` initializes to 0 (`:605`), so a first command with counter 0 is replayable.
- **Solution:** Treat `counter_diff == 0` as a replay — drop the `&& counter_diff != 0` so the equality case runs the seen-set check, or reject `counter_diff == 0` outright once any counter has been recorded.
- **Confidence:** High — code-confirmed.

### #5 — AES-CCM nonce reused across command and response directions
- **Location:** `src/encryption.cpp:158-168` (`getCurrentNonce`), `:604` & `:689-690`, `:663-665`
- **Category:** crypto-logic
- **Provenance:** Pre-existing (not the NimBLE port).
- **Observed:** Command and response nonces are both built as `nonce = session_id[8] || counter_be[8]`, then `nonce_ccm = nonce_full[3..15]`, under the **same** `session_key`. The device response counter is set to 0 at auth (`:604`) and the client command counter likewise starts at 0; there is no direction bit in the nonce.
- **Why wrong / trigger:** The first encrypted command (counter 0) and first encrypted response (counter 0) yield an identical CCM nonce under an identical key. CCM keystream depends only on key+nonce (not the AD), so equal-counter messages in the two directions share keystream — XOR of the two plaintexts leaks, and CCM nonce reuse also weakens the CBC-MAC authenticity guarantee. Every overlapping counter value between the two independent counters is a reuse.
- **Solution:** Domain-separate the directions: reserve a direction bit/byte in the nonce, or derive two sub-keys (one per direction) from the session key, so device→client and client→device never share a (key, nonce) pair.
- **Confidence:** Med — device-side reuse is code-confirmed; full impact depends on the `py-opendisplay` client using the same single-key/counter convention (*needs verification against the client*).

---

## MEDIUM

### #6 — Long blocking operations inside NimBLE callbacks stall the host task
- **Location:** `src/esp32_ble_callbacks.h:51` (`onConnect`), `:66-72` (`onDisconnect`); `src/display_service.cpp:230-233` (`bbepSleep`+`delay(50)`), `:203-209` (`pwrmgmLockTake` spin-with-`delay(1)`); `src/touch_input.cpp:416-440` (`touchResumeAfterEpdRefresh`: settle delay + GT911 I2C reinit)
- **Category:** nimble-api-misuse
- **Provenance:** Introduced by the NimBLE migration.
- **Observed:** The disconnect path can execute `pwrmgmLockTake()` (blocks until the loop releases the lock — potentially spanning the tick's SPI ops), `bbepSleep + delay(50)`, `pwrmgm(false)` (I2C PMIC shutdown, pin writes), and `touchResumeAfterEpdRefresh`. `onConnect`'s `updatemsdata()` performs sensor I2C, ADC reads, and mDNS TXT updates. All on the NimBLE host task.
- **Why wrong / trigger:** The host task also services all GAP/GATT events. Blocking it hundreds of ms (or longer if `pwrmgmLockTake` waits on a busy loop) delays connection setup, MTU exchange, and subscription events, and risks host-level timeouts. NimBLE callbacks are expected to return promptly.
- **Solution:** Falls out of the #1/#3 fix — once callbacks are flag-only, the host task never blocks.
- **Confidence:** High — code-confirmed.

### #7 — MTU advertised as 512 but any write of 257–512 bytes is silently discarded (no error response)
- **Location:** `src/ble_init.cpp:255` (`BLEDevice::setMTU(512)`); `src/esp32_ble_callbacks.h:97, 119-123`
- **Category:** error-handling
- **Provenance:** Introduced by the NimBLE migration (MTU/limit interaction).
- **Observed:**
  ```cpp
  if (value.length() > 0 && value.length() <= MAX_COMMAND_SIZE) { /* queue */ }
  else if (value.length() > MAX_COMMAND_SIZE) { writeSerial("WARNING: Command too large, dropping"); }
  ```
  `MAX_COMMAND_SIZE` is 256, but the ATT MTU (512) and the characteristic's default 512-byte max value accept larger writes, which are dropped with only a local serial log — the client's ATT write completes successfully and no NACK/notification is sent.
- **Why wrong / trigger:** A third-party client that negotiates MTU 512 and sends >256-byte frames (legal on the raw protocol) sees writes silently vanish and hangs waiting for acks. The `main.h:382-384` comment acknowledges the >256 B regression as "none known" — but the *silent* part is separately fixable.
- **Solution:** Emit a small error notification (e.g. `{0xFF, cmd, err}`), raise `MAX_COMMAND_SIZE` to the true max frame size, or cap advertised MTU nearer 259 so oversized writes can't be produced. (The full-queue drop at `:119-121` is lower risk — the per-frame ack/pipe-mask protocols give the client a recovery path.)
- **Confidence:** High — code-confirmed.

### #8 — `0x0052` force-sleep tears down NimBLE while the client is still connected (no graceful disconnect)
- **Location:** `src/main.cpp:481-523` (force path); `src/device_control.cpp:740`; `src/esp32_ble_callbacks.h:85-127`
- **Category:** race / lifetime
- **Provenance:** Interacts with NimBLE `deinit(true)` semantics.
- **Observed:** Host command `0x0052` calls `enterDeepSleep(true, overrideSeconds)`. With `force==true`, the connected-client bail (`main.cpp:492`) is skipped, and execution proceeds to `epdSessionForceOff()` → advertising stop → `delay(200)` → `BLEDevice::deinit(true)` → `esp32_ble_clear_handles()`. No `disconnect()` of the peer is issued first. The command is drained in the loop task; the NimBLE host runs `onWrite` in a separate task.
- **Why wrong / trigger:** In the force path the link is still up, so the central can still be delivering WRITE_NR frames (each firing `onWrite`, which memcpys into `commandQueue`) at the moment the loop calls `deinit(true)`. `deinit(true)` frees the GATT objects and stops the host; a callback executing against those objects concurrently is a use-after-free window. `delay(200)` narrows but does not close it. Idle-path sleeps are safe (only when `getConnectedCount()==0`); this is specific to `0x0052`.
- **Solution:** In the force path, stop accepting new work and gracefully `pServer->disconnect(connHandle)`, poll `getConnectedCount()==0` (bounded wait) before `deinit(true)`; or disable the RX characteristic first. At minimum confirm NimBLE's `deinit(true)` internally quiesces the host task before freeing.
- **Confidence:** Med — force bypass + missing disconnect are code-confirmed; exact UAF depends on NimBLE-Arduino deinit/host-task synchronization (*needs verification*).

### #9 — Replay counter is advanced by unauthenticated (pre-decrypt) data → cheap session-teardown DoS
- **Location:** `src/encryption.cpp:637-684` (`verifyNonceReplay` at `:640`, before `aes_ccm_decrypt` at `:663`)
- **Category:** crypto-logic / error-handling
- **Provenance:** Pre-existing.
- **Observed:** `decryptCommand` calls `verifyNonceReplay(nonce_full)` — which mutates `last_seen_counter` and `replay_window` — *before* the tag is verified. The `session_id` gating the nonce (`:118`) is transmitted in the clear as the first 8 bytes of every command nonce.
- **Why wrong / trigger:** An attacker who sniffs one command learns `session_id`, then forges frames with that id and an in-window counter (`last_seen ± 32`). Each forged frame passes `verifyNonceReplay` (advancing `last_seen_counter`) then fails CCM auth, incrementing `integrity_failures`; after 3, the session force-clears (`:678-682`). So 3 sniff-and-forge packets tear down an authenticated session and/or push `last_seen_counter` ahead of legitimate in-flight commands (which then fall outside the ±32 window and get rejected). Low-cost remote DoS.
- **Solution:** Do not persist replay-window/counter state until *after* successful authenticated decryption (advance only on the success path after `:666`); rate-limit / deprioritize the integrity-failure teardown so unauthenticated traffic can't cheaply reset a session.
- **Confidence:** Med — logic code-confirmed; exploitability depends on sniffing the cleartext `session_id`, which the protocol exposes.

### #10 — Seeed/IT8951 refresh always reports success — TCON busy-timeout is discarded
- **Location:** `src/display_seeed_gfx.cpp:123-127` (`seeed_gfx_wait_refresh`); consumed at `src/display_service.cpp:510, 2078`
- **Category:** error-handling
- **Provenance:** Pre-existing (Seeed display path).
- **Observed:**
  ```cpp
  bool seeed_gfx_wait_refresh(int timeout_sec) { (void)timeout_sec; delay(300); return true; }
  ```
  The library sets a real failure flag (`lib/Seeed_GFX/Extensions/Tcon.cpp:32`: `opnd_seeed_tcon_busy_timed_out = true;`) and exposes `opnd_seeed_tcon_busy_timeout_occurred()` (`display_seeed_gfx.cpp:46`), but `seeed_gfx_wait_refresh` never consults it.
- **Why wrong / trigger:** `waitforrefresh()` routes to this function for the Seeed driver, returning `true` unconditionally, so `refreshSuccess` in `directWriteFinishAndRefresh` (`:2078`) is always true even when the IT8951 stalled. The device then commits `displayed_etag = newEtag` (`:2099`) and replies `{0x00,0x73}` success. A later partial-update diff bases against an etag that doesn't match the physical panel → corrupt partial refresh, failure invisible to the client.
- **Solution:** Return `!opnd_seeed_tcon_busy_timeout_occurred()` (the flag is reset at refresh start by `seeed_gfx_epaper_begin`, `:106`) and honor `timeout_sec` instead of a fixed `delay(300)`.
- **Confidence:** High — code-confirmed including the library set-site.

### #11 — PIPE full-frame stall leaves a zombie `pipeState.active`; a later END drives SPI/refresh into a powered-off panel
- **Location:** `src/main.cpp:346-352` (watchdog) with `src/display_service.cpp:356-365` (`checkPartialWriteTimeout` / `resetPipeWriteState`), `:2353` (`handlePipeWriteStart`), `:2522` (`handlePipeWriteEnd`), `:1773` (`directWriteActivatePanel`)
- **Category:** lifetime / resource-leak
- **Provenance:** Pre-existing (interacts with disconnect cleanup #2).
- **Observed:** The only stall guard covering a PIPE *full-frame* transfer is the direct-write watchdog (`cleanupDirectWriteState(true)` when `directWriteDuration > 900000UL`). A PIPE full-frame transfer sets **both** `directWriteActive` and `pipeState.active`, but `checkPartialWriteTimeout` only clears `pipeState` when `pipeState.partial` (`:363`).
- **Why wrong / trigger:** On a 15-min stall, `cleanupDirectWriteState(true)` resets `directWrite*` and powers the panel off, but `pipeState.active` stays true (nothing calls `resetPipeWriteState()`). Before the next `0x0080` START self-heals it, a `0x0082` END is accepted (`handlePipeWriteEnd` has no `directWriteActive` check), completeness passes (both byte counters 0 and equal), and `directWriteFinishAndRefresh` calls `bbepRefresh()` + `waitforrefresh(60)` on a rail that is powered **off** — a spurious refresh plus a ~60 s blocking hang returning failure.
- **Solution:** In the direct-write watchdog (or a dedicated pipe timeout), also `if (pipeState.active) resetPipeWriteState();` when the stall fires; equivalently gate `handlePipeWriteEnd`'s refresh on `directWriteActive` for the non-partial branch.
- **Confidence:** High for the zombie state; Med for the 60 s-hang manifestation (depends on END arriving before a new START).

### #12 — Deep sleep can silently truncate active buzzer playback (no "buzzer active" gate)
- **Location:** `src/buzzer_control.cpp:130` (private `s_buzzer.active`, no exported accessor); `src/main.cpp:380-405` (deep-sleep gate), `enterDeepSleep()`
- **Category:** timing / logic
- **Provenance:** Pre-existing.
- **Observed:** The `workInFlight` gate (`main.cpp:380`) does not include buzzer state, and `enterDeepSleep()` never calls `buzzer_stop_internal()`. A melody may run up to `kBuzzerMaxTotalMs = 30000` ms, but the battery idle-hold default is ~10 s.
- **Why wrong / trigger:** If a melody is triggered over WiFi/LAN, or the BLE client disconnects mid-playback (so `getConnectedCount()==0`), the idle window can expire while `s_buzzer.active` is still true. `buzzerService()` (`main.cpp:422`) runs only *after* the non-returning `enterDeepSleep()` call at `:404`, so the melody is cut off and the buzzer enable pin / PWM are torn down abruptly.
- **Solution:** Export `bool buzzerIsActive(void)` and add it to `workInFlight`; and/or call `buzzer_stop_internal()` at the top of `enterDeepSleep()` so the buzzer is deterministically silenced before sleep.
- **Confidence:** High — code-confirmed across both files.

---

## LOW

### #13 — `esp32_restart_ble_advertising`: unchecked start result, pending flag cleared before success, redundant double-restart
- **Location:** `src/ble_init.cpp:228-246`
- **Category:** error-handling / logic
- **Observed:** `bleRestartAdvertisingPending = false;` then `delay(100); BLEDevice::startAdvertising(); updatemsdata();`. The flag is cleared before `startAdvertising()`, whose boolean result is ignored; `updatemsdata()` then does a second `stop(); … delay(50); start()` cycle (its dedupe never fires — see #14). Verified against NimBLE 2.x that `m_advertiseOnDisconnect` defaults to **false**, so this function is the only thing that restarts advertising.
- **Why wrong / trigger:** If `startAdvertising()` fails (transient controller state right after disconnect), the flag is already cleared and there is no retry until the 60 s idle MSD tick — the radio can stay dark up to a minute. The `delay(100)`+`delay(50)` also stall the loop ~150 ms per disconnect.
- **Solution:** Clear `bleRestartAdvertisingPending` only after `start()` returns true; drop/soften the pre-delay; call `updatemsdata()` (which starts advertising with fresh data) *instead of* `startAdvertising()`.
- **Confidence:** High (code); failure frequency inferred (*needs hardware verification*).

### #14 — `updatemsdata` payload dedupe is dead code → advertising stop/start churn every 60 s
- **Location:** `src/display_service.cpp:1493-1496` (statusByte includes `mloopcounter` nibble), `:1523-1528` (memcmp dedupe), `:1558-1559` (`mloopcounter++`)
- **Category:** logic
- **Observed:** `statusByte` embeds `(mloopcounter & 0x0F) << 4`, and `mloopcounter` increments every call, so consecutive payloads always differ in the top nibble and `memcmp(prev_msd_payload, msd_payload, 16) == 0` can never be true.
- **Why wrong / trigger:** The dedupe branch (and its NRF twin at `:1505-1510`) is unreachable; every idle 60 s tick runs `stop(); … delay(50); start()` — a ≥50 ms advertising gap + loop stall each minute that the dedupe intended to avoid. Cosmetic reliability cost, not a correctness bug (the rolling nibble is deliberate for scan distinguishability).
- **Solution:** Compare payloads excluding the sequence nibble, or only restart advertising when the non-rolling bytes change (prefer `setAdvertisementData` without stop/start).
- **Confidence:** High — code-confirmed.

### #15 — Duplicated struct/constant definitions across translation units — silent-divergence (ODR) hazard
- **Location:** `src/esp32_ble_callbacks.h:17-28` (`#ifndef COMMAND_QUEUE_SIZE` fallback 33/256 + `CommandQueueItem`); `src/main.h:384-399` (authoritative); `src/communication.cpp:50-57` (local `chunked_write_state_t` with literal `buffer[4096]` vs `MAX_CONFIG_SIZE`), `:65-76` (local `ResponseQueueItem data[512]`, `RESPONSE_QUEUE_SIZE_LOCAL = 10`)
- **Category:** buffer / logic
- **Observed:** The BLE callback header carries its own `#ifndef` copies of the queue geometry, and `communication.cpp` re-declares `chunked_write_state_t` and `ResponseQueueItem` locally with literal sizes. All values currently agree.
- **Why wrong / trigger:** Any future change to `MAX_CONFIG_SIZE`, `MAX_RESPONSE_SIZE`, or queue sizes in one place but not the others compiles cleanly and produces layout-mismatched access to the same global — memcpy past the real buffer, i.e. memory corruption with no diagnostic. The `#ifndef` fallback is the sharpest edge: a TU including it without `main.h` first gets a *different* `CommandQueueItem` layout.
- **Solution:** Move the queue structs/sizes into one shared header included by both; replace the `communication.cpp` literals with the named macros; add `static_assert(sizeof(...))` cross-checks.
- **Confidence:** High — duplication confirmed and currently matches.

### #16 — `0x0052` timer-sleep path sends no ACK before deinit
- **Location:** `src/device_control.cpp:717-740`
- **Category:** error-handling / logic
- **Observed:** The D-FF branch sends OK before power-off (`:721-722`) and reject branches send error frames (`:729-736`), but the normal battery timer-sleep branch calls `enterDeepSleep(true, overrideSeconds)` (`:740`) with no success response; the subsequent `deinit(true)` destroys any queued ACK.
- **Why wrong / trigger:** A host issuing `0x0052` with a duration override gets no confirmation the device accepted it, inconsistent with the reject cases (which do respond).
- **Solution:** `sendResponse({0x00,0x52,0x00,0x00})` + brief flush (as the D-FF branch does at `:721-723`) before `enterDeepSleep`, or document intentional silent sleep.
- **Confidence:** High — code-confirmed.

### #17 — `epdSessionForceOff()` in the force path is not gated on `epdRefreshInProgress`
- **Location:** `src/main.cpp:492-513`
- **Category:** ordering
- **Observed:** On the force path the early bails at `:492`/`:501` are skipped and execution reaches `epdSessionForceOff()` (`:513`) unconditionally. The idle path only reaches `enterDeepSleep` when `workInFlight` (including `epdRefreshInProgress`, `:384`) is false, so it can never power off mid-refresh — the force path has no such guard.
- **Why wrong / trigger:** If a refresh were ever in progress when `0x0052` is handled, `epdSessionForceOff()` would cut the EPD rail mid-refresh (corrupt/partial image, wedged controller). Currently `0x0052` is drained in the same loop pass as refreshes (both in the main task), so no refresh is concurrently in progress — defense-in-depth, not a live bug.
- **Solution:** Guard `epdSessionForceOff()` with `if (epdRefreshInProgress) { defer/wait }` so the invariant holds for future callers/contexts.
- **Confidence:** Med — missing guard confirmed; benign under current single-task execution.

### #18 — Button GPIO ISR has no debounce and reads the pin inside the ISR
- **Location:** `src/device_control.cpp:517-537` (`handleButtonISR`/`buttonISR`, `attachInterruptArg(... CHANGE)` at `:616`); interacts with `src/main.cpp:187-230` (`pollActivity`)
- **Category:** race / logic
- **Observed:** The ISR fires on every edge, does `digitalRead()`, and on a state change bumps `press_count` and sets `buttonEventPending`/`lastChangedButtonIndex` (volatile, consumed under `noInterrupts()` at `:431`). No debounce interval.
- **Why wrong / trigger:** A bouncing contact produces a burst of edges → multiple `press_count` increments per press and repeated activity; via `dynamicreturndata` this stamps `lastActivityMs` (`main.cpp:213`), holding the wake/idle window open longer than intended and reporting an inflated press count. Does not cause spurious sleep, only inaccurate counts / slightly delayed sleep.
- **Solution:** Per-button debounce (ignore edges within N ms, tracked in `ButtonState`), or latch in the ISR and validate in `processButtonEvents`.
- **Confidence:** High — no debounce confirmed.

### #19 — Config packet parser ignores the on-wire length field and trusts `sizeof(struct)`
- **Location:** `src/config_parser.cpp:288-609` (loop header `:289-291`; each case advances by `sizeof(struct …)`)
- **Category:** logic / buffer (bounded)
- **Observed:** The loop reads a 2-byte header (`offset++` then `packetId = configData[offset++]`) but discards the first byte, then advances `offset += sizeof(struct SystemConfig)` etc., using the compiled struct size rather than any length carried in the packet.
- **Why wrong / trigger:** If a stored/BLE-delivered body's length differs from the firmware's `sizeof(struct)` (schema drift, malformed input, an older/newer toolbox), parsing desyncs — following bytes are reinterpreted as a new `[len][id]` header, silently mis-loading later packets into `globalConfig` (pins, flags, security_config). **Not** memory-unsafe — every memcpy is guarded by `offset + sizeof(...) <= configLen - 2` — but it can load attacker-influenced garbage. The trailing outer CRC (`:616`) is warn-only and does not reject.
- **Solution:** Parse and honor the packet's declared length to advance `offset`, and validate it equals the expected struct size for known IDs before memcpy; treat a mismatch as a hard parse error.
- **Confidence:** High that the length byte is unused; Med on real-world triggerability (*verify against the wire format the toolbox emits*).

### #20 — Config reload mutates `globalConfig` in the BLE callback task while `loop()` reads it (nRF only)
- **Location:** `src/config_parser.cpp:263-264` (`memset`/repopulate in `loadGlobalConfig`), reached via `handleWriteConfig`/`reloadConfigAfterSave`; nRF dispatch at `src/ble_init.cpp:160`
- **Category:** race
- **Provenance:** Pre-existing (Bluefruit path), **not** the NimBLE port — but in-scope for the callback-vs-loop concern.
- **Observed:** On nRF, `imageDataWritten` (and thus `loadGlobalConfig`, which zeroes and rebuilds `globalConfig`) runs in the Bluefruit BLE callback context, while `loop()` concurrently reads `globalConfig.leds[...]`, `globalConfig.displays[...]`, button state (e.g. `processLedFlash`, `flashLed`). ESP32 is unaffected (commands drain in `loop()`).
- **Why wrong / trigger:** A config write arriving mid-flash lets `loop()` observe a half-`memset`/half-rebuilt `globalConfig` (e.g. `led_count` changed while `leds[]` is being overwritten) → inconsistent reads / bad pin writes.
- **Solution:** On nRF, marshal command processing to the main loop as ESP32 does, or guard `globalConfig` mutation/read with a critical section / double-buffer swap.
- **Confidence:** Med — call context confirmed; exact interleaving not instrumented (*needs verification on nRF scheduler specifics*).

### #21 — Non-atomic image-write flags read from the NimBLE host task (benign log race)
- **Location:** `src/esp32_ble_callbacks.h:92` → `src/display_service.cpp:1638-1653` (`imageWriteLogQuietFrame`); `directWriteActive` defined plain at `src/main.h:172`
- **Category:** race
- **Observed:** `onWrite` (host task) calls `imageWriteLogQuietFrame`, which reads `(directWriteActive || partialCtx.active || pipeState.active) && imgLogChunks >= 1` — all mutated by `loop()` with no atomics/locking and not `volatile`.
- **Why wrong / trigger:** Genuine concurrent read (host) vs write (loop) of non-atomic multi-byte state, but impact is confined to log suppression — a torn read only mis-logs a line. No memory-safety consequence (the payload is copied into the ring, not shared).
- **Solution:** Acceptable as-is; if desired, compute the quiet flag in `loop()` only, or snapshot it. Worth a comment noting the deliberate benign race.
- **Confidence:** High that it is a data race; High that impact is benign.

### #22 — Seeed keep-alive WARM state is inconsistent with a slept controller
- **Location:** `src/display_service.cpp:2075-2087` (refresh tail), `:280-295` (`epdSessionRelease`), `:2291` (WARM comment)
- **Category:** logic
- **Observed:** On the Seeed path the refresh tail calls `seeed_gfx_direct_sleep()` (`:2079`) unconditionally, then `cleanupDirectWriteState(false)` → `epdSessionRelease(true)`. With keep-alive enabled, Release sets `PWR_WARM` documented as "controller stays AWAKE (no bbepSleep)" — but the Seeed TCON was just slept.
- **Why wrong / trigger:** The "warm = controller awake, rail up" invariant doesn't hold for Seeed: the rail stays powered (draws current) while the TCON sleeps. Not a correctness fault — the next push runs `seeed_gfx_direct_write_reset` with `seeed_gfx_hw_initialized` still true → `g_seeed_epaper.wake()` (`display_seeed_gfx.cpp:157`), the right call for a slept-but-powered TCON. If Release takes the force-off branch instead, `seeed_gfx_direct_sleep()` is called twice (idempotent, harmless).
- **Solution:** Skip `seeed_gfx_direct_sleep()` at `:2079` when keep-alive will hold WARM, or document that Seeed WARM means "rail up, TCON asleep, re-wake on next push." Consider forcing keep-alive off for Seeed if the warm-rail draw isn't worth it.
- **Confidence:** Med — behavior confirmed; whether it matters depends on Seeed power budget (*needs verification*).

### #23 — Blocking SHT40 measurement/retry runs inside `loop()`
- **Location:** `src/sensor_sht40.cpp:79` (`delay(SHT40_MEASURE_DELAY_MS)`), `:118-156` (`read_sht40_sample` 2-pass × up-to-3-address retry with `Wire.end()`/`begin()`+`delay(2)`)
- **Category:** timing / error-handling
- **Observed:** A healthy sensor succeeds in ~12 ms, but if the preferred address doesn't ACK on pass 1, the code re-inits the bus and retries all addresses, stacking `delay()`s and possible `Wire.requestFrom` timeouts — hundreds of ms of loop stall that delays BLE command/response servicing. Bounded (every 30 s).
- **Solution:** Cap retries, avoid the full `Wire.end()/begin()` teardown on the hot path, or convert to a state-machine tick like the buzzer.
- **Confidence:** Med — inferred from retry structure; exact stall depends on Wire timeout config.

### #24 — Buzzer global-cap comment says "5 s" but the constant is 30 s
- **Location:** `src/buzzer_control.cpp:17` (`kBuzzerMaxTotalMs = 30000u`), `:144`, `:167` (comments say "5 s")
- **Category:** logic (doc/constant mismatch)
- **Observed:** No functional bug — the code uses the constant consistently and the comparisons are rollover-safe subtraction — but the stale "5 s" comments mislead about the real 30 s ceiling.
- **Solution:** Update the comments to 30 s (or make the constant match intent).
- **Confidence:** High.

### #25 — mDNS service re-registered on every WiFi reconnect without teardown
- **Location:** `src/wifi_service.cpp:73-83`, `:249-257` (`restartLanService`), `:177` / `:256` callers
- **Category:** resource-leak
- **Observed:** `restartLanService()` calls `MDNS.begin()` + `MDNS.addService(...)` on each reconnect with no matching `MDNS.end()`.
- **Why wrong / trigger:** Repeated `begin`/`addService` across reconnect cycles can duplicate the advertised record or leak responder state (build-dependent); frequently-reconnecting devices may accumulate records.
- **Solution:** `MDNS.end()` before re-`begin()`, or add the service once and only update TXT records on reconnect.
- **Confidence:** Med — depends on ESPmDNS idempotency (*needs verification against the library version*).

---

## INFORMATIONAL

### #26 — Advertising payload sits at exactly 31 bytes with all builder return values ignored
- **Location:** `src/ble_init.cpp:297-308`; `src/display_service.cpp:1535-1550`; vendored `NimBLEAdvertisementData.cpp:39-47, 269-280`
- **Observed:** Payload = name (10 B) + flags (3 B) + MFG data (18 B) = **31 bytes**, exactly `BLE_HS_ADV_MAX_SZ`. NimBLE 2.x `setManufacturerData`/`setName` *append* via `addData`, which returns `false` and drops the field once `size + length > 31`; every call site discards the result.
- **Why wrong / trigger:** Zero headroom. Any future growth (7-char ID, an extra dynamic byte, one more AD field) makes the last `set*` fail *silently* → device advertises without manufacturer data (id 9286), breaking HA discovery, with nothing logged. Also note NimBLE 2.x `set*` on a persistent `NimBLEAdvertisementData` appends rather than replaces; today this is safe only because every path rebuilds from scratch — worth a comment so a future "just update the MSD field" edit doesn't reintroduce payload growth.
- **Solution:** Check the boolean results of `setName/setFlags/setManufacturerData` and log/assert on failure.
- **Confidence:** High — confirmed against the vendored library.

### #27 — Verified clean: callback lifetime, binary-value handling, SPSC atomics, notify path (positive finding)
- **Location:** `src/ble_init.cpp:261-263, 286`; `src/main.cpp:409-410, 523-524`; vendored `NimBLEServer.cpp`, `NimBLECharacteristic.cpp`, `NimBLEAttValue.h`
- **Observed / conclusion:** The `setCallbacks` hazard class was audited and is clean: `staticServerCallbacks`/`staticCharCallbacks` are static; `setCallbacks(&staticServerCallbacks, false)` correctly prevents `~NimBLEServer` from deleting the static, and NimBLE 2.x `NimBLECharacteristic::setCallbacks` takes no ownership flag and never deletes `m_pCallbacks`, so `pTxCharacteristic->setCallbacks(&staticCharCallbacks)` is safe. `esp32_ble_clear_handles()` runs immediately after `deinit(true)` on the same task, so there is no window where the loop uses a freed `pServer`/`pTxCharacteristic`. The `onWrite` path uses `NimBLEAttValue` deep-copy with `.c_str()/.length()`, so the `84c322e` `0x00`-truncation fix is complete — no remaining Arduino-`String` conversions of binary RX payloads. The command-queue SPSC atomics (RELEASE publish / ACQUIRE consume) are correctly paired; `flushResponseQueueToBle`'s `notify(data,len)` (immediate mbuf copy) with stop-on-false backpressure is sound. **Caveat:** the response queue's non-atomic indices are safe only because it is produced and consumed exclusively on the loop task — an undocumented invariant that a single host-task `sendResponse` call would break.
- **Confidence:** High — all code-confirmed.

### #28 — RTC_DATA_ATTR wake state cannot distinguish a hidden mid-cycle reset from a true cold boot
- **Location:** `src/main.cpp:83-92, 147-153`; `src/main.h:317-318`; `src/wake_button.cpp:28`
- **Observed:** `deep_sleep_count`/`woke_from_deep_sleep` are `RTC_DATA_ATTR`, but the bootloader reloads RTC segments from the app image on every reset *except* a deep-sleep wake. A panic/WDT/brownout during an awake cycle lands in the NORMAL BOOT branch with `deep_sleep_count==0`, indistinguishable from first boot → re-runs `initDisplay()` (boot-screen redraw) and re-arms the min-wake window, spending extra energy when it should have resumed quietly.
- **Note:** Already captured in `docs/FINDINGS_DEEP_SLEEP_WAKE_BOOT_SCREEN_2026-07-07.md`. A durable fix needs a non-RTC source (NVS) or a magic-word validity check that survives non-wake resets.
- **Confidence:** High — matches documented behavior.

### #29 — Non-reentrant static scratch buffers in the crypto/config path
- **Location:** `src/encryption.cpp:662` (`static uint8_t decrypted_with_length[512]`), `:694` (`static uint8_t payload_with_length[513]`); `src/config_parser.cpp:85, 159` (`static config_storage_t config`)
- **Observed:** Safe today — command processing is serialized in `loop()` on ESP32 and each function fully consumes its buffer before returning; sizes are correct (frames capped at `MAX_COMMAND_SIZE` 256). Would break if ever called re-entrantly or from two contexts (see #20).
- **Solution:** No action now; keep the single-consumer invariant. If nRF moves to direct-callback dispatch, convert to caller-provided/stack buffers.
- **Confidence:** High — informational.

### #30 — Buzzer state machine has no locking but is currently single-threaded
- **Location:** `src/buzzer_control.cpp:129-145` (`s_buzzer`); dispatch `src/communication.cpp:652-655`, `src/main.cpp:332`
- **Observed:** `s_buzzer` is shared mutable state touched by `handleBuzzerActivate()` (stop + memcpy + re-init) and `buzzerService()`, both currently on the loop task (ESP32 enqueues; WiFi path is also loop-side). No live race — the "new melody mid-playback" case is handled (preempt via `buzzer_stop_internal()` before the memcpy). Called out because a future direct-from-callback invocation would race with zero synchronization.
- **Solution:** Keep buzzer command handling on the loop task only; document the single-thread invariant near `s_buzzer`.
- **Confidence:** High — dispatch path confirmed.

### #31 — `fb_byte_size()` / Seeed chunk path dereference `displays[0]` without a `display_count` guard
- **Location:** `src/display_seeed_gfx.cpp:87-94` (`fb_byte_size`), `:133-140`, `:166-177`
- **Observed:** `fb_byte_size()` reads `globalConfig.displays[0].pixel_width/height` unconditionally; `seeed_gfx_panel_is_4gray()` guards `display_count < 1` but these do not. Only reachable with a misconfigured/empty display config on a Seeed build (all real entry points require a configured display) — defensive only.
- **Solution:** Early-return / zero-size when `display_count < 1`, matching `seeed_gfx_prepare_hardware` (`:97`).
- **Confidence:** High — low practical impact.

### #32 — Touch loops rely on the `count <= 4` invariant without local clamping
- **Location:** `src/touch_input.cpp:471-559` (`prior_rt[4]`/`s_touch_rt[4]`), `:589-731`, `:429-443`
- **Observed:** Loops iterate `i < globalConfig.touch_controller_count` and index size-4 arrays with no `i < 4` clamp, unlike `touch_detach_all_configured_ints` (`:146`) which clamps. Safe today because `config_parser.cpp:396` caps `touch_controller_count` at 4; would become an OOB write if that cap ever regresses.
- **Solution:** Add `&& i < 4` for defense-in-depth, matching the already-clamped sibling.
- **Confidence:** High — not currently a bug; fragility note.

---

## Recommended remediation order

1. **#1 (Critical) + #2, #3, #6 (High/Medium) — single fix.** Convert all NimBLE server callbacks (`onConnect`, `onDisconnect`) to **flag-only**, servicing the flags from `loop()`, exactly as `bleRestartAdvertisingPending` already does. One change closes the SPI/rail teardown race, the pipe-state wipe race, the shared-`BLEAdvertisementData` vector race, and the host-task blocking. This is the highest-leverage fix in the report.
2. **#4, #5, #9 (High/Medium crypto).** Fix replay equality (`#4`, one-line), then direction-separate the CCM nonce (`#5`), then move replay-state advancement to the post-auth path (`#9`). Coordinate `#5` with the `py-opendisplay` client (protocol-level change).
3. **#8 (Medium).** Graceful disconnect before `deinit(true)` on the `0x0052` force path.
4. **#10, #11, #12 (Medium).** Honor the Seeed TCON timeout flag; clear zombie `pipeState` on stall; gate/silence the buzzer before deep sleep.
5. **Low / Informational.** Address opportunistically; **#15** (shared-header + `static_assert`) and **#26** (check advertising-builder results) are cheap traps worth closing early.

## Methodology / caveats
- Five agents reviewed disjoint file sets (core BLE runtime; power/sleep/wake; display/panel; config/crypto/device-control; peripherals); `main.cpp` was intentionally double-covered. Each read the actual source; the BLE core also read the vendored `NimBLE-Arduino` 2.x library to confirm callback task context and ownership semantics.
- All three of the top races were independently corroborated by multiple agents' confirmation that the ESP32 `onWrite` path is a clean SPSC enqueue and that display/config/buzzer work is single-threaded in `loop()` — which is precisely what makes the *non*-`onWrite* callbacks (`onConnect`/`onDisconnect`) the exception that breaks the model.
- Items tagged *needs verification* (#5 client-side, #8 NimBLE deinit quiescing, #19 wire format, #20 nRF scheduler, #22 power budget, #25 ESPmDNS) depend on facts outside this repo's source and should be confirmed before or alongside the fix.
