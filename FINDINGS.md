# OpenDisplay Firmware — Performance & Correctness Findings

Analysis of the `src/` tree (Arduino framework, nRF52840 + ESP32-S3/C3/C6 targets) for a
battery-powered e-paper BLE display tag. Findings were produced by a full read of every source
file, cross-checked against callers, and corroborated where possible by `cppcheck`
(`--enable=warning,performance,portability`, both `-DTARGET_NRF` and `-DTARGET_ESP32`).

On this device **wasted CPU time is wasted battery**, and the boot-screen render, image
decompression, and BLE transfer paths are the hot paths. Anything that keeps the panel rail or a
radio powered longer than necessary is treated as high impact.

Line numbers are from the tree at analysis time (commit `4f4b503`).

Legend: **severity** = high / med / low · **category** = correctness / perf.

---

## High severity

### H1. Dangling pointer to out-of-scope `errorResponse` — correctness
`src/communication.cpp:178-213`
```cpp
} else {                       // encrypt failed
    uint8_t errorResponse[] = {0xFF, (uint8_t)(command & 0xFF), 0x00};
    response = errorResponse;  // response now points at a local
    len = sizeof(errorResponse);
}                              // <-- errorResponse goes out of scope here
...
writeSerial("  Command: 0x" + String(response[0], HEX) ...);  // reads freed stack
imageCharacteristic.notify(response, len);                     // sends freed stack
```
`errorResponse` is declared inside the `else` block; every later use of `response`
(logging, `send_wifi_lan_frame`, `notify`) dereferences a pointer to a stack object whose
lifetime has ended. Independently flagged by cppcheck (`invalidLifetime`, 8 sites).
**Fix:** declare the error buffer in the function's outer scope (or `static`), so it outlives the block.

### H2. Duplicate `ButtonState` definition with different layout → out-of-bounds writes — correctness
`src/main.h:132-145` vs `src/device_control.cpp:41-55`
`main.h` owns the storage `ButtonState buttonStates[MAX_BUTTONS]` with a struct ending at
`bool inverted;`. `device_control.cpp` re-declares `ButtonState` with two extra fields
(`bool power_off; uint16_t power_off_hold_ms;`) and accesses the same array via
`extern ButtonState buttonStates[MAX_BUTTONS];`. The two translation units disagree on
`sizeof(ButtonState)` (9 vs 12 bytes after alignment). `initButtons()` writes all 32 elements
using the larger stride, so high indices land past the real array and clobber adjacent `.bss`
globals (`buttonStateCount`, `buttonEventPending`, …). ODR violation + live memory corruption.
Independently flagged by cppcheck (`ctuOneDefinitionRuleViolation`).
**Fix:** define `ButtonState` once in a shared header (with the two extra fields) and include it in both TUs.

### H3. Row buffer overflow on 4bpp panels wider than 1360 px — correctness
`src/boot_screen.cpp:574, 879, 914` · buffer `staticRowBuffer[680]` at `src/main.h:156`
For the supported `PANEL_IC_SEEED_ED103TC2_1872X1404_4GRAY` panel, `getBitsPerPixel()` returns 4
(`display_service.cpp:1191-1195`) and `pitch = w / 2 = 936`, but the boot render writes into the
680-byte `staticRowBuffer`. `memset(row, ..., pitch)` overruns by 256 bytes **per row**, the
per-pixel guards test `bytePos < pitch` (not buffer size), and `seeed_gfx_boot_write_row` then
reads 936 bytes back out. Corrupts neighbouring globals on every boot render of that panel.
The comment at `main.h:155` (“Maximum pitch: 1360px / 2 = 680”) predates the 1872-wide panel.
**Fix:** size `staticRowBuffer` for the worst-case pitch (≥936 + gray4 plane scratch), or
`return false` when `pitch > sizeof(staticRowBuffer)`.

### H4. AES-CCM nonce reuse across command/response directions — correctness (crypto)
`src/encryption.cpp:158-168` (`getCurrentNonce`), `:604` (reset), `:688-690` (`encryptResponse`)
The nonce is `session_id[8] || counter[8]` with **no direction/domain-separation bit**. On a new
session both `nonce_counter` (responses) and the inbound command counter start at 0. Command #0
and response #0 therefore use an identical `(key, nonce)` pair under the same `session_key`.
AES-CCM nonce reuse leaks the keystream XOR of the two messages and breaks tag unforgeability.
**Fix:** reserve one nonce bit as a direction flag (set for server→client), require it clear on inbound.

### H5. Replay of the most-recent command is always accepted — correctness (crypto)
`src/encryption.cpp:136`
```cpp
if (nonce_counter <= encryptionSession.last_seen_counter && counter_diff != 0) { ...dedup... }
```
The `&& counter_diff != 0` guard means a packet whose counter *equals* `last_seen_counter`
skips the “already seen” scan and is accepted. Since `last_seen_counter` only advances on a strict
increase, the latest legitimate command (and the initial counter-0 command) can be captured and
replayed indefinitely.
**Fix:** drop the `&& counter_diff != 0` guard; treat an exact match as a replay unless it is a genuine new maximum.

### H6. Busy-spin at full clock during the deep-sleep wake advertising window — perf
`src/main.cpp:85-105`
When `woke_from_deep_sleep && advertising_timeout_active`, the branch ends in a bare `return;`
with no delay/sleep, so `loop()` re-enters immediately and spins the CPU at full clock for up to
`sleep_timeout_ms` (default 10 s) on **every** deep-sleep wake — the most battery-critical path on
the device.
**Fix:** `delay(50)` (or `esp_light_sleep_start()`) before the `return`; tens of ms of connection-detection latency is irrelevant here.

### H7. Abandoned partial write leaves the EPD rail powered indefinitely — correctness (battery)
`src/display_service.cpp:1974-1985` (`partial_prepare_panel_ram` → `pwrmgm(true)`)
A partial-write start (0x76) powers the panel/PMIC, but `partialCtx` has **no timeout**. The
main-loop watchdog (`main.cpp:137-143`) and the ESP32 disconnect cleanup
(`esp32_ble_callbacks.h:50-55`) only look at `directWriteActive`, never `partialCtx.active`. If the
host disconnects or stalls mid-partial-stream, the rail stays powered until reboot — a battery killer.
**Fix:** give `partialCtx` a start timestamp checked by the same watchdog, and call
`cleanup_partial_write_state()` from the disconnect path.

### H8. Unbalanced touch-suspend counter permanently disables touch — correctness
`src/touch_input.cpp:112-116` (saturating suspend counter), gate at `:552`
`touchSuspendForEpdRefresh()` increments a counter that `processTouchInput()` gates on; several
error paths suspend without a matching resume, so touch stays dead until reboot:
- `display_service.cpp:1337-1342` — compressed-data zlib error cleanup (no resume)
- `display_service.cpp:1477-1482` — start inline-data zlib error (no resume)
- `display_service.cpp:1652-1657` — end final-flush zlib error (no resume)
- `display_service.cpp:1414-1418` — re-sent start suspends again (1→2); single resume at `:1708` leaves 1
- `display_service.cpp:1491` — partial-start aborting an active direct write never resumes it
- `main.cpp:141` — 15-min direct-write timeout cleanup (no resume)
- `display_service.cpp:1156-1170` — boot retry suspends twice, resumes once

**Fix:** move `touchResumeAfterEpdRefresh()` into `cleanupDirectWriteState()` (paired 1:1 with the
suspend in `handleDirectWriteStart`), or replace the counter with a boolean owned by the state machine.

### H9. Blocking, unbounded buzzer playback stalls BLE/loop — perf + correctness
`src/buzzer_control.cpp:72-77` (`delayMicroseconds` bit-bang loop), `:147-163` (nested playback)
`handleBuzzerActivate()` plays `outer (≤255) × pattern_count (≤255) × nsteps (≤255)` steps of up
to `255×5 = 1275 ms` each, all via blocking `delayMicroseconds`. A hostile/buggy packet can block
for hours; nothing else runs (no touch, buttons, BLE drain). On nRF the handler runs in the
Bluefruit write-callback, so a long buzz drops the BLE connection; on ESP32 it starves the loop
task (watchdog risk).
**Fix:** cap total playback duration (a few seconds); convert to a non-blocking state machine like
the LED one, or use hardware PWM (`tone()`/LEDC on ESP32, PWM peripheral on nRF).

### H10. `led_run_step()` can hang the device on infinite-repeat/zero-delay configs — correctness
`src/device_control.cpp:252-357` (esp. 259, 282, 309, 337)
The `for(;;)` state machine only `return`s when it schedules a delay or finishes. With
`grouprepeats == 255` (infinite) and all delays zero, every phase transition falls through with no
`return` → infinite loop, hard-hanging the firmware (WDT reset on ESP32, dead on nRF). Even finite
configs block for minutes (`flashLed()` busy-waits ~11 ms/call × hundreds of flashes).
**Fix:** execute at most one flash per `led_run_step()` call (return after each `flashLed()`,
scheduling a minimal delay even when the configured delay is 0).

### H11. Full blocking sensor sweep on every advertising update (incl. touch-drag / button press) — perf
`src/sensor_sht40.cpp:79,118-156` · `src/sensor_bq27220.cpp:141-181` · `src/display_service.cpp:1247-1250`
`updatemsdata()` polls SHT40 (`delay(12 ms)` per attempt, up to ~80-100 ms with the retry/bus-reinit
fallback), BQ27220 (2 I2C txns), and the battery ADC (`readBatteryVoltage()` = 10×`analogRead` +
`delay(2)` ≈ 20 ms). It is called from `processButtonEvents()` (`device_control.cpp:446`), from
`processTouchInput()` on every coordinate change (`touch_input.cpp:698`, up to 10 Hz), and from the
connect callback — so dragging a finger spends a large fraction of CPU time blocked in sensor delays.
**Fix:** cache SHT40/BQ27220/ADC readings with a 30-60 s TTL inside the poll functions; event-driven
`updatemsdata()` calls should only repack cached bytes.

---

## Medium severity

### M1. Cross-task BLE command queue indices are not `volatile`/atomic — correctness
`src/main.h:374-381` · `src/esp32_ble_callbacks.h:83-88` · `src/main.cpp:106-112`
`onWrite()` (BLE host task) does `memcpy(...); commandQueueHead = nextHead;` while `loop()` polls
`commandQueueTail != commandQueueHead`. `commandQueueHead/Tail` are plain non-`volatile` globals:
the compiler may cache them across iterations and may reorder the payload copy relative to the head
publish (no barrier) → missed or torn commands. (The pattern already exists correctly:
`bleRestartAdvertisingPending` is `volatile`.)
**Fix:** make head/tail `volatile` at minimum, or use `std::atomic<uint8_t>` with release/acquire, or a FreeRTOS queue.

### M2. Display teardown / sensor polling executed in BLE-task context — correctness
`src/esp32_ble_callbacks.h:52-55` (`onDisconnect` → `cleanupDirectWriteState(true)`),
`:44` (`onConnect` → `updatemsdata()`); nRF watchdog `main.cpp:141`; partial-refresh race below.
`cleanupDirectWriteState` runs `bbepSleep`/`delay(200)`/`pwrmgm(false)` → `SPI.end()`/`Wire.end()`
from the BLE task, racing `loop()` which may be mid-`bbepWriteData` on the same SPI bus (the
`epdRefreshInProgress` guard covers only the refresh phase, not the upload phase). `updatemsdata()`
from `onConnect` races `Wire`/statics with the loop copy. Blocks the BLE stack for hundreds of ms.
**Fix:** set a `*_pending` flag in the callback and do the work in `loop()` (mirror `bleRestartAdvertisingPending`).

### M3. Partial-refresh path never sets `epdRefreshInProgress` — correctness
`src/display_service.cpp:1618-1648`, `partial_write_to_panel:1987-2006`
Runs a blocking 60 s `waitforrefresh` without setting `epdRefreshInProgress`, unlike the direct
path (`:1688/1705`). Consumers that gate on it — disconnect-cleanup deferral
(`esp32_ble_callbacks.h:50`), advertising-restart gate (`ble_init.cpp:174`), keep-awake
(`main.cpp:171`) — all see “no refresh running” during a partial refresh.
**Fix:** set/clear `epdRefreshInProgress` around the partial-refresh wait too.

### M4. BWR/BWY second bitplane unreachable; direct-write commits garbage plane — correctness
`src/display_service.cpp:1425,1431` set bitplane mode and one-plane byte count; `directWritePlane2`
is never set true (only cleared at `:1389/1426`), and `handleDirectWriteData:1609` auto-fires
`handleDirectWriteEnd` once plane 0 completes. The red/yellow plane can never be uploaded; whatever
was left in PLANE_1 RAM is refreshed.
**Fix:** implement the plane switch (as GRAY4 does at `:1365-1383`), or reject BWR/BWY direct writes explicitly.

### M5. Config reload of a larger blob fails until reboot (`static` length) — correctness
`src/config_parser.cpp:237`
```cpp
static uint32_t configLen = MAX_CONFIG_SIZE;   // initialized once, then overwritten to prev size
```
`loadConfig()` writes `*len = config.data_len` on success, so the next `loadGlobalConfig()` call
starts with `configLen` = the previous config’s size. `loadGlobalConfig()` runs at runtime after
every BLE config write (`communication.cpp` `reloadConfigAfterSave()`); writing a config larger
than the last one fails ("Config data larger than buffer") — saved to flash but unusable until reboot.
**Fix:** make it non-`static`, or reset `configLen = MAX_CONFIG_SIZE;` at the top of `loadGlobalConfig()`.

### M6. TLV parser ignores the per-packet wire length field — correctness
`src/config_parser.cpp:253` (`offset++` discards the length byte), advancing by hardcoded
`sizeof(struct X)` per case; unknown-ID `default` at `:568-571` sets `offset = configLen - 2`,
dropping the rest of the config; trailing CRC check at `:574-582` is effectively unreachable and
non-fatal. A toolbox whose packet size differs from the firmware struct (schema drift) silently
desyncs and interprets payload bytes as packet IDs — garbage accepted into `globalConfig`
(including `securityConfig.encryption_key`). All reads stay in-bounds, so it’s corruption of meaning, not memory.
**Fix:** read + validate the length byte (`offset + pktLen <= configLen - 2`), copy
`min(pktLen, sizeof(struct))`, advance by `pktLen`; use it to skip unknown IDs; always verify the trailing CRC and fail on mismatch.

### M7. Byte-at-a-time zlib decode dispatch in the image hot path — perf
`lib/uzlib/src/od_zlib_stream.c:480-482` — `process_block_data` does `return 1;` after **every**
literal byte, so `od_zlib_stream_poll` re-dispatches through two `switch` statements per output
byte (the match-copy case at `:523-534` already loops correctly). `:313-321` — `adler_update_byte`
does `s2 %= 65521u` (a divide) per byte instead of deferring the modulo up to 5552 bytes.
For a 48 KB mono frame that is ~50k redundant dispatches + ~100k divides; ~27× that on the 1.3 MB
Seeed 4-gray frame. Biggest single CPU win for image transfer.
**Fix:** loop over literals in place until `capacity`; batch the adler32 modulo.

### M8. 256-byte decompression chunk fragments panel SPI writes — perf
`src/display_service.h:7` (`OPENDISPLAY_DECOMPRESSION_CHUNK_SIZE 256`), loop at `display_service.cpp:1877-1912`
Every 256 decompressed bytes pays a full `bbepWriteData` transaction (CS assert, DC, SPI setup):
188 transactions for an 800×480 mono frame, ~5100 for the 1.3 MB Seeed frame.
**Fix:** raise the chunk to 1-4 KB (RAM permitting) to amortize fixed per-transaction overhead.

### M9. nRF `updatemsdata()` tears down + restarts advertising every cycle — perf
`src/display_service.cpp:1276-1285`
The nRF path unconditionally does `clearData()/addData()/stop()/start()` every loop wake, even when
the 16-byte MSD payload is unchanged; `setFastTimeout(1)` means each restart burns a fast-advertising
burst. The ESP32 path right below (`:1287-1294`) already short-circuits with
`memcmp(prev_msd_payload, msd_payload, 16)`.
**Fix:** hoist the `memcmp` unchanged-payload early-out above the `#ifdef TARGET_NRF` block.

### M10. nRF advertising “boost” on button press never actually engages — correctness/perf
`src/ble_init.cpp:54-84` · `src/device_control.cpp:446-449`
The handler calls `updatemsdata()` (which restarts advertising at the *normal* interval because the
boost deadline isn’t set yet) *before* `ble_nrf_boost_advertising()` (which only sets the deadline);
`ble_nrf_advertising_tick()` merely `return`s while boosting without applying the interval. Net: the
20-30 ms boost interval only applies if a second event fires within the window.
**Fix:** call `ble_nrf_boost_advertising()` before `updatemsdata()`, or have `tick()` apply the boost interval + restart advertising when the boost begins.

### M11. `setup()` blocks up to ~34 s on WiFi connect — perf
`src/wifi_service.cpp:119-146` via `main.cpp:74`
`initWiFi()` defaults to `waitForConnection = true`: 3 retries × 10 s of `delay(500)` polling +
`delay(2000)` between retries, stalling boot while BLE advertises but queued commands go unprocessed.
`handleWiFiServer()` already handles late association asynchronously.
**Fix:** call `initWiFi(false)` from `setup()`.

### M12. WiFi radio started on every deep-sleep wake but never serviced — perf
`src/main.cpp:252` (`minimalSetup()` → `initWiFi(false)`)
The advertising-window branch (`main.cpp:85-105`) returns before `handleWiFiServer()` is reached,
so the STA radio draws power for the whole ≤10 s window with no function unless a BLE central connects.
**Fix:** skip WiFi init in the minimal wake path, or service `handleWiFiServer()` inside the advertising branch.

### M13. `String` heap churn / hex-dump logging in receive/ack + summary hot paths — perf
`src/communication.cpp:187-213` (per-response String build + per-byte hex loop, incl. every 0x0071
data-chunk ack), `:474`, `:513-523`; `src/config_parser.cpp:32` (`writeSerial(String)` **by value**)
and `printConfigSummary()` (~150+ temporary String concatenations, reprinted after every config write);
`src/main.cpp:220` (`"Loop end: " + String(...)` **every** nRF loop iteration), `:107-122`,
`esp32_ble_callbacks.h:71-82`. Under `DISABLE_USB_SERIAL` the `writeSerial` body is empty but the
argument Strings are still built — pure heap churn/fragmentation on a long-lived BLE process.
**Fix:** a `LOG(...)` macro that compiles to nothing when disabled; take `const String&`/`const char*`
in `writeSerial`; prefer `snprintf` into a stack buffer; delete/rate-limit per-iteration and per-packet lines.

### M14. Redundant full-payload memcpy chain on every encrypted packet — perf
`src/encryption.cpp:663-665` decrypt → `decrypted_with_length[512]`, `:672` copy → `plaintext`,
then `src/communication.cpp:535` copy → `decrypted_data + 2`. Two of the three copies are avoidable
on the per-packet path.
**Fix:** decrypt directly into the final `decrypted_data` region; drop the intermediate `plaintext` hop.

### M15. `sendResponse` length parameter is `uint8_t`, truncates encrypted length — correctness
`src/communication.cpp:159` `void sendResponse(uint8_t* response, uint8_t len)` vs `:169-175`
`uint16_t encrypted_len; ... len = encrypted_len;`. `encryptResponse` can return up to 284 bytes,
which truncates into the `uint8_t len` (→28). Latent today (no plaintext response exceeds ~100 B)
but a silent-corruption landmine; the `encrypted_response[600]` buffer implies large responses were intended.
**Fix:** widen `sendResponse`/`sendResponseUnencrypted` `len` to `uint16_t`.

### M16. Config file fully read + CRC’d twice on every boot — perf
`src/config_parser.cpp:778-820` + `src/factory_config.cpp:48`
`full_config_init()` → `tryProvisionFactoryEmbed()` → `hasValidStoredConfig()` → `loadConfig()`
(full flash read + bitwise CRC), then `loadGlobalConfig()` → `loadConfig()` again.
**Fix:** have the provision check reuse the already-loaded blob, or drop the pre-check and provision on `loadGlobalConfig` failure.

### M17. ~16.4 KB of RAM pinned in `static` config buffers, half redundant — perf
`src/config_parser.cpp:85,154,206,236` — two `config_storage_t` statics (4112 B each) +
two `MAX_CONFIG_SIZE` buffers (~6.4 % of the nRF52840’s 256 KB, reserved forever for occasional config work).
**Fix:** stream the payload directly into the caller’s buffer in `loadConfig` (removes one 4 KB static +
the byte-copy loop at `:189-191`); share one scratch buffer between `hasValidStoredConfig` and `loadGlobalConfig`.

### M18. Button event slot race loses ISR events — correctness
`src/device_control.cpp:426-429`
`buttonEventPending = false; changedButtonIndex = lastChangedButtonIndex; lastChangedButtonIndex = 0xFF;`
— if the ISR fires between the read and the `= 0xFF` store, the fresh index is overwritten with
0xFF while `buttonEventPending` stays true → next pass processes 0xFF (skipped) and the press is
lost. The single event slot (plus the `break` in `buttonISRGeneric` at `:537`) also drops one of two
near-simultaneous button changes.
**Fix:** guard read-and-clear with `noInterrupts()/interrupts()` (or use a per-button pending bitmask); remove the `break`.

### M19. `flashLed()` writes to unconfigured (0xFF) pins — correctness
`src/device_control.cpp:474-500`
Unlike `led_all_off()` (`:173-181`), `flashLed()` calls `digitalWrite(ledRedPin, …)` without the
`!= 0xFF` guard. On nRF this indexes `g_ADigitalPinMap[255]` out of bounds; on ESP32 it logs an
error per write, 7×16 times per flash.
**Fix:** mirror the 0xFF guards from `led_all_off()`.

### M20. Every EPD refresh blocks ~500-700 ms re-initializing the GT911 touch controller — perf
`src/touch_input.cpp:313-336,402`
`touchResumeAfterEpdRefresh()` → `delay(200)` + `touch_reinit_gt911()` → `gt911_resolve_and_init()`
adds `delay(300)` + `delay(200)` per controller (up to ~1 s in the auto-address path) after every
refresh, stalling buttons/BLE.
**Fix:** try a lightweight PID probe at the last-known address first (the `initTouchInput()`
“kept post-EPD” path at `:487-505` already does this); fall back to full reset only on probe failure;
replace fixed settle delays with a non-blocking deadline.

### M21. Non-atomic RMW of touch/WiFi shared flags — correctness
`src/touch_input.cpp:588` (also `:107`) — `s_touch_irq_mask &= ~(1u<<i);` in main context is a
load/AND/store; an ISR setting another controller’s bit in between gets erased (edge lost; timed-poll
fallback recovers, so latency not permanent loss). Same class of issue as M18.
**Fix:** wrap the clear in a critical section.

### M22. Second full-frame render pass duplicates all per-pixel work — perf
`src/boot_screen.cpp:889-931`
`planePasses = 2` for 4-gray split and BWR/BWY. For `gray4Split`, pass 2 re-runs the entire
text/QR/logo pixel pipeline to produce a byte-identical row (only the extracted bit differs). For
BWR/BWY the color-plane pass scans **all** w×h pixels (with the per-pixel rotation transform) though
only the footer swatch band can set bits.
**Fix:** in the color pass, skip rows outside `[swatchY0, swatchY1)` (whole-row for rot 0/2, x-clamp
for 1/3); for gray4Split, cache pass-1 packed rows and replay in pass 2.

---

## Low severity

- **L1. `pinToButtonIndex[64] = {0xFF}` initializes only element 0** — correctness (latent).
  `src/main.cpp:243`. Aggregate init zero-fills 1-63, so every pin but 0 reads as button index 0.
  Currently only a dead `extern` (`device_control.cpp:35`). Fix: `memset(..., 0xFF, ...)` or delete.

- **L2. nRF boost deadline uses wraparound-unsafe absolute comparison** — correctness.
  `src/ble_init.cpp:55,59,69`. `millis() < s_nrf_adv_boost_until` and `!= 0` sentinel break near the
  49.7-day wrap. Fix: store start time, compare `millis() - start < NRF_ADV_BOOST_MS` (the idiom the rest of main.cpp uses).

- **L3. `clearStoredConfig()` leaves stale secrets in RAM** — correctness.
  `src/config_parser.cpp:141`. Zeroes `globalConfig` but not `securityConfig` (AES key), wifi creds,
  or `wifiConfigured`; after the BLE clear-config command the old key/creds stay active until reboot.
  Fix: also `memset(&securityConfig, 0, …)` and reset the wifi globals.

- **L4. 3 malformed packets tear down a live session (DoS)** — correctness.
  `src/encryption.cpp:641-645,678-682`. Any unauthenticated writer can send 3 bad-tag packets to force
  `clearEncryptionSession()`. Fix: rate-limit, or only count failures that pass the replay window.

- **L5. Per-partial-write NACK unconditionally wipes `displayed_etag`** — correctness.
  `src/display_service.cpp:2017-2018`. Even pure pre-validation rejects (flags/OOB/align/etag-mismatch,
  `:1507-1537`), where panel RAM was untouched, force the host into a full-image rewrite. Fix: only
  clear the etag when panel RAM was actually modified.

- **L6. `waitforrefresh` false-fails on sub-100 ms refreshes** — correctness.
  `src/display_service.cpp:316-320`. If the panel is already not-busy on the first poll it returns
  `false`, clearing `displayed_etag` and sending a timeout response. Fix: sample busy immediately after issuing the refresh.

- **L7. security_config bound off by two lets it consume CRC bytes** — correctness.
  `src/config_parser.cpp:526` checks `<= configLen` where every other packet uses `configLen - 2`.
  No OOB, but a truncated packet is accepted. Fix: use `configLen - 2`.

- **L8. Stored-config `version` and declared inner length never validated** — correctness.
  `src/config_parser.cpp:80-90,163-172,248`. `loadConfig` checks only magic + `data_len`; the payload’s
  declared total length (bytes 0-1) is skipped without comparison. Fix: reject `version != 1` and mismatched declared length.

- **L9. `initAXP2101` hardcodes GPIO 21 and ignores its `busId`** — correctness.
  `src/display_service.cpp:625-637`. Toggles a fixed pin on every `pwrmgm(true)` (i.e. every image
  upload) regardless of board config; `busId` is validated but never used to select the Wire bus. Fix: use the configured pin/bus.

- **L10. QR capacity check ignores 12-bit mode/count header; padding count can underflow** — correctness (latent).
  `src/qr/qrcode.c:331,339-340`. `length == dataCapacity` overflows; padding wraps (clamped to 4),
  producing a malformed QR instead of an error. Not reachable from the boot screen. Fix: include header bits in the capacity check.

- **L11. `adler_update_byte` divides per byte** — perf. (folded into M7; noted separately for the CRC-style path.)

- **L12. Bitwise config CRC over up to 4 KB, multiple times per boot/write** — perf.
  `src/config_parser.cpp:211-224`. A 256-entry table (1 KB flash) cuts it ~8×. Minor at these sizes.

- **L13. Per-byte copy loop where `memcpy` suffices** — perf. `src/config_parser.cpp:189-191`.

- **L14. `saveConfig` open-failure retry re-issues the identical `open()`; `file.flush()` on a read handle** — perf/dead code.
  `src/config_parser.cpp:104-115,174`. Both no-ops to delete.

- **L15. Fixed `delay()`s throttle transfer/BLE housekeeping** — perf.
  `src/communication.cpp:214` (`delay(20)` after **every** nRF notify, incl. every data-chunk ack),
  `:348` (`delay(50)` per config-read chunk); `src/ble_init.cpp:179` (`delay(100)`),
  `src/display_service.cpp:1319` (`delay(50)`). Fix: connection-interval-aware flow control / non-blocking waits.

- **L16. Pointless `nonce_copy` bounce in `encryptResponse`** — perf. `src/encryption.cpp:703-707`.
  `memcpy(ciphertext + 2, nonce, 16)` directly.

- **L17. Per-pixel rotation transform + redundant swatch divisions in boot render** — perf.
  `src/boot_screen.cpp:918-923` (rotation `switch` per pixel; one axis is row-constant — hoist it),
  `:287,927-928,935,958` (`bootSwatchIndex`/border recomputed 2-3 divisions per pixel — compute once).
  Complements the bigger per-pixel-predicate cost noted under boot render below.

- **L18. Boot-screen glyph lookup is a linear scan of a 94-entry font table, per pixel** — perf.
  `src/boot_screen.cpp:84-89,227-233,913-961`. For each pixel in a text band, `bootTextPixelBlack`
  walks the string char-by-char and `bootGlyph` linearly scans `BOOT_FONT5X7`. Fix: direct-index LUT
  (`c - ' '`), hoist per-row band detection, render glyph columns as horizontal runs.

- **L19. ~30 ms of blocking `delay()` in footer battery telemetry, every boot** — perf.
  `src/boot_screen.cpp:706` → `readBatteryVoltage()` (`display_service.cpp:1218-1224`, `delay(10)` + 10×`delay(2)`).
  Fix: fewer/faster samples, or sample while the render loop runs.

- **L20. ~1.3 KB of VLA stack during QR generation** — perf. `src/qr/qrcode.c:275,277,348` + `codewordBytes[407]`
  + caller `qrBuf[256]`. Version is fixed at 6; replace VLAs with worst-case fixed arrays or static scratch.

- **L21. `poll_interval_ms` below 100 ms is silently ineffective** — correctness/doc.
  `src/touch_input.cpp:37,556-568` gate at 100 ms (`TOUCH_PROCESS_MIN_INTERVAL_MS`) despite the
  structs.h comment saying default 25 ms. Fix: lower the gate to the min configured interval, or fix the doc.

- **L22. Working INT line still triggers a full I2C status read every 100 ms** — perf.
  `src/touch_input.cpp:574-586`. Timed-poll fallback negates the interrupt power benefit. Fix: stretch the fallback interval when IRQs are healthy.

- **L23. Dead render code / write-only state** — perf/cleanup.
  `renderChar_4BPP/_2BPP/_1BPP` never called and `writeTextAndFill` never defined
  (`display_service.cpp:1038-1111`, `.h:31`); `staticWhiteRow[680]`/`staticLineBuffer[256]`
  (`main.h:155,157`) reachable only by dead externs; `directWriteRefreshMode`/`directWriteDataKind`/
  `directWritePlane2` write-only (`main.h:187-188`). Delete or finish.

- **L24. Identical “full”/“reduced” boot line arrays; no-op ternaries** — cleanup.
  `src/boot_screen.cpp:723-730`. `bootLinesFull`/`bootLinesReduced` are identical and
  `useZoneLayout ? 5u : 5u` selects the same either way — a reduced small-screen layout that was never implemented.

- **L25. `button_id` math makes `instance_number` a no-op** — correctness.
  `src/device_control.cpp:580-581`. `(instance*8 + pinIdx) % 8 == pinIdx`, so buttons on different
  instances sharing a byte index collide. Fix: simplify to `pinIdx` if intended, else rethink the ID scheme.

- **L26. Per-pin `delay(10)` + `delay(50)` in `initButtons()`** — perf.
  `src/device_control.cpp:599,626`. Up to ~370 ms of boot time; a single settle delay after all `pinMode` calls suffices.

- **L27. Duplicated / overwritten preferred-connection-parameter setup** — perf/cleanup.
  `src/ble_init.cpp:249-253` (repeated at `display_service.cpp:1317-1318`). `setMinPreferred(0x06)` then
  `setMinPreferred(0x12)` overwrites the first (intent was min/max), and the pair is re-applied to the same advertising object.

- **L28. WiFi LAN frames up to 4096 B feed handlers that BLE caps at 512 B** — correctness (verify).
  `src/wifi_service.cpp:211-246` → `imageDataWritten`. Framing math is correct; confirm downstream handlers tolerate 4 KB inputs.

---

## Checked and found OK (non-findings)

- `millis()`/`micros()` comparisons in `main.cpp` (93,138,148,185,202), `device_control.cpp:365`,
  `power_latch.cpp:142`, `touch_input.cpp:557/581`, `buzzer_control.cpp:72` all use the wraparound-safe
  subtraction idiom (the one exception is L2).
- MAC/tag/session-id comparisons use `constantTimeCompare` (`encryption.cpp:225-231`, used at `:118,:575`) — timing-safe.
- Chunked config write is length-bounded (`communication.cpp:431`, `receivedSize + len > 4096`); attacker `totalSize` can’t overflow the 4096 buffer.
- ESP32 `onWrite` copy is bounded by `MAX_COMMAND_SIZE` (512) matching the queue buffer.
- `decryptCommand` payload-length validation (`encryption.cpp:667-668`) correctly bounds the `memcpy`.
- Config parser clamps all instance counts to array bounds (`config_parser.cpp:359-377`), so touch/sensor/button loops can’t overrun.
- `width*height` in the direct-write path uses `uint32_t` casts (`display_service.cpp:1430`); `calc_controller_plane_bytes` can’t overflow for uint16 dims.
- SHT40/BQ27220 value conversions (sign, clamping, CRC) are correct.

---

## Recommended priority order

1. **H1, H2, H3** — memory-corruption bugs (dangling pointer, ODR/array overrun, row-buffer overflow). Fix first.
2. **H4, H5** — crypto (nonce reuse, replay acceptance). Fix before any security claim.
3. **H6, H7, H11, H9, H10** — battery/hang: busy-spin wake, stuck panel rail, per-event sensor sweep, blocking buzzer, LED hang.
4. **H8, M1, M2, M18, M21** — races and stuck-state (touch counter, non-atomic queue/flags, cross-task teardown).
5. **M7, M8, M13, M14** — the image-transfer/decompress hot path (biggest steady-state CPU/battery wins).
6. Remaining medium/low items as cleanup.
