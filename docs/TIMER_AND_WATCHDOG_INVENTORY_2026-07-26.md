# Timer, Watchdog and Timeout Inventory — `Firmware`

**Date:** 2026-07-26
**Scope:** everything tracked in this repo — `src/`, `include/`, `platformio.ini`, `scripts/`.
**Explicitly out of scope:** `.pio/libdeps/**` (bb_epaper, NimBLE-Arduino, FastEPD, Adafruit
Bluefruit), `~/.platformio/packages/**`, the precompiled `sdkconfig.h`, mbedTLS, FreeRTOS/IDF,
and every sibling repo (`py-opendisplay`, …). Where firmware hands control to one of those, the
entry is recorded as **bounded by library — not analyzed (out of scope)** with the firmware-side
call site and the argument the firmware passes.

Written as the pre-implementation map for
[PLAN_FREEZE_PROOFING_2026-07-26.md](PLAN_FREEZE_PROOFING_2026-07-26.md) and its review,
[FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md](FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md).
Every claim below was verified against source; nothing is carried over from those documents
without re-checking. Corrections to them are in the final section.

**Target shorthand.** `nRF` = `TARGET_NRF` (nRF52840, Bluefruit; commands run inline on the
Bluefruit Callback task, no queues). `ESP32` = `TARGET_ESP32` (S3/C6/C3/classic; SPSC command
ring drained by `loop()`). `OPENDISPLAY_HAS_WIFI` is `TARGET_ESP32 && OPENDISPLAY_ENABLE_WIFI` —
`esp32-N4` is ESP32 **without** WiFi, `nrf52840custom` has neither.

---

## 1. Watchdog-related build flags and code in this repo

### 1.1 `-DCONFIG_FREERTOS_WATCHDOG_TIMEOUT_S=120`

- **Mechanism:** PlatformIO build flag, present in every ESP env.
- **file:line:** [platformio.ini:53](../platformio.ini), [:83](../platformio.ini),
  [:112](../platformio.ini), [:140](../platformio.ini), [:189](../platformio.ini),
  [:209](../platformio.ini), [:229](../platformio.ini), [:253](../platformio.ini),
  [:295](../platformio.ini). `esp32-s3-E1004` ([:156](../platformio.ini)) and
  `esp32-s3-N16R8-extuart-debug` ([:278](../platformio.ini)) inherit it through
  `${env:<parent>.build_flags}`, so **all ten ESP envs carry it**; `nrf52840custom` does not.
- **Target:** ESP32 only.
- **Duration / bound:** nominally 120 s.
- **What it monitors:** nothing. **The flag is inert.** Two independent reasons, both verifiable
  from inside this repo: (a) it is not an IDF 5.x symbol — the real one is
  `CONFIG_ESP_TASK_WDT_TIMEOUT_S` — and (b) the firmware never calls any WDT API, so nothing in
  this codebase reads it. Verified firmware-side: `grep -rn "esp_task_wdt\|rtc_wdt\|NRF_WDT" src/`
  returns **zero** hits.
- **Action on expiry:** none.
- **Context:** n/a.
- **Can it fire while connected / transferring / mid-refresh?** It cannot fire at all.

### 1.2 Firmware-armed watchdogs

**There are none, on either target.** No `esp_task_wdt_add/init/reset`, no `NRF_WDT`, no
`rtc_wdt_*` anywhere in `src/`. Whatever hardware watchdog the platform enables by default is
outside this repo's control and out of scope. The only "watchdogs" this firmware owns are the
`millis()` deadlines in §2.

### 1.3 Reset-reason handling (observability only)

- **Mechanism:** `resetReasonName()` + `esp_reset_reason()` logging at boot.
- **file:line:** [main.cpp:29-44](../src/main.cpp) (name table),
  [main.cpp:82-84](../src/main.cpp) (read + log).
- **Target:** ESP32 (`#ifdef TARGET_ESP32`).
- **Bound:** n/a — one-shot at boot.
- **What it monitors:** whether the previous boot ended in `PANIC` / `INT_WDT` / `TASK_WDT` /
  `WDT` / `BROWNOUT`. Purely a log line; nothing branches on it.
- **Action:** logs. The adjacent comment at [main.cpp:95-100](../src/main.cpp) records the
  hardware-proven consequence that a non-deep-sleep reset wipes RTC memory (so `deep_sleep_count`
  reads 0 and the boot screen redraws).
- **Context:** `setup()`.

---

## 2. `millis()`-based deadlines in `src/`

Every one, grouped by subsystem.

### 2.1 Direct-write 15-minute watchdog

- **file:line:** [main.cpp:436-442](../src/main.cpp); stamp set at
  [display_service.cpp:2077](../src/display_service.cpp) (`directWriteActivatePanel`), cleared at
  [display_service.cpp:2022](../src/display_service.cpp) (`cleanupDirectWriteState`).
- **Target:** ESP32 only — the block sits inside the `#ifdef TARGET_ESP32` region of `loop()`.
  **nRF has no equivalent**; its `loop()` body is the `#else` arm
  ([main.cpp:518-530](../src/main.cpp)).
- **Duration:** `900000UL`, an inline literal at [main.cpp:438](../src/main.cpp). No named
  constant.
- **Monitors:** wall-clock age of a direct-write session, measured from START. **Nothing
  refreshes the stamp** — it is a hard cap on the whole upload + refresh window, not an
  inactivity timer.
- **Action on expiry:** `od_log_error` + `cleanupDirectWriteState(true)` → clears all
  `directWrite*` state and force-powers the panel off.
- **Context:** loop task, once per pass.
- **Fires while connected?** Yes. **During a transfer?** That is its only purpose. **Mid-EPD
  refresh?** No — `directWriteFinishAndRefresh` blocks the loop task across
  `bbepRefresh`/`waitforrefresh` ([display_service.cpp:2415-2436](../src/display_service.cpp)),
  so the check simply does not run until the refresh returns.
- **Gap:** keys on `directWriteActive`. `sendPipeNack` clears that flag via
  `cleanupDirectWriteState(true)` ([display_service.cpp:2564-2578](../src/display_service.cpp))
  while leaving `pipeState.active = true` — after a fatal pipe NACK this watchdog no longer
  applies to the latched pipe.

### 2.2 `checkPartialWriteTimeout()` — partial-write 15-minute watchdog

- **file:line:** [display_service.cpp:578-587](../src/display_service.cpp); called from
  [main.cpp:443](../src/main.cpp). Stamp set at
  [display_service.cpp:2249](../src/display_service.cpp) (legacy `0x76` START) and
  [display_service.cpp:2762](../src/display_service.cpp) (pipe-partial START).
- **Target:** ESP32 only (same `#ifdef` region as §2.1).
- **Duration:** `900000UL`, inline literal at [display_service.cpp:580](../src/display_service.cpp).
- **Monitors:** `partialCtx.active` age from START. Again a start-stamp, never refreshed.
- **Action:** `cleanup_partial_write_state()`, plus `resetPipeWriteState()` when
  `pipeState.partial` — so a pipe-partial transfer *is* cleared here.
- **Context:** loop task, once per pass.
- **Fires while connected/transferring?** Yes. Mid-refresh: same answer as §2.1 — the loop task
  is inside the refresh, so the check is deferred, not suppressed.

### 2.3 `waitforrefresh()` — panel busy-wait bound

- **file:line:** [display_service.cpp:747-775](../src/display_service.cpp); the loop bound is
  [display_service.cpp:760](../src/display_service.cpp).
- **Target:** both.
- **Duration:** `timeout * 100` iterations of `delay(10)`. Every call site passes `60` →
  **60 s**: [display_service.cpp:541](../src/display_service.cpp) (boot),
  [:1592](../src/display_service.cpp) (FastEPD boot), [:2423](../src/display_service.cpp)
  (FastEPD direct refresh), [:2432](../src/display_service.cpp) (bb_epaper direct refresh),
  [:3241](../src/display_service.cpp) and [:3244](../src/display_service.cpp) (partial refresh).
- **Monitors:** the panel BUSY line via `bbepIsBusy()`.
- **Action:** `od_log_warn("Refresh timed out")`, returns `false`. Callers treat `false` as
  refresh failure → `epdSessionRelease(false)` powers the panel down.
- **Context:** whichever task ran the refresh — loop task on ESP32, Bluefruit Callback task on
  nRF for a client-driven refresh.
- **Two short-circuits that make the 60 s cap not apply:**
  1. **FastEPD (IT8951 / E1004 driver builds):**
     [display_service.cpp:749](../src/display_service.cpp) returns `fastepd_wait_refresh(timeout)`,
     which is a stub — [display_fastepd.cpp:228-231](../src/display_fastepd.cpp) does
     `(void)timeout_sec; return !s_init_failed;`. **No wait, no bound.** The actual blocking
     happens inside the library (`fastepd_full_update` / `fastepd_direct_refresh`) —
     *bounded by library — not analyzed (out of scope)*, firmware call sites
     [display_service.cpp:1592](../src/display_service.cpp) and
     [:2422](../src/display_service.cpp), no timeout argument passed.
  2. **E1004 panel:** [display_service.cpp:752-756](../src/display_service.cpp) returns `true`
     immediately when the panel is already idle, on the grounds that `bbepRefresh` already waited
     — *bounded by library — not analyzed (out of scope)*.
- The 60 s loop itself yields (`delay(10)`), which is why a long refresh does not trip whatever
  platform WDT exists.

### 2.4 EPD keep-alive (`pwrmgmOffDeadlineMs` / `epdSessionTick`)

- **file:line:** deadline armed [display_service.cpp:505](../src/display_service.cpp)
  (`epdSessionRelease`); expiry checked [display_service.cpp:520-527](../src/display_service.cpp)
  (`epdSessionTick`); window computed [display_service.cpp:383-395](../src/display_service.cpp).
- **Target:** both.
- **Duration — config-driven:** `power_option.screen_timeout_seconds`
  ([opendisplay_structs.h:504](../include/opendisplay_structs.h), `uint8_t`, documented `@min 0
  @max 30`), clamped to `EPD_KEEPALIVE_MAX_S = 30`
  ([display_service.h:16](../src/display_service.h)) at
  [display_service.cpp:393](../src/display_service.cpp). **0 = power off immediately** (also the
  factory/old-blob default). **Forced to 0 unconditionally when an AXP2101 PMIC is in the sensor
  list** ([display_service.cpp:385-391](../src/display_service.cpp)), logging once per call when
  it overrides a nonzero value.
- **Monitors:** how long a `PWR_WARM` (post-successful-refresh) panel keeps its rail up.
- **Action:** `epdSessionForceOffLocked()` — `bbepSleep` + `delay(50)` + rail cut, or
  `fastepd_direct_sleep()` on FastEPD builds.
- **Context:** loop task via [main.cpp:353](../src/main.cpp), **and** inside `idleDelay()` at
  [main.cpp:545](../src/main.cpp) so a long `idleDelay` still expires it.
- **Fires while connected?** Yes — WARM survives disconnect by design. **During a transfer?** No:
  the tick early-returns unless `pwrmgmState == PWR_WARM`
  ([display_service.cpp:521](../src/display_service.cpp)) and a transfer is `PWR_ACTIVE`, and it
  additionally `TryTake`s the lock and skips the pass if held. **Mid-refresh?** No, same reasons.
- On battery ESP32 the effective window is `min(configured window, idle-hold)`, because
  `enterDeepSleep()` calls `epdSessionForceOff()` unconditionally at
  [main.cpp:609](../src/main.cpp).

### 2.5 Encryption session timeout

- **file:line:** [encryption.cpp:221-232](../src/encryption.cpp)
  (`checkEncryptionSessionTimeout`), reached from `isAuthenticated()`
  [encryption.cpp:194-198](../src/encryption.cpp).
- **Target:** both.
- **Duration — config-driven:** `securityConfig.session_timeout_seconds`
  ([opendisplay_structs.h:916](../include/opendisplay_structs.h), `uint16_t` LE, documented
  `0 = no timeout (persists until disconnect)`). **0 short-circuits to "still valid"** at
  [encryption.cpp:223](../src/encryption.cpp) — there is no firmware fallback constant.
- **Monitors:** age since `session_start_time`, in whole seconds
  ([encryption.cpp:224-225](../src/encryption.cpp)).
- **Action:** `clearEncryptionSession()` + return false → the command that triggered the check is
  answered `RESP_AUTH_REQUIRED` ([communication.cpp:664-670](../src/communication.cpp)).
- **Context:** whichever task dispatches the command — loop task (ESP32) or Bluefruit Callback
  task (nRF). It is a *query with side effects*: `isAuthenticated()` mutates session state.
- **Fires while connected?** Yes, by construction. **During a transfer?** **Yes** — it is
  evaluated on every command dispatch, including every pipe DATA frame, so on a nonzero
  configured value it deterministically fires mid-upload once the transfer outlives the window.
  **Mid-refresh?** No — no commands dispatch while the loop task is inside the refresh.

### 2.6 Encryption auth challenge validity — 30 s

- **file:line:** stamped [encryption.cpp:588](../src/encryption.cpp)
  (`server_nonce_time = currentTime`), checked
  [encryption.cpp:604](../src/encryption.cpp).
- **Target:** both.
- **Duration:** `30000` ms, inline literal at [encryption.cpp:604](../src/encryption.cpp).
- **Monitors:** how long an issued `AUTH_STATUS_CHALLENGE` server nonce stays acceptable.
- **Action:** the 32-byte response arm rejects the stale challenge (client must re-request).
- **Context:** command dispatch task.
- **Fires mid-transfer?** Only during the authenticate handshake, which precedes any transfer.

### 2.7 LAN read / idle timeout — 30 s

- **file:line:** [wifi_service.cpp:957-960](../src/wifi_service.cpp). Stamps at
  [:891](../src/wifi_service.cpp) (accept), [:915](../src/wifi_service.cpp) (TLS handshake
  complete), [:951](../src/wifi_service.cpp) (bytes read), [:977](../src/wifi_service.cpp)
  (valid frame about to dispatch).
- **Target:** ESP32 with `OPENDISPLAY_HAS_WIFI` only.
- **Duration:** `OD_LAN_READ_TIMEOUT_S = 30u`, from the canonical protocol header
  [opendisplay_protocol.h:984](../include/opendisplay_protocol.h) — the **only** timeout in this
  firmware that comes from the wire protocol rather than being firmware-local.
- **Monitors:** silence on an accepted LAN TCP/TLS session. Note the guard is
  `else if (drainedBytes == 0)` — the check only runs on a tick where **nothing** was read.
- **Action:** `disconnectWiFiServer()`.
- **Context:** loop task, inside `handleWiFiServer()`.
- **Fires while a BLE client is connected?** Yes — the two transports are independent, and
  `disconnectWiFiServer()` raises `bleDisconnectCleanupPending`
  ([wifi_service.cpp:812](../src/wifi_service.cpp)), which is why the `ownerStillUp` guard in
  [main.cpp:328-338](../src/main.cpp) exists. **During a LAN transfer?** Only if the client has
  been silent 30 s. **Mid-refresh?** No — `handleWiFiServer()` runs on the loop task, which is
  inside the refresh.

### 2.8 `wifiClient.setTimeout(30000)`

- **file:line:** [wifi_service.cpp:888](../src/wifi_service.cpp).
- **Target:** ESP32 + WiFi.
- **Bound:** 30 000 ms passed to the Arduino `WiFiClient` — *bounded by library — not analyzed
  (out of scope)*. Recorded because the firmware chooses the value.

### 2.9 WiFi link supervisor poll — 10 s

- **file:line:** [main.cpp:448-464](../src/main.cpp).
- **Target:** ESP32 + WiFi.
- **Duration:** `10000` ms, inline literal at [main.cpp:449](../src/main.cpp).
- **Monitors:** `WiFi.status()` vs. the cached `wifiConnected` flag.
- **Action:** on loss → `disconnectWiFiServer()`; on regain → `restartWiFiLanAfterReconnect()`.
- **Context:** loop task.
- **Fires while a BLE client is connected / transferring?** Yes to both — and via
  `disconnectWiFiServer()` it raises the shared BLE-disconnect-cleanup flag.

### 2.10 Initial blocking WiFi connect — 3 × 10 s + 2 × 2 s

- **file:line:** [wifi_service.cpp:762-787](../src/wifi_service.cpp).
- **Target:** ESP32 + WiFi, and only on the `waitForConnection == true` path (setup).
- **Duration:** `maxRetries = 3` ([:762](../src/wifi_service.cpp)) ×
  `timeoutPerRetry = 10000` ms ([:763](../src/wifi_service.cpp)), with `delay(2000)` between
  attempts ([:786](../src/wifi_service.cpp)) → **worst case ~34 s of blocking**. Inner poll is
  `delay(500)` ([:769](../src/wifi_service.cpp)); early-aborts on `WL_CONNECT_FAILED` /
  `WL_NO_SSID_AVAIL`.
- **Action on expiry:** `wifiConnected = false`, log, continue booting.
- **Context:** whichever task called `initWiFi(true)`.

### 2.11 mDNS TXT MSD update throttle — 400 ms

- **file:line:** [wifi_service.cpp:377-383](../src/wifi_service.cpp).
- **Target:** ESP32 + WiFi.
- **Duration:** `400` ms, inline literal at [:378](../src/wifi_service.cpp).
- **Monitors:** re-publish rate of the `msd` TXT record when the payload is unchanged.
- **Action:** skip the update.

### 2.12 Deep-sleep idle hold

- **file:line:** [main.cpp:486-499](../src/main.cpp) (idle branch);
  [main.cpp:374-390](../src/main.cpp) (post-wake advertising branch). `lastActivityMs` is
  refreshed by `pollActivity()` [main.cpp:257](../src/main.cpp), at end of setup
  [main.cpp:175](../src/main.cpp), and on an aborted sleep [main.cpp:590](../src/main.cpp).
- **Target:** ESP32 only.
- **Duration — config-driven:** `power_option.sleep_timeout_ms`
  ([opendisplay_structs.h:490](../include/opendisplay_structs.h), `uint16_t` LE ms — so the
  maximum expressible hold is 65 535 ms). **Fallback when 0: `DEFAULT_IDLE_HOLD_MS = 10000`**
  ([main.h:322](../src/main.h)), applied at [main.cpp:488-490](../src/main.cpp) and
  [main.cpp:375-377](../src/main.cpp).
- **Monitors:** quiet time since the last detected activity.
- **Action:** `enterDeepSleep()`.
- **Context:** loop task. Gated on `power_mode == 1` **and** `deep_sleep_time_seconds > 0`.
- **Fires while connected?** No — `pollActivity()` treats `connCount > 0` as activity in itself
  ([main.cpp:254](../src/main.cpp)), so a connected client pins the device awake indefinitely.
  `enterDeepSleep()` re-checks `getConnectedCount() > 0` at [main.cpp:588](../src/main.cpp).
  **During a transfer with the link already dropped?** `workInFlight`
  ([main.cpp:474-479](../src/main.cpp)) does **not** include `transferActive()`, so a latched
  `pipeState.active` with `connCount == 0` does not block the idle branch. **Mid-refresh?** No —
  `epdRefreshInProgress` is in `workInFlight` ([main.cpp:478](../src/main.cpp)).

### 2.13 Minimum-wake hold

- **file:line:** `minWakeTimeMs()` [main.cpp:197-200](../src/main.cpp); `minWakeHoldActive()`
  [main.cpp:202-210](../src/main.cpp); armed at [main.cpp:162-163](../src/main.cpp) (button wake)
  and [main.cpp:171-172](../src/main.cpp) (first boot / hidden reset).
- **Target:** ESP32 only.
- **Duration — config-driven:** `power_option.min_wake_time_seconds`
  ([opendisplay_structs.h:503](../include/opendisplay_structs.h), `uint16_t` LE seconds).
  **Fallback when 0: `DEFAULT_MIN_WAKE_TIME_SECONDS = 120`** ([main.h:312](../src/main.h)).
- **Monitors:** a floor on awake time after a button wake or a first boot, layered *under* the
  idle hold — sleep requires both conditions.
- **Action on expiry:** clears `minWakeWindowActive`, logs, permits sleep.
- **Context:** loop task; also re-checked defensively inside `enterDeepSleep()`
  ([main.cpp:598-601](../src/main.cpp)), which the comment notes must stay ahead of the
  advertising stop.

### 2.14 Post-wake advertising window

- **file:line:** [main.cpp:374-390](../src/main.cpp); start stamp
  [main.cpp:158](../src/main.cpp).
- **Target:** ESP32 only.
- **Duration:** same source as §2.12 (`sleep_timeout_ms`, fallback `DEFAULT_IDLE_HOLD_MS`).
  Measured from `lastActivityMs`, not from `advertising_start_time` — a connect-then-drop re-arms
  the whole window. `advertising_start_time` is used only for the log line
  ([main.cpp:384](../src/main.cpp)).
- **Action:** `enterDeepSleep()`.
- **Context:** loop task, `woke_from_deep_sleep && advertising_timeout_active` branch; the branch
  `return`s every pass after `idleDelay(50)` ([main.cpp:396](../src/main.cpp)), so the command
  drain and everything below it are **not reached** until a client connects.

### 2.15 MSD refresh cadence — 60 s

- **file:line:** [main.cpp:509-513](../src/main.cpp).
- **Target:** ESP32 only. Note it sits in the **`else` (not-`workInFlight`) branch**, so it does
  not run while a client is connected or a refresh is in flight.
- **Duration:** `60000` ms, inline literal at [main.cpp:510](../src/main.cpp).
- **Action:** `updatemsdata()` — re-polls sensors and re-pushes the BLE advertisement.
- **Interlock:** `MyBLEServerCallbacks::onConnect` sets `msdUpdatePending`
  ([esp32_ble_callbacks.h:56](../src/esp32_ble_callbacks.h)) rather than calling `updatemsdata()`
  inline, precisely because this 60 s path also drives it from the loop task.

### 2.16 nRF advertising boost — 3 s

- **file:line:** constant [ble_init.cpp:44](../src/ble_init.cpp) (`NRF_ADV_BOOST_MS = 3000`);
  armed [:47](../src/ble_init.cpp); consulted [:51](../src/ble_init.cpp) and
  [:61](../src/ble_init.cpp); serviced by `ble_nrf_advertising_tick()`
  [:59-76](../src/ble_init.cpp).
- **Target:** nRF only.
- **Duration:** 3 000 ms. Boost interval 20–30 ms (`NRF_ADV_BOOST_MIN/MAX`,
  [:42-43](../src/ble_init.cpp)); steady interval 160–1000 ms
  (`NRF_ADV_INTERVAL_MIN/MAX`, [:40-41](../src/ble_init.cpp)).
- **Action on expiry:** restore the slow interval, `Advertising.stop()` + `start(0)`.
- **Context:** loop task via [main.cpp:526](../src/main.cpp), and inside `idleDelay()` at
  [main.cpp:540](../src/main.cpp).

### 2.17 nRF link-diagnostic one-shot

- **file:line:** [ble_init.cpp:136-145](../src/ble_init.cpp).
- **Target:** nRF only.
- **Duration:** `s_link_diag_timer.begin(500, …, /*repeating=*/false)`
  ([ble_init.cpp:141](../src/ble_init.cpp)); `reset()` restarts it from now
  ([:144](../src/ble_init.cpp)). **The literal is 500; the adjacent comment says "fires ~2.5 s
  later".** See §8.
- **Action:** logs negotiated PHY/MTU/DLE if still connected. Diagnostics only — no state change.
- **Context:** FreeRTOS timer task (not the Callback task).

### 2.18 Buzzer sequencing

- **file:line:** step deadline `s_buzzer.step_until_ms`, armed
  [buzzer_control.cpp:203](../src/buzzer_control.cpp) and
  [:209](../src/buzzer_control.cpp), consumed
  [buzzer_control.cpp:243](../src/buzzer_control.cpp); global cap checked
  [buzzer_control.cpp:168](../src/buzzer_control.cpp) against `play_start_ms`
  ([:341](../src/buzzer_control.cpp)).
- **Target:** both.
- **Durations:** `kBuzzerMaxTotalMs = 30000u`
  ([buzzer_control.cpp:17](../src/buzzer_control.cpp)) — a hard 30 s cap on any playback;
  `kBuzzerDurationUnitMs = 5u` ([:15](../src/buzzer_control.cpp)) — note duration units;
  `kBuzzerInterPatternGapMs = 20u` ([:16](../src/buzzer_control.cpp)).
  Remaining budget is recomputed per step at [:182](../src/buzzer_control.cpp), so no single note
  can overshoot the cap.
- **Action on expiry:** `buzzer_stop_internal()` — tone off, drive off, state zeroed.
- **Context:** `buzzerService()`, called from `loop()` at [main.cpp:354](../src/main.cpp),
  [:483](../src/main.cpp), [:516](../src/main.cpp), [:529](../src/main.cpp), and from
  `idleDelay()` [main.cpp:546](../src/main.cpp).
- **Fires while connected/transferring?** The cap is honest wall-clock and fires whenever
  `buzzerService()` runs — but during a blocking refresh or SPI stream `buzzerService()` does not
  run, so a tone can sound past 30 s until the loop task is released. Non-blocking design: the
  handler ACKs immediately after starting ([:347-348](../src/buzzer_control.cpp)).
- **Blocking exception:** `passiveBuzzerPowerOffAlert()` uses three raw `delay(80)`
  ([buzzer_control.cpp:369-373](../src/buzzer_control.cpp)) — ~240 ms of unconditional blocking,
  reached from the power-off hold path.

### 2.19 LED flash sequencing

- **file:line:** `led_schedule_delay_ms()` [device_control.cpp:356-363](../src/device_control.cpp);
  deadline consumed [device_control.cpp:529](../src/device_control.cpp) in `processLedFlash()`.
- **Target:** both.
- **Durations:** per-step, from the config's packed `LedFlashPattern`, scaled by
  `LED_DELAY_FACTOR_MS = 100u` and floored at `LED_MIN_STEP_DELAY_MS = 1u`
  ([device_control.cpp:279-280](../src/device_control.cpp)).
- **Bound:** **none.** There is no global cap equivalent to the buzzer's `kBuzzerMaxTotalMs`; a
  long loop count in the pattern runs to completion. See §5.8.
- **Context:** `processLedFlash()` from `loop()` [main.cpp:352](../src/main.cpp) and `idleDelay()`
  [main.cpp:544](../src/main.cpp).

### 2.20 Touch poll interval / I²C backoff

- **file:line:** global floor [touch_input.cpp:588-592](../src/touch_input.cpp); per-controller
  interval [touch_input.cpp:600](../src/touch_input.cpp); timed-poll comparisons
  [:612](../src/touch_input.cpp) and [:625](../src/touch_input.cpp); I²C-fail backoff
  [:609-611](../src/touch_input.cpp).
- **Target:** both (the `transferActive()` skip at [:584-586](../src/touch_input.cpp) is
  `#ifdef TARGET_ESP32` only).
- **Durations:** `TOUCH_PROCESS_MIN_INTERVAL_MS = 100`
  ([touch_input.cpp:38](../src/touch_input.cpp)) — a hard floor on how often
  `processTouchInput()` does any work at all; per-controller
  `TouchController.poll_interval_ms` ([opendisplay_structs.h:951](../include/opendisplay_structs.h),
  `uint8_t` ms) with **firmware fallback `TOUCH_PROCESS_MIN_INTERVAL_MS` (100)** when 0 — the
  header documents the default as 25 ms (see §8); `TOUCH_I2C_FAIL_BACKOFF_MS = 100`
  ([:37](../src/touch_input.cpp)) suppresses level-triggered INT-low reads after a failure.
- **Action:** skip this pass.
- **Context:** loop task and `idleDelay()`.
- **Fires mid-transfer?** On ESP32 the whole function early-returns while
  `transferActive() || s_epd_refresh_suspend > 0` ([:584-586](../src/touch_input.cpp)). On nRF
  there is no such gate, but the transfer runs on a different task.

### 2.21 GT911 reset/settle delays

- **file:line:** `GT911_PRE_RESET_DELAY_MS = 300` / `GT911_POST_RESET_SETTLE_MS = 200`
  ([touch_input.cpp:29-30](../src/touch_input.cpp)), used at
  [:317-321](../src/touch_input.cpp), [:331-333](../src/touch_input.cpp),
  [:337-339](../src/touch_input.cpp), [:429](../src/touch_input.cpp).
  The address-select pulse train at [:250-269](../src/touch_input.cpp) adds
  `delay(10)+delay(60)+delay(1)+delay(11)+delayMicroseconds(110)+delay(6)+delay(51)`.
- **Target:** both.
- **Bound:** fixed, unconditional blocking — a full re-init can block ~1 s.
- **Context:** whichever task calls `touchResumeAfterEpdRefresh()` / init, i.e. the loop task.

### 2.22 Power-off button hold — `power_latch`

- **file:line:** `POWER_OFF_HOLD_MS = 3000` ([power_latch.cpp:21](../src/power_latch.cpp));
  stamped [:139](../src/power_latch.cpp); compared [:142](../src/power_latch.cpp).
- **Target:** ESP32 (the whole file's active body is ESP32; the nRF build gets the stubs at
  [power_latch.cpp:196](../src/power_latch.cpp)).
- **Duration:** 3 000 ms, firmware constant. Not config-driven on this path.
- **Action:** `powerOff()` → latch release + `esp_deep_sleep_start()`.
- **Context:** `powerButtonPoll()`, called from `processButtonEvents()`
  ([device_control.cpp:587](../src/device_control.cpp)).
- **Fires while connected / transferring / mid-refresh?** `processButtonEvents()` runs from the
  loop task and `idleDelay()`, so it can fire while a client is connected and while a transfer is
  latched — but not while the loop task is inside a blocking refresh or SPI stream.

### 2.23 Power-off button hold — config-driven (`device_control`)

- **file:line:** stamped [device_control.cpp:75](../src/device_control.cpp); compared
  [device_control.cpp:78](../src/device_control.cpp).
- **Target:** both.
- **Duration — config-driven:** `BinaryInputs.power_off_hold_sec`
  ([opendisplay_structs.h:862](../include/opendisplay_structs.h), `uint8_t` seconds).
  **Fallback when 0: 3 000 ms**, computed at
  [device_control.cpp:746](../src/device_control.cpp) into `ButtonState.power_off_hold_ms`
  ([structs.h:181](../src/structs.h), `uint16_t` ms).
- **Action:** `passiveBuzzerPowerOffAlert()` (≈240 ms of blocking `delay`) then
  `powerLatchTriggerOff()`.
- **Context:** `processButtonEvents()` on the loop task.
- **Note:** this is a *second, independent* power-off-hold mechanism from §2.22, with a different
  default source. Both can be armed on the same device. See §6.

### 2.24 ADC-ladder button poll and press-count window

- **file:line:** [device_control.cpp:173-200](../src/device_control.cpp).
- **Target:** ESP32 only (`registerAdcLadder` is `#ifdef TARGET_ESP32`,
  [device_control.cpp:738-742](../src/device_control.cpp)).
- **Durations:** `ADC_LADDER_POLL_MS = 5` ([:96](../src/device_control.cpp)) — poll floor;
  `5000` ms inline at [:193](../src/device_control.cpp) — press-count reset window;
  `ADC_LADDER_DEBOUNCE = 3` ([:97](../src/device_control.cpp)) — consecutive equal samples
  required.
- **Context:** loop task via `processButtonEvents()`.

### 2.25 Sensor / battery read caches (TTL, not timeouts)

| Cache | file:line | TTL | Target |
|---|---|---|---|
| SHT40 MSD poll | [sensor_sht40.cpp:239](../src/sensor_sht40.cpp), used [:245](../src/sensor_sht40.cpp) | `kSht40MsdPollTtlMs = 30000u` | both |
| BQ27220 MSD poll | [sensor_bq27220.cpp:142](../src/sensor_bq27220.cpp), used [:151](../src/sensor_bq27220.cpp) | `kBq27220MsdPollTtlMs = 30000u` | both |
| Battery voltage | [display_service.cpp:1707](../src/display_service.cpp), used [:1712](../src/display_service.cpp) | `kBatteryVoltageTtlMs = 30000u` | both |

These suppress I²C/ADC work rather than bounding it. Action on expiry is "do the read". The
uncached battery read itself blocks `delay(10) + 10 × delay(2)` ≈ 30 ms
([display_service.cpp:1690-1698](../src/display_service.cpp)).

### 2.26 Transfer start stamps (inputs to §2.1/§2.2)

| Stamp | Set | Cleared | Read by |
|---|---|---|---|
| `directWriteStartTime` | [display_service.cpp:2077](../src/display_service.cpp) | [:2022](../src/display_service.cpp), [:2375](../src/display_service.cpp) | [main.cpp:437](../src/main.cpp) |
| `partialCtx.start_time` | [display_service.cpp:2249](../src/display_service.cpp), [:2762](../src/display_service.cpp) | `cleanup_partial_write_state()` | [display_service.cpp:580](../src/display_service.cpp) |
| `imgLogStartMs` | [display_service.cpp:1862](../src/display_service.cpp) | — | [:1901](../src/display_service.cpp) — throughput logging only, no deadline |
| `partial_prepare_panel_ram` `t0` | [display_service.cpp:3249](../src/display_service.cpp) | — | debug logging only, no deadline |

`pipeState` has **no timestamp field at all** ([structs.h:106-126](../src/structs.h)) — this is
the structural reason a latched pipe has no bound of its own.

---

## 3. Retry / attempt bounds acting as implicit timeouts

### 3.1 Authentication rate limit — 10 attempts per 60 s

- **file:line:** [encryption.cpp:568-578](../src/encryption.cpp).
- **Target:** both.
- **Bound:** `auth_attempts >= 10` within `timeSinceLastAuth < 60` seconds (both inline literals,
  [:570-571](../src/encryption.cpp)). The counter resets when a request arrives ≥60 s after the
  previous one ([:576](../src/encryption.cpp)).
- **Action:** respond `AUTH_STATUS_RATE_LIMIT`, return false. The link is **not** dropped.
- **Context:** command dispatch task.
- **Note:** the counter increments on *every* authenticate attempt including the challenge
  request ([:579](../src/encryption.cpp)), so a client that re-handshakes more than 10 times a
  minute rate-limits itself.

### 3.2 Integrity-failure threshold — 3

- **file:line:** [encryption.cpp:692-696](../src/encryption.cpp) (nonce/replay path) and
  [:729-733](../src/encryption.cpp) (CCM tag path); reset on success
  [:725](../src/encryption.cpp) and at session start [:657](../src/encryption.cpp).
- **Target:** both.
- **Bound:** 3 (inline literal, twice).
- **Action:** `clearEncryptionSession()`. The link stays up; every subsequent command is answered
  `RESP_AUTH_REQUIRED`, unencrypted, forever.
- **Fires mid-transfer?** **Yes — this is the primary field wedge.** Ordinary packet loss trips
  the nonce arm, which increments the same counter as genuine tamper evidence.

### 3.3 Nonce replay window — ±32, 64-entry ring

- **file:line:** window test [encryption.cpp:131](../src/encryption.cpp); ring scan
  [:137-141](../src/encryption.cpp); ring store [:152-154](../src/encryption.cpp); ring
  declaration `uint64_t replay_window[64]`
  ([encryption_state.h:21](../src/encryption_state.h)).
- **Target:** both.
- **Bound:** `counter_diff < -32 || counter_diff > 32` → reject. The ring index is a **function
  static** at [encryption.cpp:152](../src/encryption.cpp), so `clearEncryptionSession()` memsets
  the ring ([:217](../src/encryption.cpp)) but leaves the index — a real bug today.
- **Action:** reject the frame; via §3.2 three rejections kill the session.
- **Ordering hazard:** `last_seen_counter` and the ring are committed at
  [:149-154](../src/encryption.cpp) **before** `aes_ccm_decrypt` runs at
  [:714](../src/encryption.cpp).

### 3.4 nRF notify retry — 4 attempts × `delay(5)`

- **file:line:** [communication.cpp:345-349](../src/communication.cpp).
- **Target:** nRF only.
- **Bound:** up to 4 retries, 5 ms apart → **≤20 ms of blocking** per response on backpressure.
- **Action:** give up silently (no error path if all 5 attempts fail).
- **Context:** Bluefruit Callback task (or, per the inline-fallback hazard, the BLE task).

### 3.5 ESP32 BLE notify drain cap — 16 per call

- **file:line:** [main.cpp:279](../src/main.cpp) (`bleDrain < 16`).
- **Target:** ESP32.
- **Bound:** at most 16 notifies per `flushResponseQueueToBle()` call; also breaks early when
  `notify()` returns false (mbuf exhaustion), deliberately leaving the entry queued
  ([main.cpp:288-290](../src/main.cpp)).
- **Action:** return; the remainder drains next call. Called once per loop pass **and between
  every command in the drain** ([main.cpp:421](../src/main.cpp)) and between config-read chunks
  ([communication.cpp:473](../src/communication.cpp)).

### 3.6 Response ring — 10 slots, drop-newest

- **file:line:** `RESPONSE_QUEUE_SIZE 10` ([structs.h:83](../src/structs.h)); full check
  [communication.cpp:113-116](../src/communication.cpp).
- **Target:** ESP32.
- **Action on full:** `od_log_error("Response queue full, dropping response")` and drop **the
  newest**. Backlog warning at depth ≥2 ([:126](../src/communication.cpp)).
- **Flush:** drained to empty every pass when no central is connected
  ([main.cpp:307-312](../src/main.cpp)).

### 3.7 Command ring — 33 slots, 32 usable, drop-newest

- **file:line:** `COMMAND_QUEUE_SIZE 33` ([main.h:371](../src/main.h), mirrored
  [esp32_ble_callbacks.h:19](../src/esp32_ble_callbacks.h)); producer refuses at
  `nextHead == tail` ([esp32_ble_callbacks.h:122-129](../src/esp32_ble_callbacks.h)).
- **Target:** ESP32.
- **Bound:** usable capacity is `COMMAND_QUEUE_SIZE - 1 = 32`, not 33. `PIPE_MAX_W = 32`
  ([structs.h:51](../src/structs.h); 16 under `PIPE_SMALL_DRAM_WINDOW`,
  [structs.h:47](../src/structs.h) — `esp32-N4` only).
- **Action on full:** `od_log_error("Command queue full, dropping command")`, drop the newest.
- **Flush:** **never.** There is no code path anywhere that resets `commandQueueHead/Tail`;
  stale commands survive a disconnect.

### 3.8 Loop command drain cap — 33 iterations

- **file:line:** [main.cpp:406-423](../src/main.cpp) (`while (drained < COMMAND_QUEUE_SIZE)`).
- **Target:** ESP32.
- **Bound:** a count cap only — **no wall-clock cap**. Each iteration can block for the full
  duration of the command it dispatches (up to a 60 s refresh), so the drain's true worst case is
  unbounded in time.
- **Drain-loop trap:** `tail` is cached at [main.cpp:409](../src/main.cpp) and the incremented
  value stored at [:417](../src/main.cpp) *after* dispatch — a flush performed from handler
  context would be clobbered by that store.

### 3.9 Config chunked write

- **file:line:** START [communication.cpp:495-517](../src/communication.cpp); chunks
  [communication.cpp:541-581](../src/communication.cpp).
- **Target:** both.
- **Bounds:** `MAX_CONFIG_CHUNKS = 20u`, `CONFIG_CHUNK_SIZE = 200u`,
  `CONFIG_CHUNK_SIZE_WITH_PREFIX = 202u`
  ([opendisplay_protocol.h:882-884](../include/opendisplay_protocol.h)); enforced at
  [communication.cpp:558](../src/communication.cpp).
- **Action on violation:** `chunkedWriteState.active = false` + NACK.
- **Gap:** `chunkedWriteState.active` is cleared **only** on completion
  ([:574](../src/communication.cpp)), a malformed chunk ([:558](../src/communication.cpp)), or an
  auth failure ([:550](../src/communication.cpp)). **No timer.** A client that sends `0x0040`
  START and vanishes latches it forever — and it is not covered by §2.1 or §2.2.

### 3.10 Config read chunk cap

- **file:line:** [communication.cpp:447-448](../src/communication.cpp).
- **Bound:** `maxChunks = (MAX_CONFIG_SIZE + 93) / 94`, a derived compile-time bound guaranteeing
  the loop terminates regardless of payload arithmetic. `MAX_RESPONSE_DATA_SIZE = 100u`
  ([opendisplay_protocol.h:885](../include/opendisplay_protocol.h)).

### 3.11 Pipe ACK cadence and reorder bounds

- **file:line:** in-order cadence [display_service.cpp:2854](../src/display_service.cpp);
  gap/duplicate rate limit [:2877-2881](../src/display_service.cpp) and
  [:2888-2893](../src/display_service.cpp); reorder overflow guard
  [:2874](../src/display_service.cpp); over-size frame guard
  [:2817](../src/display_service.cpp); out-of-window NACK [:2896](../src/display_service.cpp).
- **Target:** both.
- **Bounds:** `pipeState.ack_every` (negotiated, `PIPE_MAX_N = 32` / 16 on `esp32-N4`,
  [structs.h:52](../src/structs.h)/[:48](../src/structs.h)); `PIPE_REORDER_SLOTS = 33` / 17
  ([structs.h:50](../src/structs.h)/[:46](../src/structs.h)); `PIPE_REORDER_SLOT_SIZE = 248`
  ([structs.h:54](../src/structs.h)); `PIPE_ACK_MASK_BITS` from the protocol header.
- **Action on violation:** `sendPipeNack()` → sets `pipeState.error = true` and calls
  `cleanupDirectWriteState(true)` ([display_service.cpp:2564-2578](../src/display_service.cpp)).
  **`pipeState.active` stays true**, so `transferActive()` latches and the §2.1 watchdog no
  longer applies. This is the "pipe fatal-NACK latch".
- All of these are *count* bounds. There is no time bound anywhere in the pipe state machine.

### 3.12 LAN drain byte budget

- **file:line:** [wifi_service.cpp:936-992](../src/wifi_service.cpp), loop condition
  [:992](../src/wifi_service.cpp).
- **Target:** ESP32 + WiFi.
- **Bound:** `drainedBytes < sizeof(tcpReceiveBuffer)` = **16 384 bytes** per `handleWiFiServer()`
  tick ([wifi_service.cpp:40](../src/wifi_service.cpp)). Frame length is separately bounded by
  `OD_LAN_MAX_PAYLOAD = 4094u`
  ([opendisplay_protocol.h:982](../include/opendisplay_protocol.h)), enforced at
  [wifi_service.cpp:966-970](../src/wifi_service.cpp).
- **Action:** return; resume next loop pass. Like §3.8 this is a byte cap, not a time cap — each
  dispatched frame can block arbitrarily.

### 3.13 GT911 I²C retries and disable threshold

- **file:line:** `GT911_I2C_RETRIES = 3` ([touch_input.cpp:35](../src/touch_input.cpp)), used at
  [:180](../src/touch_input.cpp) and [:202](../src/touch_input.cpp) with
  `delayMicroseconds(GT911_I2C_RETRY_DELAY_US)` = 500 µs
  ([:36](../src/touch_input.cpp), used [:196](../src/touch_input.cpp), [:207](../src/touch_input.cpp),
  [:212](../src/touch_input.cpp)); `TOUCH_I2C_FAIL_DISABLE_THRESHOLD = 5`
  ([:39](../src/touch_input.cpp)), enforced [:642](../src/touch_input.cpp) and
  [:677](../src/touch_input.cpp).
- **Target:** both.
- **Action:** after 5 consecutive failures, `touch_disable_controller()`
  ([touch_input.cpp:97-112](../src/touch_input.cpp)) permanently disables that controller for the
  rest of the boot (cleared only by a re-init at [:378-379](../src/touch_input.cpp) /
  [:519-520](../src/touch_input.cpp)).
- **Underlying `Wire` transaction bound:** *bounded by library — not analyzed (out of scope)*.
  **The firmware never calls `Wire.setTimeOut()`** — verified: `grep -rn "setTimeOut" src/`
  returns nothing. `wireBeginForOpenDisplay()`
  ([display_service.cpp:785-805](../src/display_service.cpp)) sets only clock, with a 100 kHz
  fallback on a failed `begin`.

### 3.14 Boot-screen refresh retry — 1 retry, battery only

- **file:line:** [display_service.cpp:1622-1636](../src/display_service.cpp).
- **Target:** both (`nrfVbusPresent()` gates it).
- **Bound:** exactly one retry, and only when `!nrfVbusPresent()`. Retry costs
  `pwrmgm(false) + delay(200) + prepareEpdRailForBoot() + initBbepPanelSession()` plus a second
  `refreshBootScreenFull()` → a second 60 s `waitforrefresh`. Worst case ≈ **2 × 60 s of
  blocking in `setup()`**.
- **Action if both fail:** `od_log_warn("Boot screen refresh did not complete")` and continue.
- **Note:** neither boot-refresh path sets `epdRefreshInProgress` — see §4 and §5.

### 3.15 Fixed-count hardware loops

| Loop | file:line | Bound |
|---|---|---|
| External-flash bit-bang power-down (nRF) | [main.cpp:846-853](../src/main.cpp), [:872-882](../src/main.cpp) | 8 bits × `delayMicroseconds(1)`; JEDEC read 3 bytes |
| Battery ADC averaging | [display_service.cpp:1694-1698](../src/display_service.cpp) | `numSamples = 10`, `delay(2)` each |
| E1004 half-plane byte sink | [display_service.cpp:278-294](../src/display_service.cpp) | bounded by `e1004HalfPlaneBytes`; returns on `!e1004StreamOpen` |
| Buzzer note-index folding | [buzzer_control.cpp:87-91](../src/buzzer_control.cpp) | converges into `[kBuzzerMinNoteIdx, kBuzzerMaxNoteIdx]` = [117, 234] by ±1 octave steps, with a defensive clamp at [:93-95](../src/buzzer_control.cpp) |
| Boot-screen font autosizing | [boot_screen.cpp:424](../src/boot_screen.cpp), [:439](../src/boot_screen.cpp), [:448](../src/boot_screen.cpp) | monotonically decrement a scale bounded below by 1 |
| Secure-erase zero-fill | [encryption.cpp:817](../src/encryption.cpp), [:834](../src/encryption.cpp) | bounded by `fileSize` |
| Config block walk | [config_parser.cpp:314](../src/config_parser.cpp) | bounded by `configLen` |

---

## 4. Periodic ticks

Everything below runs from `loop()` and/or `idleDelay()`. **All of it starves whenever the loop
task blocks** — the two long blockers are `waitforrefresh` (up to 60 s) and the SPI/zlib streaming
inside a direct/pipe/partial write. On nRF the command handlers run on the Bluefruit Callback
task instead, so `loop()` keeps ticking during a client-driven transfer; the loop-blocking
analysis below is ESP32-specific except where noted.

| Tick | file:line | Cadence | Target | Starves when loop blocks |
|---|---|---|---|---|
| `processLedFlash()` | [main.cpp:352](../src/main.cpp), [:544](../src/main.cpp) | every pass; internally gated by `delay_until_ms` | both | LED pattern freezes mid-sequence |
| `epdSessionTick()` | [main.cpp:353](../src/main.cpp), [:545](../src/main.cpp) | every pass; acts only in `PWR_WARM` past the deadline | both | keep-alive expiry deferred (benign — a transfer is `PWR_ACTIVE` anyway) |
| `buzzerService()` | [main.cpp:354](../src/main.cpp), [:483](../src/main.cpp), [:516](../src/main.cpp), [:529](../src/main.cpp), [:546](../src/main.cpp) | every pass; gated by `step_until_ms` | both | a tone sounds past its step, and past the 30 s cap |
| `pollActivity()` | [main.cpp:356](../src/main.cpp) → [main.cpp:218-265](../src/main.cpp) | every pass | ESP32 | `lastActivityMs` not refreshed — but sleep gates are downstream and also blocked |
| Command drain | [main.cpp:406-423](../src/main.cpp) | every pass, ≤33 commands | ESP32 | n/a (it *is* the blocker) |
| `flushResponseQueueToBle()` | [main.cpp:421](../src/main.cpp), [:424](../src/main.cpp) | between every drained command + once per pass | ESP32 | ACKs stall → the client's PTO fires. Explicitly pre-flushed before the refresh at [display_service.cpp:2412](../src/display_service.cpp) to mitigate |
| `serviceBleDisconnectCleanup()` | [main.cpp:370](../src/main.cpp), [:428](../src/main.cpp) | every pass; deferred while `epdRefreshInProgress` | ESP32 | disconnect teardown deferred |
| `esp32_restart_ble_advertising()` | [main.cpp:372](../src/main.cpp), [:434](../src/main.cpp), [display_service.cpp:2439](../src/display_service.cpp) | on `bleRestartAdvertisingPending` | ESP32 | radio stays dark. Contains a hard `delay(100)` at [ble_init.cpp:241](../src/ble_init.cpp) |
| §2.1 direct-write watchdog | [main.cpp:436](../src/main.cpp) | every pass | ESP32 | deferred |
| `checkPartialWriteTimeout()` | [main.cpp:443](../src/main.cpp) | every pass | ESP32 | deferred |
| `handleWiFiServer()` | [main.cpp:447](../src/main.cpp) | every pass | ESP32+WiFi | LAN idle timeout and TLS handshake progress both stall |
| `serviceLanRoam()` | [wifi_service.cpp:856](../src/wifi_service.cpp), inside `handleWiFiServer()` | every pass; self-gated on `!wifiClient.connected() && !transferActive()` | ESP32+WiFi | roam deferred (by design) |
| WiFi link supervisor | [main.cpp:448-464](../src/main.cpp) | 10 s | ESP32+WiFi | link-loss detection deferred |
| `processButtonEvents()` | [main.cpp:481](../src/main.cpp), [:514](../src/main.cpp), [:527](../src/main.cpp), [:542](../src/main.cpp) | every pass (workInFlight branch) / every `idleDelay` chunk | both | **power-off hold cannot complete** — the user's only manual recovery is dead during a wedged refresh |
| `processTouchInput()` | [main.cpp:482](../src/main.cpp), [:515](../src/main.cpp), [:528](../src/main.cpp), [:543](../src/main.cpp) | ≥100 ms floor | both | touch dead (already suppressed during transfers on ESP32 by design) |
| MSD refresh | [main.cpp:509-513](../src/main.cpp) | 60 s, **idle branch only** | ESP32 | deferred |
| `ble_nrf_advertising_tick()` | [main.cpp:526](../src/main.cpp), [:540](../src/main.cpp) | every pass / every `idleDelay` chunk | nRF | advertising stays at the boosted 20–30 ms interval |
| `idleDelay(ms)` | [main.cpp:535-551](../src/main.cpp) | services the above every `CHECK_INTERVAL_MS = 100` ([:536](../src/main.cpp)) | both | n/a |

**`idleDelay` call sites and their arguments:** `idleDelay(50)` in the post-wake advertising
window ([main.cpp:396](../src/main.cpp)); `idleDelay(5)` on the battery idle-hold path
([:495](../src/main.cpp)) and the USB idle path ([:507](../src/main.cpp)); on nRF,
`idleDelay(globalConfig.power_option.sleep_timeout_ms)` or `idleDelay(500)`
([main.cpp:519-525](../src/main.cpp)) — i.e. **on nRF the loop cadence is itself config-driven**,
up to 65 535 ms, though the 100 ms internal chunking keeps every tick above serviced.

---

## 5. Unbounded waits inside this project

This section is the inventory's most important half: the absence of a bound.

### 5.1 `pwrmgmLockTake()` — infinite spin

- **file:line:** [display_service.cpp:401-408](../src/display_service.cpp); the spin is
  [:407](../src/display_service.cpp).
- **Target:** both (the cross-task hazard it exists for is nRF-specific: Bluefruit Callback task
  vs. loop task).
- **Bound:** **none.** `while (__atomic_exchange_n(&pwrmgmLock, 1, ACQUIRE)) { delay(1); }` — it
  yields (deliberately, per the comment at [:402-406](../src/display_service.cpp), to avoid
  priority-inversion livelock) but never gives up.
- **Callers:** `epdSessionAcquire` ([:438](../src/display_service.cpp)), `epdSessionRelease`
  ([:496](../src/display_service.cpp)), `epdSessionForceOff`
  ([:514](../src/display_service.cpp)).
- **Held across:** panel I/O including `bbepSleep`, `bbepWakeUp`, `bbepSendCMDSequence`, plus a
  raw `delay(50)` at [:430](../src/display_service.cpp) — *the busy-wait inside those is bounded
  by library — not analyzed (out of scope)*.
- **Note:** `pwrmgmLockTryTake()` ([:409-411](../src/display_service.cpp)) is the safe variant and
  is used only by `epdSessionTick` ([:520](../src/display_service.cpp)). The lock is a bare 0/1
  flag with **no owner field**, so a hung holder is indistinguishable from a busy one.

### 5.2 `powerOff()` stuck-button spin

- **file:line:** [power_latch.cpp:87-90](../src/power_latch.cpp).
- **Target:** ESP32.
- **Bound:** **none.** `while (digitalRead(buttonPin()) == LOW) { delay(20); }` — a shorted or
  stuck-low button pin holds the device here forever, with the latch still engaged.
- **Context:** called from `powerButtonPoll()` on the loop task, i.e. this hangs `loop()`.

### 5.3 FastEPD refresh — no firmware-side bound at all

- **file:line:** [display_fastepd.cpp:228-231](../src/display_fastepd.cpp).
- **Target:** ESP32 with `OPENDISPLAY_FASTEPD` (IT8951 / E1004 class).
- **Bound:** `fastepd_wait_refresh()` discards its `timeout_sec` argument entirely and returns
  `!s_init_failed`. `waitforrefresh(60)` short-circuits to it at
  [display_service.cpp:749](../src/display_service.cpp), so **the 60 s cap does not exist on
  these builds**. The real blocking lives in `fastepd_full_update()`
  ([display_fastepd.cpp:222-226](../src/display_fastepd.cpp)) and `fastepd_direct_refresh` —
  *bounded by library — not analyzed (out of scope)*; the firmware passes **no** timeout to
  either.
- **Firmware call sites of the blocking work:** [display_service.cpp:1592](../src/display_service.cpp)
  (boot), [display_service.cpp:2422-2423](../src/display_service.cpp) (direct refresh — the path
  a real transfer takes), [display_service.cpp:3288](../src/display_service.cpp)
  (`fastepd_partial_refresh`).

### 5.4 Boot refresh is invisible to every `epdRefreshInProgress` gate

- **file:line:** `refreshBootScreenFull()` [display_service.cpp:533-542](../src/display_service.cpp);
  FastEPD boot path [display_service.cpp:1588-1594](../src/display_service.cpp).
- **Target:** both.
- **Bound:** the refresh itself is bounded (60 s, §2.3) or not (§5.3) — but **neither path sets
  `epdRefreshInProgress`**. Verified: the flag is written only at
  [display_service.cpp:2415/2436](../src/display_service.cpp) and
  [:3284/3294](../src/display_service.cpp). So every reader —
  [main.cpp:322](../src/main.cpp), [main.cpp:478](../src/main.cpp),
  [ble_init.cpp:236](../src/ble_init.cpp) — mis-reads a 30–60 s boot refresh as idle. With the
  §3.14 retry that window can double.

### 5.5 Blocking sequences with no escape

| Sequence | file:line | Blocking cost |
|---|---|---|
| `pwrmgm(true)` rail bring-up | [main.cpp:717-753](../src/main.cpp) | `delay(800)` + `delay(100)` (or `delay(200)` on FastEPD) ≈ **900 ms**, unconditional |
| `initBbepPanelSession()` | [display_service.cpp:341-354](../src/display_service.cpp) | `delay(200)` plus library init |
| `prepareEpdRailForBoot()` (nRF, battery) | [display_service.cpp:172-183](../src/display_service.cpp) | a full extra `pwrmgm` off/on cycle ≈ 950 ms |
| GT911 address-select + reset | [touch_input.cpp:250-269](../src/touch_input.cpp), [:317-339](../src/touch_input.cpp) | ~150 ms + up to 3 × 500 ms reset cycles |
| `esp32_restart_ble_advertising()` | [ble_init.cpp:241](../src/ble_init.cpp) | `delay(100)` |
| `updatemsdata()` advertising re-push | [display_service.cpp:1811](../src/display_service.cpp) | `delay(50)` |
| `directWriteFinishAndRefresh` pre-refresh settle | [display_service.cpp:2414](../src/display_service.cpp) | `delay(20)` |
| `enterDeepSleep()` teardown | [main.cpp:618](../src/main.cpp), [:621](../src/main.cpp), [:637](../src/main.cpp) | `delay(200) + delay(100) + delay(100)` |
| `reboot()` | [device_control.cpp:259-269](../src/device_control.cpp), [communication.cpp:720](../src/communication.cpp) | `delay(200)`/`delay(100)` before reset — terminal, so harmless |
| `checkResetPin()` | [encryption.cpp:875](../src/encryption.cpp), [:881](../src/encryption.cpp) | `delay(100)` ×2 in `setup()` |
| Button init settle | [device_control.cpp:782](../src/device_control.cpp) | `delay(10)` **per configured pin** — up to 320 ms with 32 buttons |
| `od_log_flush()` | [od_log.cpp:64](../src/od_log.cpp) | `delay(5)` on ESP32, per call |

### 5.6 Terminal `while (1) {}`

- **file:line:** [device_control.cpp:866](../src/device_control.cpp).
- **Target:** nRF only, inside the DFU-bootloader jump.
- **Bound:** none by design — control never returns; `bootloader_util_app_start()` has already
  transferred execution. Not a freeze vector.

### 5.7 Latched state with no timer of its own

These are not loops, but they are unbounded in exactly the way that matters:

| Latch | Set | Cleared | Bounded by |
|---|---|---|---|
| `pipeState.active` after a fatal NACK | [display_service.cpp:2568](../src/display_service.cpp) | only `resetPipeWriteState()` — reachable from disconnect ([main.cpp:347](../src/main.cpp)) or a new `0x0080` | **nothing** for a non-partial pipe: §2.1 keys on `directWriteActive`, which `sendPipeNack`→`cleanupDirectWriteState(true)` just cleared |
| `chunkedWriteState.active` | [communication.cpp:496](../src/communication.cpp) | completion / malformed / auth-fail only | **nothing** |
| `encryptionSession` after a BLE disconnect | — | nothing on BLE; LAN clears it at [wifi_service.cpp:804](../src/wifi_service.cpp) and [:879](../src/wifi_service.cpp) | **nothing** (survives into the next connection) |
| `directWriteTouchSuspended` / `s_epd_refresh_suspend` | [display_service.cpp:2136](../src/display_service.cpp), [:2799](../src/display_service.cpp), [:539](../src/display_service.cpp), [:1590](../src/display_service.cpp) → `touchSuspendForEpdRefresh()` [touch_input.cpp:116-117](../src/touch_input.cpp) | paired `touchResumeAfterEpdRefresh()` [touch_input.cpp:418-421](../src/touch_input.cpp) | **nothing** — an unpaired suspend leaves touch dead for the rest of the boot |
| `roamPending` | [wifi_service.cpp:606](../src/wifi_service.cpp) | `serviceLanRoam()` when idle | **nothing** — a permanently busy device never roams (deliberate) |
| `touch` `rt->disabled` | [touch_input.cpp:100](../src/touch_input.cpp) | only a full re-init | **nothing** — permanent for the boot |

### 5.8 LED pattern with no global cap

`processLedFlash()` ([device_control.cpp:524-535](../src/device_control.cpp)) advances a state
machine whose loop counts come from config. Unlike the buzzer there is **no
`kBuzzerMaxTotalMs` equivalent**, so a pathological pattern runs indefinitely. It never blocks
the loop (each step schedules a deadline and returns), so it is a nuisance rather than a freeze —
but it is genuinely unbounded.

---

## 6. Config-driven values — consolidated

| Timeout | Struct field | Type / range | 0 means | Firmware constant backing it | Consumer |
|---|---|---|---|---|---|
| EPD keep-alive | `power_option.screen_timeout_seconds` [opendisplay_structs.h:504](../include/opendisplay_structs.h) | `uint8_t`, `@min 0 @max 30` | power off immediately | clamp `EPD_KEEPALIVE_MAX_S = 30` [display_service.h:16](../src/display_service.h) | [display_service.cpp:383-395](../src/display_service.cpp) |
| Idle hold / advertising window | `power_option.sleep_timeout_ms` [opendisplay_structs.h:490](../include/opendisplay_structs.h) | `uint16_t` LE ms (max 65 535) | fall back to default | `DEFAULT_IDLE_HOLD_MS = 10000` [main.h:322](../src/main.h) | [main.cpp:375-377](../src/main.cpp), [:487-490](../src/main.cpp); also the nRF `idleDelay` argument [main.cpp:519-520](../src/main.cpp) |
| Minimum wake hold | `power_option.min_wake_time_seconds` [opendisplay_structs.h:503](../include/opendisplay_structs.h) | `uint16_t` LE s | fall back to default | `DEFAULT_MIN_WAKE_TIME_SECONDS = 120` [main.h:312](../src/main.h) | [main.cpp:197-200](../src/main.cpp) |
| Deep-sleep duration | `power_option.deep_sleep_time_seconds` [opendisplay_structs.h:499](../include/opendisplay_structs.h) | `uint16_t` LE s | deep sleep disabled | none | [main.cpp:583-585](../src/main.cpp), [:625-628](../src/main.cpp); overridable for one cycle by command `0x0053` |
| Encryption session lifetime | `securityConfig.session_timeout_seconds` [opendisplay_structs.h:916](../include/opendisplay_structs.h) | `uint16_t` LE s | **no timeout** (persists until disconnect) | none — the 0 case returns `true` directly | [encryption.cpp:221-232](../src/encryption.cpp) |
| Power-off hold (binary inputs) | `BinaryInputs.power_off_hold_sec` [opendisplay_structs.h:862](../include/opendisplay_structs.h) | `uint8_t` s | default 3 s | literal `3000u` [device_control.cpp:746](../src/device_control.cpp) → `ButtonState.power_off_hold_ms` [structs.h:181](../src/structs.h) | [device_control.cpp:78](../src/device_control.cpp) |
| Touch poll interval | `TouchController.poll_interval_ms` [opendisplay_structs.h:951](../include/opendisplay_structs.h) | `uint8_t` ms | header says 25 ms; **firmware uses 100** | `TOUCH_PROCESS_MIN_INTERVAL_MS = 100` [touch_input.cpp:38](../src/touch_input.cpp) | [touch_input.cpp:600](../src/touch_input.cpp) |
| LAN listener idle drop | — (protocol constant, not config) | — | — | `OD_LAN_READ_TIMEOUT_S = 30u` [opendisplay_protocol.h:984](../include/opendisplay_protocol.h) | [wifi_service.cpp:957](../src/wifi_service.cpp) |

Everything not in this table is a compile-time firmware constant.

Note the **two independent power-off-hold mechanisms**: `power_latch`'s fixed
`POWER_OFF_HOLD_MS = 3000` ([power_latch.cpp:21](../src/power_latch.cpp)), keyed on
`SystemConfig.pwr_pin_3`, and `device_control`'s config-driven per-instance hold. They share the
3 s default but not the source, and both can be active on one device.

---

## 7. Analysis

### 7.1 Coverage map

Rows are failure conditions; cells name the mechanism that actually bounds them today.

| Failure | nRF | ESP32 | Covered? |
|---|---|---|---|
| **Stalled direct-write transfer** (client silent, `directWriteActive` set) | *nothing* | §2.1, 15 min | Partial — nRF has no watchdog at all |
| **Stalled pipe transfer, no fatal NACK** | *nothing* | §2.1 via `directWriteActive` (set by `directWriteActivatePanel` for both legacy and pipe), 15 min | Partial |
| **Stalled pipe transfer after a fatal NACK** | *nothing* | *nothing* — `directWriteActive` cleared, `pipeState.active` latched | **GAP** |
| **Stalled partial write** | *nothing* | §2.2, 15 min | Partial |
| **Stalled config chunked write** | *nothing* | *nothing* | **GAP** (§3.9) |
| **Idle authenticated connection** (client connected, sending nothing) | *nothing* | *nothing* — `pollActivity` treats a live link as activity ([main.cpp:254](../src/main.cpp)), pinning the device awake | **GAP** |
| **Client flooding undecryptable frames post-session-clear** | *nothing* | *nothing* | **GAP** — every frame is answered `RESP_AUTH_REQUIRED` ([communication.cpp:664-670](../src/communication.cpp)) indefinitely |
| **Wedged panel (bb_epaper BUSY stuck)** | §2.3, 60 s | §2.3, 60 s | Yes |
| **Wedged panel (FastEPD / IT8951)** | n/a | *nothing* (§5.3) | **GAP** |
| **Wedged panel during the boot refresh** | 60 s wait, but invisible to every gate (§5.4) | same | Partial |
| **Hung I²C (GT911 holding SDA low)** | §3.13 disables the controller after 5 failures — but only if the transaction *returns* | same | **GAP** — no `Wire.setTimeOut`, no bus recovery |
| **Hung I²C (AXP2101 / SHT40 / BQ27220)** | *nothing* | *nothing* | **GAP** |
| **Dead radio — advertising stopped, never restarted** | `ble_nrf_advertising_tick` only re-starts after a boost expiry | *nothing* — `esp32_restart_ble_advertising` **clears** the pending flag when `getConnectedCount() > 0` ([ble_init.cpp:232-235](../src/ble_init.cpp)) | **GAP** |
| **Lost WiFi link** | n/a | §2.9, 10 s poll | Yes |
| **Silent LAN client** | n/a | §2.7, 30 s | Yes |
| **Stuck `pwrmgmLock`** | *nothing* (§5.1) | *nothing* | **GAP** |
| **Stuck power button (latch path)** | n/a | *nothing* (§5.2) | **GAP** |
| **Command ring full** | n/a (no ring) | drop-newest + log (§3.7); never flushed | Partial — recoverable, but stale commands survive a disconnect |
| **Response ring full** | n/a | drop-newest + log (§3.6); drained when disconnected | Yes |
| **Dead encryption session with a live link** | *nothing* | *nothing* | **GAP** — invisible to the client |
| **Encryption session expiring mid-transfer** | fires (§2.5) | fires (§2.5) | *Inverted* — the timer is itself the wedge |
| **CPU/peripheral hard hang** | *nothing* (no WDT armed) | *nothing* firmware-side | **GAP** — accepted per the plan's software-only decision |

The shape of the gap: this firmware bounds **wall-clock transfer age** and **panel BUSY**, and
nothing else. There is no inactivity timer, no link-liveness timer, no I²C bound, no lock bound,
and on nRF no transfer watchdog whatsoever.

### 7.2 Conflicts and nesting

1. **§2.1/§2.2 (900 s) nest inside the drain and the refresh, not the other way round.** Both are
   evaluated once per `loop()` pass at [main.cpp:436](../src/main.cpp)/[:443](../src/main.cpp),
   *after* the command drain at [:406-423](../src/main.cpp). A single dispatched command can block
   for a 60 s refresh, so the effective resolution of both watchdogs is one refresh, not one loop
   pass. They cannot fire mid-refresh. Correct ordering, but it means "15 minutes" is really
   "15 minutes, rounded up to the next loop pass."

2. **§2.5 (encryption expiry) nests inside §2.1 and can fire during it.** Session expiry runs on
   every command dispatch, so on a nonzero `session_timeout_seconds` shorter than 900 s it fires
   *inside* the direct-write watchdog's window — and its action (`clearEncryptionSession`) does
   **not** clear `directWriteActive`. The result is a transfer that is simultaneously "active" per
   §2.1 and unable to accept a single further frame, for up to 15 minutes.

3. **§3.2 (integrity ≥ 3) and §2.1 fight.** Same shape as (2): the session dies, the transfer
   flag lives. §2.1 is the only thing that eventually releases the panel.

4. **§3.11 (`sendPipeNack`) actively *disables* §2.1.** `cleanupDirectWriteState(true)` clears the
   very flag §2.1 keys on, while leaving `pipeState.active` set. A fatal NACK therefore trades a
   bounded wedge for an unbounded one. This is the single worst ordering inversion in the
   inventory.

5. **§2.7 (LAN 30 s) and the BLE disconnect path share one flag.** `disconnectWiFiServer()` sets
   `bleDisconnectCleanupPending` ([wifi_service.cpp:812](../src/wifi_service.cpp)), so a LAN idle
   timeout enters `serviceBleDisconnectCleanup()`. The only thing preventing it from tearing down
   a live BLE transfer is the `ownerStillUp` guard at [main.cpp:328-338](../src/main.cpp) —
   which is inside `#ifdef OPENDISPLAY_HAS_WIFI`, so on `esp32-N4` the guard does not exist at
   all (there is also no LAN on that env, so the flag has no second raiser there today; the
   hazard is latent and becomes live the moment anything else raises the flag).

6. **§2.4 (keep-alive) vs. §2.12 (idle hold).** On battery ESP32, `enterDeepSleep()` calls
   `epdSessionForceOff()` unconditionally ([main.cpp:609](../src/main.cpp)), so the effective
   keep-alive is `min(screen_timeout_seconds, sleep_timeout_ms)`. With the defaults (keep-alive up
   to 30 s, idle hold 10 s) the idle hold usually wins. The code comments at
   [main.cpp:602-609](../src/main.cpp) document this deliberately; noting it because "30 s
   keep-alive" is not what a battery device actually does.

7. **§2.13 (min-wake) is checked twice, and the second check has an ordering constraint.**
   `minWakeHoldActive()` is evaluated in `loop()` ([main.cpp:383](../src/main.cpp),
   [:494](../src/main.cpp)) **and** inside `enterDeepSleep()` ([main.cpp:598](../src/main.cpp)).
   The second check must stay above the advertising stop at [main.cpp:611-616](../src/main.cpp),
   because everything past that point commits to `esp_deep_sleep_start()` — a late abort would
   leave the device awake with the radio dark. The comment at [:594-597](../src/main.cpp) says so;
   it is a real constraint on any future reordering.

8. **§2.13/§2.12 have a side effect: `minWakeHoldActive()` mutates.** It clears
   `minWakeWindowActive` when the window elapses ([main.cpp:205](../src/main.cpp)). Because it is
   called from three sites, whichever one runs first consumes the transition and logs it. Benign
   today (all three are on the loop task), but it is a query with side effects, same class of
   defect as `isAuthenticated()`.

9. **Two power-off holds, one device.** §2.22 (fixed 3 s, `power_latch`) and §2.23 (config,
   default 3 s, `device_control`) both run from `processButtonEvents()`
   ([device_control.cpp:586-587](../src/device_control.cpp)). If a device configures a
   `BinaryInputs` power-off pin that is also `SystemConfig.pwr_pin_3`, both arm on the same press
   with independent thresholds.

10. **§2.15 (60 s MSD) vs. `onConnect`'s `msdUpdatePending`.** Both drive `updatemsdata()`, which
    mutates the shared advertisement vector and re-pushes it with a `delay(50)`
    ([display_service.cpp:1811](../src/display_service.cpp)). They are serialised only because
    both run on the loop task — the `onConnect` callback deliberately defers rather than calling
    inline ([esp32_ble_callbacks.h:53-56](../src/esp32_ble_callbacks.h)). Any future producer on
    another task breaks this.

11. **§3.5 (16-notify drain cap) vs. §3.6 (10-slot ring).** The drain cap exceeds the ring size,
    so it is never the binding constraint; the binding one is `notify()` returning false. Not a
    conflict, but the 16 is dead weight.

### 7.3 Impact of the freeze-proofing plan

Per existing mechanism, with the plan phase that touches it.

| Existing mechanism | Verdict | Plan reference |
|---|---|---|
| §1.1 `-DCONFIG_FREERTOS_WATCHDOG_TIMEOUT_S=120` | **DELETES** (or renames) | Phase 2 `[L3]` |
| §1.3 reset-reason logging | LEAVES | — |
| §2.1 direct-write 15-min watchdog | **MODIFIES** — kept as a backstop, raised to 20 min | Phase 6 `[H3]` |
| §2.2 `checkPartialWriteTimeout` | **MODIFIES** — same, 20 min | Phase 6 `[H3]` |
| §2.3 `waitforrefresh(60)` (bb_epaper) | LEAVES | — |
| §2.3 `waitforrefresh` → FastEPD stub | **MODIFIES** — implement a real IT8951 LUT-busy poll honouring `timeout_sec`; wrap `fastepd_direct_refresh` | Phase 2 `[X3]` |
| §2.4 EPD keep-alive | LEAVES | — |
| §2.5 encryption session timeout | **DELETES** — age-based expiry removed; session lifetime := connection lifetime | Phase 5 |
| §2.6 auth challenge 30 s | LEAVES | — |
| §2.7 LAN 30 s idle | LEAVES — the BLE/LAN asymmetry is called out as deliberate | Phase 7 |
| §2.8 `wifiClient.setTimeout(30000)` | LEAVES | — |
| §2.9 WiFi 10 s supervisor | LEAVES (but its `disconnectWiFiServer` path gains the owner guard) | Phase 4/5 `[H2]` |
| §2.10 blocking WiFi connect | LEAVES | — |
| §2.11 mDNS 400 ms throttle | LEAVES | — |
| §2.12 deep-sleep idle hold | LEAVES — explicitly stated as requiring no change; the 5-min BLE idle drop makes `connCount` fall so the existing hold elapses naturally | Phase 7 |
| §2.13 min-wake hold | LEAVES | — |
| §2.14 advertising window | LEAVES, but Phase 4 must skip `esp32_set_ble_connectable` during the post-wake window | Phase 4 |
| §2.15 60 s MSD cadence | LEAVES | — |
| §2.16 nRF advertising boost | LEAVES | — |
| §2.17 nRF link-diag one-shot | LEAVES | — |
| §2.18 buzzer 30 s cap + sequencing | LEAVES; `abortToKnownState` adds a stop call | Phase 3 |
| §2.19 LED sequencing | LEAVES; `abortToKnownState` adds a stop call. **The plan does not add a global LED cap** — `[X6]` notes it and defers | Phase 3 / review `X6` |
| §2.20 touch poll interval | LEAVES; `touchForceResume()` added alongside | Phase 3 `[M3]` |
| §2.22 `power_latch` 3 s hold | LEAVES (the *spin* it can reach is bounded — see §5.2 row) | Phase 2 |
| §2.23 config power-off hold | LEAVES | — |
| §2.24 ADC ladder poll | LEAVES | — |
| §2.25 sensor TTLs | LEAVES | — |
| §3.1 auth rate limit 10/60 s | LEAVES | — |
| §3.2 integrity ≥ 3 | **MODIFIES** — nonce failures no longer increment it; CCM-tag trigger stays and must now drop the link | Phase 1, Phase 5 |
| §3.3 nonce window ±32 / 64-ring | **MODIFIES** — symmetric ±128 with a 256-entry ring (fallback ±64); `replay_window_index` moved into the session; `counter_diff == 0` exemption removed; check/commit split around CCM | Phase 1 `[M1]` |
| §3.4 nRF notify retry ×4 | LEAVES | — |
| §3.5 16-notify drain cap | LEAVES | — |
| §3.6 response ring 10 | **MODIFIES** — adds an overflow flag serviced in loop, gated on `transferActive()`; adds a redundant `flushResponseQueue()` | Phase 7, Phase 3 (`[L1]`: the flush is redundant on ESP32) |
| §3.7 command ring 33/32 | **MODIFIES** — adds `flushCommandQueue()`, fixes the `main.h:365-370` off-by-one comment, optionally bumps to 34 off `esp32-N4`. Overflow **keeps** drop-newest; it does not drop the link | Phase 7 `[H1]` |
| §3.8 drain count cap | **MODIFIES** — adds a 2 s wall-clock cap alongside; adds the drain-abort check between [main.cpp:415](../src/main.cpp) and [:416](../src/main.cpp) | Phase 2, Phase 3 `[M5]` |
| §3.9 config chunked write | **SUBSUMES** — no timer of its own; covered by the supervisor predicate, and `resetChunkedWriteState()` is added | Phase 6 `[X4]` |
| §3.11 pipe NACK latch | **MODIFIES** — `error_since_ms` added to `PipeWriteState` + a 10 s `pipeErrorTick()` hardware-release deadline | Phase 5 `[L2]` |
| §3.12 LAN 16 KB drain budget | LEAVES | — |
| §3.13 GT911 retries / disable | **MODIFIES** — `Wire.setTimeOut(25)` after every `Wire.begin()` plus a nine-clock SDA recovery keyed on `i2c_fail_streak` | Phase 2 `[X1]` |
| §3.14 boot-refresh retry | LEAVES, but §5.4's flag fix makes it visible | Phase 2 `[X2]` |
| §5.1 `pwrmgmLockTake` infinite spin | **MODIFIES** — returns `bool` with a **60 s** deadline, fail-closed, no steal | Phase 2 `[C2]` |
| §5.2 `powerOff` stuck-button spin | **MODIFIES** — 10 s bound, then drop the latch anyway | Phase 2 |
| §5.3 FastEPD unbounded refresh | **MODIFIES** — see §2.3 row | Phase 2 `[X3]` |
| §5.4 boot refresh invisible | **MODIFIES** — set/clear `epdRefreshInProgress` around both boot paths | Phase 2 `[X2]` |
| §5.7 `encryptionSession` surviving disconnect | **MODIFIES** — cleared on BLE disconnect (deferred on nRF behind the in-flight depth counter) | Phase 5 `[H4]` |
| §5.7 `directWriteTouchSuspended` unpaired | **MODIFIES** — `touchForceResume()` clears it and asserts the counter reached 0 | Phase 3 `[M3]` |
| §5.7 `roamPending`, `rt->disabled` | LEAVES | — |
| §5.8 LED unbounded pattern | LEAVES | review `X6` |
| §2.12 `workInFlight` gate | **MODIFIES** — `transferActive()` added to the disjunction | Phase 6 `[X5]` |
| `esp32_restart_ble_advertising` clearing the flag on a stale count | **MODIFIES** — `advertisingHealthTick()` at 30 s | Phase 4 `[X7]` |

**New mechanisms the plan introduces** (for completeness, so a future reader can tell them from
the existing set): the 10-minute progress supervisor (`g_lastProgressMs`, 600 000 ms, Phase 6);
the 5-minute BLE idle disconnect (`OD_BLE_IDLE_DISCONNECT_MS = 300000`, Phase 7); the 10 s pipe
error-release deadline (Phase 5); the 60 s `pwrmgmLock` deadline and the 10 s `powerOff` bound
(Phase 2); the 2 s drain cap (Phase 2); the 30 s advertising health tick (Phase 4); the ~10 s
idle-incumbent eviction threshold (Phase 4 `[M2]`).

**Silent conflicts to watch:**

- **The plan's 20-minute backstop must sit above the supervisor's 10 minutes, and both above the
  60 s lock deadline.** As written the hierarchy is 10 s (pipe error) < 60 s (lock, refresh) <
  300 s (BLE idle) < 600 s (supervisor) < 1200 s (backstops). That is consistent — but note the
  20-minute backstops key on a *start* stamp while the supervisor keys on *progress*, so on a slow
  legitimate transfer the backstop can fire first. A 20-minute upload is not achievable on any
  current panel, so this is theoretical; it stops being theoretical if E1004 payload sizes grow.
- **The 5-minute BLE idle timer and the 60 s `pwrmgmLock` deadline can overlap.** A client
  connected but silent for 5 minutes while another context holds `pwrmgmLock` will be dropped by
  the idle timer; `abortToKnownState`'s `epdSessionForceOff()` then needs the lock it cannot get,
  and must honour the new fail-closed return rather than spinning.
- **Phase 5 disables §2.5 but §2.6 (the 30 s challenge validity) stays.** They are different
  timers on the same subsystem; the plan does not mention §2.6 at all. It is correct to leave it,
  but the plan's "encryption is scoped to the life of the connection and nothing else" sentence
  reads as if no encryption-side timer remains. One does.
- **The plan touches `esp32_restart_ble_advertising` (Phase 4 `[X7]`) but not the `delay(100)` at
  [ble_init.cpp:241](../src/ble_init.cpp)**, which now runs on every health-tick-forced restart.
- **`abortToKnownState`'s buzzer/LED stop has no counterpart bound.** §5.8 stays unbounded; the
  abort only stops a sequence that is already running when the abort fires.

### 7.4 Numbers summary — shortest to longest

Every firmware-controlled duration, sorted. Blocking `delay()`s under 10 ms are omitted.

| Duration | Mechanism | Where |
|---|---|---|
| 500 µs | GT911 I²C retry gap | [touch_input.cpp:36](../src/touch_input.cpp) |
| 1 ms | `pwrmgmLockTake` spin yield | [display_service.cpp:407](../src/display_service.cpp) |
| 1 ms | `workInFlight` loop yield | [main.cpp:484](../src/main.cpp) |
| 5 ms | ADC ladder poll floor | [device_control.cpp:96](../src/device_control.cpp) |
| 5 ms | nRF notify retry gap | [communication.cpp:347](../src/communication.cpp) |
| 5 ms | `od_log_flush` settle (ESP32) | [od_log.cpp:64](../src/od_log.cpp) |
| 5 ms | buzzer duration unit | [buzzer_control.cpp:15](../src/buzzer_control.cpp) |
| 5 ms | `idleDelay` argument, idle paths | [main.cpp:495](../src/main.cpp), [:507](../src/main.cpp) |
| 10 ms | `waitforrefresh` poll interval | [display_service.cpp:761](../src/display_service.cpp) |
| 10 ms | per-button init settle | [device_control.cpp:782](../src/device_control.cpp) |
| 12 ms | SHT40 measurement wait | [sensor_sht40.cpp:16](../src/sensor_sht40.cpp) |
| 20 ms | buzzer inter-pattern gap | [buzzer_control.cpp:16](../src/buzzer_control.cpp) |
| 20 ms | pre-refresh settle | [display_service.cpp:2414](../src/display_service.cpp) |
| 20 ms | `powerOff` stuck-button poll | [power_latch.cpp:88](../src/power_latch.cpp) |
| 20–30 ms | nRF boosted advertising interval | [ble_init.cpp:42-43](../src/ble_init.cpp) |
| 50 ms | `idleDelay` argument, post-wake window | [main.cpp:396](../src/main.cpp) |
| 50 ms | advertising re-push settle | [display_service.cpp:1811](../src/display_service.cpp) |
| 50 ms | `epdSessionForceOffLocked` post-sleep | [display_service.cpp:430](../src/display_service.cpp) |
| 80 ms ×3 | power-off buzzer alert | [buzzer_control.cpp:369-373](../src/buzzer_control.cpp) |
| 100 ms | `idleDelay` internal chunk | [main.cpp:536](../src/main.cpp) |
| 100 ms | touch process floor / poll fallback | [touch_input.cpp:38](../src/touch_input.cpp) |
| 100 ms | touch I²C fail backoff | [touch_input.cpp:37](../src/touch_input.cpp) |
| 100 ms | LED delay factor | [device_control.cpp:279](../src/device_control.cpp) |
| 100 ms | advertising restart settle | [ble_init.cpp:241](../src/ble_init.cpp) |
| 160–1000 ms | nRF steady advertising interval | [ble_init.cpp:40-41](../src/ble_init.cpp) |
| 200 ms | GT911 post-reset settle | [touch_input.cpp:29](../src/touch_input.cpp) |
| 200 ms | panel init settle | [display_service.cpp:346](../src/display_service.cpp), [:354](../src/display_service.cpp) |
| 300 ms | GT911 pre-reset delay | [touch_input.cpp:30](../src/touch_input.cpp) |
| 400 ms | mDNS MSD TXT throttle | [wifi_service.cpp:378](../src/wifi_service.cpp) |
| 500 ms | nRF link-diag one-shot | [ble_init.cpp:141](../src/ble_init.cpp) |
| 500 ms | WiFi connect inner poll | [wifi_service.cpp:769](../src/wifi_service.cpp) |
| ~900 ms | `pwrmgm(true)` rail bring-up | [main.cpp:721](../src/main.cpp) + [:749](../src/main.cpp) |
| 2 000 ms | WiFi inter-retry delay | [wifi_service.cpp:786](../src/wifi_service.cpp) |
| 3 000 ms | nRF advertising boost | [ble_init.cpp:44](../src/ble_init.cpp) |
| 3 000 ms | power-off hold (latch, fixed) | [power_latch.cpp:21](../src/power_latch.cpp) |
| 3 000 ms | power-off hold default (config) | [device_control.cpp:746](../src/device_control.cpp) |
| 5 000 ms | ADC ladder press-count window | [device_control.cpp:193](../src/device_control.cpp) |
| 10 000 ms | WiFi link supervisor poll | [main.cpp:449](../src/main.cpp) |
| 10 000 ms | WiFi connect timeout per retry | [wifi_service.cpp:763](../src/wifi_service.cpp) |
| 10 000 ms | `DEFAULT_IDLE_HOLD_MS` | [main.h:322](../src/main.h) |
| ≤30 000 ms | EPD keep-alive (`EPD_KEEPALIVE_MAX_S`) | [display_service.h:16](../src/display_service.h) |
| 30 000 ms | buzzer total playback cap | [buzzer_control.cpp:17](../src/buzzer_control.cpp) |
| 30 000 ms | auth challenge validity | [encryption.cpp:604](../src/encryption.cpp) |
| 30 000 ms | LAN idle drop (`OD_LAN_READ_TIMEOUT_S`) | [opendisplay_protocol.h:984](../include/opendisplay_protocol.h) |
| 30 000 ms | `wifiClient.setTimeout` | [wifi_service.cpp:888](../src/wifi_service.cpp) |
| 30 000 ms | sensor / battery read TTLs | [sensor_sht40.cpp:239](../src/sensor_sht40.cpp), [sensor_bq27220.cpp:142](../src/sensor_bq27220.cpp), [display_service.cpp:1707](../src/display_service.cpp) |
| 60 000 ms | auth rate-limit window (10 attempts) | [encryption.cpp:570](../src/encryption.cpp) |
| 60 000 ms | MSD refresh cadence | [main.cpp:510](../src/main.cpp) |
| 60 000 ms | `waitforrefresh` cap (bb_epaper only) | [display_service.cpp:760](../src/display_service.cpp) with `timeout = 60` |
| ≤65 535 ms | `sleep_timeout_ms` idle hold / advertising window; nRF `idleDelay` argument | [opendisplay_structs.h:490](../include/opendisplay_structs.h) |
| 120 000 ms | `DEFAULT_MIN_WAKE_TIME_SECONDS` | [main.h:312](../src/main.h) |
| ≤65 535 s | `min_wake_time_seconds`, `deep_sleep_time_seconds`, `session_timeout_seconds` | config |
| 900 000 ms | direct-write watchdog | [main.cpp:438](../src/main.cpp) |
| 900 000 ms | partial-write watchdog | [display_service.cpp:580](../src/display_service.cpp) |
| ∞ | `pwrmgmLockTake`, `powerOff` button spin, FastEPD refresh, pipe NACK latch, chunked-write latch, encryption session across a BLE disconnect | §5 |

The visible hierarchy: everything the firmware bounds today is either **under a minute** (panel,
LAN, radio, sensors) or **at fifteen minutes** (transfers). There is nothing in between — which
is precisely the band the plan's 5-minute idle and 10-minute supervisor are meant to fill.

---

## 8. Corrections to existing docs

Verified against source; each of these is wrong or imprecise in the plan or the review.

1. **"Every ESP env passes `-DCONFIG_FREERTOS_WATCHDOG_TIMEOUT_S=120`" — correct, but only via
   inheritance.** Review `L3` cites `platformio.ini:189`. Two envs
   (`esp32-s3-E1004` [:156](../platformio.ini), `esp32-s3-N16R8-extuart-debug`
   [:278](../platformio.ini)) do not list the flag; they inherit it through
   `${env:<parent>.build_flags}`. The conclusion (all ten ESP envs, flag inert) holds — but a
   `grep` for the flag returns nine lines, not ten, and anyone deleting it must check the
   inheriting envs still compile.

2. **The plan's `[L3]` says "the real TWDT is 5 s/panic on IDLE0."** That claim is sourced from
   the precompiled `sdkconfig.h`, which is out of scope for this inventory. What is verifiable
   from inside this repo: **the firmware arms nothing** (`grep -rn "esp_task_wdt" src/` → no
   hits), so whatever the platform default is, this codebase neither sets it nor feeds it. State
   it that way rather than quoting a framework value.

3. **The buzzer's global cap is 30 s, not 5 s.** `kBuzzerMaxTotalMs = 30000u`
   ([buzzer_control.cpp:17](../src/buzzer_control.cpp)), but the in-code comments at
   [buzzer_control.cpp:167](../src/buzzer_control.cpp) ("Global 5 s cap") and
   [:144](../src/buzzer_control.cpp) ("for the 5 s total cap") both say 5 s. Neither the plan nor
   the review mentions the buzzer cap, so this is a source-comment defect rather than a doc
   defect — flagged here because `[X6]` claims "nothing bounds a stuck `buzzer_control`
   sequence," which is **wrong**: a 30 s cap exists. What is genuinely unbounded is the **LED**
   sequence (§5.8), not the buzzer.

4. **The nRF link-diag timer literal is 500, not 2500.** [ble_init.cpp:141](../src/ble_init.cpp)
   passes `500`; the comment on the next line ([:144](../src/ble_init.cpp)) and the block comment
   at [:82](../src/ble_init.cpp) both say "~2.5 s later". One of the two is wrong. Neither
   plan nor review covers this; it is diagnostics-only, so the consequence is a misleading log
   phase label, not a freeze.

5. **`TouchController.poll_interval_ms`'s documented default is not the firmware's fallback.**
   The header says `0 = default 25 ms`
   ([opendisplay_structs.h:951](../include/opendisplay_structs.h)); the firmware falls back to
   `TOUCH_PROCESS_MIN_INTERVAL_MS` = **100 ms** ([touch_input.cpp:600](../src/touch_input.cpp)).
   Even an explicit 25 is floored to 100 by the global gate at
   [touch_input.cpp:589-592](../src/touch_input.cpp), so **the config field cannot produce a poll
   faster than 100 ms on this firmware**. Not mentioned anywhere; relevant to `[X1]`, which
   assumes touch polling is frequent enough to detect an I²C wedge quickly.

6. **Review `L4`'s line-drift note is itself slightly off.** It corrects
   `communication.cpp:113-116` → `:117-121` for "the ring-full check". The ring-full *check* is
   [communication.cpp:113-116](../src/communication.cpp) (`nextHead == responseQueueTail` at `:113`, the
   `od_log_error` at `:114`); `:117-121` is the memcpy/enqueue that follows. Both the plan's
   original and the review's correction point at roughly the right block; neither is exact.

7. **Review `L4` says `display_fastepd.cpp:222-231` "lands on `fastepd_full_update` at
   `:227-231`".** `fastepd_full_update` is [display_fastepd.cpp:222-226](../src/display_fastepd.cpp);
   `fastepd_wait_refresh` is [:228-231](../src/display_fastepd.cpp). The two are transposed. The
   substantive finding (`X3`) is correct.

8. **Plan `[H3]` describes `checkPartialWriteTimeout` as living at
   `display_service.cpp:578-587`** — correct — **but the plan's §Context item 2 implies it is the
   only bound on a pipe transfer.** It bounds `partialCtx` only; for a *non-partial* pipe transfer
   after a fatal NACK there is genuinely nothing, which the review states correctly under
   "Verified CORRECT" item 3. The plan text should not be read as offering partial coverage there.

9. **Neither document inventories the second power-off hold.** `power_latch`'s fixed
   `POWER_OFF_HOLD_MS = 3000` ([power_latch.cpp:21](../src/power_latch.cpp)) is entirely separate
   from `BinaryInputs.power_off_hold_sec`. Phase 2 bounds the `powerOff()` spin
   ([power_latch.cpp:87-90](../src/power_latch.cpp)) but says nothing about the fact that the
   `device_control` path reaches `powerLatchTriggerOff()`
   ([device_control.cpp:80](../src/device_control.cpp)) rather than `powerOff()`, so the two
   shutdown routes have different blocking profiles.

10. **Neither document mentions that the *entire* recovery surface depends on `loop()`.** Every
    bound in §2 and §3 except `waitforrefresh` and the nRF notify retry is evaluated from
    `loop()` or `idleDelay()`. On ESP32 that includes `processButtonEvents()`, so during a wedged
    blocking refresh **even the physical power-off hold does not work**. This matters for the
    plan's "residual risk: recoverable by power cycle" claim — on a latching device, the power
    button is itself software-mediated ([device_control.cpp:60-84](../src/device_control.cpp),
    [power_latch.cpp:124-144](../src/power_latch.cpp)) and is not a recovery path while the loop
    task is blocked.
