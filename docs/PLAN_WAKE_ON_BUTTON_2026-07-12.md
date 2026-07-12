# Wake-on-Button-Press from Deep Sleep — Implementation Plan

## Context

The ESP32 path of this firmware supports timer-based deep sleep for battery devices (`power_mode == 1`, `deep_sleep_time_seconds > 0`). Today the only wake source armed on the idle/timer sleep path is the RTC timer (`src/main.cpp:463`), with an explicit TODO at `src/main.cpp:464` to add button wake. The user wants: any configured button on a wake-capable pin should wake the display from deep sleep; on button wake, the device should stay awake for a minimum window (default 120 s) so the user can interact; this window should be unified with the existing first-boot 120 s holdoff (`FIRST_BOOT_DEEP_SLEEP_DELAY_MS`); it must be correct with battery latches (MOSFET and D-FF) and must not disturb the idle timer, post-wake advertising window, deep-sleep timer, power-off path, or nRF52840 builds.

**Deliverable 0 (user request):** save the consolidated architecture report below to `docs/architecture-deep-sleep-power-buttons.md` as the first implementation step.

---

## Step 0 — Write `docs/architecture-deep-sleep-power-buttons.md`

Create the file with exactly the following content (update line anchors if the implementation lands after other changes):

~~~markdown
# Architecture: Deep Sleep, Battery Latch, and Buttons (ESP32 path)

Status: as of branch `feat/pipe-partial`, 2026-07-12. All behavior described here is
ESP32-only (`TARGET_ESP32`); the nRF52840 target compiles these subsystems as no-op
stubs and has no deep sleep.

## 1. Deep sleep

There are two distinct deep-sleep entry points.

### 1.1 Timer deep sleep — `enterDeepSleep(bool force)` (`src/main.cpp:431-470`)

The normal battery low-power path. Guards, in order:

1. `globalConfig.power_option.power_mode != 1` (not battery) → return, no sleep.
2. `deep_sleep_time_seconds == 0` (disabled) → return.
3. A BLE client is connected and `!force` → return (live link aborts sleep).

Then: sets RTC-persisted `woke_from_deep_sleep = true`, stops advertising,
`BLEDevice::deinit(true)` + handle clear, arms the timer wake
(`esp_sleep_enable_timer_wakeup(deep_sleep_time_seconds * 1e6)`, `main.cpp:463`),
flushes the log, calls `powerLatchHoldForSleep()` (`main.cpp:468`) so a latched
device keeps its power rail across sleep, and enters `esp_deep_sleep_start()`.

Wake source on this path today: **timer only**. (`main.cpp:464` carries the TODO
for button wake.)

Callers:
- `main.cpp:250` — post-wake advertising window timed out.
- `main.cpp:363` — main idle quiet-window elapsed.
- `device_control.cpp:711` — BLE command `0x0052` (`force = true`).

### 1.2 Power-off deep sleep — `powerOff()` (`src/power_latch.cpp:83-106`)

User-initiated shutdown on the MOSFET-latch path (3 s long-press on `pwr_pin_3`, or
a `binary_inputs` button flagged `power_off`). Waits for button release, drives the
latch pin LOW, `esp_sleep_config_gpio_isolate()`, `gpio_hold_en()`, then — under
`#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP` — arms
`esp_deep_sleep_enable_gpio_wakeup(1ULL << buttonPin(), ESP_GPIO_WAKEUP_GPIO_LOW)`
on the shutdown button and calls `esp_deep_sleep_start()`. Wake source: **button
only** (or hardware re-latch if the rail actually drops).

### 1.3 Wake-cause detection (`src/main.cpp:66-83`, in `setup()`)

`esp_sleep_get_wakeup_cause()` is called once. Any cause other than
`ESP_SLEEP_WAKEUP_UNDEFINED` is treated identically as "woke from deep sleep":
sets `is_deep_sleep_wake` / `woke_from_deep_sleep`, increments RTC-persisted
`deep_sleep_count`. The specific cause (timer vs GPIO) is logged but not acted on.

### 1.4 Boot sequence differences on wake

`setup()` order: serial init → wake detect → `full_config_init()` (which calls
`powerLatchBegin()` at `config_parser.cpp:856`) → `initio()` → **if cold boot only:**
`initDisplay()` (EPD power + full refresh) → `ble_init()` → **if cold boot only:**
`initWiFi(false)` → `updatemsdata()`, `initButtons()`, `initTouchInput()` →
**if wake:** arm the post-wake advertising window (`advertising_timeout_active = true`,
`advertising_start_time = millis()`) → `lastActivityMs = millis()`.

On a deep-sleep wake, display init, boot screen, and WiFi are skipped; the e-paper
image is retained.

## 2. Sleep/wake timers

All timing state lives in `src/main.h`; config fields come from the binary TLV
config blob (LittleFS-persisted), not NVS.

| Timer / window | Definition | Default | Purpose |
|---|---|---|---|
| Deep sleep duration | `PowerOption.deep_sleep_time_seconds` (`structs.h:56`, uint16) | 0 = disabled | Timer wake interval |
| Idle quiet window ("stay awake") | `PowerOption.sleep_timeout_ms` (`structs.h:47`, uint16 ms) | 0 → `DEFAULT_IDLE_HOLD_MS` | Quiet time before sleeping; also the post-wake advertising window length |
| Idle hold fallback | `DEFAULT_IDLE_HOLD_MS` (`main.h:325`) | 10 000 ms | Fallback when `sleep_timeout_ms == 0` |
| First-boot holdoff | `FIRST_BOOT_DEEP_SLEEP_DELAY_MS` (`main.h:328`) | 120 000 ms | Cold-boot grace period before first sleep (gated by `deep_sleep_count == 0`) |
| Direct-write timeout | inline `main.cpp:293` | 900 000 ms | PIPE/direct-write session timeout |
| Power-off long-press | `POWER_OFF_HOLD_MS` (`power_latch.cpp:21`) | 3 000 ms | Latch shutdown hold |

Runtime/RTC state: `RTC_DATA_ATTR bool woke_from_deep_sleep` (`main.h:312`),
`RTC_DATA_ATTR uint32_t deep_sleep_count` (`main.h:313`),
`advertising_timeout_active` / `advertising_start_time` (`main.h:316-317`),
`lastActivityMs` (`main.h:323`), first-boot-delay state (`main.h:329-331`).

### 2.1 Loop decision flow (`src/main.cpp:219-389`)

`pollActivity()` (`main.cpp:144-187`) stamps `lastActivityMs` whenever BLE queues,
connection state, touch, LAN state, or the button MSD payload (`memcmp` of
`dynamicreturndata`) change — so a button press resets the idle timer indirectly.

- **Post-wake window branch** (`main.cpp:226-260`), active while
  `woke_from_deep_sleep && advertising_timeout_active`: a BLE connect exits the
  window into full setup; otherwise after `sleep_timeout_ms` (or 10 s) of quiet →
  `enterDeepSleep()`.
- **Normal branch**: drains BLE work; computes `workInFlight`; when idle applies the
  first-boot 120 s holdoff, then the idle quiet window → `enterDeepSleep()`.

## 3. Battery latch

Two mechanisms, both in `src/power_latch.cpp` (ESP32-only; no-op stubs otherwise),
selected by `SystemConfig.device_flags` bits and sharing pins `pwr_pin_2`/`pwr_pin_3`
(`structs.h:29-32`):

| Flag | Bit | Mechanism | pwr_pin_2 | pwr_pin_3 |
|---|---|---|---|---|
| `DEVICE_FLAG_BATTERY_LATCH` | 1<<3 | Self-holding MOSFET/load-switch | latch enable | active-low shutdown button |
| `DEVICE_FLAG_PWR_LATCH_DFF` | 1<<4 | 74AHC1G79 D flip-flop | D (`PWR_HOLD`) | CP clock (`PWR_LOCK`) |

Key behaviors:
- `powerLatchBegin()` (`power_latch.cpp:110-122`, called from `full_config_init`):
  releases any RTC hold from a prior sleep (`gpio_hold_dis`), engages the D-FF
  (drive D high, pulse CP), sets the MOSFET shutdown button to `INPUT_PULLUP`.
- `powerLatchHoldForSleep()` (`power_latch.cpp:148-167`): before timer deep sleep,
  drives the hold pin HIGH and latches it with `gpio_hold_en()` +
  `gpio_deep_sleep_hold_en()` (skipped on C6, which lacks that API) so the rail
  stays up through deep sleep. **A latched battery device therefore does enter
  timer deep sleep and self-wakes — it does not cut power on the idle path.**
- `powerOff()` (MOSFET) — see §1.2. `dffLatchRelease()` (D-FF) clocks Q low → hard
  power cut, no deep sleep; re-power is via the hardware button re-latching the FF.
- BLE command `0x0052` (`device_control.cpp:700-714`): D-FF → hard off; otherwise
  `enterDeepSleep(true)`.
- There is **no low-battery cutoff logic**; battery voltage (ADC or BQ27220) is
  measured and reported only.

## 4. Buttons

Configured via the repeatable `binary_inputs` TLV packet (id `0x25`), struct
`BinaryInputs` (`structs.h:247-268`): up to 4 instances × 8 pins = 32 buttons
(`MAX_BUTTONS`, `structs.h:400`). Per-pin bitmasks: `input_flags` (active pins),
`invert` (active-low), `pullups`, `pulldowns`, `power_off_flags`;
`power_off_hold_sec`; `reserved[12]` spare bytes. Parsed by fixed-size `memcpy`
(`config_parser.cpp:381-391`) — struct size is ABI.

Runtime:
- `initButtons()` (`device_control.cpp:551-658`): pinMode with per-pin pull config,
  `attachInterruptArg(pin, buttonISR, idx, CHANGE)`, 50 ms boot settle that resets
  `press_count` and discards spurious startup edges. Skips `0xFF` pins and the
  GT911 touch INT pin.
- ISR (`device_control.cpp:512-527`, `IRAM_ATTR`): edge-triggered; updates
  `current_state`, increments 4-bit `press_count` on press, flags
  `buttonEventPending`.
- `processButtonEvents()` (`device_control.cpp:420-452`): packs
  `(button_id | press_count<<3 | state<<7)` into `dynamicreturndata[byte_index]`
  and re-publishes the BLE advertising MSD. Buttons report to the host; they do
  not drive local page changes.
- Power-off: `pollConfiguredPowerOffButtons()` (`device_control.cpp:50-81`) for
  flagged buttons, `powerButtonPoll()` (`power_latch.cpp:124-145`) for the
  dedicated `pwr_pin_3` button.

Gaps (pre-feature): no RTC-capability validation of button pins, no
ext0/ext1/`rtc_gpio_*` usage, and normal buttons never arm a deep-sleep wake —
only the dedicated latch shutdown button does, and only on the power-off path.

## 5. Variant awareness

A single `TARGET_ESP32` macro covers all ESP32 chips (envs: classic ESP32, S3, C3,
C6). The only per-chip guards are `#if !defined(CONFIG_IDF_TARGET_ESP32C6)` around
`gpio_deep_sleep_hold_en()` and `#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP` around the
GPIO wake call in `powerOff()`. Wake-capable pins differ by chip: classic ESP32
and S2/S3 wake via ext0/ext1 on RTC GPIOs; C3/C6 wake via
`esp_deep_sleep_enable_gpio_wakeup()` on their low-power GPIO range.
~~~

---

## Verified framework facts (arduino-esp32 core 3.3.9, checked in `~/.platformio/packages/framework-arduinoespressif32-libs/<chip>/include/`)

| Capability | esp32 classic | esp32s3 | esp32c3 | esp32c6 |
|---|---|---|---|---|
| ext0 (`SOC_PM_SUPPORT_EXT0_WAKEUP`) | yes | yes | — | — |
| ext1 (`SOC_PM_SUPPORT_EXT1_WAKEUP`) | yes (ALL_LOW / ANY_HIGH only) | yes (ANY_LOW / ANY_HIGH) | — | yes (+ per-pin mode) |
| `SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP` | — | — | yes (GPIO0–5) | yes (GPIO0–7) |
| Wake-valid pins | 0,2,4,12–15,25–27,32–39 | 0–21 | 0–5 | 0–7 |

- `esp_sleep_is_valid_wakeup_gpio(gpio_num_t)` exists unguarded on every chip (`esp_sleep.h:250`) — use it as the runtime capability check; **no hand-rolled pin tables**.
- `esp_sleep_get_ext1_wakeup_status()` (unguarded) and `esp_sleep_get_gpio_wakeup_status()` (guarded by `SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP`) identify the waking pin(s). There is **no ext0 status API** — remember the armed ext0 pin in `RTC_DATA_ATTR`.
- `rtc_gpio_pullup_en` / `rtc_gpio_pulldown_en` exist in `driver/rtc_io.h`.
- `gpio_deep_sleep_hold_en()` only affects pads on which `gpio_hold_en()` was called (driver/gpio.h:433–450) — so `powerLatchHoldForSleep()` cannot freeze wake-button pads; **power_latch.cpp needs no changes**.
- `CONFIG_ESP_SLEEP_GPIO_ENABLE_INTERNAL_RESISTORS=y` in shipped sdkconfigs — for C3/C6 GPIO wake, `esp_deep_sleep_start()` auto-enables the pull opposite the wake level.
- ext1 note (esp_sleep.h:354–361): with RTC_PERIPH powered down, IDF maintains configured pulls via the HOLD feature automatically; `esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ON)` is a fallback only.
- `ESP_EXT1_WAKEUP_ANY_LOW` does **not exist** on classic ESP32.

**Unverifiable locally (precompiled libs — confirm on hardware, see validation list):**
1. Two successive `esp_deep_sleep_enable_gpio_wakeup()` calls with different levels accumulate per-pin (true in IDF v5.3–5.5 source; check return code of the second call at runtime).
2. IDF forces RTC_PERIPH on when ext0 is armed on classic ESP32.
3. Sleep-current impact of held pulls — hardware measurement.

---

## Step 1 — New module `src/wake_button.h` / `src/wake_button.cpp`

Follow the `power_latch.cpp` pattern: entire implementation inside `#if defined(TARGET_ESP32)`, empty stubs in `#else` (nRF unaffected). Include the local `DEVICE_FLAG_*` fallback defines exactly as `power_latch.cpp:10-15` does.

Public API:
```cpp
// Arm all eligible button-wake sources for the upcoming timer deep sleep.
// Never fails; logs each pin's disposition. No-op on nRF.
void armButtonWakeSources();
// Classify wake cause and log the waking pin(s). Call once, early in setup().
// Returns true for EXT0/EXT1/GPIO causes. Safe pre-config (reads sleep regs only).
bool detectButtonWake(esp_sleep_wakeup_cause_t cause);
```

Internal state: `RTC_DATA_ATTR static uint8_t s_ext0WakePin = 0xFF;` (survives sleep; only way to log the ext0 wake pin). Externs for `buttonStates[]`, `buttonStateCount`, `globalConfig` (same style as `device_control.cpp:42`).

### Wake-mask construction (all variants)

Candidates:
1. Every initialized `buttonStates[i]` pin (these already exclude `0xFF` and the GT911 touch INT pin). Wake level = pressed level = `inverted ? LOW : HIGH`.
2. `system_config.pwr_pin_3` as an **active-low** candidate **only when** `DEVICE_FLAG_BATTERY_LATCH` is set and the pin is valid — so the MOSFET-latch power button also wakes from timer sleep (mirrors `powerOff()` semantics).

Exclusions, each logged:
- `power_option.sleep_flags` bit 0 set (`SLEEP_FLAG_BUTTON_WAKE_DISABLE`) → arm nothing (feature default-ON per the "any button press should wake" principle).
- Pin == `pwr_pin_2` always; pin == `pwr_pin_3` when `DEVICE_FLAG_PWR_LATCH_DFF` (**critical**: on D-FF boards pwr_pin_3 is the flip-flop CP clock — a wake-armed pull could clock the latch off).
- `!esp_sleep_is_valid_wakeup_gpio(pin)` → warn "pin N not wake-capable on this chip; timer-only for this button".
- Pin currently reading its pressed level at sleep entry → skip this cycle, log "held at sleep entry" (prevents instant-wake ping-pong; pin re-qualifies next sleep entry after release).

Build 64-bit `lowMask` (wake on LOW) and `highMask` (wake on HIGH).

### Per-variant arming

```cpp
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP            // C3 (GPIO0-5), C6 (GPIO0-7)
    if (lowMask)  esp_deep_sleep_enable_gpio_wakeup(lowMask,  ESP_GPIO_WAKEUP_GPIO_LOW);
    if (highMask) err = esp_deep_sleep_enable_gpio_wakeup(highMask, ESP_GPIO_WAKEUP_GPIO_HIGH);
    // If the second call errors (mixed-polarity accumulation is the unverified item):
    // log, keep the first group, never abort sleep. Pulls auto-configured by IDF.
#elif SOC_PM_SUPPORT_EXT1_WAKEUP                 // classic, S2, S3
  #if CONFIG_IDF_TARGET_ESP32
    // classic has no ANY_LOW: HIGH group -> ext1 ANY_HIGH; LOW group -> ext0 (ONE pin,
    // lowest-numbered), s_ext0WakePin = pin; warn for each additional low pin not armed.
    if (highMask) esp_sleep_enable_ext1_wakeup(highMask, ESP_EXT1_WAKEUP_ANY_HIGH);
    if (lowMask)  esp_sleep_enable_ext0_wakeup(firstLowPin, 0);
  #else
    // S2/S3: larger polarity group -> ext1 (ANY_HIGH or ANY_LOW); other group's first
    // pin -> ext0; warn extras. (One ext1 call only — a second call replaces the first.)
  #endif
    // Pull retention: rtc_gpio_pullup_en()/rtc_gpio_pulldown_en() per the button's
    // configured pullups/pulldowns bitmasks (and INPUT_PULLUP for the latch button).
    // Pins with no internal pull and unknown external hardware: warn "floating wake
    // pin may cause spurious wakes" but still arm.
#endif
```

Timer wake (`esp_sleep_enable_timer_wakeup`, main.cpp:463) is **always left armed** — ext0/ext1/gpio/timer coexist. Both masks empty → single log "no wake-capable buttons — timer-only deep sleep".

### `detectButtonWake(cause)`

Switch on cause: `EXT0` (log `s_ext0WakePin`), `EXT1` (log `esp_sleep_get_ext1_wakeup_status()` mask), `GPIO` under `#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP` (log `esp_sleep_get_gpio_wakeup_status()` mask) → return true; `TIMER` → log "timer wake", return false; default → false.

**Decision:** a button wake is logged and arms the stay-awake window but does **not** inject a synthetic press into the BLE MSD payload. The press happened while the ISR was dead; `initButtons()`'s settle pass resets `press_count`/state from live pin levels anyway, so a synthetic event would be erased or double-counted. If the button is still held when `initButtons()` runs, its state lands in MSD naturally.

---

## Step 2 — Shared minimum-wake-time (the timer refactor)

### Config (`src/structs.h`)

`PowerOption` (structs.h:44-61): split `reserved[7]` (line 60) →
```cpp
uint16_t min_wake_time_seconds; // Min awake window after first boot or button wake; 0 = default 120 s
uint8_t reserved[5];
```
Struct size unchanged → the fixed-size `memcpy` parse and all existing config blobs stay valid; old blobs read 0 → default 120 s. (`sleep_timeout_ms` is uint16 **ms**, max 65.5 s — cannot hold 120 s, hence a new seconds field.)

Add near structs.h:63:
```cpp
#define SLEEP_FLAG_BUTTON_WAKE_DISABLE (1u << 0)  // power_option.sleep_flags bit 0
```
(`sleep_flags` bits are all reserved today — verified in firmware and the toolbox schema.)

### One mechanism: a minimum-wake hold deadline (`src/main.h` + `src/main.cpp`)

**Semantics: the hold is a floor layered under the existing quiet-window logic, not a replacement.** Sleep requires both the existing idle/advertising quiet condition AND the hold expired. `pollActivity()`/`lastActivityMs` untouched — interaction keeps extending the quiet window inside and beyond the floor. On timer wake the hold is never armed → behavior bit-identical to today.

`src/main.h`: delete lines 328-331 (`FIRST_BOOT_DEEP_SLEEP_DELAY_MS`, `firstBootDelayInitialized/Elapsed/Start`); add next to the advertising globals (~line 316):
```cpp
static constexpr uint16_t DEFAULT_MIN_WAKE_TIME_SECONDS = 120;
bool minWakeWindowActive = false;   // armed in setup() on first boot or button wake
uint32_t minWakeWindowStartMs = 0;
```

`src/main.cpp` helpers (near `pollActivity()`):
```cpp
static uint32_t minWakeTimeMs() {
    uint16_t s = globalConfig.power_option.min_wake_time_seconds;
    return (uint32_t)(s ? s : DEFAULT_MIN_WAKE_TIME_SECONDS) * 1000UL;
}
static bool minWakeHoldActive() {
    if (!minWakeWindowActive) return false;
    if (millis() - minWakeWindowStartMs >= minWakeTimeMs()) {
        minWakeWindowActive = false;
        writeSerial("Minimum wake window elapsed, deep sleep permitted");
        return false;
    }
    return true;
}
```

Documented behavior delta: the first-boot holdoff now counts from end of setup() rather than from the first quiet loop pass — makes "awake ≥ 120 s from power-on" a real guarantee.

---

## Step 3 — `src/main.cpp` edits

1. **setup() wake-cause block (main.cpp:66-83):** after line 68, add `bool woke_by_button = detectButtonWake(wakeup_reason);` — replaces the raw numeric log at line 74 with named cause + waking pin mask.
2. **Window arming (main.cpp:122-133):** keep the existing advertising-window arm; extend:
```cpp
if (is_deep_sleep_wake) {
    advertising_timeout_active = true;
    advertising_start_time = millis();
    if (woke_by_button) {
        minWakeWindowActive = true;
        minWakeWindowStartMs = millis();
        writeSerial("Button wake: holding awake >= " + String(minWakeTimeMs()) + " ms");
    }
} else if (deep_sleep_count == 0) {   // first boot (RTC count survives soft resets)
    minWakeWindowActive = true;
    minWakeWindowStartMs = millis();
}
```
(Arm regardless of power_mode — every consuming path is already gated by `power_mode == 1` / `deep_sleep_time_seconds > 0`, so the flag is inert on wired devices.)
3. **Post-wake advertising branch (main.cpp:246):** `if (idle_duration >= advertising_timeout_ms)` → `if (idle_duration >= advertising_timeout_ms && !minWakeHoldActive())`. Timer wake: hold never armed → unchanged short window. Button wake: window ends at max(quiet window, min wake time); `idleDelay(50)` at line 258 keeps servicing buttons/touch throughout.
4. **Delete the first-boot block (main.cpp:336-352)** — superseded by the hold.
5. **Idle gate (main.cpp:359):** `if (idleMs < idleHoldMs)` → `if (idleMs < idleHoldMs || minWakeHoldActive())`. Also covers connect-then-drop during a button-wake window (`woke_from_deep_sleep` cleared at line 231 → normal idle logic still honors the floor).
6. **`enterDeepSleep()` (main.cpp:431-470):**
   - After the connected-client guard (line 448), defense-in-depth: `if (!force && minWakeHoldActive()) return;` (`force` from BLE 0x0052 bypasses — command behavior unchanged).
   - Replace the TODO at line 464 with `armButtonWakeSources();` — after `esp_sleep_enable_timer_wakeup()` (463), **before** `powerLatchHoldForSleep()` (468). Ordering: latch-hold manipulation then can't disturb freshly configured RTC pulls, and `gpio_hold_en` applies only to the latch pin.

## Step 4 — Supporting edits

- `src/config_parser.cpp` (~line 681, config dump): print `min_wake_time_seconds` and the `sleep_flags` button-wake bit (log-based verification).
- `src/power_latch.cpp`: **no changes** — `powerOff()`/`dffLatchRelease()` hard-off paths (with `esp_sleep_config_gpio_isolate`) keep their existing single-button wake arming; `enterDeepSleep()` never calls isolate, so new wake pins stay live through timer sleep.
- Companion (separate repo, non-blocking): `opendisplay.org/httpdocs/firmware/toolbox/config.yaml` power_option — name `sleep_flags` bit 0 `button_wake_disable`; carve `min_wake_time_seconds` (2 bytes) from reserved.

---

## Step 5 — Protocol change: `0x0052` optional 2-byte sleep-duration payload

**Requirement:** `0x0052` gains an optional 2-byte big-endian payload of seconds — e.g. `00 52 00 FF` commands "sleep for 255 seconds." The value overrides `deep_sleep_time_seconds` for **exactly one** deep-sleep cycle.

### Backward compatibility — verified against the dispatcher

`imageDataWritten()` (`communication.cpp:498-506`) only requires `len >= 2` and parses the big-endian command ID from bytes 0-1; `case 0x0052` (`communication.cpp:655`) currently calls `handleDeepSleepCommand()` with no payload pointer, ignoring any trailing bytes. Therefore:
- **New host → old firmware:** `00 52 00 FF` matches `0x0052`; the extra 2 bytes are ignored; device sleeps with the config duration. Graceful degradation, no error path.
- **Old host → new firmware:** bare `00 52` yields zero payload bytes → no override → config duration. Bit-identical to today.
- Big-endian seconds matches both the user-specified example and the command-ID framing convention; the `data + 2, len - 2` payload convention matches every other parameterized command (0x70, 0x73, 0x77, …).

### Changes

1. **`src/communication.cpp:655-657`:** `case 0x0052: handleDeepSleepCommand(data + 2, len - 2); break;`
2. **`src/device_control.h:16` / `src/device_control.cpp:700-715`:** `void handleDeepSleepCommand(const uint8_t* payload, uint16_t payloadLen)`. Parse:
   - `payloadLen >= 2` → `overrideSeconds = ((uint16_t)payload[0] << 8) | payload[1]` (extra bytes beyond 2 ignored for forward compatibility). `0x0000` = explicit "no override" (sentinel, uses config).
   - `payloadLen == 1` → malformed: log warning, treat as no payload.
   - `payloadLen == 0` → no override (legacy behavior).
   - **D-FF path:** payload is meaningless (hard power cut has no timer, no self-wake) — log "duration payload ignored (D-FF hard power off)" if `overrideSeconds != 0`, then proceed with the existing ACK + `powerLatchPowerOff()` unchanged.
   - **Non-DFF path — eligibility pre-check with NACK (consistency decision, see below):** before calling `enterDeepSleep`, the handler checks the same two config guards `enterDeepSleep` enforces and reports rejection using the codebase's standard response convention (success `{0x00, cmd, 0x00, 0x00}` / error `{0xFF, cmd, code, 0x00}`, as in `handleLedActivate`, device_control.cpp:374-417):
     - `power_mode != 1` → NACK `{0xFF, 0x52, 0x02, 0x00}`, return (no sleep — wired guard, unchanged eligibility).
     - `deep_sleep_time_seconds == 0` → NACK `{0xFF, 0x52, 0x01, 0x00}`, return (**the payload does NOT enable sleep on a config-disabled device**).
     - Otherwise: `enterDeepSleep(true, overrideSeconds);`
     Backward compatible: today's non-DFF `0x0052` sends no response at all, so old hosts ignore the new unsolicited NACK; new hosts gain observable rejection.
3. **`enterDeepSleep()` signature (`src/main.cpp:431`, decl in main.h):** `void enterDeepSleep(bool force = false, uint16_t overrideSleepSeconds = 0)`. Inside:
   - **All existing guards unchanged** (power_mode 432, `deep_sleep_time_seconds == 0` 437, BLE-connected 444). The override never changes *eligibility* — only the duration of an otherwise-permitted sleep.
   - After the guards: `uint16_t sleepSeconds = overrideSleepSeconds ? overrideSleepSeconds : globalConfig.power_option.deep_sleep_time_seconds;` — timer arm (line 462-463) uses `sleepSeconds`; the entry log prints the effective duration and whether it came from the command override.

**Consistency decision — why the override does NOT enable sleep when `deep_sleep_time_seconds == 0`:** the two config guards in `enterDeepSleep` are uniformly absolute today — both reject even the forced BLE command — and `deep_sleep_time_seconds == 0` is the firmware-wide deep-sleep disable switch (idle gate, first-boot holdoff, and `enterDeepSleep` all treat 0 as "disabled"; structs.h:56 "0 if not used"). Letting a payload bypass one guard but not the other would be asymmetric, and would give the same command different *eligibility* with vs without payload (bare `0x0052` already rejects on such devices). Rule adopted: **payload = duration only; eligibility is config's alone.** The legitimate future use case (host-driven duty cycling on a device with no autonomous timer) should be an explicit opt-in — e.g. a new `sleep_flags` bit — not a silent side effect of the duration payload; noted as out of scope.
4. **One-cycle semantics by construction:** the override is threaded as a **parameter**, never stored in a global and never `RTC_DATA_ATTR`. The idle-timer callers (`main.cpp:250`, `main.cpp:363`) use the default `0` → config duration. If the guarded entry aborts (wired device), the override is discarded with the call — it cannot leak into a later sleep. After the override sleep, wake → boot reinitializes everything → every subsequent cycle uses config. No clearing logic needed, no invalid state possible.
5. **Interaction with button wake:** none required — `armButtonWakeSources()` runs identically; a button can cut an override sleep short, and the subsequent wake windows behave per the boot-side table (timer wake → short window; button wake → 120 s floor).
6. **Docs:** note the payload in `docs/architecture-deep-sleep-power-buttons.md` §3.1 and in the companion host-side command reference (separate repo, non-blocking).

### Logic table addendum (replaces/refines rows 9-10)

| # | Trigger | Latch | Payload | Behavior |
|---|---|---|---|---|
| 9a | `0x0052`, D-FF | D-FF | none | ACK, hard power cut — unchanged |
| 9b | `0x0052`, D-FF | D-FF | N seconds | payload logged + ignored, ACK, hard power cut (no timer exists once power drops) |
| 10a | `0x0052`, non-DFF | any/none | none or `0x0000` | `enterDeepSleep(true, 0)` — config duration; buttons + timer armed |
| 10b | `0x0052`, non-DFF | any/none | N seconds | sleeps N seconds (config overridden this cycle only); buttons + timer armed; next idle sleep uses config |
| 10c | `0x0052`, non-DFF, config `deep_sleep_time_seconds == 0` | any/none | any (or none) | rejected: NACK `{0xFF, 0x52, 0x01, 0x00}`, no sleep — payload does not enable a config-disabled device |
| 10d | `0x0052`, non-DFF, `power_mode != 1` | any/none | any | rejected: NACK `{0xFF, 0x52, 0x02, 0x00}`, no sleep (wired guard unchanged) |
| 10e | `0x0052`, malformed 1-byte payload | any | 1 byte | warning logged, treated as 10a |

---

## Logic / state table

Sleep-entry rows (non-forced `enterDeepSleep()`; timer source armed in every "sleeps" row):

| # | power_mode / deep_sleep_time | Latch | Buttons | Armed wake sources at sleep entry | Notes |
|---|---|---|---|---|---|
| 1 | wired (≠1) or time=0 | any | any | never sleeps (early return 432/437) | unchanged |
| 2 | battery, >0 | none | none | timer only; log "timer-only" | unchanged + 1 log |
| 3 | battery, >0 | none | all non-capable pins | timer only; per-pin warn | e.g. C3 button on GPIO9 |
| 4 | battery, >0 | none | active-low capable | classic: ext0 (first) + warn rest; S3: ext1 ANY_LOW; C3/C6: gpio LOW | |
| 5 | battery, >0 | none | active-high capable | classic/S3: ext1 ANY_HIGH; C3/C6: gpio HIGH | |
| 6 | battery, >0 | none | mixed polarity | classic: ext1 ANY_HIGH + ext0 one low; S3: ext1 larger group + ext0 first of other; C3/C6: two gpio calls (hw-verify) | warns list unarmed pins |
| 7 | battery, >0 | MOSFET | rows 2–6 | same + pwr_pin_3 joins low group if capable; pwr_pin_2 excluded, held HIGH via gpio_hold_en | power button wakes; hold unaffected |
| 8 | battery, >0 | D-FF | rows 2–6 | same, but pwr_pin_2 AND pwr_pin_3 force-excluded (CP clock) | latch cannot be clocked off |
| 9 | BLE 0x0052, D-FF | D-FF | any | hard power cut — no wake sources; duration payload ignored/logged | see Step 5 addendum |
| 10 | BLE 0x0052, non-DFF | any | any | timer (config or 2-byte payload override, one cycle) + buttons | see Step 5 addendum rows 10a-10e |
| 11 | MOSFET 3 s power-off | MOSFET | any | unchanged `powerOff()`: isolate + gpio wake on pwr_pin_3 only | untouched path |
| 12 | any sleeps-row, button held at entry | any | held pin | held pin skipped (logged); if only wake pin → timer-only this cycle | ping-pong mitigation |

Boot-side rows:

| Wake cause | woke_from_deep_sleep | Window(s) armed |
|---|---|---|
| UNDEFINED, deep_sleep_count==0 (true first boot OR hidden mid-cycle reset — see below) | false | min-wake hold (120 s default); full display init |
| UNDEFINED, deep_sleep_count>0 | false | none — defensive row only; unreachable in practice (see "Hidden mid-cycle resets") |
| TIMER | true | short advertising window only — unchanged |
| EXT0 / EXT1 / GPIO | true | advertising window + min-wake floor |
| GPIO after MOSFET `powerOff()` | per prior RTC flag | as row above when flag set; additionally gets the floor (improvement) |
| other (UART/ULP — never armed) | true | treated like timer wake (default case) — safe |

### Hidden mid-cycle resets (wake cause UNDEFINED after a crash) — focused analysis

**Question: when a hidden mid-cycle reset occurs (panic / WDT / brownout / `esp_restart()` between deep-sleep cycles), what is correct behavior — and should it do a full display init? Answer: yes, full display init is correct and required, and the plan's first-boot rule already delivers the right window behavior. Details:**

**Empirical fact (changes the state model):** `docs/FINDINGS_DEEP_SLEEP_WAKE_BOOT_SCREEN_2026-07-07.md` captured this exact scenario on hardware (reTerminal E1001): a PANIC on the wake path → `RTC_SW_CPU_RST` → `=== NORMAL BOOT ===  Deep sleep count (RTC): 0` — the count was 2 immediately before the panic. `RTC_DATA_ATTR` variables do **not** survive non-deep-sleep resets on this platform: the second-stage bootloader reloads RTC memory segments from the app image on every reset except a deep-sleep wake. So after any panic/WDT/SW/brownout reset, the device boots with `deep_sleep_count = 0`, `woke_from_deep_sleep = false`, `rebootFlag = 1`, `displayed_etag = 0` — **indistinguishable from a true first boot.** The `UNDEFINED + count>0` table row is defensive only; the code comment at `main.cpp:79-81` claiming RTC survives soft resets is contradicted by the captured log and must be fixed during implementation.

**Why full display init is correct here:**
1. **Panel controller state is unknown.** The crash may have hit mid-refresh; an EPD controller abandoned mid-waveform (rails up, charge on the panel) must be re-initialized. `initDisplay()` (rail power cycle + controller init + full refresh) is the only path that guarantees a known-good panel state. Skipping it to preserve the image would trade a cosmetic flash for an unverifiable panel state.
2. **Host re-sync is forced automatically.** `rebootFlag = 1` (reloaded initializer) is advertised in MSD, and `displayed_etag = 0` makes any partial/etag-gated write NACK into a full push. The crash recovery contract is self-healing — full init + boot screen is consistent with it.
3. **Diagnostic value.** The boot screen appearing on a battery device is precisely how the 2026-07-07 panic was discovered. Silent "seamless" recovery would hide crash loops on headless devices.

**Window behavior on a hidden reset (what this plan produces):** since the reset presents as `count == 0`, the 120 s min-wake hold arms. This is desirable: after an abnormal reset the device stays connectable long enough for the host to re-push the image (and for a developer to capture logs). It is also **not a behavior change** — today's first-boot gate (`!woke_from_deep_sleep && deep_sleep_count == 0`) already fires after a crash for the same reason, so the crash reboot already waits 2 minutes before its first sleep. A crash *loop* therefore costs ~120 s awake per crash; that is accepted — a crash loop is a firmware bug to fix, not a state to optimize battery for.

**If the defensive row ever fires** (`UNDEFINED + count>0`, e.g. a future IDF/bootloader that preserves RTC segments): no hold is armed, the device full-inits the display, and sleeps after the normal idle quiet window — safe, no invalid state. If distinguishing a true cold boot from a crash reset ever becomes necessary, the correct tool is `esp_reset_reason()` (already logged at `main.cpp:66-67`: `ESP_RST_POWERON` vs `PANIC`/`SW`/`WDT`/`BROWNOUT`), not RTC counters.

### Advertising continuity during the awake windows (EXT0/EXT1/GPIO wake) — verified

The button-wake windows do not introduce any state where the device is awake with advertising stopped:

1. **Start:** on every deep-sleep wake, `ble_init_esp32()` starts advertising unconditionally in setup (`ble_init.cpp:331`) before the windows are armed, so the post-wake branch always begins with the radio advertising.
2. **Only two deliberate stop sites exist while disconnected, and neither strands the device:**
   - `updatemsdata()` (`display_service.cpp:1376-1389`) stops advertising only to swap in a fresh MSD payload and restarts it ~50 ms later in the same call (pre-existing refresh blip, unchanged by this feature).
   - `enterDeepSleep()` stops advertising (`main.cpp:450-456`) — but every abort guard precedes that point, and after the stop there is no return path: it always reaches `esp_deep_sleep_start()`. **Ordering requirement:** the new `if (!force && minWakeHoldActive()) return;` guard MUST be inserted after the connected-client guard (line 448) and before the advertising stop (line 450), so an aborted sleep can never leave advertising stopped. (Step 3 item 6 places it there.)
3. **Disconnect healing:** a connect-then-drop sets `bleRestartAdvertisingPending` (`esp32_ble_callbacks.h:69`), serviced in both loop branches (window branch `main.cpp:236-238`, normal branch `main.cpp:288-290`). The pending flag is also in `pollActivity()`'s watch list (`main.cpp:174`), so a pending restart stamps `lastActivityMs` — no window can expire while a re-advertise is owed. `esp32_restart_ble_advertising()` defers while `epdRefreshInProgress` (`ble_init.cpp:251-253`), but that flag also sets `workInFlight` and counts as activity, so the device stays awake and retries.
4. **The hold adds no stop point:** when `minWakeHoldActive()` keeps a branch alive past its quiet timeout, the branch only runs `idleDelay(50)`/`idleDelay(5)` (buttons/touch serviced); advertising is untouched. While a client is connected, advertising is off — standard single-connection BLE peripheral behavior, not a gap.

Net guarantee: for the entire button-wake awake window (advertising window + 120 s min-wake floor), the device is either advertising, momentarily restarting advertising to refresh MSD, connected to a client, or deferring a restart behind an in-flight refresh that itself holds the device awake.

**Implementation additions from this analysis:**
- Fix the incorrect comment at `main.cpp:79-81` (RTC does not survive soft resets; a non-zero count on NORMAL BOOT is not an expected signal).
- Validation task: force a mid-cycle crash (`abort()` or a test command) between sleep cycles → verify full display init, `count` reset to 0, 120 s hold armed, host re-push succeeds (etag 0 → full write).
- **Optional hardening, out of scope for this feature:** brownout-aware boot. The findings doc (source #4) documents the loop: brownout → boot → full-refresh current spike → brownout again. A follow-up could special-case `ESP_RST_BROWNOUT` (defer the full refresh, shorten the awake window, sleep quickly to let the battery recover). Not part of the wake-on-button change.

**No invalid states:** `minWakeWindowActive` on a wired device is inert (all consumers power_mode-gated); the hold self-clears by time (single `millis()` compare, wraparound-safe subtraction); every failure mode degrades to "timer-only sleep" or "shorter window" — never to no-sleep or no-wake lockup. The only loop-like scenario (button held across sleep entry) is skipped at arming time and self-limiting regardless. No dynamic allocation, no exceptions, no unbounded loops in any new code.

**No new latency:** `armButtonWakeSources()` runs only inside `enterDeepSleep()` (already a slow teardown with delays); `detectButtonWake()` once in setup(); loop() gains exactly two short-circuit `minWakeHoldActive()` calls in branches that only execute when idle. The hot BLE/WiFi path (main.cpp:264-334), `pollActivity()`, idle-timer math, and deep-sleep timer are untouched.

---

## Validation tasks

**Compile matrix** (all must build clean): `pio run` for every ESP32 env (esp32-s3-* variants, esp32-c3-N4/N16, esp32-c6-N4, esp32-N4) plus the nRF env (must compile against wake_button stubs with no behavior change).

**Hardware, per variant (S3 + C3/C6 + classic where available):**
1. Timer wake regression: sleeps, wakes after `deep_sleep_time_seconds`, log shows "timer wake", short window, re-sleeps after quiet window.
2. Button wake: press during sleep → boots, log shows named cause + pin mask + "holding awake >= 120000 ms"; device connectable ≥ 120 s with no interaction, then sleeps.
3. `min_wake_time_seconds` override (e.g. 30) honored for both first boot and button wake.
4. First boot after flash/battery insert: ≥ 120 s before first sleep.
5. Button held at sleep entry: "held at sleep entry" logged, timer-only sleep, no wake ping-pong.
6. Non-capable pin (e.g. C3 button on GPIO > 5): warning logged, timer wake unaffected.
7. Mixed polarity on C3/C6: second `esp_deep_sleep_enable_gpio_wakeup` returns ESP_OK and both polarities wake (unverified item 1).
8. MOSFET latch board: timer sleep keeps rail up; pwr_pin_3 wakes from timer sleep; 3 s hold power-off + button-on still works; sleep current measured vs pre-change.
9. D-FF board: 0x0052 hard-off unchanged; config-button wake works; pwr_pin_2/3 never appear in the logged wake mask.
10. BLE 0x0052 on non-DFF: sleeps immediately even inside the 120 s window (force path); button press wakes it.
10a. `0x0052` payload matrix: bare `00 52` → config duration (regression); `00 52 00 1E` → wakes after 30 s, next idle sleep uses config again (one-cycle check); `00 52 00 00` → config duration; `00 52 00 FF` on a device with `deep_sleep_time_seconds = 0` → NACK `{0xFF, 0x52, 0x01, 0x00}`, stays awake; same on a wired device (`power_mode != 1`) → NACK `{0xFF, 0x52, 0x02, 0x00}`; 1-byte payload → warning + config duration; payload sent to a D-FF board → ignored-payload log + hard off unchanged.
10b. Cross-version compatibility: new host with payload against previous firmware release → device sleeps with config duration, no error (backward-compat check).
11. Idle-timer/advertising-window timings identical to current firmware logs on timer-wake cycles.
12. Log checks: config dump prints `min_wake_time_seconds`/`sleep_flags`; sleep entry prints per-pin arming dispositions; boot prints named wake cause + pin mask.

## Critical files

- `src/wake_button.h` / `src/wake_button.cpp` — new module
- `src/main.cpp` — setup() wake detect (66-83), window arming (122-133), advertising branch (246), first-boot block removal (336-352), idle gate (359), enterDeepSleep (448, 464)
- `src/main.h` — remove first-boot statics (328-331), add min-wake globals (~316)
- `src/structs.h` — `PowerOption.min_wake_time_seconds` from reserved (60), `SLEEP_FLAG_BUTTON_WAKE_DISABLE` (~63)
- `src/config_parser.cpp` — config dump line (~681)
- `src/communication.cpp` — 0x0052 dispatch passes payload (655-657)
- `src/device_control.cpp` / `.h` — `handleDeepSleepCommand(payload, len)` (700-715, decl device_control.h:16)
- `docs/architecture-deep-sleep-power-buttons.md` — new (Step 0 content above)
- `src/power_latch.cpp` — reference only, must remain unchanged
