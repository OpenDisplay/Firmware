# Architecture: Deep Sleep, Battery Latch, and Buttons (ESP32 path)

Status: as of branch `feat/button-wake`, 2026-07-12 (includes the wake-on-button
feature — see `docs/IMPLEMENTATION_WAKE_ON_BUTTON_2026-07-12.md`). All behavior
described here is ESP32-only (`TARGET_ESP32`); the nRF52840 target compiles these
subsystems as no-op stubs and has no deep sleep.

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

Wake sources on this path: **timer plus every wake-capable configured button**
(`armButtonWakeSources()` in `src/wake_button.cpp`, called after the timer arm and
before `powerLatchHoldForSleep()`). Button wake is default-on; `power_option.
sleep_flags` bit 0 (`SLEEP_FLAG_BUTTON_WAKE_DISABLE`) opts a device out. The
duration can be overridden for one cycle by the `0x0052` payload (see §3.1).

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
`ESP_SLEEP_WAKEUP_UNDEFINED` is treated as "woke from deep sleep": sets
`is_deep_sleep_wake` / `woke_from_deep_sleep`, increments RTC-persisted
`deep_sleep_count`. `detectButtonWake()` (`src/wake_button.cpp`) classifies the
cause: EXT0/EXT1/GPIO (button) wakes additionally arm the minimum-wake hold
(default 120 s, `PowerOption.min_wake_time_seconds`) so the device stays
connectable; timer wakes keep the short advertising window only.

**RTC persistence caveat:** `RTC_DATA_ATTR` variables survive deep sleep only.
On any other reset (panic, WDT, `esp_restart()`, brownout) the second-stage
bootloader reloads the RTC memory segments from the app image, so
`deep_sleep_count`, `woke_from_deep_sleep`, `rebootFlag`, and `displayed_etag`
all return to their initializers. This was observed on hardware in
`FINDINGS_DEEP_SLEEP_WAKE_BOOT_SCREEN_2026-07-07.md` (count 2 → panic → NORMAL
BOOT with count 0). Consequence: a hidden mid-cycle reset is indistinguishable
from a true first boot (`UNDEFINED` + `count == 0`) and takes the full cold-boot
path — display re-init (guaranteeing a known-good panel state after a possible
mid-refresh crash), `rebootFlag = 1` advertised, `displayed_etag = 0` forcing a
host full re-push, and the first-boot sleep holdoff. This is the intended
crash-recovery behavior. (The comment at `main.cpp:79-81` claiming RTC survives
soft resets is contradicted by the captured log.) To distinguish a cold boot
from a crash reset, use `esp_reset_reason()` — `ESP_RST_POWERON` vs
`PANIC`/`SW`/`WDT`/`BROWNOUT` — not RTC counters.

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

### 3.1 D-FF hard power off vs non-DFF deep sleep (BLE `0x0052`)

The same host command produces fundamentally different power states depending on
the latch hardware:

- **D-FF boards (`DEVICE_FLAG_PWR_LATCH_DFF`): hard power off.** The 74AHC1G79
  flip-flop's Q output gates the power rail. `dffLatchRelease()`
  (`power_latch.cpp:63-81`) drives D (`pwr_pin_2`) LOW and pulses CP
  (`pwr_pin_3`) → Q latches low → the rail is physically cut. This is not a
  sleep state: the MCU loses power entirely, so **no firmware wake source
  (timer, GPIO, button) exists** — RTC memory, wake configuration, and held
  GPIOs are all gone. The only way back is the hardware power button, which
  re-clocks the flip-flop and re-latches Q high, producing a cold boot
  (`ESP_RST_POWERON`, `deep_sleep_count == 0`, full display init).
  `handleDeepSleepCommand()` sends the ACK *before* releasing the latch, since
  nothing can be sent afterward.
- **Non-DFF boards (MOSFET latch or no latch): timer deep sleep.** `0x0052`
  calls `enterDeepSleep(true)` — the MCU stays powered (a MOSFET latch is held
  via `powerLatchHoldForSleep()`), the RTC timer wake stays armed, and the
  device self-wakes after `deep_sleep_time_seconds`. `force = true` bypasses
  the connected-client guard because the command inherently arrives over a
  live BLE link.

Implication for button wake: on the non-DFF path, `0x0052` flows through
`enterDeepSleep()`, so button wake sources armed there also apply to a
commanded sleep. On D-FF boards, wake arming is meaningless after `0x0052`
(power is cut), and during normal *timer* sleep `pwr_pin_3` must never be
armed as a wake pin — it is the flip-flop's CP clock input, and driving or
pulling it could clock the latch and power the board off
(`armButtonWakeSources()` force-excludes it on D-FF boards).

**`0x0052` payload (protocol extension):** an optional 2-byte big-endian
seconds payload overrides the configured sleep duration for exactly one cycle
(`00 52 00 FF` = "sleep 255 s"); `0x0000` or no payload = config duration. The
payload changes only the duration, never eligibility: on non-DFF boards the
command is NACKed with `{0xFF, 0x52, 0x01, 0x00}` when
`deep_sleep_time_seconds == 0` (deep sleep disabled in config) and
`{0xFF, 0x52, 0x02, 0x00}` when `power_mode != 1` (not battery). D-FF boards
ignore the payload (logged) — a hard power cut has no timer. Backward
compatible both directions: old firmware ignores the extra bytes; old hosts
send no payload and may ignore the new NACKs.
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

### Wake-capability matrix (verified against arduino-esp32 core 3.3.9)

| Capability | esp32 classic | esp32s3 | esp32c3 | esp32c6 |
|---|---|---|---|---|
| ext0 (`SOC_PM_SUPPORT_EXT0_WAKEUP`) | yes | yes | — | — |
| ext1 (`SOC_PM_SUPPORT_EXT1_WAKEUP`) | yes (ALL_LOW / ANY_HIGH only) | yes (ANY_LOW / ANY_HIGH) | — | yes (+ per-pin mode) |
| `SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP` | — | — | yes (GPIO0–5) | yes (GPIO0–7) |
| Wake-valid pins | 0,2,4,12–15,25–27,32–39 | 0–21 | 0–5 | 0–7 |

- `esp_sleep_is_valid_wakeup_gpio(gpio_num_t)` exists unguarded on every chip — the
  correct runtime capability check (no hand-rolled pin tables needed).
- `esp_sleep_get_ext1_wakeup_status()` / `esp_sleep_get_gpio_wakeup_status()`
  identify the waking pin(s); there is **no ext0 status API** (the armed ext0 pin
  must be remembered in RTC memory).
- `gpio_deep_sleep_hold_en()` only affects pads on which `gpio_hold_en()` was
  called, so the power-latch hold cannot freeze other (e.g. wake-button) pads.
- `CONFIG_ESP_SLEEP_GPIO_ENABLE_INTERNAL_RESISTORS=y` in the shipped sdkconfigs:
  for C3/C6 GPIO deep-sleep wake, IDF auto-enables the pull opposite the wake level.
- `ESP_EXT1_WAKEUP_ANY_LOW` does **not exist** on classic ESP32 — multiple
  active-low wake buttons on classic silicon require ext0 (one pin) alongside
  ext1 ANY_HIGH for active-high buttons.

### 5.1 Button-wake capability matrix (as implemented in `armButtonWakeSources()`)

| | Classic ESP32 (`esp32-N4`) | ESP32-S2 *(no env; code covers it)* | ESP32-S3 (`esp32-s3-*`) | ESP32-C3 (`esp32-c3-N4/N16`) | ESP32-C6 (`esp32-c6-N4`) | nRF52840 (`nrf52840custom`) |
|---|---|---|---|---|---|---|
| **Wake-capable pins** | RTC GPIOs: 0, 2, 4, 12–15, 25–27, 32–39 (18 pins) | GPIO 0–21 (22 pins) | GPIO 0–21 (22 pins) | GPIO 0–5 only (6 pins) | GPIO 0–7 only (8 pins) | n/a — no deep sleep path |
| **Mechanism firmware uses** | ext1 ANY_HIGH + ext0 | ext1 (one mode) + ext0 | ext1 (one mode) + ext0 | `esp_deep_sleep_enable_gpio_wakeup` | `esp_deep_sleep_enable_gpio_wakeup` (LP GPIO; ext1 exists but unused) | none |
| **Active-high buttons wakeable** | all (ext1 ANY_HIGH) | all, if HIGH is the larger group; else 1 via ext0 | same as S2 | all on GPIO 0–5 | all on GPIO 0–7 | 0 |
| **Active-low buttons wakeable** | **1 only** (ext0; classic has no ANY_LOW) | all, if LOW is the larger group; else 1 via ext0 | same as S2 | all on GPIO 0–5 | all on GPIO 0–7 | 0 |
| **Mixed polarity** | all HIGH + 1 LOW | larger group in full + 1 of the other | same as S2 | both groups in full (two gpio-wake calls — needs hardware confirm) | both groups in full (same caveat) | n/a |
| **Which pin woke us, in logs** | ext1: status mask; ext0: pin remembered in RTC RAM (`s_ext0WakePin`) | same | same | status mask | status mask | n/a |
| **Pull handling during sleep** | `rtc_gpio_pullup/pulldown_en` re-asserted per config | same | same | auto: IDF enables the pull opposite the wake level | same as C3 | n/a |

Practical readings:

- **The worst case is classic ESP32 with multiple active-low buttons** — the
  common wiring (button to GND, internal pullup). Only the lowest-numbered one
  wakes; the rest are logged as "not armed … timer-only for this button." A
  classic-ESP32 board needing several wake buttons should wire them
  active-high (or use an S3).
- **C3 is the most pin-constrained**: a button on GPIO 6+ can never wake
  (logged as not wake-capable). C6 extends the window to GPIO 7.
- **S3 is the most capable** target: 22 wake-capable pins and whole-group wake
  for either polarity; only a *mixed* population costs anything (the minority
  polarity gets one ext0 slot).
- Ceilings before pin capability applies: 32 configured buttons max
  (`MAX_BUTTONS`), minus exclusions — the latch hold pin (`pwr_pin_2`), the
  D-FF clock pin (`pwr_pin_3` on D-FF boards), the touch INT pin, and any
  button held down at sleep entry (skipped for that cycle).
- On MOSFET-latch boards, `pwr_pin_3` (the power button, active-low) joins the
  LOW group — on classic ESP32 it competes for the single ext0 slot with any
  other active-low button (lowest pin number wins).
