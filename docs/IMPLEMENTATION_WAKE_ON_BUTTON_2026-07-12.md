# Implementation Summary: Wake-on-Button-Press from Deep Sleep

**Date:** 2026-07-12  **Branch:** `feat/button-wake` (from `feat/pipe-partial`)
**Plan:** `docs/PLAN_WAKE_ON_BUTTON_2026-07-12.md`
**Build status:** all five environments compile clean — `esp32-N4` (classic),
`esp32-s3-N16R8`, `esp32-c3-N4`, `esp32-c6-N4`, `nrf52840custom` — with no
warnings in project sources.

## What was implemented

### New module: `src/wake_button.h` / `src/wake_button.cpp`

Follows the `power_latch.cpp` pattern (whole implementation in
`#if defined(TARGET_ESP32)`, no-op stubs otherwise). Two functions:

- **`armButtonWakeSources()`** — called from `enterDeepSleep()` between
  `esp_sleep_enable_timer_wakeup()` and `powerLatchHoldForSleep()`. Builds
  candidate list from every initialized `buttonStates[]` entry (wake level =
  pressed level) plus `pwr_pin_3` as an active-low candidate on
  `DEVICE_FLAG_BATTERY_LATCH` boards. Exclusions, each logged:
  `SLEEP_FLAG_BUTTON_WAKE_DISABLE` (arm nothing), `pwr_pin_2` (latch hold pin),
  `pwr_pin_3` on D-FF boards (flip-flop CP clock — arming could cut power),
  pins failing `esp_sleep_is_valid_wakeup_gpio()`, and pins held at their
  pressed level at sleep entry (instant-wake ping-pong mitigation). Per-variant
  arming:
  - **C3/C6** (`SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP`): `esp_deep_sleep_enable_gpio_wakeup()`
    once per polarity group; both return codes checked and logged; a failed
    group degrades to timer-only, never aborts the sleep.
  - **Classic ESP32** (`CONFIG_IDF_TARGET_ESP32`): HIGH group → ext1 ANY_HIGH;
    LOW group → ext0 on the lowest-numbered pin (classic silicon has no
    ANY_LOW); additional LOW pins logged as unarmed.
  - **S2/S3**: larger polarity group → ext1 (ANY_HIGH or ANY_LOW); first pin of
    the other group → ext0; extras logged as unarmed.
  - ext0/ext1 pads get their configured pulls re-asserted through the RTC IO
    registers (`rtc_gpio_pullup_en`/`rtc_gpio_pulldown_en`); pads with no
    internal pull get a "floating wake pin" warning but are still armed.
  - The timer wake source is always left armed alongside the buttons.
- **`detectButtonWake(int cause)`** — called once in `setup()`. Classifies
  EXT0 (logs the armed pin from `RTC_DATA_ATTR s_ext0WakePin`; there is no ext0
  status register), EXT1 / GPIO (logs the waking pin mask from the status
  APIs), TIMER, and default. Returns true only for the button causes. Does not
  inject synthetic button press events (the press occurred while the ISR was
  dead; `initButtons()`'s settle pass would erase or double-count one).

### Shared minimum-wake window (timer refactor)

- `PowerOption.min_wake_time_seconds` (uint16, carved from `reserved[7]` →
  field + `reserved[5]`; struct size unchanged, old config blobs read 0 →
  default) and `SLEEP_FLAG_BUTTON_WAKE_DISABLE` (sleep_flags bit 0) in
  `src/structs.h`.
- `DEFAULT_MIN_WAKE_TIME_SECONDS = 120`, `minWakeWindowActive`,
  `minWakeWindowStartMs` in `src/main.h`; the four
  `FIRST_BOOT_DEEP_SLEEP_DELAY_MS` / `firstBootDelay*` lines deleted.
- `minWakeTimeMs()` / `minWakeHoldActive()` helpers in `src/main.cpp`. The hold
  is a **floor layered under the existing quiet-window logic**: sleep requires
  both the idle/advertising quiet condition AND the hold expired. Armed in
  `setup()` on (a) button wake and (b) first boot (`deep_sleep_count == 0`,
  which — per the RTC-reload finding — also covers hidden mid-cycle resets).
  Timer wakes never arm it, so their behavior is unchanged.
- Consumers: post-wake advertising branch
  (`idle_duration >= advertising_timeout_ms && !minWakeHoldActive()`), idle
  gate (`idleMs < idleHoldMs || minWakeHoldActive()`), and a defense-in-depth
  guard in `enterDeepSleep()` placed **before** the advertising stop so an
  aborted sleep can never leave the radio dark. The old first-boot block in
  `loop()` was deleted (superseded).

### `enterDeepSleep()` changes (`src/main.cpp`)

New signature `enterDeepSleep(bool force = false, uint16_t overrideSleepSeconds = 0)`
(defaults in the `main.h` declaration and the `device_control.cpp` local
re-declaration). All three pre-existing guards byte-identical. After the guards:
`sleepSeconds = override ? override : config`, used for the timer arm and the
entry log (tagged "(host override, one cycle)" vs "(config)"). The TODO comment
at the old line 464 is now the `armButtonWakeSources()` call. The incorrect
"RTC memory survives soft resets" comment in `setup()` was rewritten to state
the bootloader-reload behavior (per the 2026-07-07 findings capture).

### `0x0052` protocol extension

- `communication.cpp`: dispatch passes `data + 2, len - 2` (dispatcher already
  guarantees `len >= 2`).
- `device_control.cpp/.h`: `handleDeepSleepCommand(const uint8_t*, uint16_t)`.
  Big-endian 2-byte seconds payload; bytes beyond 2 ignored (forward compat);
  1-byte payload logs a warning and is treated as absent; `0x0000` = explicit
  no-override. **Eligibility pre-checks with NACK** (payload = duration only,
  never eligibility): `power_mode != 1` → `{0xFF, 0x52, 0x02, 0x00}`;
  `deep_sleep_time_seconds == 0` → `{0xFF, 0x52, 0x01, 0x00}`. D-FF path
  byte-identical to before (ACK + hard power off) plus an ignored-payload log.
  No ACK on the successful non-DFF path (unchanged from legacy).
- `config_parser.cpp`: config dump prints `Min Wake Time` and
  `Button Wake: enabled/disabled (sleep_flags bit0)`.

## Differences between implementation and plan

| # | Difference | Evaluation |
|---|---|---|
| 1 | **EXT0/EXT1 cases in `detectButtonWake()` are guarded by `SOC_PM_SUPPORT_EXT0/EXT1_WAKEUP`** — not in the plan. | **Required fix, found by the build matrix.** The plan's verified-framework-facts said `esp_sleep_get_ext1_wakeup_status()` is *declared* unguarded on every chip — true, but the **symbol does not link on C3** (no ext1 hardware; precompiled libs omit it). The esp32-c3-N4 build failed at link; guarding the cases is correct since those causes cannot occur on chips without the hardware, and the `default` case covers them defensively. This validates the plan's caveat that precompiled-lib behavior could not be fully verified offline. |
| 2 | `detectButtonWake` takes `int`, not `esp_sleep_wakeup_cause_t`; no separate `wokeByButton()` accessor. | Plan-sanctioned option (header must compile on nRF without `esp_sleep.h`). The accessor was unnecessary — `setup()` captures the return value in a local that spans both uses. |
| 3 | `pwr_pin_2`/`pwr_pin_3` exclusions apply only when those pins are **valid** (`!= 0 && != 0xFF`). | Refinement, agent-initiated. `0` is the "unset" sentinel for these fields; without the validity check, a real button on GPIO0 would be silently excluded on every board with no latch configured. Correct — accepted. |
| 4 | Pull configuration for wake pads is read from `globalConfig.binary_inputs[instance_index]` via `ButtonState.pin_offset`. | The plan's preferred fallback: `ButtonState` does not cache pull bits, but it does cache `instance_index`/`pin_offset`, so the config lookup is exact — no lossy heuristic needed. |
| 5 | C3/C6 path checks **both** `esp_deep_sleep_enable_gpio_wakeup()` return codes (plan required only the second, mixed-polarity call). | Strictly more defensive; no behavior downside. |
| 6 | Config dump lines placed at contextually adjacent anchors (Min Wake Time under Deep Sleep Time; Button Wake under Sleep Flags) rather than one block. | Cosmetic; better readability of the dump. |
| 7 | nRF branch of `handleDeepSleepCommand` gained `(void)payload; (void)payloadLen;`. | Warning hygiene only; no behavior. |
| 8 | S2/S3 "larger group" tie-break: equal group sizes put the HIGH group on ext1. | Plan didn't specify tie behavior; either choice is valid — at most one pin of the other polarity is relegated to ext0, which the plan requires anyway. |

Everything else matches the plan exactly: struct layout and offsets, flag bit,
API signatures, guard ordering (hold guard before advertising stop; arming
between timer arm and latch hold), one-cycle override-by-parameter, NACK codes,
hold-as-floor semantics, first-boot refactor, D-FF exclusions, and the
RTC-comment correction.

## Validation performed

1. **Diff review vs plan** — every hunk in all 9 changed/new files checked
   against the plan's steps; deviations enumerated above and each evaluated.
2. **Cross-agent consistency** — the two implementation agents worked disjoint
   file sets against a shared `enterDeepSleep(bool, uint16_t)` contract;
   signatures, field names, and macro names line up exactly (verified by
   compile, not just inspection).
3. **Compile matrix** — 5/5 environments SUCCESS after the C3 link fix
   (difference #1). No warnings in project sources.
4. **Logic checks** —
   - All new loops bounded (≤ 33 candidates, 16 hex nibbles, 64 warn-mask bits);
     no unbounded loops, no recursion, no dynamic allocation beyond the
     codebase's existing `String` logging idiom, no exceptions (none used
     anywhere in the codebase).
   - Min-wake hold self-clears by time with wraparound-safe `millis()`
     subtraction; both call sites short-circuit so the mutation only runs when
     the gate is actually consulted; a stray armed hold on a wired device is
     provably inert (every consumer sits behind `power_mode == 1` gates).
   - Every wake-arming failure degrades to timer-only sleep — no failure mode
     yields no-sleep or no-wake.
   - Advertising continuity invariant preserved: the hold guard aborts before
     the advertising stop; past the stop, `enterDeepSleep()` unconditionally
     reaches `esp_deep_sleep_start()`.
   - `0x0052` payload-length underflow impossible (`len >= 2` dispatcher
     guard); `0xFFFF` payload (≈18.2 h) safe in the 64-bit µs conversion;
     override cannot leak across cycles (parameter, never stored).

## Not yet validated (requires hardware — see plan's validation task list)

- Actual button wake per variant (classic ext0/ext1, S3, C3/C6 gpio-wake), and
  the mixed-polarity double `esp_deep_sleep_enable_gpio_wakeup()` accumulation
  on C3/C6 (plan's flagged-unverified item; the code checks and logs both
  return codes).
- MOSFET-latch board: latch held through timer sleep with wake buttons armed;
  power-button wake; `powerOff()` regression; sleep-current delta from RTC
  pulls.
- D-FF board: `0x0052` hard-off regression; `pwr_pin_2/3` never in the logged
  wake mask.
- 120 s window timing, `min_wake_time_seconds` override, first-boot window,
  held-button skip, `0x0052` payload matrix and NACKs, cross-version
  compatibility against the previous firmware release.
