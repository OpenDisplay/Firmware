# Buzzer Music Implementation (feat/buzzer-music)

Implementation notes for the quarter-tone musical buzzer rework. Protocol
surface is documented in [buzzer-protocol.md](buzzer-protocol.md); this file
records **what was built, why, and how to test it**.

---

## 1. What changed

Three coupled changes to the passive-buzzer subsystem:

| Area | Before | After |
|------|--------|-------|
| Frequency map | Linear: `400 + 11600·(idx−1)/254` Hz (~45.7 Hz/step, non-musical) | Quarter-tone scale: `Freq(idx) = 13.75 × 2^(idx/24)`, precomputed centi-Hz table |
| Waveform | GPIO bit-bang (`digitalWrite` + `delayMicroseconds` busy-wait) | Native hardware PWM (nRF52 PWM peripheral / ESP32 LEDC) |
| Playback | Blocking — handler played to completion (≤5 s) before ACK | Non-blocking state machine ticked from `loop()`; ACK on accept (≤30 s cap) |

### Files

- [src/buzzer_control.h](../src/buzzer_control.h) — generated `enum BuzzerNote : uint8_t`
  (256 note names, values = protocol indices); `buzzerService()` declaration.
- [src/buzzer_control.cpp](../src/buzzer_control.cpp) — centi-Hz table, octave
  folding, non-blocking `s_buzzer` state machine, reworked `handleBuzzerActivate`.
- [src/buzzer_hw.h](../src/buzzer_hw.h) / [src/buzzer_hw.cpp](../src/buzzer_hw.cpp)
  — **new** platform PWM layer (`buzzer_hw_tone_start` / `buzzer_hw_tone_stop`).
- [src/main.cpp](../src/main.cpp) — `buzzerService()` ticked at the top of
  `loop()`, inside `idleDelay()`, and at every other site that services
  buttons/touch/LED (the `workInFlight` branch and both loop tails), so the
  buzzer advances anywhere input is kept alive.
- [docs/buzzer-protocol.md](buzzer-protocol.md) — protocol spec updated to match.

---

## 2. Frequency scale

```
Freq(idx) = 13.75 × 2^(idx / 24)        idx 1..255;  idx 0 = nNone (rest)
```

- 24 quarter-tones per octave, anchored at A-1 = 13.75 Hz. One index = 50 cents.
- Because the anchor is A-1, **every standard 12-TET note lands on an even
  index**: `nA4 = 120` = 440.00 Hz exactly, `nC5 = 126` = 523.25 Hz,
  `nG8 = 212` = 6271.93 Hz.
- Stored as `kBuzzerCentiHzTable[256]` — **uint32 centi-Hz** (Hz × 100), 1 KB
  rodata. Not float: ESP32-C3/C6 are soft-float, and both PWM backends consume
  integer math. Entry [0] = 0 is the `nNone` sentinel.
- Regeneration one-liners live in the table/enum comments
  ([buzzer_control.cpp](../src/buzzer_control.cpp), [buzzer_control.h](../src/buzzer_control.h)).

### Octave folding

The old 400 Hz / 12 000 Hz limits are retained as `kBuzzerFreqMinCentiHz` /
`kBuzzerFreqMaxCentiHz` and derived (compile-time, `static_assert`-verified)
into playable index bounds **117–234** (403.48–11 839.82 Hz). Out-of-range
indices are shifted ±24 (one octave) at a time until in range — pitch class is
preserved, so melodies can be authored at natural pitch in any octave.

`nNone` (0) passes through folding untouched and means *silence for the step's
duration* — a musical rest that preserves rhythm.

---

## 3. Hardware PWM layer (`buzzer_hw`)

```c
bool buzzer_hw_tone_start(uint8_t pin, uint32_t centihz, uint8_t duty_percent);
void buzzer_hw_tone_stop(uint8_t pin);   // idempotent; leaves pin driven LOW
```

- **nRF52** (`ARDUINO_ARCH_NRF52`): Adafruit `HardwarePWM` instance **HwPWM3**
  (nRF52840-only instance; HwPWM0–2 left for core features) with cooperative
  `takeOwnership`/`releaseOwnership` (token `"BZZ!"`). Prescaler walk
  DIV_1→DIV_128 picks the smallest divider whose rounded COUNTERTOP fits 15 bits
  → ≤ ~0.5 cent pitch error across the whole range. Coexists with the SoftDevice
  and is immune to BLE interrupt jitter (the bit-bang tone audibly warbled
  during BLE traffic).
- **ESP32** (`ARDUINO_ARCH_ESP32`): LEDC, shimmed on `ESP_ARDUINO_VERSION_MAJOR`:
  - **3.x** (pioarduino envs): `ledcAttach(pin, hz, 10)` — integer Hz,
    ≤ 4 cents error at the 403 Hz bottom of the playable window.
  - **2.x** (legacy `platform = espressif32` envs): reserved channel 7,
    `ledcSetup(7, centihz/100.0, 10)` — double frequency, full precision.
- Unknown platforms hit `#error` at compile time.

---

## 4. Non-blocking playback

Modeled on the existing LED pattern (`s_led` + `processLedFlash()`):

- `handleBuzzerActivate` (0x0077): validate (unchanged two-pass validator and
  error codes 0x01–0x06, plus a defensive >256-byte length guard) → **preempt**
  any playing melody → **copy the payload** into `s_buzzer.melody[256]` (the
  ESP32 command queue recycles the incoming buffer on return — the copy is
  mandatory) → start the first step → **ACK immediately** ("accepted &
  started").
- `buzzerService()` ticks the state machine: advances steps/pattern-gaps/outer
  repeats on `millis()` deadlines (wraparound-safe), enforces the 30 s cap and
  20 ms inter-pattern gap as *states*, not delays. Called from the top of
  `loop()` and from inside `idleDelay()` so melodies keep advancing during idle
  windows (worst-case step-timing drift ±100 ms there; tight during normal
  operation).
- Hardware PWM sustains the tone between ticks with **zero CPU**.
- `passiveBuzzerPowerOffAlert()` stays **blocking** (runs during shutdown):
  preempts any melody, then two 80 ms beeps at `nG8` (6271.93 Hz — nearest
  quarter-tone to the old ~6200 Hz alert, so it sounds unchanged).

### Deviations from the original plan

1. **Env-name flavor mapping**: in `platformio.ini`, `esp32-N4` is the
   pioarduino/core-3.x env and `esp32-c3-N4` is the legacy core-2.x env
   (the plan's examples had them swapped). One of each was built.
2. **Min/max index derivation** uses recursive `constexpr` scans (the nRF52
   core compiles `-std=gnu++11`, which forbids loops in `constexpr`); results
   are locked by `static_assert(117 / 234)`.
3. **Added a defensive length guard** (`len > 256` → error 0x05) before the
   payload copy; unreachable within a single BLE MTU but prevents any
   overflow by construction.
4. **`idleDelay()` also ticks `buzzerService()`** (added during review):
   without it, a melody started before a long idle wait would freeze mid-note
   with the tone sustaining for seconds.

---

## 5. Test plan

### Build matrix (all verified compiling)

```
pio run -e nrf52840custom     # nRF52840, HardwarePWM path
pio run -e esp32-N4           # ESP32, Arduino core 3.x, LEDC ledcAttach path
pio run -e esp32-c3-N4        # ESP32-C3, Arduino core 2.x, LEDC ledcSetup shim
```

### Bench tests over BLE

Each string below is a complete command frame (2-byte big-endian opcode +
payload) ready to paste into the opendisplay.org **Debug CMD (Hex)** box. The
strings are written **without spaces or `0x`** on purpose: `sendHexCommand()`
([ble-common.js](../../opendisplay.org/httpdocs/js/ble-common.js)) validates the
raw string length and slices the command ID **before** stripping non-hex
characters, so any spaces make an even-byte frame fail with
`Invalid hex command format`. The web tool encrypts the frame automatically once
authenticated (every command except `0x0050`/`0x0043`), so send the plaintext
below — don't pre-encrypt. Expected response for each accepted command:
`00 77 00 00`, arriving **immediately** (not after playback).

**T1 — Exact octave (scale correctness).** A4 440 Hz 200 ms, rest 50 ms,
A5 880 Hz 200 ms. The two tones must sound exactly one octave apart — the old
linear map could not produce a true octave.

```
0077000101037828000A9028
```

**T2 — Octave folding (fold-up).** nC4 (idx 102, below the playable window)
then nC5 (idx 126), sent as two commands. Both 200 ms; they must sound
**identical** (both 523.25 Hz).

```
0077000101016628
0077000101017E28
```

**T3 — Octave folding (fold-down).** idx 255 folds to idx 231 → 10 857 Hz
(high but audible), not 21.7 kHz.

```
007700010101FF28
```

**T4 — Non-blocking + preempt.** Send in order: a 5 s melody (A4, 1250 ms × 4
repeats), then a firmware-version read (must respond *while* the tone plays),
then a short C6 beep (must cut the long melody off — preempt):

```
00770004010178FA
0043
0077000101019614
```

**T5 — Jitter (the PWM win).** One steady A4 for 1 s while BLE traffic /
display writes are active. Pitch must be rock-steady on nRF52, where the old
bit-bang warbled under SoftDevice load.

```
00770001010178C8
```

**T6 — C-major scale, C5→C6** (100 ms per note; also a pleasant sanity check
that steps are musical):

```
0077000101087E148214861488148C14901494149614
```

**T7 — Quarter-tone glissando** (12 quarter-tones up from A4, 50 ms each —
should sound like a smooth microtonal slide, each step audibly *half* a
semitone):

```
00770001010C780A790A7A0A7B0A7C0A7D0A7E0A7F0A800A810A820A830A
```

**T8 — Rest rhythm + patterns + repeat** (protocol doc's worked example: two
patterns with a rest and the 20 ms inter-pattern gap, played twice — verifies
rests keep time and gaps/repeats survive the non-blocking rewrite):

```
0077000202029414000A01AC28
```

**T9 — A little tune** ("Twinkle Twinkle" opening, C5 C5 G5 G5 A5 A5 G5):

```
0077000101077E147E148C148C14901490148C28
```

**T10 — "Ode to Joy"** (Beethoven's 9th, first 4 measures: E E F G · G F E D ·
C C D E · E. D D, in octave 5 at ~400 ms/quarter — 15 notes, 6.4 s total,
exercises the longer cap and a dotted-quarter/eighth/half rhythm):

```
00770001010F8650865088508C508C508850865082507E507E50825086508678822882A0
```

**T11 — Power-off alert.** Hold the power-off button: the two-beep chirp must
sound essentially identical to the previous firmware (6271.93 vs ~6200 Hz =
+20 cents), and must cleanly cut off any melody still playing.

### Error-path spot checks (unchanged codes)

| Send | Meaning | Expected response |
|------|---------|-------------------|
| `0077000100` | `pattern_count = 0` | `FF 77 04 00` |
| `0077050101017828` | instance 5 (absent) | `FF 77 02 00` |
| `0077000101027828` | declares 2 steps, sends 1 | `FF 77 05 00` |
| `0077000101017828EE` | trailing byte | `FF 77 06 00` |
