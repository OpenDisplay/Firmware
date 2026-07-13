# Passive Buzzer Protocol

Configuration and BLE control for passive (piezo) buzzers on OpenDisplay panels.
Config block **0x29**, activate opcode **0x0077**.

This document is written against the firmware in
[src/buzzer_control.cpp](../src/buzzer_control.cpp),
[src/buzzer_control.h](../src/buzzer_control.h),
[src/structs.h](../src/structs.h),
[src/config_parser.cpp](../src/config_parser.cpp),
[src/communication.cpp](../src/communication.cpp),
[src/buzzer_hw.cpp](../src/buzzer_hw.cpp), and
[src/buzzer_hw.h](../src/buzzer_hw.h). It applies to both the
`nrf52840custom` and `esp32-s3-*` targets — the handler and its non-blocking
playback state machine are shared across both.

---

## 1. Overview

A *passive* buzzer has no internal oscillator: it emits sound only while driven
with an AC / square-wave signal, and its pitch is set by the drive frequency.
The firmware therefore generates the waveform itself and plays caller-supplied
**patterns** — ordered lists of (frequency, duration) steps — over BLE.

Up to **4 buzzer instances** are supported. Each is described by a config block
at provisioning time and addressed by instance index at runtime.

Two entry points exist:

| Trigger | Source | Behaviour |
|---------|--------|-----------|
| `0x0077` activate command | client→device over BLE | Plays a caller-defined pattern |
| Power-off alert | internal ([device_control.cpp:77](../src/device_control.cpp#L77)) | Fixed two-beep chirp before power latch off |

> **Non-blocking note.** The firmware generates the waveform with the native
> hardware PWM peripheral (nRF52 PWM / ESP32 LEDC — see `buzzer_hw_tone_start`)
> and sequences steps from a state machine ticked by the main loop
> (`buzzerService`), not with `delay()`. The `0x0077` handler validates the
> payload, starts playback, and returns **immediately** — the ACK means
> "accepted & playback started," **not** "finished playing." Playback continues
> in the background; a new `0x0077` received while a melody is playing
> **preempts** it (stops the current melody and starts the new one). The
> power-off alert (§8) is the one exception that still blocks.

---

## 2. Configuration block — `0x29` `passive_buzzer`

Repeatable, **max 4 instances**. Fixed **32-byte** record
([structs.h:209-216](../src/structs.h#L209-L216); `static_assert` enforced in
[buzzer_control.cpp](../src/buzzer_control.cpp)). Parsed in
[config_parser.cpp:409-416](../src/config_parser.cpp#L409-L416); instances beyond
the 4th are skipped with a warning.

| Offset | Field | Type | Meaning |
|-------:|-------|------|---------|
| 0 | `instance_number` | `uint8` | Instance index (0-based) |
| 1 | `drive_pin` | `uint8` | GPIO driving the buzzer (via transistor). `0xFF` = unconfigured |
| 2 | `enable_pin` | `uint8` | Optional enable/FET gate. `0xFF` = unused |
| 3 | `flags` | `uint8` | Bit 0 `BUZZER_FLAG_ENABLE_ACTIVE_HIGH`; others reserved |
| 4 | `duty_percent` | `uint8` | PWM duty 1–100. `0` or `>100` → defaults to **50** |
| 5–31 | `reserved[27]` | — | Reserved, must be 0 |

**Enable-pin polarity.** When `enable_pin != 0xFF`, the firmware asserts it
around playback. If `BUZZER_FLAG_ENABLE_ACTIVE_HIGH` is set the pin is driven
HIGH to enable; otherwise it is active-low
([buzzer_control.cpp](../src/buzzer_control.cpp)).

**Init.** At boot `initPassiveBuzzers()` sets each configured `drive_pin` as an
output driven LOW and de-asserts each `enable_pin`
([buzzer_control.cpp](../src/buzzer_control.cpp)).

---

## 3. Activate command — `0x0077`

### 3.1 Framing

The 2-byte **big-endian** opcode `0x00 0x77` is stripped by the dispatcher before
the handler runs ([communication.cpp:647-649](../src/communication.cpp#L647-L649)).
The layout below describes the **post-opcode** payload; `data[0]` is the first
byte after the opcode. All fields are single bytes (endianness N/A).

### 3.2 Payload layout

```
byte 0        instance          buzzer instance index
byte 1        outer_repeat      whole-sequence repeat count; 0 is treated as 1
byte 2        pattern_count     number of patterns that follow (must be >= 1)

then, repeated pattern_count times — one pattern:
  byte        nsteps            number of steps in this pattern
  then, repeated nsteps times — one step:
    byte      freq_idx          0 = rest/silence; 1..255 -> tone (see §4)
    byte      dur_unit          duration in 5 ms units (effective ms = dur_unit * 5)
```

A pattern with `nsteps = 0` is legal (contributes only its inter-pattern gap).

### 3.3 Validation

The handler performs a two-pass parse. It first walks the whole payload to verify
every declared pattern/step fits exactly within `len`, then plays it. Rejections
(§6) are returned **before** any sound is produced. The payload must terminate
**exactly** at the end of the last step — trailing bytes are an error (code `0x06`).

---

## 4. Frequency mapping

`freq_idx` is an 8-bit index, **not** a raw frequency. The index maps onto a
**quarter-tone musical scale** — 24 equal steps per octave, anchored at
A-1 = 13.75 Hz:

```
Freq(idx) = 13.75 × 2^(idx / 24)                 for idx in 1..255
```

- `0` → **silence / rest** (`nNone` — pin held low for the step duration; a
  musical rest that preserves melody rhythm).
- `1..255` → a tone. One index step is exactly **one quarter-tone** (50 cents),
  so two indices = one semitone and 24 indices = one octave. Convenient
  landmarks: `idx = 120` → **A4 = 440.00 Hz** exactly, `idx = 255` →
  21714.33 Hz. All standard 12-TET notes land on **even** indices (see the
  appendix).

Tones are generated by **hardware PWM** (nRF52 PWM peripheral / ESP32 LEDC)
from a precomputed centi-Hz table, so the actual pitch is accurate to within a
few cents across the range.

### Octave folding

The buzzer's usable output is bounded by the retained hardware limits **400 Hz**
and **12 000 Hz** (`kBuzzerFreqMinCentiHz` / `kBuzzerFreqMaxCentiHz`), which
restrict the directly playable indices to **117–234** (403.48 Hz … 11 839.82 Hz).

An index outside that window is **not** clamped or rejected: it is shifted
**±24 (one octave) at a time** until it lands inside 117–234, preserving its
pitch class. For example `idx = 102` (nC4) folds **up** to `idx = 126`
(nC5, 523.25 Hz), and `idx = 255` folds **down** to `idx = 231` (10 857.16 Hz).
Folding happens centrally at the single table lookup, so clients may compose a
melody in **any** octave and trust that every note sounds at its nearest
in-range octave.

**Duty cycle** comes from the config (`duty_percent`, default 50%), clamped so the
on-time is at least 1 µs and strictly less than the period.

---

## 5. Timing & playback semantics

| Constant | Value | Meaning |
|----------|------:|---------|
| `kBuzzerDurationUnitMs` | 5 ms | One `dur_unit` |
| `kBuzzerInterPatternGapMs` | 20 ms | Silent gap inserted **between** patterns |
| `kBuzzerMaxTotalMs` | 30000 ms | Hard cap on total playback wall-time |

Playback order (advanced by the `buzzerService` state machine):

1. Repeat the whole sequence `outer_repeat` times.
2. Within each repetition, play patterns in order; insert a 20 ms gap **between**
   consecutive patterns (not before the first, not after the last).
3. Within each pattern, play its steps in order. Per step: assert enable, drive
   `Freq(freq_idx)` for `dur_unit * 5 ms`.

**Non-blocking servicing.** Playback is a state machine ticked from the main
loop (`buzzerService`, called beside the LED flasher). The handler returns as
soon as the melody starts; steps advance on subsequent loop passes. Loop passes
are fast during normal operation, so step boundaries are tight; the one
exception is the ESP32 deep-sleep idle window (`idleDelay(50)`), where step
timing can drift by up to **±50 ms** — the same tolerance the LED flasher
already accepts.

**30-second cap.** Before each step the elapsed time since playback start is
checked. At/after 30000 ms playback stops immediately (`capped`), and a step that
would overrun the cap is truncated to the remaining time. This bounds total
playback wall-time regardless of `outer_repeat` or payload size.

At the end (normal, capped, or **preempted** by a new `0x0077`) the enable pin
is de-asserted and the drive pin is driven low.

Per-step maximum duration is `255 * 5 = 1275 ms` (`dur_unit` is a single byte).

---

## 6. Responses

The device replies on the notify channel with a fixed **4-byte** frame:

```
byte 0   status     0x00 = success, 0xFF = error (NACK)
byte 1   0x77       opcode echo (low byte)
byte 2   code       error code (0x00 on success)
byte 3   0x00       reserved
```

| Status | code | Meaning | Cause |
|--------|------|---------|-------|
| `0x00` | `0x00` | Success | Payload accepted; playback started (sent immediately, before the melody finishes) |
| `0xFF` | `0x01` | Truncated header | Post-opcode `len < 3` |
| `0xFF` | `0x02` | Invalid instance | `instance >= passive_buzzer_count` |
| `0xFF` | `0x03` | Unconfigured buzzer | Selected instance has `drive_pin == 0xFF` |
| `0xFF` | `0x04` | No patterns | `pattern_count == 0` |
| `0xFF` | `0x05` | Truncated body | A declared pattern/step runs past `len` |
| `0xFF` | `0x06` | Trailing bytes | Payload does not end exactly at the last step |

The ACK is sent right after validation; it does not report anything that
happens during playback. A later truncation by the 30 s cap, or preemption by a
subsequent `0x0077`, is **not** an error and produces no further response.

---

## 7. Worked example

Play on **instance 0**: two "beep-beep" patterns, whole thing twice.

- Pattern A: 2 steps — 1 kHz for 100 ms, then rest 50 ms.
- Pattern B: 1 step — 2 kHz for 200 ms.

Compute indices with `idx = round(24 × log2(hz / 13.75))`:
- 1 kHz → `idx = 148` (`0x94`; `Freq(148) = 987.77 Hz` — nB5)
- 2 kHz → `idx = 172` (`0xAC`; `Freq(172) = 1975.53 Hz` — nB6)

Durations: 100 ms → `20` units, 50 ms rest → `10` units (`freq_idx = 0`),
200 ms → `40` units.

Full BLE write (opcode + payload):

```
00 77                      ; opcode 0x0077 (big-endian)
00                         ; instance = 0
02                         ; outer_repeat = 2
02                         ; pattern_count = 2
   02                      ; pattern A: nsteps = 2
      94 14                ;   step: idx 148 (~1kHz), 100 ms
      00 0A                ;   step: rest, 50 ms
   01                      ; pattern B: nsteps = 1
      AC 28                ;   step: idx 172 (~2kHz), 200 ms
```

Total nominal time ≈ `2 * (100 + 50 + 20gap + 200) = 740 ms`, well under the cap.
Expected response: `00 77 00 00`.

---

## 8. Power-off alert (internal)

`passiveBuzzerPowerOffAlert()` is **not** reachable over BLE. It is invoked when
a button's power-off hold threshold is met, immediately before
`powerLatchTriggerOff()`. It plays a fixed two-beep chirp — `nG8` (idx 212) ≈
6271.93 Hz (the nearest quarter-tone to the old ~6200 Hz target), 80 ms on /
80 ms gap / 80 ms on — on the first buzzer whose `drive_pin` is neither `0` nor
`0xFF`. Unlike the `0x0077` path this routine stays **blocking** (it runs
synchronously during shutdown, where there is nothing left to starve), and it
preempts any melody still playing before it beeps.

---

## 9. Client checklist

- Address a buzzer by its config `instance_number`; confirm it is configured
  (else `0x03`).
- Convert desired Hz to `freq_idx` with the inverse of §4:
  `idx = clamp(round(24 × log2(hz / 13.75)), 1, 255)`; use `0` for a rest.
- Don't worry about octave range: any octave is safe to send. Notes outside the
  playable 117–234 window are octave-folded into range (§4), preserving pitch
  class — so you can author melodies at their natural pitch and let the firmware
  place them.
- Quantise durations to 5 ms units (`dur_unit = round(ms / 5)`, max 255).
- Keep total nominal time under 30000 ms or expect silent truncation.
- Ensure the payload ends exactly at the final step (no padding) to avoid `0x06`.
- Playback is non-blocking and the ACK arrives immediately (accepted & started),
  so you may pipeline other commands behind a long buzzer play. Note that
  sending another `0x0077` before the melody finishes **preempts** it.

---

## Appendix A. Note-name ↔ index reference

The firmware header (`enum BuzzerNote : uint8_t` in
[src/buzzer_control.h](../src/buzzer_control.h)) names every index so melodies
can be authored by note rather than by number, e.g. `{nA4, 40, nNone, 10, nA5, 40}`.

**Naming convention:** `n<Note><s?><octave><p?>`

- `n` prefix on every enumerator; the note letter is `A`–`G`.
- `s` = sharp (e.g. `nCs5` = C♯5).
- optional trailing `p` = **plus one quarter-tone** (the odd index halfway to the
  next semitone), e.g. `nC5p`, `nGs5p`.
- `nNone = 0` is the rest/silence sentinel.
- octave `−1` is spelled `m1`: the five sub-C0 tones are
  `nAm1p, nAsm1, nAsm1p, nBm1, nBm1p` at indices **1–5** (index 0 would be nA-1
  but is reserved as `nNone`).

**Index arithmetic.** Octaves increment at C. **C0 = idx 6** and **each octave =
24 indices**, so:

| Anchor | Index | Frequency |
|--------|------:|----------:|
| C0 | 6 | 16.35 Hz |
| C1 | 30 | 32.70 Hz |
| C2 | 54 | 65.41 Hz |
| C3 | 78 | 130.81 Hz |
| C4 | 102 | 261.63 Hz |
| C5 | 126 | 523.25 Hz |
| C6 | 150 | 1046.50 Hz |
| C7 | 174 | 2093.00 Hz |
| C8 | 198 | 4186.01 Hz |
| C9 | 222 | 8372.02 Hz |
| C10 | 246 | 16744.04 Hz |

All standard 12-TET notes land on **even** indices; the odd indices between them
are the `p` (+quarter-tone) microtones. For any note, take its octave's C anchor
and add the semitone offset ×2 (C +0, C♯ +2, D +4, D♯ +6, E +8, F +10, F♯ +12,
G +14, G♯ +16, A +18, A♯ +20, B +22), then +1 for a `p` variant. Landmarks:
`nA4 = 120` = 440.00 Hz, `nC5 = 126` = 523.25 Hz, `nG8 = 212` = 6271.93 Hz. The
table spans `nAm1p = 1` (14.15 Hz) to `nE10p = 255` (21714.33 Hz).

**C4–C6 reference** (12-TET semitones; frequencies from
`round(100 × 13.75 × 2^(idx/24)) / 100`):

| Note | Index | Freq (Hz) | Note | Index | Freq (Hz) |
|------|------:|----------:|------|------:|----------:|
| nC4  | 102 | 261.63 | nC5  | 126 | 523.25 |
| nCs4 | 104 | 277.18 | nCs5 | 128 | 554.37 |
| nD4  | 106 | 293.66 | nD5  | 130 | 587.33 |
| nDs4 | 108 | 311.13 | nDs5 | 132 | 622.25 |
| nE4  | 110 | 329.63 | nE5  | 134 | 659.26 |
| nF4  | 112 | 349.23 | nF5  | 136 | 698.46 |
| nFs4 | 114 | 369.99 | nFs5 | 138 | 739.99 |
| nG4  | 116 | 392.00 | nG5  | 140 | 783.99 |
| nGs4 | 118 | 415.30 | nGs5 | 142 | 830.61 |
| nA4  | 120 | 440.00 | nA5  | 144 | 880.00 |
| nAs4 | 122 | 466.16 | nAs5 | 146 | 932.33 |
| nB4  | 124 | 493.88 | nB5  | 148 | 987.77 |
| nC6  | 150 | 1046.50 |      |     |        |

Every octave repeats the same pattern shifted by 24 indices; use the formula
above for notes outside this range. Remember the playable window is idx 117–234
(§4) — `nG4p` (403.48 Hz) up to `nFs9` (11 839.82 Hz). Notes below or above that
window are octave-folded into range at playback, preserving pitch class.
