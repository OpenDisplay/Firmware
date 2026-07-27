# Firmware Audit — 2026-07-13

- **Branch:** `feat/less-latency`
- **HEAD commit:** `74b3f76`
- **Scope:** All first-party sources in `src/` (41 files, ~13.6k lines). Excludes
  `.pio/`, `libdeps`, and bundled third-party (`src/qr/qrcode.c`, bb_epaper,
  Seeed_GFX/TFT_eSPI, uzlib) except where first-party code calls them.
- **Targets audited (both `#ifdef` paths):** nRF52840 (`TARGET_NRF`, Bluefruit /
  single-core FreeRTOS) and ESP32 S3/C3/C6/classic (`TARGET_ESP32`, Arduino /
  dual-core). Read `platformio.ini` and `docs/epd-panel-power-session.md`,
  `docs/architecture-deep-sleep-power-buttons.md` for architecture context.

This is a mature codebase with an existing `FINDINGS.md`, extensive inline
rationale, and prior hardware-verified fixes. No CRITICAL or HIGH defect survived
verification. Findings are MEDIUM/LOW: latent hazards and forward-compat gaps,
not active crashes. Several constructs that *look* suspicious are deliberate and
were confirmed correct (listed under "Verified-correct by design").

## Executive summary

| ID | Sev | Location | Title |
|----|-----|----------|-------|
| M1 | MEDIUM | communication.cpp:352 | `handleReadConfig` puts a 4 KB buffer on the BLE-callback stack (nRF) |
| M2 | MEDIUM | config_parser.cpp:605-608 | Unknown config packet ID silently aborts the rest of the parse yet marks config loaded |
| L1 | LOW | main.h:140 / display_service.cpp:41 | `connectionRequested` defined `uint8_t`, externed `bool` (type/ODR mismatch) |
| L2 | LOW | communication.cpp:411-425 | `handleWriteConfig` `len==201` drops a byte and wedges chunked-write state |
| L3 | LOW | display_service.cpp:2786-2791 | zlib/gray4 overflow bytes written to the controller before the overflow check aborts |
| L4 | LOW | buzzer_control.cpp:71-78 | `buzzer_drive_tone_sw` busy-waits up to ~5 s, starving BLE/keep-alive servicing |
| L5 | LOW | power_latch.cpp:87-89 | `powerOff()` button-release wait has no timeout (stuck button pins the CPU) |

Counts: CRITICAL 0, HIGH 0, MEDIUM 2, LOW 5.

---

## M1 — 4 KB stack buffer in the BLE write-callback context (nRF)

- **STATUS: RESOLVED**, on both of its independent counts.
  1. The buffer itself is gone: `handleReadConfig` now takes its scratch from
     `getConfigScratch()` rather than declaring `uint8_t configData[4096]` on the
     stack. This landed separately from the BLE work.
  2. The context is gone: Phase 3 of
     `docs/PLAN_BLE_TRANSPORT_ABSTRACTION_2026-07-27.md` moved nRF command
     dispatch off the Bluefruit callback task onto `loop()`, so no command
     handler runs on the small callback stack any more. This class of finding —
     "handler X is unsafe because of the stack it runs on" — is now structurally
     impossible on both targets.
- **Severity:** MEDIUM (latent stack overflow / crash)
- **Location:** `src/communication.cpp:352` (`handleReadConfig`), reached from
  `imageDataWritten` case `0x0040` (`communication.cpp:584-589`).

### Description
`handleReadConfig()` declares `uint8_t configData[4096];` and
`uint32_t configLen = 4096;` on the stack. On nRF52840 the BLE write callback
(`imageDataWritten`) executes on the Adafruit Bluefruit callback task, not the
Arduino loop task, and that task's stack is limited. Every *other* large config
buffer in the codebase is deliberately `static` to avoid exactly this:
`loadConfig` uses `static config_storage_t config` (config_parser.cpp:159),
`loadGlobalConfig` uses `static uint8_t configData[MAX_CONFIG_SIZE]`
(config_parser.cpp:273), `hasValidStoredConfig` uses `static uint8_t buf[...]`.
`handleReadConfig` is the lone stack allocation of this size, and it runs in the
most stack-constrained context.

### Concrete failure scenario
On nRF, a client sends command `0x0040` (read config). `imageDataWritten` runs on
the Bluefruit callback task; `handleReadConfig` pushes 4 KB + locals onto that
task's stack. If the callback task stack headroom is below ~4.3 KB (Bluefruit
defaults are a few KB), the write smashes past the stack guard → hard fault /
silent corruption of adjacent task state. It has evidently not tripped in
practice (config reads work), so the current stack must be large enough — hence
latent, not active — but it is fragile against any future stack-tightening or
added locals on this path.

### Fix direction
Make `configData` `static` (matching `loadGlobalConfig`/`hasValidStoredConfig`),
or read+notify config in a smaller streaming window. No behavior change; removes
the stack pressure.

---

## M2 — Unknown config packet ID aborts the remainder of the parse

- **Severity:** MEDIUM (silent config truncation; forward-incompatibility)
- **Location:** `src/config_parser.cpp:605-608` (`default:` case in
  `loadGlobalConfig`'s packet loop).

### Description
```c
default:
    writeSerial("WARNING: Unknown packet ID 0x" + String(packetId, HEX) + ", skipping");
    offset = configLen - 2; // Skip to CRC
    break;
```
The log says "skipping", but the code does not skip *this* packet — it jumps
`offset` to the CRC, terminating the `while (offset < configLen - 2)` loop. Every
packet after the first unrecognized ID is dropped. `globalConfig.loaded` is still
set `true` at the end (config_parser.cpp:621), so the caller treats the partial
config as valid. The format has no generic per-packet length the parser
consults (the loop does `offset++` then reads the ID but never uses the skipped
byte as a length), so there is no safe way to skip a single unknown packet — but
the current behavior is worse than failing: it silently loads a truncated config.

### Concrete failure scenario
A newer toolbox emits a config containing a packet type this firmware build does
not know (e.g. a future `0x2D`), positioned *before* the `0x20` display packet.
On load, the parser hits `default`, jumps to CRC, and the display (and any later
packets) are never parsed: `display_count` stays 0. The device boots with no
display, `globalConfig.loaded == true`, and the only trace is one WARNING line —
no NACK, no fallback. The same happens for any real-world packet reordering that
puts an unknown ID ahead of required packets.

### Fix direction
Either treat an unknown packet ID as a hard parse failure
(`globalConfig.loaded = false; return false;`) so the factory-embed / retry path
engages, or add and honor a real per-packet length field so unknown packets can
be skipped forward-compatibly. At minimum, correct the "skipping" log to reflect
that parsing stops.

---

## L1 — `connectionRequested` type mismatch across translation units

- **Severity:** LOW (undefined behavior in principle; benign in practice)
- **Location:** definition `src/main.h:140` (`uint8_t connectionRequested = 0;`),
  declaration `src/display_service.cpp:41` (`extern bool connectionRequested;`).

### Description
The object is defined as `uint8_t` but externed as `bool` in another TU. This is
an ODR / type violation. It only works because `bool` and `uint8_t` share size
and representation on both toolchains, and the only read
(`display_service.cpp:1505`, `((connectionRequested & 0x01) << 2)`) masks to one
bit. Still UB by the standard and a trap for future changes (e.g. if the value is
ever set to 2+).

### Concrete failure scenario
No current runtime failure. If someone later assigns `connectionRequested = 2`
via the `uint8_t` definition and another TU reads it as `bool`, the compiler is
entitled to assume it is 0/1 and mis-optimize the read.

### Fix direction
Make the `extern` declaration `uint8_t` to match the definition (ideally move the
declaration into a shared header so both TUs agree).

---

## L2 — `handleWriteConfig` with `len == 201` drops a byte and wedges chunked state

- **Severity:** LOW (edge input; non-standard client framing)
- **Location:** `src/communication.cpp:406-428`.

### Description
```c
if (len > 200) {
    chunkedWriteState.active = true; ...
    if (len >= 202) { /* parse 2-byte total header */ }
    else {                                  // len == 201 only
        chunkedWriteState.totalSize = len;  // 201
        chunkedWriteState.expectedChunks = 1;
        uint16_t chunkSize = (len < 200) ? len : 200;   // = 200
        memcpy(chunkedWriteState.buffer, data, chunkSize);   // drops byte 201
        chunkedWriteState.receivedSize = chunkSize;          // 200
        chunkedWriteState.receivedChunks = 1;                // == expectedChunks
    }
    /* ACK and return — no save in this path */
}
```
For `len == 201` the handler copies only 200 of the 201 bytes (drops one),
declares `expectedChunks == receivedChunks == 1`, ACKs, and returns without
saving (saving only happens in `handleWriteConfigChunk`). The transfer is
"complete" by its own counters yet the config is never persisted, and
`chunkedWriteState.active` stays set until the next `0x0041`.

### Concrete failure scenario
A non-standard client (the reference client uses the 202-byte `[len:2][data:200]`
chunk-0 framing) sends a single `0x0041` whose payload is exactly 201 bytes. The
device ACKs, silently drops the last byte, saves nothing, and leaves chunked
state armed. A following `0x0042` chunk would then save a byte-short buffer;
absent that, the write is silently lost.

### Fix direction
Handle `len == 201` in the non-chunked branch (it fits a single save), or copy
all `len` bytes and treat a lone oversize-but-single write as a complete
`saveConfig`. Reject rather than partially accept.

---

## L3 — Overflow bytes reach the controller before the size check aborts

- **Severity:** LOW (harmless in practice; writes past the intended window)
- **Location:** `src/display_service.cpp:2786-2791` (`zlib_stream_to_direct_write`),
  and the gray4 branch at 2778-2783.

### Description
```c
bbepWriteData(&bbep, decompressionChunk, bytesOut);
directWriteBytesWritten += (uint32_t)bytesOut;
...
if (directWriteBytesWritten > directWriteDecompressedTotal) {
    return false;
}
```
A decompressed chunk is written to the panel controller *before* the running
total is checked against the negotiated decompressed size. A corrupt or
oversized zlib stream can therefore push bytes past the address window before the
next-iteration check aborts and NACKs.

### Concrete failure scenario
A malformed compressed image whose inflated output exceeds the size negotiated at
START causes up to one 2048-byte chunk to be streamed past the panel's addressed
region before `directWriteBytesWritten > directWriteDecompressedTotal` trips.
SSD16xx/IT8951 controllers ignore writes past the window, and the transfer is
then aborted and the panel powered down, so no lasting corruption — but the write
should be bounded before it is issued.

### Fix direction
Clamp `bytesOut` to `directWriteDecompressedTotal - directWriteBytesWritten`
before `bbepWriteData`, and fail if the stream still has more to emit.

---

## L4 — Buzzer playback busy-waits up to ~5 s, starving the main loop

- **STATUS: RESOLVED**, independently of the BLE transport work and before it.
  The software square-wave busy-loop is gone: `buzzer_drive_tone_sw` no longer
  exists. Tone generation is now hardware PWM (`buzzer_hw_tone_start` in
  `src/buzzer_hw.cpp` — nRF PWM peripheral, ESP32 LEDC), and playback is a
  millis()-poll state machine (`buzzer_run` / `buzzerService` over `s_buzzer`,
  `buzzer_control.cpp`) driven from `loop()` and `idleDelay()`. Nothing blocks:
  `buzzerService()` returns immediately while a step is still timing out. The
  5 s cap survives as `kBuzzerMaxTotalMs`, now enforced across polls rather than
  inside a spin.

  Note for anyone tracing this finding's history: a Phase 5 note briefly claimed
  L4 was still open and made worse by moving nRF dispatch to `loop()`. That was
  wrong — it reasoned from this document's original text without checking the
  code, which had already been rewritten. The reasoning would have been correct
  had the busy-wait still existed.
- **Severity:** LOW (bounded; degrades BLE latency / keep-alive during a tone)
- **Location:** `src/buzzer_control.cpp:71-78` (`buzzer_drive_tone_sw`), driven by
  `handleBuzzerActivate` (buzzer_control.cpp:148-175).

### Description
Tone generation is a software square-wave busy-loop
(`while (micros()-start < total_us) { digitalWrite; delayMicroseconds; ... }`)
with no yielding. `handleBuzzerActivate` runs it for the whole pattern, capped at
`kBuzzerMaxTotalMs = 5000`. It executes inside the command drain (ESP32 loop task
/ nRF BLE callback), so for the duration nothing else in the loop runs: no
command-queue drain, no `epdSessionTick`, no response flush, no touch/button
poll.

### Concrete failure scenario
A `0x0077` buzzer command with a long pattern blocks the main loop for up to ~5 s.
During that window BLE responses queue without flushing (10-slot ring), a warm
EPD panel's keep-alive tick cannot fire, and touch/button events are not polled.
Recoverable once the tone ends, but a 5 s stall is user-visible and can overflow
the response ring under concurrent traffic.

### Fix direction
Use hardware PWM/LEDC (ESP32) / a timer for tone generation, or chunk the
software loop and service the loop between steps; at minimum lower the cap.

---

## L5 — `powerOff()` release-wait has no timeout

- **Severity:** LOW (intended shutdown path; unbounded only on hardware fault)
- **Location:** `src/power_latch.cpp:87-89` (MOSFET-latch `powerOff`); analogous
  device-side release-wait exists for the D-FF path.

### Description
```c
pinMode(buttonPin(), INPUT_PULLUP);
while (digitalRead(buttonPin()) == LOW) {
    delay(20);
}
```
The shutdown routine waits for the user to release the power button before
cutting the latch, with no upper bound. A stuck-low pin (mechanical fault, solder
short, or a pin misconfigured active-low) never satisfies the exit condition.

### Concrete failure scenario
On a device whose shutdown button pad is shorted to ground (or a config that
mislabels polarity), a power-off request enters `powerOff()` and spins forever in
the release-wait, never reaching `esp_deep_sleep_start()`. The only recovery is
the hardware/interrupt watchdog; the device cannot be powered off through the
intended path.

### Fix direction
Add a bounded wait (e.g. a few seconds) after which the latch is cut regardless,
and/or feed the watchdog inside the loop.

---

## Verified-correct by design (checked, NOT flagged)

These were examined closely and confirmed intentional/safe; noted so a re-audit
doesn't re-open them:

- **`pwrmgmLock` try-lock + `delay(1)` spin** (display_service.cpp:204-217):
  deliberate priority-inversion avoidance on nRF (Bluefruit callback task vs loop
  task). `epdSessionTick` try-locks and skips, so it can never rail-cut mid-init.
- **EPD keep-alive state machine** (`PWR_OFF/WARM/ACTIVE`, deadline poll):
  millis()-wrap-safe (`(int32_t)(millis()-deadline) >= 0`), and the
  ACTIVE-only-teardown invariant on disconnect is intentional (a WARM panel
  survives disconnect).
- **RTC_DATA_ATTR crash-recovery semantics** (main.cpp:82-91): the "count 0 after
  a hidden reset takes the cold path" behavior is the documented, hardware-verified
  design.
- **PIPE_WRITE reorder sizing** (`33 = W+1`, `seq % 33` collision-free for
  W ≤ 32): the span-based window rule bounds occupancy, and the `>= PIPE_REORDER_SLOTS`
  guard is defensive. `pipeChunkReceived`'s `received_count`-bounded prefix
  correctly avoids phantom-ack wraparound in the first 32 chunks.
- **Replay window ±32 vs pipe W ≤ 32** (encryption.cpp:128-135, comm.cpp:630-635):
  the replay counter advances at decrypt for every accepted 0x0081, keeping the
  in-flight delta within the window; drops/dupes don't desync it.
- **SPSC command queue acquire/release atomics** (esp32_ble_callbacks.h:105-118,
  main.cpp:313-331): correct release/acquire pairing between the BLE callback
  producer and the loop consumer.
- **`directWriteComputeGeometry` row-padding** (display_service.cpp:1762-1779):
  row-padded sizing matches the sender for width-not-divisible-by-8 panels
  (documented prior fix).
- **`streamGray4Bytes` / two-plane split** (display_service.cpp:1702-1720):
  plane-boundary handling and the `remaining`-clamp in callers prevent overrun.
- **`enterDeepSleep` early-return ordering vs `epdSessionForceOff`**
  (main.cpp:462-526): the mains early-returns are effectively unreachable
  (deep sleep is battery-gated), and the WARM-panel-expires-via-tick reasoning
  holds.

## Coverage statement

Every file under `src/` was read in full and cross-checked against its callers:

Fully audited: `main.cpp`, `main.h`, `structs.h`, `display_service.cpp`,
`display_service.h`, `communication.cpp`, `communication.h`, `config_parser.cpp`,
`config_parser.h`, `device_control.cpp`, `device_control.h`, `ble_init.cpp`,
`ble_init.h`, `esp32_ble_callbacks.h`, `encryption.cpp`, `encryption.h`,
`encryption_state.h`, `wake_button.cpp`, `wake_button.h`, `power_latch.cpp`,
`power_latch.h`, `boot_screen.cpp`, `boot_screen.h`, `touch_input.cpp`,
`touch_input.h`, `sensor_sht40.cpp`, `sensor_sht40.h`, `sensor_bq27220.cpp`,
`sensor_bq27220.h`, `buzzer_control.cpp`, `buzzer_control.h`, `wifi_service.cpp`,
`wifi_service.h`, `display_seeed_gfx.cpp`, `display_seeed_gfx.h`,
`factory_config.cpp`, `factory_config.h`, `generated/factory_config_data.cpp`
(empty stub), `driver.h` (empty), `logo_bitmap.h` (data table).

Not fully assessed (out of scope / low leverage):
- `src/qr/qrcode.c` and `qrcode.h`: bundled third-party. Verified only the
  first-party call site (`boot_screen.cpp:662-667`), which correctly guards the
  buffer with `qrcode_getBufferSize` before use.
- `logo_bitmap.h`: reviewed only that dimensions/stride constants are consistent
  with `bootLogoPixelBlack`'s indexing; the bitmap payload itself was not
  byte-verified.
- Third-party libraries (`bb_epaper`, `Seeed_GFX`/`TFT_eSPI`, `uzlib`, Bluefruit,
  ESP32 Arduino/IDF, mbedTLS/CC310): behavior assumed per their contracts; only
  first-party assumptions about them were checked (e.g. bb_epaper address-window
  semantics, uzlib streaming states, Wire I2C error codes).
- Runtime/timing behavior (actual Bluefruit callback-task stack size, exact panel
  BUSY timing, real deep-sleep current) cannot be confirmed by static reading;
  M1 and the buzzer/keep-alive interactions (L4) would benefit from on-hardware
  stack-high-water and loop-latency measurement.
