# NM-EPD-420-4C — device support notes

RockBase IoT **NM-EPD-420-4C**: ESP32-S3 board, 4.2" **400×300 four-colour
(B/W/R/Y)** e-paper, 25–30 s full refresh, no partial refresh.

Upstream hardware repo: <https://github.com/RockBase-iot/NM-EPD-420>
(that repo ships factory *test* firmware, not an application firmware).

Status: **uncommitted work in progress on branch `nm-epd-420`**. The current
firmware has been exercised on an NM-EPD-420-4C: full-screen refresh, physical
buttons, deep-sleep BOOT wake, and battery reporting work. AHT20 advertisement
data requires a host-side compatibility update before Home Assistant can expose
temperature and humidity entities. Read [Remaining Hardware Checks](#6-remaining-hardware-checks)
before treating this as a production configuration.

---

## 1. Build target — no new PlatformIO env

The board is ESP32-S3 with **16 MB QIO flash + OPI PSRAM**, which is exactly what
`esp32-s3-N16R8` already describes. Per `CLAUDE.md` ("do not just create new pio
targets for new devices — check whether the existing ones work with a new
config"), no env was added:

```bash
pio run -e esp32-s3-N16R8
```

Everything device-specific — panel, pins, colour scheme, battery divider — lives
in the runtime config packet, not in build flags. The only build-time choice that
could differ is Wi-Fi: `esp32-s3-N16R8` builds with `-DOPENDISPLAY_ENABLE_WIFI`
(LAN transport). Set `communication_modes` to `0x5` (BLE|WiFi) if you want it.

## 2. The panel, and why the PanelIC value is `58`

Three names describe the same panel class, so it is worth writing down which one
this firmware uses.

| Layer | Name | Notes |
|---|---|---|
| Fitted panel | **GDEY0420F51** (a.k.a. GDEY042F51) | Good Display 4.2" B/W/R/Y, controller **HX8717**, 400×300, 84.8×63.6 mm active, full update 28 s, EOL |
| Panel's own successor | **GDEM042F52** | Good Display's announced replacement for the GDEY042F51; controller **JD79668** |
| bb_epaper / OpenDisplay | `EP42YR_400x300` = `OD_PANEL_IC_EP42YR_400X300` = **58** | documented in-tree as "GDEM042F52" |
| M3 / EPD-nRF5 driver line | `OD_PANEL_IC_JD79668_420_BWRY` = **1004** | same panel, named after the controller instead of after the bb_epaper panel id |

The existing bb_epaper panel ID is a candidate for the fitted HX8717, not a
verified drop-in replacement for its GxEPD2 driver:

1. Good Display publishes **GDEM042F52 as the drop-in upgrade for GDEY042F51**
   ("we recommend upgrading to our new model GDEM042F52"), which only makes sense
   if the interface and command set carry over.
2. `GxEPD2_420c_GDEY0420F51::_InitDisplay()` and bb_epaper's
  `epd42yr_init_full` share a controller family and 400×300 TRES, but the
  latter omits several power/timing registers and uses different BTST values.
3. `src/display_service.cpp: mapEpd()` already maps `0x003A` to
  `EP42YR_400x300`, so a software path exists; it has not been tested with
  this HX8717 panel.

For a first hardware trial, no framebuffer or new build target is required;
the panel is addressed as `panel_ic_type: 58` —
the same value the opendisplay.org preset `ep42yr-400x300` (index 11) carries.

### Firmware change made on this branch

One `case` in `src/display_service.cpp` (`mapEpd()`), mapping the M3-line
controller-named value onto the same bb_epaper panel:

```c
case OD_PANEL_IC_JD79668_420_BWRY: return EP42YR_400x300; // 1004
```

Before this, a device configured with `panel_ic_type: 1004` silently fell through
to `EP_PANEL_UNDEFINED` → "No display found". Nothing else in the render or
transfer path needed touching: `getBitsPerPixel()` already returns 2 for
`OD_COLOR_SCHEME_BWRY`, and the partial-refresh handlers already NACK any
`bpp != 1` panel, which matches the panel's "no partial refresh" reality.

## 3. Pin map

From `src/config.h` in the hardware repo, cross-checked against
`docs/sch/SCH_NM-EPD-420-V1.0.pdf`.

| Function | GPIO | Config field |
|---|---|---|
| EPD SCK | 2 | `displays[0].clk_pin` |
| EPD MOSI | 1 | `displays[0].data_pin` |
| EPD CS | 46 | `displays[0].cs_pin` |
| EPD DC | 4 | `displays[0].dc_pin` |
| EPD RST | 5 | `displays[0].reset_pin` |
| EPD BUSY | 6 | `displays[0].busy_pin` |
| Battery ADC | 3 | `power_option.battery_sense_pin` |
| Battery ADC enable (HIGH) | 43 | `power_option.battery_sense_enable_pin` |
| I²C SCL (AHT20 / ES8311) | 38 | `data_buses[0].pin_1` |
| I²C SDA | 39 | `data_buses[0].pin_2` |
| USER key (active LOW) | 45 | `binary_inputs[0].input_pin_1` |
| BOOT key (active LOW) | 0 | `binary_inputs[0].input_pin_2`; ESP32-S3 ROM download strap and deep-sleep wake |
| SD/LoRa SPI | 7,8,9,10,11,12,13,14 | not OpenDisplay config concepts, never claimed |
| Module enables, all active HIGH | 47 LoRa EN, 44 codec EN, 41 audio PA, 40 AHT20 gate | flag bit 5 defaults them LOW; AHT20 briefly HIGH at wake, see §5 |

`battery_sense_pin: 3` with a 1:2 divider → `voltage_scaling_factor: 161`
(`3.3 V / 4095 counts × 2 = 0.001612 V/count`). That is the same pin, divider and
factor the M5Stack PaperS3 preset uses, which is the cross-check.

## 4. Files on this branch

- `devices/nm-epd-420-4c.yaml` — full device config in `tools/od-device-cli.py`
  YAML form. It currently encodes to a **229-byte** packet (version 1, CRC-16 over
  the body); verified by a decode→encode round-trip.
- `devices/nm-epd-420.simple-config-preset.json` — the fragment to merge into
  `OpenDisplay/opendisplay.org`'s
  `httpdocs/firmware/toolbox/simple-config-presets.json` plus the matching
  `simple-config-id-registry.json` entry. `driverBoards` index **32**; the display
  preset is the existing `ep42yr-400x300` (11); power reuses `battery-650` (9)
  with board-specific 600 mAh, 40 s idle hold and 1800 s timer wake overrides.
- `src/display_service.cpp` — the `1004 → EP42YR_400x300` mapping above.
- `include/opendisplay_structs.h` — new `OD_DEVICE_FLAG_NM420_INIT` (bit 5), the
  matching `DeviceFlags` changelog entry, and the group comment now reading
  "bits 6-7 reserved". Additive only, so no version bump: it accrues under
  *Unreleased*, exactly as the 2.1 notes describe.
- `src/main.h` / `src/main.cpp` — `DEVICE_FLAG_NM420_INIT`, default-off GPIO
  initialization and deep-sleep output hold on GPIO40/41/43/44/47.
- `src/config_parser.cpp` — calls `nm420_init()` after a config loads when the flag
  is set, next to the existing `ws_pp_init()` hook.
- `src/sensor_aht20.*` — one CRC-checked temperature/humidity sample at each
  boot/wake, published in the SHT40-compatible 3-byte MSD dynamic slot 7–9.
- `bin/NM-EPD-420-4C_factory_full.bin` — current all-in-one ESP32-S3 image:
  bootloader, partition table, `boot_app0`, application, and the NM420 factory
  config. Flash the complete image at offset `0x0`.

### Flashing

The combined image can be flashed at **`0x0`** with esptool or a compatible
ESP32-S3 flashing utility. It provisions its embedded configuration only when
the device has no stored configuration. A board already holding an older config
must receive the current YAML over BLE:

```powershell
$env:PYTHONUTF8 = '1'
uv run tools/od-device-cli.py write-config --addr AA:BB:CC:DD:EE:FF `
  --input devices/nm-epd-420-4c.yaml
```

To rebuild the combined image with the current configuration:

```powershell
$env:PYTHONUTF8 = '1'
$env:OPENDISPLAY_FACTORY_CONFIG_HEX = uv run tools/od-device-cli.py encode-config `
  --input devices/nm-epd-420-4c.yaml
$env:PATH = "$HOME\.platformio\packages\xtensa-esp-elf\bin;$env:PATH"
pio run -e esp32-s3-N16R8
Copy-Item .pio\build\esp32-s3-N16R8\firmware.factory.bin `
  bin\NM-EPD-420-4C_factory_full.bin
```

## 5. Low-power peripherals and wake-time sensing

All four peripheral enable lines are active HIGH:

| Peripheral | Enable pin | Level this branch forces |
|---|---|---|
| SX126x LoRa module | 47 — `PIN_MODULE_EN` / LoRa EN | LOW |
| ES8311 codec | 44 — `PIN_CODEC_EN` | LOW |
| Audio PA | 41 — `PA_CTRL` | LOW |
| AHT20 | 40 — `PIN_TEMP_CTL` | LOW except for one bounded measurement after each boot/wake |

The OpenDisplay config format has **no field for a spare output pin**: a pin that
belongs to no display / bus / input / LED / sensor block does not exist as far as
the firmware is concerned, so `nm-epd-420-4c.yaml` cannot switch them off. Left
alone they keep the ESP32 reset default — input, no pull — and each rail then sits
wherever the board's own resistors leave it, which is not a guarantee that the
part is off.

The NM420 flag selects a board hook to set these rails LOW after config load.
It also parks ADC_EN GPIO43 LOW between voltage reads; the ADC reader still
pulses it HIGH for its measurement. Immediately before deep sleep the firmware
holds all five outputs LOW with ESP32-S3 pad hold, releasing it after wake and
reestablishing LOW before the next reading. This does not control the interval
between a cold power-on reset and successful config load: board pull resistors
and real current measurements must be checked.

```c
/* include/opendisplay_structs.h */
#define OD_DEVICE_FLAG_NM420_INIT (1u << 5)   /* bit 5; bits 6-7 stay reserved */

/* src/main.cpp — called from config_parser.cpp after a config loads, and only
   when device_flags bit 5 is set */
void nm420_init(){ /* 47, 44, 41, 40, 43 default LOW; release sleep holds */ }
```

The config sets `device_flags: 0x20`. Because the switch is a config bit and not a
`#define` or a board name, one firmware binary still covers every other device,
and an NM-EPD-420 flashed without the bit behaves exactly as it did before.

Deep sleep starts after a 40-second idle hold and has a 1800-second timer fallback.
GPIO45 (USER) is not an ESP32-S3 deep-sleep wake pin, but GPIO0 (BOOT) is an RTC
pad, so `sleep_flags: 0` arms it as an active-LOW wake source. The firmware skips
arming a button already held at sleep entry, avoiding an immediate wake loop.

GPIO0 remains an ESP32-S3 ROM-download strap: release BOOT before reset or power-on
unless entering the ROM downloader intentionally. Its board pull-up keeps it HIGH
while asleep; hardware testing must confirm the pull-up is strong enough through the
full deep-sleep current range.

Per peripheral:

- **AHT20 temperature/humidity.** SensorType 7 at I²C `0x38` powers GPIO40 for
  a 100 ms startup period, then performs up to two CRC-checked measurements. A
  successful sample is written in the three-byte SHT40-compatible MSD format at
  dynamic bytes 7–9 and logged as `AHT20: wake-time sample ...`; a failed sample
  publishes `FF FF FF` and logs `AHT20: wake-time sample failed ...`. The rail is
  then driven LOW and is not polled again while awake. With no other I²C client,
  `Wire` is closed before supply removal. Board-level I²C pull-ups may still
  back-power an unpowered sensor, so measure the AHT20 rail after shutdown rather
  than assuming zero leakage.
- **Home Assistant compatibility.** The current RockBase IoT/OpenDisplay Home
  Assistant integration. That version has
  `SensorType.SHT40 = 4` but does not define AHT20 type 7, and the integration
  creates ambient entities only for SHT40. It therefore ignores this board's
  valid AHT20 configuration even when the firmware advertises a good sample.
  Do not change the config to type 4: firmware would send SHT40 I²C commands to
  the AHT20 and publish an invalid reading. The host-side fix is to add type 7
  to py-opendisplay and create equivalent ambient entities in the HA integration.
- **ES8311 codec + PA, LMD4737 PDM mic.** Audio has no protocol block at all.
- **SD card / SX126x LoRa.** No protocol block; the HSPI pins (7-14) are never
  claimed. The module is nevertheless genuinely powered off now, rather than merely
  unused.

## 6. Remaining hardware checks

Full refresh, buttons, BOOT wake, and battery reporting have been verified. The
following still need targeted measurements or broader visual tests:

1. **Colour packing order.** `EP42YR_400x300` uses `u8Colors_4clr_v2`; the
   host-side 2 bpp BWRY packing has to match it or red and yellow come out
   swapped. Send a full-swatch test image first.
2. **Battery scaling.** 161 is derived. Compare `read-msd` against a multimeter
  on the cell over its operating range.
3. **Rotation.** `rotation: 0` assumes the panel is mounted in its native 400×300
   landscape orientation. Change to 1/2/3 if the image lands rotated.
4. **BUSY polarity and long refreshes.** The hardware repo drives BUSY as "high
  while refreshing";
   bb_epaper's UC81xx path expects the same. If every refresh times out, suspect
   this first.
5. **AHT20 rail and host integration.** Check GPIO40 falls LOW after a successful
  and failed sample, then update py-opendisplay and Home Assistant before relying
  on HA temperature/humidity entities.

## 7. Verification done so far

**Uncommitted.** No commit exists; all changes sit in the working tree on branch
`nm-epd-420`. The rule applied here: a change that has not been exercised on
NM-EPD-420-4C hardware does not get committed.

- Config packet encodes and decodes cleanly (`od-device-cli.py`): 229 bytes,
  CRC valid, sensor type 7 / MSD slot 7, device flag 0x20, GPIO0+GPIO45 button
  configuration, and a 1800-second timer fallback round-trip.
- `pio run -e esp32-s3-N16R8` compiled and linked the full firmware, generating
  `firmware.bin` and the combined `firmware.factory.bin`. On this Windows machine PlatformIO
  Core 6.1.19 did not automatically expose the already-installed compiler;
  prepending `%USERPROFILE%\\.platformio\\packages\\xtensa-esp-elf\\bin` to PATH
  for that one command resolved it. The only compiler warning was in existing
  Wi-Fi code, unrelated to this board.
- `pio run -e nrf52840custom` also compiled and linked successfully with the
  shared SHT40 MSD packing function used by the new AHT20 path.
- NM-EPD-420-4C hardware test: full refresh, physical buttons, BOOT deep-sleep
  wake, and battery reporting work. See [Remaining Hardware Checks](#6-remaining-hardware-checks)
  for the work that remains before release.
