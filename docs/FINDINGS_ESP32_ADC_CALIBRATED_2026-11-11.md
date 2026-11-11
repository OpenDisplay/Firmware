# Findings: eFuse-calibrated battery ADC on an M5Paper (2026-11-11)

Hardware A/B of `OD_BATTERY_SENSE_FLAG_CALIBRATED_MV` (commit `5feed84`,
branch `feat/esp32-adc-calibrated-battery`).

## Setup

- Board: original M5Paper, ESP32-D0WDQ6-V3 ("bathroom" board), battery sense
  GPIO35 through a 2:1 divider, on USB after a night of charging plus a short
  run on battery.
- App image only flashed; config written over BLE.
- **Calibrated**: original config with `battery_sense_flags: 2`,
  `voltage_scaling_factor: 1999` (from the "kitchen" M5Paper: 2101 mV at the
  pin for a 4.20 V cell).
- **Legacy**: original config, flags 0, factor 174 (the current M5Paper
  preset, calibrated on the kitchen board's raw count).
- Voltage read from the advertisement (10 mV units), one boot per run,
  ~100-130 s each. The firmware caches the reading for 30 s.

## Results

| Run | Method | Advertised battery |
|---|---|---|
| 1 | calibrated | 4.42 V first reading after boot, then 4.24 V |
| 2 | calibrated | 4.23 V (all readings) |
| 3 | legacy (this build) | 4.13 V (all readings) |
| ref | legacy (official 2.27.1, the same morning) | 4.13 V, later 4.14 V |

- On this chip the legacy raw-count scaling reads about 2.5 % lower than the
  calibrated path (4.13 vs 4.23-4.24 V), whereas factor 174 made the kitchen
  board read exactly 4.20 V at full charge. That is the chip-to-chip spread
  the calibrated path removes.
- 4.23-4.24 V on USB right after charging is plausible for a 4.20 V CV charge
  plus eFuse calibration tolerance (~1-2 %). **Not checked with a
  multimeter**, so absolute accuracy is still unverified.
- The first reading after a boot is sometimes high (4.42 V calibrated here;
  4.33 V legacy after a brownout reboot earlier). It does not repeat on every
  boot and clears by the next 30 s refresh. Not investigated; possibly the
  divider/charger settling at power-up.

## On battery (charger out of the loop)

Same board, unplugged at ~09:29:40 after charging, 120 s sleep so each RTC
wake takes a fresh reading. The method was switched over BLE while on
battery, then switched back to bracket any drift of the cell.

| Time | Method | Advertised battery |
|---|---|---|
| 09:32:35 | calibrated | 4.20 V |
| 09:37:04 | calibrated | 4.19 V |
| 09:44:13 | calibrated | 4.20 V |
| 09:46:42 - 09:53:48 | legacy (factor 174) | 4.11, 4.10, 4.11, 4.10 V |
| 09:56:21 | calibrated | 4.19 V |

The calibrated path reads 4.19-4.20 V on a cell 3-27 min off the charger,
which is where a fully charged Li-ion rests; the legacy path reads 4.10-4.11
V on this chip (about 2.2 % low). The bracket rules out cell drift. Still no
multimeter reading.

## Not verified

- Absolute accuracy against a multimeter.
- The kitchen board with the calibrated flag (should read ~4.20 V at full).
- Lower in the discharge curve (3.3-3.8 V).
- Other ESP32 variants (S3/C3/C6 calibration schemes).

## Final state

The board was put back on official 2.27.1 with its original config (readback
identical).
