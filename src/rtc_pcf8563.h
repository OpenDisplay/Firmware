#ifndef RTC_PCF8563_H
#define RTC_PCF8563_H

#include <stdint.h>

// NXP PCF8563 / M5Stack BM8563 RTC as a power-off wake source
// (SensorType OD_SENSOR_TYPE_PCF8563). Used only together with the MOSFET
// power latch (DEVICE_FLAG_BATTERY_LATCH): instead of holding the rail through
// ESP32 deep sleep, the sleep path arms the RTC and releases the latch; the
// RTC's open-drain INT line switches the board back on (M5Paper wiring).

// True when an RTC sensor block is configured AND the MOSFET latch is enabled.
bool rtcWakeConfigured(void);

// Call once per boot after full_config_init() (the board's power hold is
// already asserted by then). Clears the timer/alarm flags and disables both
// interrupts so INT releases, and returns true if one of them had fired, i.e.
// this cold boot is an RTC wake. No I2C traffic unless rtcWakeConfigured().
bool rtcConsumeWake(void);

// Arm a wake `seconds` from now. <= 255 s: 1 Hz countdown timer; <= 255 min:
// 1/60 Hz countdown (rounded up to whole minutes, first tick may come up to a
// minute early); longer: minute/hour alarm. Returns false on any I2C failure,
// in which case the caller must keep the rail held and use ESP32 deep sleep.
bool rtcArmWake(uint32_t seconds);

#endif
