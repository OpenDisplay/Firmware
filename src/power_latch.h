#pragma once

// Soft battery-latch / power-button support for devices that gate their own
// power rail through a MCU-controlled MOSFET. Enabled at runtime by the
// DEVICE_FLAG_BATTERY_LATCH device_flags bit; the latch pin comes from
// system_config.pwr_pin_2 and the optional active-low shutdown button from
// pwr_pin_3 (0xFF = none).
//
// All functions are no-ops on non-ESP32 targets and when the flag is clear, so
// callers in shared code need no guards.

// Assert the keep-alive latch at boot so the device stays powered on battery.
// Safe to call on every boot, including timer-wake from deep sleep.
void powerLatchBegin();

// Poll the hardware power button; perform a clean power-off on a long press.
// Cheap; intended to be called from the existing button-polling path.
void powerButtonPoll();

// Keep the latch closed across OpenDisplay's timer-wake deep sleep so the
// device retains power and can wake itself. Call immediately before
// esp_deep_sleep_start().
void powerLatchHoldForSleep();
