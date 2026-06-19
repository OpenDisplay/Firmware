#pragma once

// Power-latch support (ESP32). DEVICE_FLAG_BATTERY_LATCH (bit 3): MOSFET hold on
// pwr_pin_2 + optional long-press shutdown on pwr_pin_3. DEVICE_FLAG_PWR_LATCH_DFF
// (bit 4): 74AHC1G79 D-FF; pwr_pin_2=D, pwr_pin_3=CP; release via command 0x0052 or binary_inputs power_off_flags
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

// Release 74AHC1G79 D-FF latch (pwr_latch_dff only). Used by command 0x0052.
bool powerLatchDffConfigured(void);
bool powerLatchMosfetConfigured(void);
void powerLatchPowerOff();
void powerLatchTriggerOff();
