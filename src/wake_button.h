#pragma once

// Button wake from timer deep sleep (ESP32). Arms every wake-capable configured
// button — plus the MOSFET-latch power button on DEVICE_FLAG_BATTERY_LATCH
// boards — as a deep-sleep wake source alongside the timer, and classifies the
// wake cause on the next boot. Default-on; OD_SLEEP_FLAG_BUTTON_WAKE_DISABLE
// (power_option.sleep_flags bit 0) opts a device out.
//
// Both functions are no-ops / false on non-ESP32 targets, so callers in shared
// code need no guards.

// Arm all eligible button-wake sources for the upcoming timer deep sleep.
// Never fails; logs each pin's disposition. Call from enterDeepSleep() after
// esp_sleep_enable_timer_wakeup() — the timer stays armed alongside — and
// before powerLatchHoldForSleep().
// includeChargePin: low-battery sleeps only. Also arms power_option.charge_state_pin
// at its charging level, so plugging in a charger wakes the device early.
void armButtonWakeSources(bool includeChargePin = false);

// Classify the wake cause and log it with the waking pin(s). Call once, early
// in setup(); safe pre-config (reads sleep registers only). Returns true for
// the button causes (EXT0/EXT1/GPIO), but false when the only waking pin is
// the charge pin armed by armButtonWakeSources(true), so a charger plug-in is
// never delivered to the host as a synthetic button click. Takes
// esp_sleep_wakeup_cause_t as int so this header also compiles on targets
// without esp_sleep.h.
bool detectButtonWake(int wakeupCause);

#if defined(TARGET_ESP32)
// Map the last button-wake cause to a configured buttonStates[] index, or -1.
int wakeButtonFindIndex(void);
#endif
