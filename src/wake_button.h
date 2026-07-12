#pragma once

// Button wake from timer deep sleep (ESP32). Arms every wake-capable configured
// button — plus the MOSFET-latch power button on DEVICE_FLAG_BATTERY_LATCH
// boards — as a deep-sleep wake source alongside the timer, and classifies the
// wake cause on the next boot. Default-on; SLEEP_FLAG_BUTTON_WAKE_DISABLE
// (power_option.sleep_flags bit 0) opts a device out.
//
// Both functions are no-ops / false on non-ESP32 targets, so callers in shared
// code need no guards.

// Arm all eligible button-wake sources for the upcoming timer deep sleep.
// Never fails; logs each pin's disposition. Call from enterDeepSleep() after
// esp_sleep_enable_timer_wakeup() — the timer stays armed alongside — and
// before powerLatchHoldForSleep().
void armButtonWakeSources();

// Classify the wake cause and log it with the waking pin(s). Call once, early
// in setup(); safe pre-config (reads sleep registers only). Returns true for
// the button causes (EXT0/EXT1/GPIO). Takes esp_sleep_wakeup_cause_t as int so
// this header also compiles on targets without esp_sleep.h.
bool detectButtonWake(int wakeupCause);
