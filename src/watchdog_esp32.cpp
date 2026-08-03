// ESP32 implementation of the portable watchdog interface.
//
// The whole file is gated on TARGET_ESP32, so an nRF build compiles it to an
// empty translation unit -- no build_src_filter changes needed across the CI
// environments.
//
// Only odWatchdogBootInit() does real work here: it is the reset-reason decode
// that used to live inline in main.cpp under #ifdef TARGET_ESP32, moved out so
// both targets reach it through one portable call. Arming is a stub.
//
// See docs/PLAN_NRF_HARDWARE_WATCHDOG_2026-08-01.md (W-6).

#include "watchdog.h"

#ifdef TARGET_ESP32

#include <Arduino.h>
#include <esp_system.h>
#include "od_log.h"

// Distinguishes a hidden mid-cycle reset (PANIC/WDT/BROWNOUT/SW) from a real
// power-on or deep-sleep wake; any reset here clears the wake cause, so the
// next boot takes the NORMAL BOOT branch and redraws the boot screen.
static const char* resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

void odWatchdogBootInit(void) {
    esp_reset_reason_t r = esp_reset_reason();
    od_log_info("Reset reason: %s (%d)", resetReasonName(r), (int)r);
}

bool odWatchdogInSafeMode(void) {
    return false;
}

void odWatchdogArm(void) {
    // No hardware watchdog is armed by this firmware on ESP32.
    //
    // Be precise about what that does and does not mean, because the obvious
    // wording ("no watchdog on this target") is FALSE and would mislead whoever
    // reads the log next. The IDF task watchdog IS enabled and initialised in the
    // shipped sdkconfigs -- CONFIG_ESP_TASK_WDT_EN=y, CONFIG_ESP_TASK_WDT_INIT=y,
    // CONFIG_ESP_TASK_WDT_TIMEOUT_S=5 -- but nothing it watches would catch a
    // wedged loop():
    //
    //   - S3 and classic ESP32 set CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y,
    //     while CONFIG_ARDUINO_RUNNING_CORE=1 puts loopTask on CPU1. A spin in
    //     loop() starves IDLE1, which nothing is subscribed to.
    //   - C3 and C6 initialise the task watchdog but subscribe no idle task at all.
    //
    // Closing this is a one-liner -- esp_task_wdt_add(NULL) on the loop task -- and
    // the feed sites it would need are already wired by this module. Deliberately
    // not done here: the timeout would be bounded by different spans than nRF's
    // (FastEPD refresh, WiFi/TLS handshakes) and needs its own analysis.
    //
    // Logged once, at boot, so the gap is explicit rather than assumed.
    od_log_info("[WDT] no firmware watchdog on ESP32; IDF TWDT is enabled but no "
                "task that would catch a wedged loop() is subscribed");
}

void odWatchdogFeed(void) {
}

void odWatchdogBreadcrumb(uint8_t phase) {
    (void)phase;
}

#endif  // TARGET_ESP32
