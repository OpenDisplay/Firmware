#include "power_latch.h"

#if defined(TARGET_ESP32)

#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_sleep.h"  // pulls in SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
#include "structs.h"

#ifndef DEVICE_FLAG_BATTERY_LATCH
#define DEVICE_FLAG_BATTERY_LATCH (1 << 3)
#endif

extern struct GlobalConfig globalConfig;

namespace {

// A self-holding battery-latch MOSFET: once the rail is powered the latch holds
// in hardware, so firmware only ever drives the pin LOW (in powerOff) to cut
// power. The latch pin and the optional shutdown button come from the device
// config (pwr_pin_2 / pwr_pin_3)
constexpr uint32_t POWER_OFF_HOLD_MS = 1500;

// Only arm long-press shutdown after the button has been seen released once, so a
// press still held from a GPIO-low wake can't instantly re-trigger it.
bool buttonReleasedSinceBoot = false;
bool pressing = false;
uint32_t pressStartMs = 0;

// 0 is the config's "unset" sentinel and GPIO0 is a strapping pin, so neither 0
// nor 0xFF is a usable latch/button pin.
bool validPin(uint8_t pin) { return pin != 0 && pin != 0xFF; }

bool latchEnabled() {
    return (globalConfig.system_config.device_flags & DEVICE_FLAG_BATTERY_LATCH) &&
           validPin(globalConfig.system_config.pwr_pin_2);
}
gpio_num_t latchPin() { return (gpio_num_t)globalConfig.system_config.pwr_pin_2; }
int buttonPin() { return globalConfig.system_config.pwr_pin_3; }
bool hasButton() { return validPin(globalConfig.system_config.pwr_pin_3); }

void powerOff() {
    const gpio_num_t latch = latchPin();
    // Wait for the button to be released before sleeping: we arm a GPIO-low wake
    // below, so sleeping while it is still held would immediately wake us again.
    if (hasButton()) {
        pinMode(buttonPin(), INPUT_PULLUP);
        while (digitalRead(buttonPin()) == LOW) {
            delay(20);
        }
    }
    // Open the latch and hold it across sleep. On battery this collapses the rail
    // to a true zero-drain off (a hardware power button revives it). On USB the
    // rail is fed by USB regardless, so the chip can't power down: it enters deep
    // sleep but USB brings it straight back up — i.e. an immediate reboot.
    gpio_hold_dis(latch);
    gpio_set_direction(latch, GPIO_MODE_OUTPUT);
    gpio_set_level(latch, 0);
    esp_sleep_config_gpio_isolate();
    #if defined(CONFIG_IDF_TARGET_ESP32C6)
    // The ESP32-C6 does not support or need the global hold function.
    // It is intentionally left blank here.
    #else
    // Compiles for ESP32, ESP32-S3, and ESP32-C3 based on your platformio.ini
    gpio_deep_sleep_hold_en();
    #endif
    gpio_hold_en(latch);
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    if (hasButton()) {
        pinMode(buttonPin(), INPUT_PULLUP);
        esp_deep_sleep_enable_gpio_wakeup(1ULL << buttonPin(), ESP_GPIO_WAKEUP_GPIO_LOW);
    }
#endif
    esp_deep_sleep_start();
}

}  // namespace

void powerLatchBegin() {
    if (!latchEnabled()) return;
    // Release any pin-hold left by a prior deep sleep so the latch returns to
    // hardware control (a USB wake from powerOff would otherwise keep it held
    // open), then arm the shutdown button.
    gpio_hold_dis(latchPin());
    if (hasButton()) pinMode(buttonPin(), INPUT_PULLUP);
}

void powerButtonPoll() {
    if (!latchEnabled() || !hasButton()) return;
    const bool down = digitalRead(buttonPin()) == LOW;
    if (!down) {
        buttonReleasedSinceBoot = true;
        pressing = false;
        return;
    }
    if (!buttonReleasedSinceBoot) {
        return;  // still the press that powered the device on
    }
    if (!pressing) {
        pressing = true;
        pressStartMs = millis();
        return;
    }
    if (millis() - pressStartMs >= POWER_OFF_HOLD_MS) {
        powerOff();
    }
}

void powerLatchHoldForSleep() {
    if (!latchEnabled()) return;
    gpio_set_direction(latchPin(), GPIO_MODE_OUTPUT);
    gpio_set_level(latchPin(), 1);
    #if defined(CONFIG_IDF_TARGET_ESP32C6)
    // The ESP32-C6 does not support or need the global hold function.
    // It is intentionally left blank here.
    #else
    // Compiles for ESP32, ESP32-S3, and ESP32-C3 based on your platformio.ini
    gpio_deep_sleep_hold_en();
    #endif
    gpio_hold_en(latchPin());
}

#else  // not ESP32

// Battery latch uses ESP-IDF sleep / GPIO-hold APIs; no nRF implementation yet.
// The DEVICE_FLAG_BATTERY_LATCH config layer is platform-neutral — implement
// these three here (Nordic sd_power_system_off / GPIO retention) to support it.
void powerLatchBegin() {}
void powerButtonPoll() {}
void powerLatchHoldForSleep() {}

#endif
