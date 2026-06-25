#include "power_latch.h"

#if defined(TARGET_ESP32)

#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_sleep.h"  // pulls in SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
#include "structs.h"

#ifndef DEVICE_FLAG_BATTERY_LATCH
#define DEVICE_FLAG_BATTERY_LATCH (1 << 3)
#endif
#ifndef DEVICE_FLAG_PWR_LATCH_DFF
#define DEVICE_FLAG_PWR_LATCH_DFF (1 << 4)
#endif

extern struct GlobalConfig globalConfig;

namespace {

constexpr uint32_t POWER_OFF_HOLD_MS = 3000;

bool buttonReleasedSinceBoot = false;
bool pressing = false;
uint32_t pressStartMs = 0;

bool validPin(uint8_t pin) { return pin != 0 && pin != 0xFF; }

bool dffLatchEnabled() {
    return (globalConfig.system_config.device_flags & DEVICE_FLAG_PWR_LATCH_DFF) &&
           validPin(globalConfig.system_config.pwr_pin_2) &&
           validPin(globalConfig.system_config.pwr_pin_3);
}

bool latchEnabled() {
    return (globalConfig.system_config.device_flags & DEVICE_FLAG_BATTERY_LATCH) &&
           validPin(globalConfig.system_config.pwr_pin_2);
}

gpio_num_t latchPin() { return (gpio_num_t)globalConfig.system_config.pwr_pin_2; }
int buttonPin() { return globalConfig.system_config.pwr_pin_3; }
bool hasButton() { return validPin(globalConfig.system_config.pwr_pin_3); }

static void dffClockPulse(uint8_t cpPin) {
    pinMode(cpPin, OUTPUT);
    digitalWrite(cpPin, LOW);
    delayMicroseconds(50);
    digitalWrite(cpPin, HIGH);
    delayMicroseconds(50);
}

static void dffLatchEngage() {
    const uint8_t dPin = globalConfig.system_config.pwr_pin_2;
    const uint8_t cpPin = globalConfig.system_config.pwr_pin_3;
    gpio_hold_dis((gpio_num_t)dPin);
    gpio_hold_dis((gpio_num_t)cpPin);
    pinMode(dPin, OUTPUT);
    digitalWrite(dPin, HIGH);
    delayMicroseconds(50);
    dffClockPulse(cpPin);
}

static void dffLatchRelease() {
    const uint8_t dPin = globalConfig.system_config.pwr_pin_2;
    const uint8_t cpPin = globalConfig.system_config.pwr_pin_3;
    gpio_hold_dis((gpio_num_t)dPin);
    gpio_hold_dis((gpio_num_t)cpPin);
    pinMode(dPin, OUTPUT);
    digitalWrite(dPin, LOW);
    delayMicroseconds(50);
    dffClockPulse(cpPin);
    gpio_set_direction((gpio_num_t)dPin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)dPin, 0);
    esp_sleep_config_gpio_isolate();
#if !defined(CONFIG_IDF_TARGET_ESP32C6)
    gpio_deep_sleep_hold_en();
#endif
    gpio_hold_en((gpio_num_t)dPin);
    pinMode(cpPin, OUTPUT);
    digitalWrite(cpPin, LOW);
}

void powerOff() {
    const gpio_num_t latch = latchPin();
    if (hasButton()) {
        pinMode(buttonPin(), INPUT_PULLUP);
        while (digitalRead(buttonPin()) == LOW) {
            delay(20);
        }
    }
    gpio_hold_dis(latch);
    gpio_set_direction(latch, GPIO_MODE_OUTPUT);
    gpio_set_level(latch, 0);
    esp_sleep_config_gpio_isolate();
#if !defined(CONFIG_IDF_TARGET_ESP32C6)
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
    if (dffLatchEnabled()) {
        dffLatchEngage();
        return;
    }
    if (!latchEnabled()) {
        return;
    }
    gpio_hold_dis(latchPin());
    if (hasButton()) {
        pinMode(buttonPin(), INPUT_PULLUP);
    }
}

void powerButtonPoll() {
    if (dffLatchEnabled() || !latchEnabled() || !hasButton()) {
        return;
    }
    const bool down = digitalRead(buttonPin()) == LOW;
    if (!down) {
        buttonReleasedSinceBoot = true;
        pressing = false;
        return;
    }
    if (!buttonReleasedSinceBoot) {
        return;
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
    if (dffLatchEnabled()) {
        const uint8_t dPin = globalConfig.system_config.pwr_pin_2;
        gpio_hold_dis((gpio_num_t)dPin);
        gpio_set_direction((gpio_num_t)dPin, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)dPin, 1);
#if !defined(CONFIG_IDF_TARGET_ESP32C6)
        gpio_deep_sleep_hold_en();
#endif
        gpio_hold_en((gpio_num_t)dPin);
        return;
    }
    if (!latchEnabled()) {
        return;
    }
    gpio_set_direction(latchPin(), GPIO_MODE_OUTPUT);
    gpio_set_level(latchPin(), 1);
#if !defined(CONFIG_IDF_TARGET_ESP32C6)
    gpio_deep_sleep_hold_en();
#endif
    gpio_hold_en(latchPin());
}

void powerLatchPowerOff() {
    if (!dffLatchEnabled()) {
        return;
    }
    dffLatchRelease();
}

bool powerLatchMosfetConfigured(void) {
    return latchEnabled();
}

void powerLatchTriggerOff() {
    if (dffLatchEnabled()) {
        dffLatchRelease();
    } else if (latchEnabled()) {
        powerOff();
    }
}

bool powerLatchDffConfigured(void) {
    return dffLatchEnabled();
}

#else  // not ESP32

void powerLatchBegin() {}
void powerButtonPoll() {}
void powerLatchHoldForSleep() {}

bool powerLatchDffConfigured(void) {
    return false;
}

void powerLatchPowerOff() {}

bool powerLatchMosfetConfigured(void) {
    return false;
}

void powerLatchTriggerOff() {}

#endif
