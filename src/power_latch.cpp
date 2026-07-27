// Soft power latch: a MOSFET or D-flip-flop that holds the device rail on, and the
// long-press power button that releases it.
//
// The whole file used to sit under #if defined(TARGET_ESP32), with a block of empty
// stubs for every other target. That guard was inherited, not required: the latch is
// a board feature (DEVICE_FLAG_BATTERY_LATCH / DEVICE_FLAG_PWR_LATCH_DFF plus
// pwr_pin_2/pwr_pin_3), not a SoC feature, and every predicate, the press state
// machine and the D-FF clock pulse are plain Arduino GPIO. Only two things are
// genuinely ESP32-specific, and they are shimmed below rather than fencing the file:
// pad-hold across deep sleep, and the deep-sleep park after the rail is cut.
//
// No nRF board ships this hardware today, so the flags are simply never set there and
// every entry point returns early exactly as the old stubs did. The difference is that
// an nRF board that DOES get a latch now works without reviving a deleted arm.

#include "power_latch.h"

#include <Arduino.h>

#include "od_log.h"
#include "structs.h"

#if defined(TARGET_ESP32)
#include "driver/gpio.h"
#include "esp_sleep.h"  // pulls in SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
#endif

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

// --- platform shims ----------------------------------------------------------
// ESP32 pad-hold freezes a pin's level so it survives deep sleep and the reset that
// ends it -- which is what keeps the rail latched while the CPU is off. nRF has no
// deep-sleep path here and its pads hold their level for as long as the core runs,
// so there is nothing to freeze and these are no-ops.

inline void padHoldDisable(uint8_t pin) {
#if defined(TARGET_ESP32)
    gpio_hold_dis((gpio_num_t)pin);
#else
    (void)pin;
#endif
}

inline void padHold(uint8_t pin) {
#if defined(TARGET_ESP32)
#if !defined(CONFIG_IDF_TARGET_ESP32C6)
    gpio_deep_sleep_hold_en();
#endif
    gpio_hold_en((gpio_num_t)pin);
#else
    (void)pin;
#endif
}

// Same, preceded by the all-pad isolate the rail-cut paths do (but the
// hold-for-sleep path deliberately does not -- kept distinct rather than merged,
// so the ESP32 call sequences stay byte-for-byte what they were).
inline void padIsolateAndHold(uint8_t pin) {
#if defined(TARGET_ESP32)
    esp_sleep_config_gpio_isolate();
#endif
    padHold(pin);
}

// Wait to die after the rail has been cut. On latch hardware the rail normally
// drops before this returns; it is the fallback for when it does not. ESP32 waits
// in deep sleep (lowest current, and the button can wake it); elsewhere there is no
// such state, so park loudly rather than returning into a loop() whose rail is
// supposed to be gone.
[[noreturn]] void parkAfterRailCut(int wakePin, bool haveWakePin) {
#if defined(TARGET_ESP32)
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    if (haveWakePin) {
        pinMode(wakePin, INPUT_PULLUP);
        esp_deep_sleep_enable_gpio_wakeup(1ULL << wakePin, ESP_GPIO_WAKEUP_GPIO_LOW);
    }
#else
    (void)wakePin;
    (void)haveWakePin;
#endif
    esp_deep_sleep_start();
#else
    (void)wakePin;
    (void)haveWakePin;
    od_log_warn("Power latch: rail cut but still running -- parking");
    od_log_flush();
    for (;;) {
        delay(1000);
    }
#endif
}

// --- configuration -----------------------------------------------------------

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

uint8_t latchPin() { return globalConfig.system_config.pwr_pin_2; }
int buttonPin() { return globalConfig.system_config.pwr_pin_3; }
bool hasButton() { return validPin(globalConfig.system_config.pwr_pin_3); }

// --- D flip-flop latch --------------------------------------------------------

void dffClockPulse(uint8_t cpPin) {
    pinMode(cpPin, OUTPUT);
    digitalWrite(cpPin, LOW);
    delayMicroseconds(50);
    digitalWrite(cpPin, HIGH);
    delayMicroseconds(50);
}

void dffLatchEngage() {
    const uint8_t dPin = globalConfig.system_config.pwr_pin_2;
    const uint8_t cpPin = globalConfig.system_config.pwr_pin_3;
    padHoldDisable(dPin);
    padHoldDisable(cpPin);
    pinMode(dPin, OUTPUT);
    digitalWrite(dPin, HIGH);
    delayMicroseconds(50);
    dffClockPulse(cpPin);
}

void dffLatchRelease() {
    const uint8_t dPin = globalConfig.system_config.pwr_pin_2;
    const uint8_t cpPin = globalConfig.system_config.pwr_pin_3;
    padHoldDisable(dPin);
    padHoldDisable(cpPin);
    pinMode(dPin, OUTPUT);
    digitalWrite(dPin, LOW);
    delayMicroseconds(50);
    dffClockPulse(cpPin);
    pinMode(dPin, OUTPUT);
    digitalWrite(dPin, LOW);
    padIsolateAndHold(dPin);
    pinMode(cpPin, OUTPUT);
    digitalWrite(cpPin, LOW);
}

// --- MOSFET latch -------------------------------------------------------------

[[noreturn]] void powerOff() {
    const uint8_t latch = latchPin();
    if (hasButton()) {
        pinMode(buttonPin(), INPUT_PULLUP);
        while (digitalRead(buttonPin()) == LOW) {
            delay(20);
        }
    }
    padHoldDisable(latch);
    pinMode(latch, OUTPUT);
    digitalWrite(latch, LOW);
    padIsolateAndHold(latch);
    parkAfterRailCut(buttonPin(), hasButton());
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
    padHoldDisable(latchPin());
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
        padHoldDisable(dPin);
        pinMode(dPin, OUTPUT);
        digitalWrite(dPin, HIGH);
        padHold(dPin);
        return;
    }
    if (!latchEnabled()) {
        return;
    }
    pinMode(latchPin(), OUTPUT);
    digitalWrite(latchPin(), HIGH);
    padHold(latchPin());
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
