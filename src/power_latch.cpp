#include "power_latch.h"

#if defined(OPENDISPLAY_XTEINK_X4)

#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_sleep.h"

namespace {

// GPIO13 gates the battery-latch MOSFET: driving it LOW opens the latch and cuts
// the battery rail entirely. The latch self-holds in hardware once powered, so we
// never drive it HIGH (see powerLatchBegin). DIO flash mode frees GPIO13 (the QIO
// write-protect pin) — see the build env.
constexpr gpio_num_t LATCH_PIN = GPIO_NUM_13;
// The inset "dimple" button (active-low) — firmware reads it for long-press
// shutdown. Power-ON is a separate protruding hardware button wired to the power
// path/reset, not to a GPIO we read.
constexpr gpio_num_t POWER_BTN_PIN = GPIO_NUM_3;
constexpr uint32_t POWER_OFF_HOLD_MS = 1500;

// Only arm long-press shutdown after the dimple has been seen released once, so a
// press still held from a GPIO3-low wake (USB) can't instantly re-trigger it.
bool buttonReleasedSinceBoot = false;
bool pressing = false;
uint32_t pressStartMs = 0;

void powerOff() {
    // Wait for the dimple to be released before sleeping: we arm a GPIO3-low wake
    // below, so sleeping while it is still held would immediately wake us again.
    pinMode(POWER_BTN_PIN, INPUT_PULLUP);
    while (digitalRead(POWER_BTN_PIN) == LOW) {
        delay(20);
    }
    // Open the latch and hold it across sleep. On battery the rail collapses to a
    // true zero-drain off (the hardware power button revives it); on USB the rail
    // stays powered, so we deep-sleep and wake on the next dimple press.
    gpio_hold_dis(LATCH_PIN);
    gpio_set_direction(LATCH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LATCH_PIN, 0);
    esp_sleep_config_gpio_isolate();
    gpio_deep_sleep_hold_en();
    gpio_hold_en(LATCH_PIN);
    pinMode(POWER_BTN_PIN, INPUT_PULLUP);
    esp_deep_sleep_enable_gpio_wakeup(1ULL << POWER_BTN_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
}

}  // namespace

void powerLatchBegin() {
    // Mirror crosspoint, the reference firmware for this board: the battery-latch
    // MOSFET self-holds in hardware once the rail is powered, so firmware never
    // drives GPIO13 HIGH. It is only ever driven LOW (in powerOff) to cut power.
    // Release any pin-hold left by a prior deep sleep so GPIO13 returns to hardware
    // control (a USB wake from powerOff would otherwise keep the latch held open),
    // then arm the power button.
    gpio_hold_dis(LATCH_PIN);
    pinMode(POWER_BTN_PIN, INPUT_PULLUP);
}

void powerButtonPoll() {
    const bool down = digitalRead(POWER_BTN_PIN) == LOW;
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
    gpio_set_direction(LATCH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LATCH_PIN, 1);
    gpio_deep_sleep_hold_en();
    gpio_hold_en(LATCH_PIN);
}

#else  // not XTEINK X4

void powerLatchBegin() {}
void powerButtonPoll() {}
void powerLatchHoldForSleep() {}

#endif
