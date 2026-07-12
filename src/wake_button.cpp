#include "wake_button.h"

#if defined(TARGET_ESP32)

#include <Arduino.h>
#include "esp_sleep.h"
#include "soc/soc_caps.h"
#include "driver/gpio.h"
#if SOC_PM_SUPPORT_EXT1_WAKEUP
#include "driver/rtc_io.h"  // RTC pull retention for ext0/ext1 pads
#endif
#include "structs.h"

#ifndef DEVICE_FLAG_BATTERY_LATCH
#define DEVICE_FLAG_BATTERY_LATCH (1 << 3)
#endif
#ifndef DEVICE_FLAG_PWR_LATCH_DFF
#define DEVICE_FLAG_PWR_LATCH_DFF (1 << 4)
#endif

extern struct GlobalConfig globalConfig;
extern ButtonState buttonStates[MAX_BUTTONS];
extern uint8_t buttonStateCount;
void writeSerial(String message, bool newLine = true);

// There is no ext0 status register, so the armed pin is remembered across the
// sleep for detectButtonWake() to name. 0xFF = ext0 not armed this cycle.
RTC_DATA_ATTR static uint8_t s_ext0WakePin = 0xFF;

namespace {

bool validPin(uint8_t pin) { return pin != 0 && pin != 0xFF; }

struct WakeCandidate {
    uint8_t pin;
    bool wakeHigh;   // wake level == pressed level
    bool pullup;     // configured internal pulls (retained across sleep)
    bool pulldown;
};

String maskToHex(uint64_t mask) {
    String s = "";
    bool started = false;
    for (int shift = 60; shift >= 0; shift -= 4) {
        uint8_t nib = (mask >> shift) & 0xF;
        if (!started && nib == 0 && shift != 0) continue;
        started = true;
        s += (char)(nib < 10 ? ('0' + nib) : ('A' + nib - 10));
    }
    return s;
}

#if !SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP && SOC_PM_SUPPORT_EXT1_WAKEUP
// With RTC_PERIPH powered down, IDF maintains pulls configured through the RTC
// IO registers via the HOLD feature — the digital-domain pinMode pulls do not
// survive, so re-assert them here for every armed pad. A pad with no internal
// pull depends on external hardware to idle at the released level.
void retainWakePull(const WakeCandidate& c) {
    const gpio_num_t gpio = (gpio_num_t)c.pin;
    if (c.pullup) {
        rtc_gpio_pullup_en(gpio);
        rtc_gpio_pulldown_dis(gpio);
    } else if (c.pulldown) {
        rtc_gpio_pulldown_en(gpio);
        rtc_gpio_pullup_dis(gpio);
    } else {
        writeSerial("Wake: pin " + String(c.pin) + " has no internal pull - floating wake pin may cause spurious wakes");
    }
}

void warnUnarmedPins(uint64_t mask, const char* reason) {
    for (uint8_t pin = 0; pin < 64; pin++) {
        if (mask & (1ULL << pin)) {
            writeSerial("Wake: pin " + String(pin) + " not armed (" + String(reason) + ") - timer-only for this button");
        }
    }
}
#endif

}  // namespace

void armButtonWakeSources() {
    if (globalConfig.power_option.sleep_flags & SLEEP_FLAG_BUTTON_WAKE_DISABLE) {
        writeSerial("Button wake disabled (sleep_flags) - timer-only deep sleep");
        return;
    }
    const uint8_t deviceFlags = globalConfig.system_config.device_flags;
    const uint8_t pwrPin2 = globalConfig.system_config.pwr_pin_2;
    const uint8_t pwrPin3 = globalConfig.system_config.pwr_pin_3;

    // Candidates: every initialized button (initButtons() already excluded 0xFF
    // and the GT911 INT pin), plus the MOSFET-latch power button so it wakes
    // from timer sleep the same way it wakes from powerOff().
    WakeCandidate candidates[MAX_BUTTONS + 1];
    uint8_t candidateCount = 0;
    for (uint8_t i = 0; i < buttonStateCount && i < MAX_BUTTONS; i++) {
        const ButtonState& btn = buttonStates[i];
        if (!btn.initialized || btn.instance_index >= 4) continue;
        const BinaryInputs& input = globalConfig.binary_inputs[btn.instance_index];
        WakeCandidate& c = candidates[candidateCount++];
        c.pin = btn.pin;
        c.wakeHigh = !btn.inverted;
        c.pullup = (input.pullups & (1 << btn.pin_offset)) != 0;
        c.pulldown = (input.pulldowns & (1 << btn.pin_offset)) != 0;
    }
    if ((deviceFlags & DEVICE_FLAG_BATTERY_LATCH) && validPin(pwrPin3)) {
        WakeCandidate& c = candidates[candidateCount++];
        c.pin = pwrPin3;
        c.wakeHigh = false;      // shutdown button is active-low
        c.pullup = true;         // powerLatchBegin() keeps it INPUT_PULLUP
        c.pulldown = false;
    }

    uint64_t lowMask = 0;
    uint64_t highMask = 0;
    WakeCandidate eligible[MAX_BUTTONS + 1];
    uint8_t eligibleCount = 0;
    s_ext0WakePin = 0xFF;
    for (uint8_t i = 0; i < candidateCount; i++) {
        const WakeCandidate& c = candidates[i];
        if (validPin(pwrPin2) && c.pin == pwrPin2) {
            // pwr_pin_2 is the latch hold pin (MOSFET enable / D-FF D input);
            // it is an output, never a wake button.
            writeSerial("Wake: pin " + String(c.pin) + " is the power latch pin - not armed");
            continue;
        }
        if ((deviceFlags & DEVICE_FLAG_PWR_LATCH_DFF) && validPin(pwrPin3) && c.pin == pwrPin3) {
            // On D-FF boards pwr_pin_3 is the 74AHC1G79 CP clock: a wake-armed
            // pull or level change could clock the latch off and cut power.
            writeSerial("Wake: pin " + String(c.pin) + " is the D-FF latch clock - not armed");
            continue;
        }
        if (!esp_sleep_is_valid_wakeup_gpio((gpio_num_t)c.pin)) {
            writeSerial("Wake: pin " + String(c.pin) + " not wake-capable on this chip - timer-only for this button");
            continue;
        }
        if (digitalRead(c.pin) == (c.wakeHigh ? HIGH : LOW)) {
            // Already at its wake level: arming would wake instantly and
            // ping-pong. The pin re-qualifies next sleep entry after release.
            writeSerial("Wake: pin " + String(c.pin) + " held at sleep entry - skipped this cycle");
            continue;
        }
        if (c.wakeHigh) highMask |= 1ULL << c.pin;
        else lowMask |= 1ULL << c.pin;
        eligible[eligibleCount++] = c;
    }

    if (lowMask == 0 && highMask == 0) {
        writeSerial("No wake-capable buttons - timer-only deep sleep");
        return;
    }

#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    // C3/C6: LP GPIO wake; esp_deep_sleep_start() auto-enables the pull
    // opposite each wake level (CONFIG_ESP_SLEEP_GPIO_ENABLE_INTERNAL_RESISTORS).
    if (lowMask) {
        esp_err_t err = esp_deep_sleep_enable_gpio_wakeup(lowMask, ESP_GPIO_WAKEUP_GPIO_LOW);
        if (err != ESP_OK) {
            writeSerial("Wake: gpio LOW arm failed (err " + String(err) + ") - timer-only for mask 0x" + maskToHex(lowMask));
        } else {
            writeSerial("Wake: gpio LOW armed, mask 0x" + maskToHex(lowMask));
        }
    }
    if (highMask) {
        // Mixed polarity needs the second call to accumulate per-pin; on error
        // keep the LOW group and never abort the sleep.
        esp_err_t err = esp_deep_sleep_enable_gpio_wakeup(highMask, ESP_GPIO_WAKEUP_GPIO_HIGH);
        if (err != ESP_OK) {
            writeSerial("Wake: gpio HIGH arm failed (err " + String(err) + ") - timer-only for mask 0x" + maskToHex(highMask));
        } else {
            writeSerial("Wake: gpio HIGH armed, mask 0x" + maskToHex(highMask));
        }
    }
#elif SOC_PM_SUPPORT_EXT1_WAKEUP
    uint64_t armedMask = 0;
#if CONFIG_IDF_TARGET_ESP32
    // Classic ESP32 ext1 has no ANY_LOW mode: the HIGH group rides ext1
    // ANY_HIGH; ext0 covers exactly one LOW pin (the lowest-numbered).
    if (highMask) {
        esp_sleep_enable_ext1_wakeup(highMask, ESP_EXT1_WAKEUP_ANY_HIGH);
        armedMask |= highMask;
        writeSerial("Wake: ext1 ANY_HIGH armed, mask 0x" + maskToHex(highMask));
    }
    if (lowMask) {
        const uint8_t firstLowPin = (uint8_t)__builtin_ctzll(lowMask);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)firstLowPin, 0);
        s_ext0WakePin = firstLowPin;
        armedMask |= 1ULL << firstLowPin;
        writeSerial("Wake: ext0 LOW armed on pin " + String(firstLowPin));
        warnUnarmedPins(lowMask & ~(1ULL << firstLowPin), "classic ESP32 ext0 takes one LOW pin");
    }
#else
    // S2/S3: one ext1 call only (a second replaces the first), so the larger
    // polarity group takes ext1 and the other group's first pin takes ext0.
    if (__builtin_popcountll(highMask) >= __builtin_popcountll(lowMask)) {
        if (highMask) {
            esp_sleep_enable_ext1_wakeup(highMask, ESP_EXT1_WAKEUP_ANY_HIGH);
            armedMask |= highMask;
            writeSerial("Wake: ext1 ANY_HIGH armed, mask 0x" + maskToHex(highMask));
        }
        if (lowMask) {
            const uint8_t firstLowPin = (uint8_t)__builtin_ctzll(lowMask);
            esp_sleep_enable_ext0_wakeup((gpio_num_t)firstLowPin, 0);
            s_ext0WakePin = firstLowPin;
            armedMask |= 1ULL << firstLowPin;
            writeSerial("Wake: ext0 LOW armed on pin " + String(firstLowPin));
            warnUnarmedPins(lowMask & ~(1ULL << firstLowPin), "ext1 taken by HIGH group, ext0 takes one pin");
        }
    } else {
        esp_sleep_enable_ext1_wakeup(lowMask, ESP_EXT1_WAKEUP_ANY_LOW);
        armedMask |= lowMask;
        writeSerial("Wake: ext1 ANY_LOW armed, mask 0x" + maskToHex(lowMask));
        if (highMask) {
            const uint8_t firstHighPin = (uint8_t)__builtin_ctzll(highMask);
            esp_sleep_enable_ext0_wakeup((gpio_num_t)firstHighPin, 1);
            s_ext0WakePin = firstHighPin;
            armedMask |= 1ULL << firstHighPin;
            writeSerial("Wake: ext0 HIGH armed on pin " + String(firstHighPin));
            warnUnarmedPins(highMask & ~(1ULL << firstHighPin), "ext1 taken by LOW group, ext0 takes one pin");
        }
    }
#endif
    // Re-assert configured pulls through the RTC IO registers on every armed
    // pad so each pin idles at its released level through the sleep.
    for (uint8_t i = 0; i < eligibleCount; i++) {
        if (armedMask & (1ULL << eligible[i].pin)) {
            retainWakePull(eligible[i]);
        }
    }
#endif
}

bool detectButtonWake(int wakeupCause) {
    switch ((esp_sleep_wakeup_cause_t)wakeupCause) {
        // The EXT0/EXT1 causes are guarded by hardware capability: the enum
        // values exist on every chip, but the status symbol does not link on
        // chips without ext1 (C3), and neither cause can occur where the
        // hardware is absent — the default case covers them defensively.
#if SOC_PM_SUPPORT_EXT0_WAKEUP
        case ESP_SLEEP_WAKEUP_EXT0:
            writeSerial("Wake-up reason: EXT0 button (pin " + String(s_ext0WakePin) + ")");
            return true;
#endif
#if SOC_PM_SUPPORT_EXT1_WAKEUP
        case ESP_SLEEP_WAKEUP_EXT1:
            writeSerial("Wake-up reason: EXT1 button (pin mask 0x" + maskToHex(esp_sleep_get_ext1_wakeup_status()) + ")");
            return true;
#endif
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
        case ESP_SLEEP_WAKEUP_GPIO:
            writeSerial("Wake-up reason: GPIO button (pin mask 0x" + maskToHex(esp_sleep_get_gpio_wakeup_status()) + ")");
            return true;
#endif
        case ESP_SLEEP_WAKEUP_TIMER:
            writeSerial("Wake-up reason: timer");
            return false;
        default:
            writeSerial("Wake-up reason: " + String(wakeupCause) + " (not a button)");
            return false;
    }
}

#else  // not ESP32

void armButtonWakeSources() {}

bool detectButtonWake(int wakeupCause) {
    (void)wakeupCause;
    return false;
}

#endif
