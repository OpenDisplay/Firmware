#include "buzzer_hw.h"
#include <Arduino.h>

// ---------------------------------------------------------------------------
// nRF52 (Adafruit core) -- HardwarePWM instance HwPWM3.
//   HwPWM0..2 are left for core/other features; HwPWM3 exists on nRF52840.
//   16 MHz base clock, 15-bit COUNTERTOP. We walk the prescaler DIV_1..DIV_128
//   and pick the smallest divider whose top = round(clk*100 / centihz) fits in
//   15 bits (<= 32767), giving <= ~0.5 cent pitch error across the range.
//   Ownership is taken/released cooperatively so the peripheral is only held
//   while a tone is actually sounding. Runs alongside the SoftDevice.
// ---------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_NRF52)

#include "HardwarePWM.h"

// Non-zero cooperative ownership token ("BZZ!").
static const uint32_t kBuzzerPwmToken = 0x425A5A21u;
static const uint32_t kBuzzerPwmClockHz = 16000000u;   // DIV_1 base clock
static const uint32_t kBuzzerPwmMaxTop = 32767u;       // 15-bit COUNTERTOP

bool buzzer_hw_tone_start(uint8_t pin, uint32_t centihz, uint8_t duty_percent) {
    if (centihz == 0) {
        return false;
    }
    if (duty_percent == 0 || duty_percent > 100) {
        duty_percent = 50;
    }

    // Prescaler walk: DIV_1..DIV_128 map to clock shifts of 0..7.
    uint8_t div_exp = 0;
    uint32_t top = 0;
    for (div_exp = 0; div_exp <= 7; div_exp++) {
        uint32_t clk = kBuzzerPwmClockHz >> div_exp;
        uint64_t t = ((uint64_t)clk * 100u + (centihz / 2u)) / centihz;   // round
        if (t == 0) {
            t = 1;
        }
        if (t <= kBuzzerPwmMaxTop) {
            top = (uint32_t)t;
            break;
        }
    }
    if (top == 0) {
        return false;   // frequency too low even at DIV_128 (should not happen in-range)
    }

    // Ownership: take it only if we don't already hold it.
    if (!HwPWM3.isOwner(kBuzzerPwmToken)) {
        if (!HwPWM3.takeOwnership(kBuzzerPwmToken)) {
            return false;   // owned by something else
        }
    }

    // PRESCALER/COUNTERTOP must be set while stopped; writePin -> begin() then
    // reloads them and restarts the sequence.
    if (HwPWM3.enabled()) {
        HwPWM3.stop();
    }
    HwPWM3.setClockDiv((uint8_t)div_exp);          // PWM_PRESCALER_PRESCALER_DIV_x == div_exp
    HwPWM3.setMaxValue((uint16_t)top);
    HwPWM3.addPin(pin);

    uint32_t val = (top * (uint32_t)duty_percent) / 100u;
    if (val == 0) {
        val = 1;
    }
    if (val >= top) {
        val = top - 1;
    }
    HwPWM3.writePin(pin, (uint16_t)val);
    return true;
}

void buzzer_hw_tone_stop(uint8_t pin) {
    if (HwPWM3.isOwner(kBuzzerPwmToken)) {
        if (HwPWM3.checkPin(pin)) {
            HwPWM3.removePin(pin);
        }
        HwPWM3.stop();
        HwPWM3.releaseOwnership(kBuzzerPwmToken);
    }
    // Always leave the pin defined and LOW.
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

// ---------------------------------------------------------------------------
// ESP32 -- LEDC, shimmed across the build matrix. pioarduino envs ship Arduino
// core 3.x (ESP_ARDUINO_VERSION_MAJOR >= 3, pin-based LEDC API); the legacy
// `platform = espressif32` envs ship 2.x (channel-based API). Frequency is set
// from centi-Hz -- integer Hz on 3.x (<= 4 cents at the 403 Hz bottom of the
// playable range), full double precision on 2.x.
// ---------------------------------------------------------------------------
#elif defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>

static const uint8_t kBuzzerLedcResBits = 10;          // 10-bit -> duty 0..1023
static const uint32_t kBuzzerLedcMaxDuty = 1023u;

// Track the pin currently attached to LEDC (0xFF = none). tone_stop is called
// on every rest step and at melody end even when nothing was attached; detaching
// an unattached pin makes the core log `ledcDetach(): pin N is not attached`, so
// we only attach/detach against this tracked state (keeps tone_stop idempotent).
static uint8_t s_ledc_pin = 0xFF;

#if ESP_ARDUINO_VERSION_MAJOR >= 3

bool buzzer_hw_tone_start(uint8_t pin, uint32_t centihz, uint8_t duty_percent) {
    if (centihz == 0) {
        return false;
    }
    if (duty_percent == 0 || duty_percent > 100) {
        duty_percent = 50;
    }
    uint32_t freq_hz = (centihz + 50u) / 100u;         // round to integer Hz
    if (freq_hz == 0) {
        return false;
    }
    // Release any prior attachment before re-attaching (normal flow stops first,
    // but stay robust to a start-over-start).
    if (s_ledc_pin != 0xFF) {
        ledcDetach(s_ledc_pin);
        s_ledc_pin = 0xFF;
    }
    if (!ledcAttach(pin, freq_hz, kBuzzerLedcResBits)) {
        return false;
    }
    s_ledc_pin = pin;
    uint32_t duty = (kBuzzerLedcMaxDuty * (uint32_t)duty_percent) / 100u;
    if (duty == 0) {
        duty = 1;
    }
    ledcWrite(pin, duty);
    return true;
}

void buzzer_hw_tone_stop(uint8_t pin) {
    if (s_ledc_pin == pin) {
        ledcWrite(pin, 0);
        ledcDetach(pin);
        s_ledc_pin = 0xFF;
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

#else   // ESP_ARDUINO_VERSION_MAJOR < 3 (legacy 2.x core)

static const uint8_t kBuzzerLedcChannel = 7;           // reserved buzzer channel

bool buzzer_hw_tone_start(uint8_t pin, uint32_t centihz, uint8_t duty_percent) {
    if (centihz == 0) {
        return false;
    }
    if (duty_percent == 0 || duty_percent > 100) {
        duty_percent = 50;
    }
    double freq_hz = (double)centihz / 100.0;          // full precision
    ledcSetup(kBuzzerLedcChannel, freq_hz, kBuzzerLedcResBits);
    ledcAttachPin(pin, kBuzzerLedcChannel);
    s_ledc_pin = pin;
    uint32_t duty = (kBuzzerLedcMaxDuty * (uint32_t)duty_percent) / 100u;
    if (duty == 0) {
        duty = 1;
    }
    ledcWrite(kBuzzerLedcChannel, duty);
    return true;
}

void buzzer_hw_tone_stop(uint8_t pin) {
    if (s_ledc_pin == pin) {
        ledcWrite(kBuzzerLedcChannel, 0);
        ledcDetachPin(pin);
        s_ledc_pin = 0xFF;
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

#endif  // ESP_ARDUINO_VERSION_MAJOR

// ---------------------------------------------------------------------------
#else
#error "buzzer_hw: unsupported platform (need ARDUINO_ARCH_NRF52 or ARDUINO_ARCH_ESP32)"
#endif
