#include "buzzer_control.h"
#include "structs.h"
#include <Arduino.h>
#include <string.h>

extern struct GlobalConfig globalConfig;
void sendResponse(uint8_t* response, uint8_t len);

static_assert(sizeof(PassiveBuzzerConfig) == 32, "PassiveBuzzerConfig must be 32 bytes");

static constexpr uint32_t kBuzzerFreqMinHz = 400u;
static constexpr uint32_t kBuzzerFreqMaxHz = 12000u;
static constexpr uint8_t kBuzzerDurationUnitMs = 5u;
static constexpr uint32_t kBuzzerInterPatternGapMs = 20u;

static uint32_t buzzer_index_to_hz(uint8_t idx) {
    if (idx == 0) {
        return 0;
    }
    uint32_t fmin = kBuzzerFreqMinHz;
    uint32_t fmax = kBuzzerFreqMaxHz;
    uint32_t span = fmax - fmin;
    return fmin + (span * (uint32_t)(idx - 1)) / 254u;
}

static void buzzer_set_enable(const PassiveBuzzerConfig* b, bool on) {
    if (b->enable_pin == 0xFF) {
        return;
    }
    bool high = on;
    if ((b->flags & BUZZER_FLAG_ENABLE_ACTIVE_HIGH) == 0) {
        high = !high;
    }
    digitalWrite(b->enable_pin, high ? HIGH : LOW);
}

static void buzzer_drive_off(const PassiveBuzzerConfig* b) {
    pinMode(b->drive_pin, OUTPUT);
    digitalWrite(b->drive_pin, LOW);
}

static void buzzer_drive_tone_sw(const PassiveBuzzerConfig* b, uint32_t hz, uint32_t ms) {
    uint8_t duty = b->duty_percent;
    if (duty == 0 || duty > 100) {
        duty = 50;
    }
    if (ms == 0) {
        return;
    }
    if (hz == 0 || duty == 0) {
        buzzer_drive_off(b);
        delay(ms);
        return;
    }

    uint8_t pin = b->drive_pin;
    uint32_t period_us = 1000000u / hz;
    if (period_us < 2u) {
        period_us = 2u;
    }
    uint32_t on_us = (period_us * (uint32_t)duty) / 100u;
    if (on_us == 0u) {
        on_us = 1u;
    }
    if (on_us >= period_us) {
        on_us = period_us - 1u;
    }
    uint32_t off_us = period_us - on_us;

    const uint32_t total_us = ms * 1000u;
    const uint32_t start_us = micros();
    while ((uint32_t)(micros() - start_us) < total_us) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(on_us);
        digitalWrite(pin, LOW);
        delayMicroseconds(off_us);
    }
}

void initPassiveBuzzers(void) {
    for (uint8_t i = 0; i < globalConfig.passive_buzzer_count; i++) {
        const PassiveBuzzerConfig* b = &globalConfig.passive_buzzers[i];
        if (b->drive_pin == 0xFF) {
            continue;
        }
        pinMode(b->drive_pin, OUTPUT);
        digitalWrite(b->drive_pin, LOW);
        if (b->enable_pin != 0xFF) {
            pinMode(b->enable_pin, OUTPUT);
            buzzer_set_enable(b, false);
        }
    }
}

void handleBuzzerActivate(uint8_t* data, uint16_t len) {
    if (len < 3) {
        uint8_t err[] = {0xFF, 0x75, 0x01, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }
    uint8_t inst = data[0];
    if (inst >= globalConfig.passive_buzzer_count) {
        uint8_t err[] = {0xFF, 0x75, 0x02, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }
    PassiveBuzzerConfig* b = &globalConfig.passive_buzzers[inst];
    if (b->drive_pin == 0xFF) {
        uint8_t err[] = {0xFF, 0x75, 0x03, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }

    uint8_t outer = data[1];
    if (outer == 0) {
        outer = 1;
    }
    uint8_t pattern_count = data[2];
    if (pattern_count == 0) {
        uint8_t err[] = {0xFF, 0x75, 0x04, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }

    uint16_t scan = 3;
    for (uint8_t pi = 0; pi < pattern_count; pi++) {
        if (scan >= len) {
            uint8_t err[] = {0xFF, 0x75, 0x05, 0x00};
            sendResponse(err, sizeof(err));
            return;
        }
        uint8_t nsteps = data[scan++];
        uint32_t need = (uint32_t)nsteps * 2u;
        if (scan + need > len) {
            uint8_t err[] = {0xFF, 0x75, 0x05, 0x00};
            sendResponse(err, sizeof(err));
            return;
        }
        scan = (uint16_t)(scan + need);
    }
    if (scan != len) {
        uint8_t err[] = {0xFF, 0x75, 0x06, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }

    for (uint8_t rep = 0; rep < outer; rep++) {
        uint16_t poff = 3;
        for (uint8_t pi = 0; pi < pattern_count; pi++) {
            uint8_t nsteps = data[poff++];
            for (uint8_t si = 0; si < nsteps; si++) {
                uint8_t fidx = data[poff++];
                uint8_t dunit = data[poff++];
                uint32_t hz = buzzer_index_to_hz(fidx);
                uint32_t ms = (uint32_t)dunit * (uint32_t)kBuzzerDurationUnitMs;
                buzzer_set_enable(b, true);
                buzzer_drive_tone_sw(b, hz, ms);
            }
            if (pi + 1 < pattern_count) {
                delay(kBuzzerInterPatternGapMs);
            }
        }
    }

    buzzer_set_enable(b, false);
    buzzer_drive_off(b);

    uint8_t ok[] = {0x00, 0x75, 0x00, 0x00};
    sendResponse(ok, sizeof(ok));
}
