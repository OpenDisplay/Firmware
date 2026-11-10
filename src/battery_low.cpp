#include "battery_low.h"
#include "structs.h"
#include "sensor_bq27220.h"
#include "display_service.h"
#include "od_log.h"

#include <Arduino.h>

extern struct GlobalConfig globalConfig;

static constexpr float kHysteresisV = 0.10f;
static constexpr uint8_t kSocTripThreshold = 5;
static constexpr uint8_t kSocRecoverThreshold = 10;
static constexpr uint8_t kDebounceStrikes = 3;
static constexpr uint32_t kPollIntervalMs = 60000u;

// Indexed by CapacityEstimator. Index 0 (unset) is 0.0f, meaning "never low".
static constexpr float kCutoffVolts[6] = {
    0.0f,   // unset
    3.350f, // OD_CAPACITY_EST_LI_ION
    3.000f, // OD_CAPACITY_EST_LIFEPO4
    3.600f, // OD_CAPACITY_EST_SUPERCAP
    2.700f, // OD_CAPACITY_EST_LITHIUM_PRIMARY
    3.300f, // OD_CAPACITY_EST_SEEED_LI_ION
};

struct BatteryLowLatch {
    bool low;
    bool screenDrawn;
    uint8_t lowStrikes;
};

// RTC_DATA_ATTR on ESP32 so the latch survives a deep-sleep reboot.
// Without it the low-battery screen would redraw on every wake, and the
// debounce strike count would never accumulate across the short wake windows
// between deep sleeps. nRF never deep sleeps, so a plain static is enough
// there.
#ifdef TARGET_ESP32
RTC_DATA_ATTR static BatteryLowLatch s_latch = {false, false, 0};
#else
static BatteryLowLatch s_latch = {false, false, 0};
#endif

static float cutoffForEstimator(uint8_t estimator) {
    if (estimator >= (sizeof(kCutoffVolts) / sizeof(kCutoffVolts[0]))) {
        return 0.0f;
    }
    return kCutoffVolts[estimator];
}

bool isBatteryLow(void) {
    if (globalConfig.power_option.power_mode != OD_POWER_MODE_BATTERY) {
        s_latch.low = false;
        s_latch.lowStrikes = 0;
        return false;
    }

    const uint8_t estimator = globalConfig.power_option.capacity_estimator;
    const float cutoff = cutoffForEstimator(estimator);
    if (cutoff <= 0.0f) {
        s_latch.low = false;
        s_latch.lowStrikes = 0;
        return false;
    }

    // LITHIUM_PRIMARY cells are not rechargeable, so a charge-pin reading is
    // meaningless for them and must not suppress the low check.
    const bool lithiumPrimary = isEstimatorLithiumPrimary(estimator);
    const bool charging = chargerIsCharging();
    if (charging && !lithiumPrimary) {
        s_latch.low = false;
        return false;
    }

    const bool gaugeHealthy = bq27220IsConfigured() && bq27220GaugeHealthy();

    if (s_latch.low) {
        // A primary cell rebounds at rest after load, so a voltage recovery
        // would cycle the device through reset and re-trip. Only a battery
        // swap (power-on reset) clears this latch.
        if (lithiumPrimary) {
            return true;
        }
        if (gaugeHealthy) {
            const uint8_t soc = bq27220Soc();
            if (soc >= kSocRecoverThreshold) {
                od_log_info("Battery low: recovered (SOC %u%%)", soc);
                s_latch.low = false;
                return false;
            }
            // Mirrors the trip path's unlearned-gauge fallback: a gauge that
            // has not learned the pack's real capacity can keep reporting a
            // low SOC long after voltage has genuinely recovered, which would
            // otherwise latch the device low forever.
            const float gaugeVoltage = readBatteryVoltage();
            const bool unlearned = gaugeVoltage >= 0.0f &&
                                   gaugeVoltage > (cutoff + kHysteresisV);
            if (unlearned) {
                od_log_info("Battery low: recovered (unlearned gauge, %d mV)",
                            (int)(gaugeVoltage * 1000.0f));
                s_latch.low = false;
                return false;
            }
            return true;
        }
        const float voltage = readBatteryVoltage();
        if (voltage >= 0.0f && voltage > (cutoff + kHysteresisV)) {
            od_log_info("Battery low: recovered (%d mV)", (int)(voltage * 1000.0f));
            s_latch.low = false;
            return false;
        }
        return true;
    }

    if (gaugeHealthy) {
        const uint8_t soc = bq27220Soc();
        if (soc > kSocTripThreshold) {
            return false;
        }
        // A gauge that has not yet learned the pack's real capacity can report
        // a low SOC while its own voltage reading is nowhere near empty.
        // Distrust the SOC in that case and fall back to the voltage table.
        const float gaugeVoltage = readBatteryVoltage();
        const bool unlearned = gaugeVoltage >= 0.0f && gaugeVoltage > (cutoff + kHysteresisV);
        if (unlearned) {
            return false;
        }
        return true;
    }

    const float voltage = readBatteryVoltage();
    if (voltage < 0.0f) {
        return false;
    }
    return voltage <= cutoff;
}

bool batteryLowStateActive(void) {
    return s_latch.low;
}

void batteryLowPoll(void) {
    static uint32_t lastPollMs = 0;
    static bool havePolled = false;

    if (havePolled && (uint32_t)(millis() - lastPollMs) < kPollIntervalMs) {
        return;
    }
    lastPollMs = millis();
    havePolled = true;

    // isBatteryLow() already applies the recovery hysteresis, so a false
    // result here is trusted immediately. Debounce only guards against a
    // single sagged reading tripping the latch, not against recovery.
    const bool lowNow = isBatteryLow();
    if (!lowNow) {
        s_latch.lowStrikes = 0;
        return;
    }

    if (s_latch.low) {
        return;
    }

    s_latch.lowStrikes++;
    if (s_latch.lowStrikes < kDebounceStrikes) {
        od_log_debug("Battery low: strike %u/%u", s_latch.lowStrikes, kDebounceStrikes);
        return;
    }

    od_log_info("Battery low: tripped");
    s_latch.low = true;
    s_latch.screenDrawn = false; // forces the low-battery screen to draw once
    s_latch.lowStrikes = 0;
}
