// Host test for src/sensor_axp2101.h -- the AXP2101 battery-voltage helpers.
//
// Build and run from the repo root:
//
//   g++ -std=c++17 -Wall -Wextra -Werror -O1 -fsanitize=undefined,address -I include -I src tools/test_sensor_axp2101.cpp -o /tmp/test_sensor_axp2101
//   /tmp/test_sensor_axp2101
//
// Like tools/test_link_owner.cpp, this is as much a written-down statement of
// the intended semantics as it is a test. It covers:
//
//   - VBAT decoding with upper unused bits clear and set
//   - ADC bit-0 enable while preserving every other bit
//   - Battery-present status set and clear
//   - Default address selection for zero and 0xFF
//   - Configured address preservation
//   - No configured AXP2101 (walked via a SensorData-shaped array)
//   - Short or failed I2C reads (walked via a fake bus driving the same helpers)
//
// The pure helpers under test have no Arduino/Wire dependency, so no hostshim
// is required. The fake-bus scenarios call the SAME helpers the production
// reader calls, but they re-implement its call sequence rather than driving
// axp2101BatteryVoltageVolts() itself. That ordering and the Wire I/O are what
// on-device testing covers, not this file.

#include "sensor_axp2101.h"
#include "opendisplay_structs.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

// --- tiny harness ----------------------------------------------------------
static int g_checks = 0;
static int g_failures = 0;

static void check(bool cond, const char* what) {
    g_checks++;
    if (!cond) {
        g_failures++;
        std::printf("FAIL: %s\n", what);
    }
}

extern "C" void __ubsan_on_report(void) {
    std::printf("FAIL: UBSan report\n");
    std::exit(1);
}

// Fake bus for the "short or failed I2C reads" case. Records writes so the test
// can verify targeted ADC-enable semantics, and lets each register return either
// a canned byte or an error.
struct FakeBus {
    // Canned responses for the three registers the reader touches. If .ok is
    // false, the read fails (mirrors an I2C NAK or a short read).
    struct Reply {
        bool ok;
        uint8_t value;
    };
    Reply power_status{true, 0x08};                // battery present by default
    Reply adc_ctrl{true, 0x00};                    // channel currently disabled
    Reply vbat_hi{true, 0x0F};                     // 0x0FB8 -> 4024 mV
    Reply vbat_lo{true, 0xB8};

    // Recorded writes.
    bool adc_write_seen = false;
    uint8_t adc_write_value = 0xFF;
};

// Executes the same production sequence readBatteryVoltageUncached's AXP2101
// arm does, but against FakeBus. Returns millivolts on success, -1 on any
// failure -- mirroring axp2101BatteryVoltageVolts()'s -1.0f return.
static int fake_read_vbat_mv(FakeBus& bus, uint8_t addr) {
    (void)addr;
    if (!bus.power_status.ok) return -1;
    if (!axp2101_batt_present(bus.power_status.value)) return -1;
    if (!bus.adc_ctrl.ok) return -1;
    bool changed = false;
    const uint8_t adc_new = axp2101_adc_enable_bit0(bus.adc_ctrl.value, &changed);
    if (changed) {
        bus.adc_write_seen = true;
        bus.adc_write_value = adc_new;
        // (Production delays AXP2101_ADC_SETTLING_MS here; no-op in the test.)
    }
    if (!bus.vbat_hi.ok || !bus.vbat_lo.ok) return -1;
    return (int)axp2101_decode_vbat_mv(bus.vbat_hi.value, bus.vbat_lo.value);
}

int main(void) {
    // ---- VBAT decoding with upper unused bits clear -------------------------
    {
        // 0x0F B8 -- the ESPHome-observed sample -- must decode to 4024 mV.
        check(axp2101_decode_vbat_mv(0x0F, 0xB8) == 4024, "vbat decode 0x0FB8 == 4024 mV");
        check(axp2101_decode_vbat_mv(0x00, 0x00) == 0, "vbat decode 0x0000 == 0 mV");
        check(axp2101_decode_vbat_mv(0x00, 0xFF) == 255, "vbat decode 0x00FF == 255 mV");
        check(axp2101_decode_vbat_mv(0x01, 0x00) == 256, "vbat decode 0x0100 == 256 mV");
        check(axp2101_decode_vbat_mv(0x3F, 0xFF) == 16383, "vbat decode 0x3FFF == 16383 mV");
    }

    // ---- VBAT decoding with upper unused bits SET ---------------------------
    {
        // Bits 7:6 of reg 0x34 are unused; the decoder must mask them off so
        // spurious garbage in that field cannot inflate the reading.
        check(axp2101_decode_vbat_mv(0xC0, 0x00) == 0,
              "vbat unused bits 0xC0 masked -> 0 mV");
        check(axp2101_decode_vbat_mv(0xCF, 0xB8) == 4024,
              "vbat 0xCFB8 (unused bits set) decodes same as 0x0FB8");
        check(axp2101_decode_vbat_mv(0xFF, 0xFF) == 16383,
              "vbat 0xFFFF decodes as 0x3FFF -> 16383 mV");
    }

    // ---- ADC bit-0 enable while preserving every other bit ------------------
    {
        // From all-zero: bit 0 flips 0->1, no other bit changes.
        bool changed = false;
        uint8_t next = axp2101_adc_enable_bit0(0x00, &changed);
        check(next == 0x01, "adc enable from 0x00 -> 0x01");
        check(changed, "adc enable from 0x00 flags channel change");

        // From every other bit set: bit 0 flips 0->1, all other bits preserved.
        changed = false;
        next = axp2101_adc_enable_bit0(0xFE, &changed);
        check(next == 0xFF, "adc enable from 0xFE preserves 7..1, sets bit 0 -> 0xFF");
        check(changed, "adc enable from 0xFE flags channel change");

        // Already enabled (0x01): value unchanged, no channel-change flag.
        changed = true;
        next = axp2101_adc_enable_bit0(0x01, &changed);
        check(next == 0x01, "adc enable from 0x01 stays 0x01");
        check(!changed, "adc enable from 0x01 does not flag channel change");

        // Already enabled with siblings set: value unchanged, no flag.
        changed = true;
        next = axp2101_adc_enable_bit0(0xA5, &changed);
        check(next == 0xA5, "adc enable from 0xA5 stays 0xA5 (bit 0 already set)");
        check(!changed, "adc enable from 0xA5 does not flag channel change");

        // NULL out-pointer must be tolerated.
        next = axp2101_adc_enable_bit0(0x00, nullptr);
        check(next == 0x01, "adc enable with null out-pointer still returns 0x01");
    }

    // ---- Battery-present status set and clear -------------------------------
    {
        check(!axp2101_batt_present(0x00), "batt present false when reg 0x00 is zero");
        check(axp2101_batt_present(0x08), "batt present true when bit 3 set");
        check(!axp2101_batt_present(0xF7), "batt present false when only bit 3 clear");
        check(axp2101_batt_present(0xFF), "batt present true when every bit set");
        // Bit 3 is battery, bit 5 is VBUS -- they are DISTINCT and must not
        // shadow each other (this is exactly what mainline had swapped).
        check(!axp2101_batt_present(0x20), "0x20 alone (VBUS bit) does NOT imply battery present");
        check(axp2101_vbus_present(0x20), "0x20 alone is VBUS present");
        check(!axp2101_vbus_present(0x08), "0x08 alone (batt bit) does NOT imply VBUS present");
    }

    // ---- Default address selection for zero and 0xFF ------------------------
    {
        check(axp2101_resolve_addr(0x00) == 0x34, "configured 0x00 -> default 0x34");
        check(axp2101_resolve_addr(0xFF) == 0x34, "configured 0xFF -> default 0x34");
    }

    // ---- Configured address preservation ------------------------------------
    {
        // A valid 7-bit address must be respected verbatim -- no defaulting.
        check(axp2101_resolve_addr(0x34) == 0x34, "configured 0x34 respected");
        check(axp2101_resolve_addr(0x35) == 0x35, "configured 0x35 respected");
        check(axp2101_resolve_addr(0x01) == 0x01, "configured 0x01 respected (edge)");
        check(axp2101_resolve_addr(0x7F) == 0x7F, "configured 0x7F respected (top of 7-bit)");
    }

    // ---- No configured AXP2101 ---------------------------------------------
    // The reader must not consult AXP2101 when the parsed sensor set has no
    // entry of type OD_SENSOR_TYPE_AXP2101. This mirrors the walk that
    // axp2101_config() performs in sensor_axp2101.cpp against globalConfig.
    {
        auto find_axp = [](const SensorData* sensors, uint8_t count) -> const SensorData* {
            for (uint8_t i = 0; i < count; i++) {
                if (sensors[i].sensor_type == OD_SENSOR_TYPE_AXP2101) return &sensors[i];
            }
            return nullptr;
        };

        SensorData empty{};
        check(find_axp(&empty, 0) == nullptr, "no sensors -> no AXP2101 found");

        SensorData sht{};
        sht.sensor_type = OD_SENSOR_TYPE_SHT40;
        check(find_axp(&sht, 1) == nullptr, "SHT40 only -> no AXP2101 found");

        SensorData bq{};
        bq.sensor_type = OD_SENSOR_TYPE_BQ27220;
        check(find_axp(&bq, 1) == nullptr, "BQ27220 only -> no AXP2101 found");

        SensorData mixed[3]{};
        mixed[0].sensor_type = OD_SENSOR_TYPE_SHT40;
        mixed[1].sensor_type = OD_SENSOR_TYPE_AXP2101;
        mixed[1].bus_id = 0;
        mixed[1].i2c_addr_7bit = 0xFF;
        mixed[2].sensor_type = OD_SENSOR_TYPE_BQ27220;
        const SensorData* hit = find_axp(mixed, 3);
        check(hit == &mixed[1], "mixed sensor set finds the AXP2101 entry");
        check(axp2101_resolve_addr(hit->i2c_addr_7bit) == 0x34,
              "PhotoPainter-style entry (addr 0xFF) resolves to default 0x34");
    }

    // ---- Short or failed I2C reads -----------------------------------------
    // Every failure mode returns -1 (matching axp2101BatteryVoltageVolts's
    // -1.0f) and does NOT touch the ADC-enable write when a preceding step
    // has already failed. Success case verifies the ADC-enable is issued
    // exactly once and only when the channel actually flipped.
    {
        // Success baseline: pre-shutdown ESPHome sample.
        FakeBus b;
        int mv = fake_read_vbat_mv(b, 0x34);
        check(mv == 4024, "success path decodes 4024 mV");
        check(b.adc_write_seen, "success path issued ADC-enable write");
        check(b.adc_write_value == 0x01, "success path wrote only bit 0 to reg 0x30");
    }
    {
        // Power-status read failure -> -1, no ADC-enable write.
        FakeBus b;
        b.power_status.ok = false;
        int mv = fake_read_vbat_mv(b, 0x34);
        check(mv == -1, "power-status read failure returns -1");
        check(!b.adc_write_seen, "power-status failure aborts before ADC-enable write");
    }
    {
        // Battery-present clear -> -1, no ADC-enable write.
        FakeBus b;
        b.power_status.value = 0x00;
        int mv = fake_read_vbat_mv(b, 0x34);
        check(mv == -1, "battery-absent returns -1");
        check(!b.adc_write_seen, "battery-absent aborts before ADC-enable write");
    }
    {
        // ADC ctrl read failure -> -1, no ADC-enable write.
        FakeBus b;
        b.adc_ctrl.ok = false;
        int mv = fake_read_vbat_mv(b, 0x34);
        check(mv == -1, "adc-ctrl read failure returns -1");
        check(!b.adc_write_seen, "adc-ctrl read failure aborts before ADC-enable write");
    }
    {
        // ADC channel already enabled: no write issued (targeted enable is a no-op).
        FakeBus b;
        b.adc_ctrl.value = 0x81;   // bit 0 already set, other bits present
        int mv = fake_read_vbat_mv(b, 0x34);
        check(mv == 4024, "already-enabled channel still decodes correctly");
        check(!b.adc_write_seen, "already-enabled channel does NOT re-write reg 0x30");
    }
    {
        // VBAT high-byte short read -> -1.
        FakeBus b;
        b.vbat_hi.ok = false;
        int mv = fake_read_vbat_mv(b, 0x34);
        check(mv == -1, "VBAT high-byte read failure returns -1");
        check(b.adc_write_seen, "VBAT read failure still records the earlier ADC-enable");
    }
    {
        // VBAT low-byte short read -> -1.
        FakeBus b;
        b.vbat_lo.ok = false;
        int mv = fake_read_vbat_mv(b, 0x34);
        check(mv == -1, "VBAT low-byte read failure returns -1");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
