// AXP2101 battery-voltage reader for the normal OpenDisplay battery path.
//
// Sits alongside sensor_bq27220 in shape: the sensor-specific configuration
// lookup, address defaulting, bus restoration, and voltage getter all live
// here. display_service.cpp's readBatteryVoltageUncached() consults this file
// when the configured sensor set contains an AXP2101 (sensor type 3), which is
// the case for the Waveshare ESP32-S3-PhotoPainter esp32-s3-wspp preset.
//
// The register decoding and bit manipulation are kept as pure `static inline`
// helpers here so tools/test_sensor_axp2101.cpp can exercise them without a
// PlatformIO build. See tools/README.md.
#ifndef SENSOR_AXP2101_H
#define SENSOR_AXP2101_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// AXP2101 register layout used by the battery-voltage path. display_service.cpp
// carries its own AXP2101_* constants for the PMIC init and shutdown block, and
// four values appear in both places: the 0x34 slave address, 0x00 power status,
// 0x30 ADC channel control, and 0x34 VBAT high. The duplication is deliberate --
// those belong to that block, and the pure helpers below must compile as a
// standalone unit for the host test.
#define AXP2101_DEFAULT_ADDR_7BIT             0x34u
#define AXP2101_REG_POWER_STATUS_ADDR         0x00u
#define AXP2101_REG_ADC_CHANNEL_CTRL_ADDR     0x30u
#define AXP2101_REG_VBAT_H_ADDR               0x34u  // VBAT high byte; read as 2-byte block with 0x35

// Datasheet reg 0x00: bit 3 = battery presence, bit 5 = VBUS presence.
// The uncalled readAXP2101Data() diagnostic in display_service.cpp reads these
// the other way round (0x20 for battery, 0x08 for VBUS). That is a pre-existing
// defect in unreachable code, flagged in the pull request and left unchanged.
#define AXP2101_POWER_STATUS_BATT_PRESENT_BIT (1u << 3)
#define AXP2101_POWER_STATUS_VBUS_PRESENT_BIT (1u << 5)

// Datasheet reg 0x30: bit 0 enables the VBAT-voltage ADC channel. Every other
// bit is a different channel or an unrelated ADC control and must be preserved.
#define AXP2101_ADC_ENABLE_VBAT_BIT           (1u << 0)

// Post-enable settling window before the first VBAT read is meaningful. Only
// applied when the caller has just flipped bit 0 from 0 to 1.
#define AXP2101_ADC_SETTLING_MS               2u

// Address defaulting: `0` and `0xFF` (SensorData.i2c_addr_7bit sentinel values,
// see include/opendisplay_structs.h) both mean "use the AXP2101 default 0x34".
// Any other value is respected as the configured 7-bit address.
static inline uint8_t axp2101_resolve_addr(uint8_t configured) {
    if (configured == 0u || configured == 0xFFu) {
        return AXP2101_DEFAULT_ADDR_7BIT;
    }
    return configured;
}

// Battery-present interpretation of the power-status register.
static inline bool axp2101_batt_present(uint8_t status_reg) {
    return (status_reg & AXP2101_POWER_STATUS_BATT_PRESENT_BIT) != 0u;
}

// VBUS-present interpretation of the power-status register. Reported in the
// battery reader's debug line; it does not affect the voltage returned.
static inline bool axp2101_vbus_present(uint8_t status_reg) {
    return (status_reg & AXP2101_POWER_STATUS_VBUS_PRESENT_BIT) != 0u;
}

// Returns the value to write back to reg 0x30 so bit 0 is set, with every other
// bit preserved. `out_channel_changed` receives `true` iff the caller has just
// flipped the channel from disabled to enabled -- the only case where the
// caller must allow AXP2101_ADC_SETTLING_MS before the first VBAT sample.
static inline uint8_t axp2101_adc_enable_bit0(uint8_t current, bool* out_channel_changed) {
    const bool was_off = (current & AXP2101_ADC_ENABLE_VBAT_BIT) == 0u;
    if (out_channel_changed != NULL) {
        *out_channel_changed = was_off;
    }
    return (uint8_t)(current | AXP2101_ADC_ENABLE_VBAT_BIT);
}

// VBAT decoding. Reg 0x34 holds the six valid high bits (bits 7:6 are unused
// and must be masked), reg 0x35 holds the full low byte. Result is millivolts
// at 1 mV per count.
static inline uint16_t axp2101_decode_vbat_mv(uint8_t hi, uint8_t lo) {
    const uint16_t high_bits = (uint16_t)(hi & 0x3Fu);
    return (uint16_t)((high_bits << 8) | (uint16_t)lo);
}

struct SensorData;

#ifdef __cplusplus
extern "C" {
#endif

// Reads the current battery voltage in volts through the OpenDisplay bus
// abstraction. The caller passes the parsed sensor set rather than this module
// reaching for a global (see the no-extern rule in CLAUDE.md); the AXP2101 entry
// is located here so the lookup stays with the sensor-specific code, matching
// sensor_bq27220.
//
// Returns `-1.0f` on any failure -- no AXP2101 in `sensors`, bus init failed,
// PMIC not answering, short read, or battery absent per reg 0x00 -- so the
// caller can fall through to its next battery source. Never allocates. Emits an
// od_log_warn on transaction failure.
float axp2101BatteryVoltageVolts(const struct SensorData* sensors, uint8_t sensor_count);

#ifdef __cplusplus
}
#endif

#endif  // SENSOR_AXP2101_H
