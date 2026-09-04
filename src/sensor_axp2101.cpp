#include "sensor_axp2101.h"

#include "structs.h"
#include "display_service.h"
#include "od_log.h"

#include <Arduino.h>
#include <Wire.h>

static const SensorData* axp2101_config(const SensorData* sensors, uint8_t count) {
    if (sensors == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < count; i++) {
        if (sensors[i].sensor_type == OD_SENSOR_TYPE_AXP2101) {
            return &sensors[i];
        }
    }
    return nullptr;
}

static uint8_t axp2101_bus_id(const SensorData* s) {
    uint8_t bid = s->bus_id;
    if (bid == 0xFF) {
        bid = 0;
    }
    return bid;
}

// Register block read: write `reg`, then requestFrom `len` bytes into the
// caller-supplied `out`. Returns false on any transaction failure or short read.
// Allocates nothing. Mirrors bq27220_read_block() in sensor_bq27220.cpp.
static bool axp2101_read_block(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(addr, (size_t)len, true) != len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        out[i] = Wire.read();
    }
    return true;
}

static bool axp2101_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

float axp2101BatteryVoltageVolts(const SensorData* sensors, uint8_t sensor_count) {
    const SensorData* s = axp2101_config(sensors, sensor_count);
    if (s == nullptr) {
        return -1.0f;
    }
    const uint8_t bus = axp2101_bus_id(s);
    if (!initOrRestoreWireForBus(bus)) {
        od_log_warn("AXP2101: bus %u init failed", bus);
        return -1.0f;
    }
    const uint8_t addr = axp2101_resolve_addr(s->i2c_addr_7bit);
    od_log_debug("AXP2101: addr=0x%02X bus=%u", addr, bus);

    uint8_t status = 0;
    if (!axp2101_read_block(addr, AXP2101_REG_POWER_STATUS_ADDR, &status, 1)) {
        od_log_warn("AXP2101: power-status read failed @0x%02X", addr);
        return -1.0f;
    }
    const bool batt_present = axp2101_batt_present(status);
    const bool vbus_present = axp2101_vbus_present(status);
    od_log_debug("AXP2101: power_status=0x%02X batt=%d vbus=%d",
                 status, (int)batt_present, (int)vbus_present);
    if (!batt_present) {
        od_log_debug("AXP2101: battery not present, returning -1");
        return -1.0f;
    }

    uint8_t adc_ctrl = 0;
    if (!axp2101_read_block(addr, AXP2101_REG_ADC_CHANNEL_CTRL_ADDR, &adc_ctrl, 1)) {
        od_log_warn("AXP2101: ADC ctrl read failed @0x%02X", addr);
        return -1.0f;
    }
    bool channel_changed = false;
    const uint8_t adc_ctrl_new = axp2101_adc_enable_bit0(adc_ctrl, &channel_changed);
    od_log_debug("AXP2101: adc_ctrl=0x%02X -> 0x%02X (changed=%d)",
                 adc_ctrl, adc_ctrl_new, (int)channel_changed);
    if (channel_changed) {
        if (!axp2101_write_reg(addr, AXP2101_REG_ADC_CHANNEL_CTRL_ADDR, adc_ctrl_new)) {
            od_log_warn("AXP2101: ADC ctrl enable failed @0x%02X", addr);
            return -1.0f;
        }
        delay(AXP2101_ADC_SETTLING_MS);
    }

    uint8_t vbat_raw[2] = {0, 0};
    if (!axp2101_read_block(addr, AXP2101_REG_VBAT_H_ADDR, vbat_raw, 2)) {
        od_log_warn("AXP2101: VBAT read failed @0x%02X", addr);
        return -1.0f;
    }
    const uint16_t mv = axp2101_decode_vbat_mv(vbat_raw[0], vbat_raw[1]);
    od_log_debug("AXP2101: VBAT raw=[0x%02X,0x%02X] -> %u mV (%.3fV)",
                 vbat_raw[0], vbat_raw[1], (unsigned)mv, (float)mv / 1000.0f);
    return (float)mv / 1000.0f;
}
