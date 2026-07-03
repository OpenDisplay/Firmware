#include "sensor_bq27220.h"
#include "structs.h"
#include "display_service.h"

#include <Arduino.h>
#include <Wire.h>

extern struct GlobalConfig globalConfig;
extern uint8_t dynamicreturndata[11];
void writeSerial(String message, bool newLine = true);

static_assert(sizeof(SensorData) == 30, "SensorData must remain 30 bytes");

#ifndef CHARGER_FLAG_ENABLE_ACTIVE_LOW
#define CHARGER_FLAG_ENABLE_ACTIVE_LOW (1u << 0)
#endif
#ifndef CHARGER_FLAG_STATE_ACTIVE_LOW
#define CHARGER_FLAG_STATE_ACTIVE_LOW (1u << 1)
#endif

#define BQ27220_CMD_VOLTAGE 0x08u
#define BQ27220_CMD_SOC 0x2Cu

#define BQ27220_MSD_CHARGING_BIT 0x80u

static float s_batt_v = -1.0f;
static uint8_t s_soc = 0xFF;
static bool s_gauge_ok = false;

static bool validPin(uint8_t pin) { return pin != 0 && pin != 0xFF; }

static uint8_t bq27220_addr_7bit(const SensorData* s) {
    uint8_t a = s->i2c_addr_7bit;
    if (a == 0 || a == 0xFF) {
        return 0x55;
    }
    return a;
}

static uint8_t bq27220_bus_id(const SensorData* s) {
    uint8_t bid = s->bus_id;
    if (bid == 0xFF) {
        bid = 0;
    }
    return bid;
}

static bool bq27220_ensure_bus(const SensorData* s) {
    return initOrRestoreWireForBus(bq27220_bus_id(s));
}

static bool bq27220_read_block(const SensorData* s, uint8_t cmd, uint8_t* buf, uint8_t len) {
    uint8_t addr = bq27220_addr_7bit(s);
    if (!bq27220_ensure_bus(s)) {
        return false;
    }
    Wire.beginTransmission(addr);
    Wire.write(cmd);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    int n = Wire.requestFrom(addr, (size_t)len, true);
    if (n != (int)len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

static const SensorData* bq27220_config(void) {
    for (uint8_t i = 0; i < globalConfig.sensor_count; i++) {
        if (globalConfig.sensors[i].sensor_type == SENSOR_TYPE_BQ27220) {
            return &globalConfig.sensors[i];
        }
    }
    return nullptr;
}

static uint8_t bq27220_msd_start(const SensorData* s) {
    return s->msd_data_start_byte;
}

static void write_dynamic_byte(uint8_t idx, uint8_t value) {
    if (idx <= 10) {
        dynamicreturndata[idx] = value;
    }
}

bool bq27220IsConfigured(void) {
    return bq27220_config() != nullptr;
}

float bq27220BatteryVoltageVolts(void) {
    return s_gauge_ok ? s_batt_v : -1.0f;
}

void initChargerGpio(void) {
    const uint8_t flags = globalConfig.power_option.charger_flags;
    const uint8_t en = globalConfig.power_option.charge_enable_pin;
    if (validPin(en)) {
        pinMode(en, OUTPUT);
        const bool activeLow = (flags & CHARGER_FLAG_ENABLE_ACTIVE_LOW) != 0;
        digitalWrite(en, activeLow ? LOW : HIGH);
    }
    const uint8_t st = globalConfig.power_option.charge_state_pin;
    if (validPin(st)) {
        pinMode(st, INPUT_PULLUP);
    }
}

static bool charger_gpio_charging(void) {
    const uint8_t st = globalConfig.power_option.charge_state_pin;
    if (!validPin(st)) {
        return false;
    }
    const bool activeLow = (globalConfig.power_option.charger_flags & CHARGER_FLAG_STATE_ACTIVE_LOW) != 0;
    const int level = digitalRead(st);
    return activeLow ? (level == HIGH) : (level == LOW);
}

void initBq27220Sensors(void) {
    initChargerGpio();
    const SensorData* s = bq27220_config();
    if (!s) {
        return;
    }
    if (!bq27220_ensure_bus(s)) {
        writeSerial("BQ27220: bus init failed", true);
        return;
    }
    uint8_t raw[2];
    if (!bq27220_read_block(s, BQ27220_CMD_VOLTAGE, raw, 2)) {
        writeSerial("BQ27220: not found @0x" + String(bq27220_addr_7bit(s), HEX), true);
        return;
    }
    writeSerial("BQ27220: fuel gauge @0x" + String(bq27220_addr_7bit(s), HEX), true);
}

static constexpr uint32_t kBq27220MsdPollTtlMs = 30000u;

void pollBq27220ForMsd(void) {
    const SensorData* s = bq27220_config();
    if (!s) {
        return;
    }
    static uint32_t lastPollMs = 0;
    static bool havePolled = false;
    if (havePolled && (uint32_t)(millis() - lastPollMs) < kBq27220MsdPollTtlMs) {
        return;
    }
    lastPollMs = millis();
    havePolled = true;
    uint8_t raw[2];
    if (!bq27220_read_block(s, BQ27220_CMD_VOLTAGE, raw, 2)) {
        s_gauge_ok = false;
        s_batt_v = -1.0f;
        s_soc = 0xFF;
        return;
    }
    uint16_t mv = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    s_batt_v = mv / 1000.0f;
    s_gauge_ok = mv > 0;

    uint8_t soc = 0xFF;
    if (bq27220_read_block(s, BQ27220_CMD_SOC, &soc, 1)) {
        if (soc > 100) {
            soc = 100;
        }
        s_soc = soc;
    } else {
        s_soc = 0xFF;
    }

    const bool charging = charger_gpio_charging();

    const uint8_t msdIdx = bq27220_msd_start(s);
    if (msdIdx <= 10) {
        if (!s_gauge_ok || s_soc > 100) {
            write_dynamic_byte(msdIdx, 0xFF);
        } else {
            uint8_t packed = (uint8_t)(s_soc & 0x7Fu);
            if (charging) {
                packed |= BQ27220_MSD_CHARGING_BIT;
            }
            write_dynamic_byte(msdIdx, packed);
        }
    }
}
