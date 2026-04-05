#include "sensor_sht40.h"
#include "structs.h"
#include "display_service.h"

#include <Arduino.h>
#include <Wire.h>

extern struct GlobalConfig globalConfig;
extern uint8_t dynamicreturndata[11];

static_assert(sizeof(SensorData) == 30, "SensorData must remain 30 bytes");

static uint8_t sht40_crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; bit--) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31u);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint8_t sht40_addr_7bit(const SensorData* s) {
    uint8_t a = s->i2c_addr_7bit;
    if (a == 0 || a == 0xFF) {
        return 0x44;
    }
    return a;
}

static uint8_t sht40_msd_start(const SensorData* s) {
    uint8_t st = s->msd_data_start_byte;
    if (st == 0xFF || st == 0) {
        return 7;
    }
    return st;
}

static bool read_sht40_sample(uint8_t addr7, int16_t* temp_centi, uint16_t* rh_centi) {
    initOrRestoreWireForOpenDisplay();
    Wire.beginTransmission(addr7);
    Wire.write(0xFD);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        return false;
    }
    delay(10);
    int n = Wire.requestFrom((int)addr7, 6);
    if (n != 6) {
        return false;
    }
    uint8_t b[6];
    for (int i = 0; i < 6; i++) {
        b[i] = Wire.read();
    }
    if (sht40_crc8(b, 2) != b[2]) {
        return false;
    }
    if (sht40_crc8(b + 3, 2) != b[5]) {
        return false;
    }
    uint16_t rawT = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
    uint16_t rawRh = (uint16_t)(((uint16_t)b[3] << 8) | b[4]);
    float tc = -45.0f + 175.0f * ((float)rawT / 65535.0f);
    float rh = -6.0f + 125.0f * ((float)rawRh / 65535.0f);
    if (rh < 0.0f) {
        rh = 0.0f;
    }
    if (rh > 100.0f) {
        rh = 100.0f;
    }
    *temp_centi = (int16_t)(tc * 100.0f);
    *rh_centi = (uint16_t)(rh * 100.0f);
    return true;
}

static void write_sht40_invalid(uint8_t start) {
    dynamicreturndata[start] = 0xFF;
    dynamicreturndata[start + 1] = 0xFF;
    dynamicreturndata[start + 2] = 0xFF;
    if ((uint16_t)start + 3u < 11u) {
        dynamicreturndata[start + 3] = 0;
    }
}

static int round_centi_to_deci(int16_t c) {
    if (c >= 0) {
        return (int)((c + 5) / 10);
    }
    return (int)((c - 5) / 10);
}

// MSD (3 bytes LE): v = rh_deci | (tu << 10); rh_deci 0..1000 = 0..100.0% RH (0.1% steps);
// tu = temp(0.1°C) + 400. Decode: t_deci = (v>>10 & 0x7FF) - 400; temp_centi = t_deci*10; rh_centi = (v&0x3FF)*10
static void write_sht40_msd(uint8_t start, int16_t temp_centi, uint16_t rh_centi) {
    int t_deci = round_centi_to_deci(temp_centi);
    if (t_deci < -400) {
        t_deci = -400;
    }
    if (t_deci > 1250) {
        t_deci = 1250;
    }
    uint32_t tu = (uint32_t)(t_deci + 400);
    uint32_t rh_d = ((uint32_t)rh_centi + 5u) / 10u;
    if (rh_d > 1000u) {
        rh_d = 1000u;
    }
    uint32_t v = (rh_d & 0x3FFu) | (tu << 10);
    dynamicreturndata[start] = (uint8_t)(v & 0xFFu);
    dynamicreturndata[start + 1] = (uint8_t)((v >> 8) & 0xFFu);
    dynamicreturndata[start + 2] = (uint8_t)((v >> 16) & 0xFFu);
    if ((uint16_t)start + 3u < 11u) {
        dynamicreturndata[start + 3] = 0;
    }
}

void initSht40Sensors(void) {
    for (uint8_t i = 0; i < globalConfig.sensor_count; i++) {
        const SensorData* s = &globalConfig.sensors[i];
        if (s->sensor_type != SENSOR_TYPE_SHT40) {
            continue;
        }
        uint8_t addr = sht40_addr_7bit(s);
        initOrRestoreWireForOpenDisplay();
        Wire.beginTransmission(addr);
        Wire.write(0x94);
        (void)Wire.endTransmission();
        delay(2);
    }
}

void pollSht40SensorsForMsd(void) {
    for (uint8_t i = 0; i < globalConfig.sensor_count; i++) {
        const SensorData* s = &globalConfig.sensors[i];
        if (s->sensor_type != SENSOR_TYPE_SHT40) {
            continue;
        }
        uint8_t start = sht40_msd_start(s);
        if (start > 8) {
            continue;
        }
        int16_t tc = 0;
        uint16_t rhc = 0;
        if (!read_sht40_sample(sht40_addr_7bit(s), &tc, &rhc)) {
            write_sht40_invalid(start);
            continue;
        }
        write_sht40_msd(start, tc, rhc);
    }
}
