#include "sensor_aht20.h"
#include "sensor_sht40.h"
#include "display_service.h"
#include "structs.h"
#include "od_log.h"

#include <Arduino.h>
#include <Wire.h>

extern struct GlobalConfig globalConfig;

static uint8_t aht20Crc(const uint8_t* data, uint8_t length) {
    uint8_t crc = 0xFF;
    for (uint8_t index = 0; index < length; index++) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static bool aht20Command(uint8_t address, const uint8_t* command, uint8_t length) {
    Wire.beginTransmission(address);
    Wire.write(command, length);
    return Wire.endTransmission() == 0;
}

static bool aht20Read(uint8_t address, int16_t* temperatureCenti, uint16_t* humidityCenti) {
    const uint8_t initialize[] = {0xBE, 0x08, 0x00};
    if (!aht20Command(address, initialize, sizeof(initialize))) return false;
    delay(10);

    const uint8_t trigger[] = {0xAC, 0x33, 0x00};
    for (uint8_t attempt = 0; attempt < 2; attempt++) {
        if (!aht20Command(address, trigger, sizeof(trigger))) return false;
        delay(85);
        if (Wire.requestFrom(address, (size_t)7, true) != 7) continue;

        uint8_t sample[7];
        for (uint8_t index = 0; index < sizeof(sample); index++) sample[index] = Wire.read();
        if ((sample[0] & 0x88) != 0x08 || aht20Crc(sample, 6) != sample[6]) continue;

        const uint32_t humidityRaw = ((uint32_t)sample[1] << 12) |
                                     ((uint32_t)sample[2] << 4) | (sample[3] >> 4);
        const uint32_t temperatureRaw = ((uint32_t)(sample[3] & 0x0F) << 16) |
                                        ((uint32_t)sample[4] << 8) | sample[5];
        *humidityCenti = (uint16_t)(((uint64_t)humidityRaw * 10000u + 524288u) / 1048576u);
        *temperatureCenti = (int16_t)(((uint64_t)temperatureRaw * 20000u + 524288u) / 1048576u - 5000);
        return true;
    }
    return false;
}

void pollAht20SensorsForMsd(void) {
    static bool sampledThisBoot = false;
    if (sampledThisBoot) return;

    for (uint8_t index = 0; index < globalConfig.sensor_count; index++) {
        const SensorData* sensor = &globalConfig.sensors[index];
        if (sensor->sensor_type != OD_SENSOR_TYPE_AHT20) continue;
        sampledThisBoot = true;

        uint8_t start = sensor->msd_data_start_byte;
        if (start == 0 || start == 0xFF) start = 7;
        if (start > 8) return;

#if defined(TARGET_ESP32) && defined(CONFIG_IDF_TARGET_ESP32S3)
        const bool gated = (globalConfig.system_config.device_flags & OD_DEVICE_FLAG_NM420_INIT) != 0;
        if (gated) {
            digitalWrite(40, HIGH);
            delay(100);
        }
#else
        const bool gated = false;
#endif
        int16_t temperatureCenti = 0;
        uint16_t humidityCenti = 0;
        uint8_t busId = sensor->bus_id == 0xFF ? 0 : sensor->bus_id;
        const uint8_t address = (sensor->i2c_addr_7bit == 0 || sensor->i2c_addr_7bit == 0xFF)
                                    ? 0x38 : sensor->i2c_addr_7bit;
        bool valid = initOrRestoreWireForBus(busId) &&
                     aht20Read(address, &temperatureCenti, &humidityCenti);
        if (gated) {
            if (globalConfig.sensor_count == 1 && globalConfig.touch_controller_count == 0) {
                invalidateOpenDisplayWire();
            }
            digitalWrite(40, LOW);
        }
        if (valid) {
            writeTempHumidityMsd(start, temperatureCenti, humidityCenti);
            od_log_info("AHT20: wake-time sample %.2f C, %.2f %%RH (bus %u, addr 0x%02X)",
                        temperatureCenti / 100.0f, humidityCenti / 100.0f, busId, address);
        } else {
            writeTempHumidityMsdInvalid(start);
            od_log_warn("AHT20: wake-time sample failed (bus %u, addr 0x%02X)", busId, address);
        }
        return;
    }
}