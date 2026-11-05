#include "rtc_pcf8563.h"

#ifdef TARGET_ESP32
#include "display_service.h"
#include "power_latch.h"
#include "structs.h"
#include "od_log.h"

#include <Arduino.h>
#include <Wire.h>

extern struct GlobalConfig globalConfig;

// PCF8563 / BM8563 register map (the two are register-compatible).
#define RTC_DEFAULT_ADDR   0x51
#define RTC_REG_CTRL1      0x00  // bit5 STOP
#define RTC_REG_CTRL2      0x01  // bit3 AF, bit2 TF, bit1 AIE, bit0 TIE
#define RTC_REG_SECONDS    0x02  // bit7 VL
#define RTC_REG_ALARM_MIN  0x09  // 0x09..0x0C: minute/hour/day/weekday, bit7 AE = disabled
#define RTC_REG_TIMER_CTRL 0x0E  // bit7 TE, bits1:0 TD
#define RTC_REG_TIMER      0x0F

#define RTC_CTRL2_FLAGS    0x0C
#define RTC_CTRL2_AIE      0x02
#define RTC_CTRL2_TIE      0x01
#define RTC_TIMER_ENABLE   0x80
#define RTC_TIMER_1HZ      0x02
#define RTC_TIMER_1_60HZ   0x03  // also the lowest-power setting while disabled
#define RTC_ALARM_OFF      0x80

static const SensorData* rtcSensor(void) {
    for (uint8_t i = 0; i < globalConfig.sensor_count; i++) {
        if (globalConfig.sensors[i].sensor_type == OD_SENSOR_TYPE_PCF8563) {
            return &globalConfig.sensors[i];
        }
    }
    return nullptr;
}

// Selects the sensor's bus and returns its address, or 0 on failure.
static uint8_t rtcBegin(void) {
    const SensorData* s = rtcSensor();
    if (!s) return 0;
    const uint8_t busId = (s->bus_id == 0xFF) ? 0 : s->bus_id;
    if (!initOrRestoreWireForBus(busId)) return 0;
    return (s->i2c_addr_7bit == 0 || s->i2c_addr_7bit == 0xFF) ? RTC_DEFAULT_ADDR : s->i2c_addr_7bit;
}

static bool rtcWrite(uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data, len);
    return Wire.endTransmission() == 0;
}

static bool rtcWrite1(uint8_t addr, uint8_t reg, uint8_t value) {
    return rtcWrite(addr, reg, &value, 1);
}

static bool rtcRead(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, (size_t)len, true) != len) return false;
    for (uint8_t i = 0; i < len; i++) data[i] = Wire.read();
    return true;
}

static uint8_t bcdToBin(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static uint8_t binToBcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

// Interrupts off, flags cleared, timer stopped, every alarm field disabled.
static bool rtcQuiesce(uint8_t addr) {
    static const uint8_t alarmsOff[4] = {RTC_ALARM_OFF, RTC_ALARM_OFF, RTC_ALARM_OFF, RTC_ALARM_OFF};
    return rtcWrite1(addr, RTC_REG_CTRL2, 0x00) &&
           rtcWrite1(addr, RTC_REG_TIMER_CTRL, RTC_TIMER_1_60HZ) &&
           rtcWrite(addr, RTC_REG_ALARM_MIN, alarmsOff, sizeof(alarmsOff));
}

bool rtcWakeConfigured(void) {
    return rtcSensor() != nullptr && powerLatchMosfetConfigured();
}

bool rtcConsumeWake(void) {
    if (!rtcWakeConfigured()) return false;
    const uint8_t addr = rtcBegin();
    uint8_t ctrl2 = 0;
    if (addr == 0 || !rtcRead(addr, RTC_REG_CTRL2, &ctrl2, 1)) {
        od_log_warn("RTC: not reachable at boot");
        return false;
    }
    // INT is level-mode (TI_TP = 0): it holds the board on until this clears.
    if (!rtcQuiesce(addr)) od_log_warn("RTC: failed to clear wake flags");
    return (ctrl2 & RTC_CTRL2_FLAGS) != 0;
}

bool rtcArmWake(uint32_t seconds) {
    if (seconds == 0 || !rtcWakeConfigured()) return false;
    const uint8_t addr = rtcBegin();
    // Clock running (STOP = 0) or neither the countdown nor the alarm advances.
    if (addr == 0 || !rtcWrite1(addr, RTC_REG_CTRL1, 0x00) || !rtcQuiesce(addr)) {
        od_log_warn("RTC: arm failed (I2C)");
        return false;
    }
    if (seconds <= 255) {
        return rtcWrite1(addr, RTC_REG_TIMER, (uint8_t)seconds) &&
               rtcWrite1(addr, RTC_REG_TIMER_CTRL, RTC_TIMER_ENABLE | RTC_TIMER_1HZ) &&
               rtcWrite1(addr, RTC_REG_CTRL2, RTC_CTRL2_TIE);
    }
    const uint32_t minutes = (seconds + 59) / 60;
    if (minutes <= 255) {
        return rtcWrite1(addr, RTC_REG_TIMER, (uint8_t)minutes) &&
               rtcWrite1(addr, RTC_REG_TIMER_CTRL, RTC_TIMER_ENABLE | RTC_TIMER_1_60HZ) &&
               rtcWrite1(addr, RTC_REG_CTRL2, RTC_CTRL2_TIE);
    }
    // Beyond 255 min (deep_sleep_time_seconds tops out near 18 h): minute+hour
    // alarm, which matches once within the next 24 h.
    uint8_t now[3];
    if (!rtcRead(addr, RTC_REG_SECONDS, now, sizeof(now))) return false;
    const uint32_t nowMin = (uint32_t)bcdToBin(now[2] & 0x3F) * 60u + bcdToBin(now[1] & 0x7F);
    const uint32_t target = (nowMin + minutes) % (24u * 60u);
    const uint8_t alarm[4] = {binToBcd((uint8_t)(target % 60)), binToBcd((uint8_t)(target / 60)),
                              RTC_ALARM_OFF, RTC_ALARM_OFF};
    return rtcWrite(addr, RTC_REG_ALARM_MIN, alarm, sizeof(alarm)) &&
           rtcWrite1(addr, RTC_REG_CTRL2, RTC_CTRL2_AIE);
}

#endif // TARGET_ESP32
