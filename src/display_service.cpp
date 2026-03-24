#include "display_service.h"

#include <Arduino.h>
#include <bb_epaper.h>
#include <string.h>
#include <Wire.h>
#include "structs.h"
#include "communication.h"
#include "encryption.h"
#include "boot_screen.h"
#include "uzlib.h"

#ifdef TARGET_NRF
extern "C" {
#include "nrf_soc.h"
}
#include <bluefruit.h>
#endif

#ifdef TARGET_ESP32
#include <BLEDevice.h>
#endif

extern BBEPDISP bbep;
extern struct GlobalConfig globalConfig;
extern uint8_t msd_payload[16];
extern uint8_t dynamicreturndata[11];
extern uint8_t rebootFlag;
extern uint8_t activeLedInstance;
extern bool connectionRequested;
extern uint8_t mloopcounter;
extern bool displayPowerState;
extern uint32_t directWriteStartTime;
extern uint8_t* directWriteCompressedBuffer;
extern uint32_t directWriteCompressedReceived;
extern uint32_t directWriteCompressedSize;
extern uint8_t directWriteRefreshMode;
extern uint32_t directWriteTotalBytes;
extern uint16_t directWriteHeight;
extern uint16_t directWriteWidth;
extern uint32_t directWriteDecompressedTotal;
extern uint32_t directWriteBytesWritten;
extern bool directWritePlane2;
extern bool directWriteBitplanes;
extern bool directWriteCompressed;
extern bool directWriteActive;
extern uint8_t compressedDataBuffer[];
extern uint8_t dictionaryBuffer[];
extern uint8_t decompressionChunk[];

#ifdef TARGET_ESP32
extern BLEAdvertisementData* advertisementData;
extern BLEServer* pServer;
extern BLEService* pService;
#endif

void pwrmgm(bool onoff);
String getChipIdHex();
void writeSerial(String message, bool newLine);
void bbepInitIO(BBEPDISP *pBBEP, uint8_t u8DC, uint8_t u8RST, uint8_t u8BUSY, uint8_t u8CS, uint8_t u8MOSI, uint8_t u8SCK, uint32_t u32Speed);
void bbepWakeUp(BBEPDISP *pBBEP);
void bbepSendCMDSequence(BBEPDISP *pBBEP, const uint8_t *pSeq);
void bbepRefresh(BBEPDISP *pBBEP, int iMode);
void bbepSleep(BBEPDISP *pBBEP, int iMode);
void bbepSetAddrWindow(BBEPDISP *pBBEP, int x, int y, int cx, int cy);
void bbepStartWrite(BBEPDISP *pBBEP, int iPlane);
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen);
bool bbepIsBusy(BBEPDISP *pBBEP);
void flashLed(uint8_t color, uint8_t brightness);
#define AXP2101_SLAVE_ADDRESS 0x34
#define AXP2101_REG_POWER_STATUS 0x00
#define AXP2101_REG_DC_ONOFF_DVM_CTRL 0x80
#define AXP2101_REG_LDO_ONOFF_CTRL0 0x90
#define AXP2101_REG_DC_VOL0_CTRL 0x82
#define AXP2101_REG_LDO_VOL2_CTRL 0x94
#define AXP2101_REG_LDO_VOL3_CTRL 0x95
#define AXP2101_REG_POWER_WAKEUP_CTL 0x26
#define AXP2101_REG_ADC_CHANNEL_CTRL 0x30
#define AXP2101_REG_ADC_DATA_BAT_VOL_H 0x34
#define AXP2101_REG_ADC_DATA_VBUS_VOL_H 0x36
#define AXP2101_REG_ADC_DATA_SYS_VOL_H 0x38
#define AXP2101_REG_BAT_PERCENT_DATA 0xA4
#define AXP2101_REG_PWRON_STATUS 0x20
#define AXP2101_REG_IRQ_ENABLE1 0x40
#define AXP2101_REG_IRQ_ENABLE2 0x41
#define AXP2101_REG_IRQ_ENABLE3 0x42
#define AXP2101_REG_IRQ_ENABLE4 0x43
#define AXP2101_REG_IRQ_STATUS1 0x44
#define AXP2101_REG_IRQ_STATUS2 0x45
#define AXP2101_REG_IRQ_STATUS3 0x46
#define AXP2101_REG_IRQ_STATUS4 0x47
#define AXP2101_REG_LDO_ONOFF_CTRL1 0x91
#define TRANSMISSION_MODE_CLEAR_ON_BOOT (1 << 7)
#define FONT_BASE_WIDTH 8
#define FONT_BASE_HEIGHT 8
#define FONT_SMALL_THRESHOLD 264

extern const uint8_t writelineFont[] PROGMEM;
extern uint8_t staticWhiteRow[680];
extern uint8_t staticRowBuffer[680];
extern char staticLineBuffer[256];

int bbepSetPanelType(BBEPDISP *pBBEP, int iPanel);
void bbepSetRotation(BBEPDISP *pBBEP, int iRotation);

int mapEpd(int id){
    switch(id) {
        case 0x0000: return EP_PANEL_UNDEFINED;
        case 0x0001: return EP42_400x300;
        case 0x0002: return EP42B_400x300;
        case 0x0003: return EP213_122x250;
        case 0x0004: return EP213B_122x250;
        case 0x0005: return EP293_128x296;
        case 0x0006: return EP294_128x296;
        case 0x0007: return EP295_128x296;
        case 0x0008: return EP295_128x296_4GRAY;
        case 0x0009: return EP266_152x296;
        case 0x000A: return EP102_80x128;
        case 0x000B: return EP27B_176x264;
        case 0x000C: return EP29R_128x296;
        case 0x000D: return EP122_192x176;
        case 0x000E: return EP154R_152x152;
        case 0x000F: return EP42R_400x300;
        case 0x0010: return EP42R2_400x300;
        case 0x0011: return EP37_240x416;
        case 0x0012: return EP37B_240x416;
        case 0x0013: return EP213_104x212;
        case 0x0014: return EP75_800x480;
        case 0x0015: return EP75_800x480_4GRAY;
        case 0x0016: return EP75_800x480_4GRAY_V2;
        case 0x0017: return EP29_128x296;
        case 0x0018: return EP29_128x296_4GRAY;
        case 0x0019: return EP213R_122x250;
        case 0x001A: return EP154_200x200;
        case 0x001B: return EP154B_200x200;
        case 0x001C: return EP266YR_184x360;
        case 0x001D: return EP29YR_128x296;
        case 0x001E: return EP29YR_168x384;
        case 0x001F: return EP583_648x480;
        case 0x0020: return EP296_128x296;
        case 0x0021: return EP26R_152x296;
        case 0x0022: return EP73_800x480;
        case 0x0023: return EP73_SPECTRA_800x480;
        case 0x0024: return EP74R_640x384;
        case 0x0025: return EP583R_600x448;
        case 0x0026: return EP75R_800x480;
        case 0x0027: return EP426_800x480;
        case 0x0028: return EP426_800x480_4GRAY;
        case 0x0029: return EP29R2_128x296;
        case 0x002A: return EP41_640x400;
        case 0x002B: return EP81_SPECTRA_1024x576;
        case 0x002C: return EP7_960x640;
        case 0x002D: return EP213R2_122x250;
        case 0x002E: return EP29Z_128x296;
        case 0x002F: return EP29Z_128x296_4GRAY;
        case 0x0030: return EP213Z_122x250;
        case 0x0031: return EP213Z_122x250_4GRAY;
        case 0x0032: return EP154Z_152x152;
        case 0x0033: return EP579_792x272;
        case 0x0034: return EP213YR_122x250;
        case 0x0035: return EP37YR_240x416;
        case 0x0036: return EP35YR_184x384;
        case 0x0037: return EP397YR_800x480;
        case 0x0038: return EP154YR_200x200;
        case 0x0039: return EP266YR2_184x360;
        case 0x003A: return EP42YR_400x300;
        case 0x003B: return EP75_800x480_GEN2;
        case 0x003C: return EP75_800x480_4GRAY_GEN2;
        case 0x003D: return EP215YR_160x296;
        case 0x003E: return EP1085_1360x480;
        case 0x003F: return EP31_240x320;
        case 0x0040: return EP75YR_800x480;
        default: return EP_PANEL_UNDEFINED;
    }
}

bool waitforrefresh(int timeout){
    for (size_t i = 0; i < (size_t)(timeout * 10); i++){
        delay(100);
        if(i % 5 == 0) writeSerial(".", false);
        if(!bbepIsBusy(&bbep)){
            if(i == 0){
                writeSerial("ERROR: Epaper not busy after refresh command - refresh may not have started", true);
                return false;
            }
            writeSerial(".", true);
            writeSerial("Refresh took ", false);
            writeSerial((String)((float)i / 10), false);
            writeSerial(" seconds", true);
            delay(200);
            return true;
        }
    }
    writeSerial("Refresh timed out", true);
    return false;
}

void initDataBuses(){
    writeSerial("=== Initializing Data Buses ===", true);
    if(globalConfig.data_bus_count == 0){
        writeSerial("No data buses configured", true);
        return;
    }
    for(uint8_t i = 0; i < globalConfig.data_bus_count; i++){
        struct DataBus* bus = &globalConfig.data_buses[i];
        if(bus->bus_type == 0x01){ // I2C bus
            writeSerial("Initializing I2C bus " + String(i) + " (instance " + String(bus->instance_number) + ")", true);
            if(bus->pin_1 == 0xFF || bus->pin_2 == 0xFF){
                writeSerial("ERROR: Invalid I2C pins for bus " + String(i) + " (SCL=" + String(bus->pin_1) + ", SDA=" + String(bus->pin_2) + ")", true);
                continue;
            }
            uint32_t busSpeed = (bus->bus_speed_hz > 0) ? bus->bus_speed_hz : 100000;
            #ifdef TARGET_ESP32
            pinMode(bus->pin_1, INPUT);
            pinMode(bus->pin_2, INPUT);
            if(bus->pullups & 0x01){
                pinMode(bus->pin_1, INPUT_PULLUP);
            }
            if(bus->pullups & 0x02){
                pinMode(bus->pin_2, INPUT_PULLUP);
            }
            if(bus->pulldowns & 0x01){
                pinMode(bus->pin_1, INPUT_PULLDOWN);
            }
            if(bus->pulldowns & 0x02){
                pinMode(bus->pin_2, INPUT_PULLDOWN);
            }
            #endif
            #ifdef TARGET_NRF
            pinMode(bus->pin_1, INPUT);
            pinMode(bus->pin_2, INPUT);
            if(bus->pullups & 0x01){
                pinMode(bus->pin_1, INPUT_PULLUP);
            }
            if(bus->pullups & 0x02){
                pinMode(bus->pin_2, INPUT_PULLUP);
            }
            #endif
            if(i == 0){
                #ifdef TARGET_ESP32
                Wire.begin(bus->pin_2, bus->pin_1); // SDA, SCL
                Wire.setClock(busSpeed);
                #endif
                #ifdef TARGET_NRF
                Wire.begin(); // Uses default I2C pins
                Wire.setClock(busSpeed);
                writeSerial("NOTE: nRF52840 using default I2C pins (config pins: SCL=" + String(bus->pin_1) + ", SDA=" + String(bus->pin_2) + ")", true);
                #endif
                writeSerial("I2C bus " + String(i) + " initialized: SCL=pin" + String(bus->pin_1) + ", SDA=pin" + String(bus->pin_2) + ", Speed=" + String(busSpeed) + "Hz", true);
            } else {
                writeSerial("WARNING: I2C bus " + String(i) + " configured but not initialized (only first bus supported)", true);
                writeSerial("  SCL=pin" + String(bus->pin_1) + ", SDA=pin" + String(bus->pin_2) + ", Speed=" + String(busSpeed) + "Hz", true);
            }
        }
        else if(bus->bus_type == 0x02){
            writeSerial("SPI bus " + String(i) + " detected (not yet implemented)", true);
            writeSerial("  Instance: " + String(bus->instance_number), true);
        }
        else{
            writeSerial("WARNING: Unknown bus type 0x" + String(bus->bus_type, HEX) + " for bus " + String(i), true);
        }
    }
    writeSerial("=== Data Bus Initialization Complete ===", true);
}

void initio(){
    if(globalConfig.led_count > 0){
        for (uint8_t i = 0; i < globalConfig.led_count; i++) {
            struct LedConfig* led = &globalConfig.leds[i];
            bool invertRed = (led->led_flags & 0x01) != 0;
            bool invertGreen = (led->led_flags & 0x02) != 0;
            bool invertBlue = (led->led_flags & 0x04) != 0;
            bool invertLed4 = (led->led_flags & 0x08) != 0;
                if (led->led_1_r != 0xFF) {
                    pinMode(led->led_1_r, OUTPUT);
                    digitalWrite(led->led_1_r, invertRed ? HIGH : LOW);
                }
                if (led->led_2_g != 0xFF) {
                    pinMode(led->led_2_g, OUTPUT);
                    digitalWrite(led->led_2_g, invertGreen ? HIGH : LOW);
                }
                if (led->led_3_b != 0xFF) {
                    pinMode(led->led_3_b, OUTPUT);
                    digitalWrite(led->led_3_b, invertBlue ? HIGH : LOW);
                }
                if (led->led_4 != 0xFF) {
                    pinMode(led->led_4, OUTPUT);
                    digitalWrite(led->led_4, invertLed4 ? HIGH : LOW);
                }
        }
        for (uint8_t i = 0; i < globalConfig.led_count; i++) {
            if (globalConfig.leds[i].led_type == 0) {
                activeLedInstance = i;
                flashLed(0xE0, 15);
                flashLed(0x1C, 15);
                flashLed(0x03, 15);
                flashLed(0xFF, 15);
            }
        }
    }
    if(globalConfig.system_config.pwr_pin != 0xFF){
    pinMode(globalConfig.system_config.pwr_pin, OUTPUT);
    digitalWrite(globalConfig.system_config.pwr_pin, LOW);
    }
    else{
        writeSerial("Power pin not set", true);
    }
    initDataBuses();
    initSensors();
}

void scanI2CDevices(){
    writeSerial("=== Scanning I2C Bus for Devices ===", true);
    uint8_t deviceCount = 0;
    uint8_t foundDevices[128];
    for(uint8_t address = 0x08; address < 0x78; address++){
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        if(error == 0){
            foundDevices[deviceCount] = address;
            deviceCount++;
            writeSerial("I2C device found at address 0x" + String(address, HEX) + " (" + String(address) + ")", true);
        }
        else if(error == 4){
            writeSerial("ERROR: Unknown error at address 0x" + String(address, HEX), true);
        }
    }
    if(deviceCount == 0){
        writeSerial("No I2C devices found on bus", true);
    } else {
        writeSerial("Found " + String(deviceCount) + " I2C device(s)", true);
        writeSerial("Device addresses: ", true);
        String addrList = "";
        for(uint8_t i = 0; i < deviceCount; i++){
            if(i > 0) addrList += ", ";
            addrList += "0x" + String(foundDevices[i], HEX);
        }
        writeSerial(addrList, true);
    }
    writeSerial("=== I2C Scan Complete ===", true);
}

void initSensors(){
    writeSerial("=== Initializing Sensors ===", true);
    if(globalConfig.sensor_count == 0){
        writeSerial("No sensors configured", true);
        return;
    }
    for(uint8_t i = 0; i < globalConfig.sensor_count; i++){
        struct SensorData* sensor = &globalConfig.sensors[i];
        writeSerial("Initializing sensor " + String(i) + " (instance " + String(sensor->instance_number) + ")", true);
        writeSerial("  Type: 0x" + String(sensor->sensor_type, HEX), true);
        writeSerial("  Bus ID: " + String(sensor->bus_id), true);
        if(sensor->sensor_type == 0x0003){ // AXP2101 PMIC
            writeSerial("  Detected AXP2101 PMIC sensor", true);
        }
        else if(sensor->sensor_type == 0x0001){ // Temperature sensor
            writeSerial("  Temperature sensor (initialization not implemented)", true);
        }
        else if(sensor->sensor_type == 0x0002){ // Humidity sensor
            writeSerial("  Humidity sensor (initialization not implemented)", true);
        }
        else{
            writeSerial("  Unknown sensor type 0x" + String(sensor->sensor_type, HEX), true);
        }
    }
    writeSerial("=== Sensor Initialization Complete ===", true);
}

void initAXP2101(uint8_t busId){
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);
    delay(100);
    digitalWrite(21, HIGH);
    writeSerial("=== Initializing AXP2101 PMIC ===", true);
    if(busId >= globalConfig.data_bus_count){
        writeSerial("ERROR: Invalid bus ID " + String(busId) + " (only " + String(globalConfig.data_bus_count) + " buses configured)", true);
        return;
    }
    struct DataBus* bus = &globalConfig.data_buses[busId];
    if(bus->bus_type != 0x01){
        writeSerial("ERROR: Bus " + String(busId) + " is not an I2C bus", true);
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX) + " (error: " + String(error) + ")", true);
        return;
    }
    writeSerial("AXP2101 detected at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX), true);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t status = Wire.read();
            writeSerial("Power status: 0x" + String(status, HEX), true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_VOL0_CTRL);
    Wire.write(0x12);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DCDC1 voltage set to 3.3V", true);
    } else {
        writeSerial("ERROR: Failed to set DCDC1 voltage", true);
    }
    delay(10);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    uint8_t dcEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            dcEnable = Wire.read();
        }
    }
    dcEnable |= 0x01;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    Wire.write(dcEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DCDC1 enabled (3.3V)", true);
    } else {
        writeSerial("ERROR: Failed to enable DCDC1", true);
    }
    delay(10);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    uint8_t aldoEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldoEnable = Wire.read();
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL2_CTRL);
    error = Wire.endTransmission();
    uint8_t aldo3VolReg = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldo3VolReg = Wire.read();
        }
    }
    aldo3VolReg = (aldo3VolReg & 0xE0) | 0x1C;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL2_CTRL);
    Wire.write(aldo3VolReg);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO3 voltage set to 3.3V", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL3_CTRL);
    error = Wire.endTransmission();
    uint8_t aldo4VolReg = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldo4VolReg = Wire.read();
        }
    }
    aldo4VolReg = (aldo4VolReg & 0xE0) | 0x1C;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL3_CTRL);
    Wire.write(aldo4VolReg);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO4 voltage set to 3.3V", true);
    }
    aldoEnable |= 0x0C;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    Wire.write(aldoEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO3 and ALDO4 enabled (3.3V)", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t wakeupCtl = Wire.read();
            writeSerial("Wakeup control: 0x" + String(wakeupCtl, HEX), true);
            if(wakeupCtl & 0x01){
                writeSerial("Wakeup already enabled", true);
            } else {
                Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
                Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
                Wire.write(wakeupCtl | 0x01);
                error = Wire.endTransmission();
                if(error == 0){
                    writeSerial("Wakeup enabled", true);
                }
            }
        }
    }
    writeSerial("=== AXP2101 PMIC Initialization Complete ===", true);
}

void readAXP2101Data(){
    writeSerial("=== Reading AXP2101 PMIC Data ===", true);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX), true);
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_CHANNEL_CTRL);
    Wire.write(0xFF);
    error = Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t status1 = Wire.read();
            uint8_t status2 = Wire.read();
            writeSerial("Power Status 1: 0x" + String(status1, HEX), true);
            writeSerial("Power Status 2: 0x" + String(status2, HEX), true);
            bool batteryPresent = (status1 & 0x20) != 0;
            bool charging = (status1 & 0x04) != 0;
            bool vbusPresent = (status1 & 0x08) != 0;
            writeSerial("Battery Present: " + String(batteryPresent ? "Yes" : "No"), true);
            writeSerial("Charging: " + String(charging ? "Yes" : "No"), true);
            writeSerial("VBUS Present: " + String(vbusPresent ? "Yes" : "No"), true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_PWRON_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t pwronStatus = Wire.read();
            writeSerial("Power On Status: 0x" + String(pwronStatus, HEX), true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_BAT_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t batVolH = Wire.read();
            uint8_t batVolL = Wire.read();
            uint16_t batVolRaw = ((uint16_t)batVolH << 4) | (batVolL & 0x0F);
            float batVoltage = batVolRaw * 0.5;
            writeSerial("Battery Voltage: " + String(batVoltage, 1) + " mV (" + String(batVoltage / 1000.0, 2) + " V)", true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_VBUS_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t vbusVolH = Wire.read();
            uint8_t vbusVolL = Wire.read();
            uint16_t vbusVolRaw = ((uint16_t)vbusVolH << 4) | (vbusVolL & 0x0F);
            float vbusVoltage = vbusVolRaw * 1.7;
            writeSerial("VBUS Voltage: " + String(vbusVoltage, 1) + " mV (" + String(vbusVoltage / 1000.0, 2) + " V)", true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_SYS_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t sysVolH = Wire.read();
            uint8_t sysVolL = Wire.read();
            uint16_t sysVolRaw = ((uint16_t)sysVolH << 4) | (sysVolL & 0x0F);
            float sysVoltage = sysVolRaw * 1.4;
            writeSerial("System Voltage: " + String(sysVoltage, 1) + " mV (" + String(sysVoltage / 1000.0, 2) + " V)", true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_BAT_PERCENT_DATA);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t batPercent = Wire.read();
            if(batPercent <= 100){
                writeSerial("Battery Percentage: " + String(batPercent) + "%", true);
            } else {
                writeSerial("Battery Percentage: Not available (fuel gauge may be disabled)", true);
            }
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t dcEnable = Wire.read();
            writeSerial("DC Enable Status: 0x" + String(dcEnable, HEX), true);
            writeSerial("  DCDC1: " + String((dcEnable & 0x01) ? "ON" : "OFF"), true);
            writeSerial("  DCDC2: " + String((dcEnable & 0x02) ? "ON" : "OFF"), true);
            writeSerial("  DCDC3: " + String((dcEnable & 0x04) ? "ON" : "OFF"), true);
            writeSerial("  DCDC4: " + String((dcEnable & 0x08) ? "ON" : "OFF"), true);
            writeSerial("  DCDC5: " + String((dcEnable & 0x10) ? "ON" : "OFF"), true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t aldoEnable = Wire.read();
            writeSerial("ALDO Enable Status: 0x" + String(aldoEnable, HEX), true);
            writeSerial("  ALDO1: " + String((aldoEnable & 0x01) ? "ON" : "OFF"), true);
            writeSerial("  ALDO2: " + String((aldoEnable & 0x02) ? "ON" : "OFF"), true);
            writeSerial("  ALDO3: " + String((aldoEnable & 0x04) ? "ON" : "OFF"), true);
            writeSerial("  ALDO4: " + String((aldoEnable & 0x08) ? "ON" : "OFF"), true);
        }
    }
    writeSerial("=== AXP2101 Data Read Complete ===", true);
}

void powerDownAXP2101(){
    writeSerial("=== Powering Down AXP2101 PMIC Rails ===", true);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX) + " (error: " + String(error) + ")", true);
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_IRQ_ENABLE1);
    Wire.write(0x00);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_ENABLE2);
        Wire.write(0x00);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_ENABLE3);
        Wire.write(0x00);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_ENABLE4);
        Wire.write(0x00);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS1);
        Wire.write(0xFF);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS2);
        Wire.write(0xFF);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS3);
        Wire.write(0xFF);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS4);
        Wire.write(0xFF);
        error = Wire.endTransmission();
        if(error == 0){
            writeSerial("All IRQs disabled and status cleared", true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    uint8_t dcEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            dcEnable = Wire.read();
        }
    }
    dcEnable &= 0x01;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    Wire.write(dcEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DC2-5 disabled (DC1 kept enabled)", true);
    } else {
        writeSerial("ERROR: Failed to disable DC2-5", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL1);
    Wire.write(0x00);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("BLDO1-2, CPUSLDO, DLDO1-2 disabled", true);
    } else {
        writeSerial("ERROR: Failed to disable BLDO/CPUSLDO/DLDO rails", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    uint8_t aldoEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldoEnable = Wire.read();
        }
    }
    aldoEnable &= ~0x0F;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    Wire.write(aldoEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO1-4 disabled", true);
    } else {
        writeSerial("ERROR: Failed to disable ALDO rails", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
    error = Wire.endTransmission();
    uint8_t wakeupCtrl = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            wakeupCtrl = Wire.read();
        }
    }
    if(!(wakeupCtrl & 0x04)) {
        wakeupCtrl |= 0x04;
    }
    if(wakeupCtrl & 0x08) {
        wakeupCtrl &= ~0x08;
    }
    if(!(wakeupCtrl & 0x10)) {
        wakeupCtrl |= 0x10;
    }
    wakeupCtrl |= 0x80;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
    Wire.write(wakeupCtrl);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("AXP2101 wake-up configured and sleep mode enabled", true);
    } else {
        writeSerial("ERROR: Failed to configure AXP2101 sleep mode", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_CHANNEL_CTRL);
    Wire.write(0x00);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("All ADC channels disabled", true);
    } else {
        writeSerial("ERROR: Failed to disable ADC channels", true);
    }
    writeSerial("=== AXP2101 PMIC Rails Powered Down ===", true);
}

static void renderChar_4BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, int fontScale) {
    for (int col = 0; col < charWidth; col += fontScale) {
        uint8_t fontByte;
        int fontCol = col / fontScale;
        if (fontCol == 0 || fontCol > 7) {
            fontByte = 0x00;
        } else {
            fontByte = fontData[fontCol - 1];
        }
        uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
        uint8_t pixelNibble = (pixelBit == 1) ? 0x0 : 0xF;
        for (int s = 0; s < fontScale; s++) {
            int pixelX = startX + charIdx * charWidth + col + s;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            int bytePos = pixelX / 2;
            if (bytePos >= pitch) break;
            if ((pixelX % 2) == 0) {
                rowBuffer[bytePos] = (rowBuffer[bytePos] & 0x0F) | (pixelNibble << 4);
            } else {
                rowBuffer[bytePos] = (rowBuffer[bytePos] & 0xF0) | pixelNibble;
            }
        }
    }
}

static void renderChar_2BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, uint8_t colorScheme, int fontScale) {
    uint8_t whiteCode = (colorScheme == 5) ? 0x03 : 0x01;
    int pixelsPerByte = 4;
    for (int col = 0; col < charWidth; col += pixelsPerByte) {
        uint8_t pixelByte = 0;
        for (int p = 0; p < pixelsPerByte; p++) {
            int pixelX = startX + charIdx * charWidth + col + p;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            uint8_t fontByte;
            int fontCol = (col + p) / fontScale;
            if (fontCol == 0 || fontCol > 7) {
                fontByte = 0x00;
            } else {
                fontByte = fontData[fontCol - 1];
            }
            uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
            uint8_t pixelValue = (pixelBit == 1) ? 0x00 : whiteCode;
            pixelByte |= (pixelValue << (6 - p * 2));
        }
        int bytePos = (startX + charIdx * charWidth + col) / 4;
        if (bytePos < pitch) {
            rowBuffer[bytePos] = pixelByte;
        }
    }
}

static void renderChar_1BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, int fontScale) {
    for (int col = 0; col < charWidth; col += fontScale) {
        uint8_t fontByte;
        int fontCol = col / fontScale;
        if (fontCol == 0 || fontCol > 7) {
            fontByte = 0x00;
        } else {
            fontByte = fontData[fontCol - 1];
        }
        uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
        for (int s = 0; s < fontScale; s++) {
            int pixelX = startX + charIdx * charWidth + col + s;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            int bytePos = pixelX / 8;
            int bitPos = 7 - (pixelX % 8);
            if (bytePos < pitch) {
                if (pixelBit == 1) {
                    rowBuffer[bytePos] &= ~(1 << bitPos);
                }
            }
        }
    }
}

void initDisplay(){
    writeSerial("=== Initializing Display ===", true);
    if(globalConfig.display_count > 0){
    pwrmgm(true);
    memset(&bbep, 0, sizeof(BBEPDISP));
    int panelType = mapEpd(globalConfig.displays[0].panel_ic_type);
    bbepSetPanelType(&bbep, panelType);
    bbepSetRotation(&bbep, globalConfig.displays[0].rotation * 90);
    bbepInitIO(&bbep, globalConfig.displays[0].dc_pin, globalConfig.displays[0].reset_pin, globalConfig.displays[0].busy_pin, globalConfig.displays[0].cs_pin, globalConfig.displays[0].data_pin, globalConfig.displays[0].clk_pin, 8000000);
    writeSerial(String("Height: ") + String(globalConfig.displays[0].pixel_height), true);
    writeSerial(String("Width: ") + String(globalConfig.displays[0].pixel_width), true);
    bbepWakeUp(&bbep);
    bbepSendCMDSequence(&bbep, bbep.pInitFull);
    if (! (globalConfig.displays[0].transmission_modes & TRANSMISSION_MODE_CLEAR_ON_BOOT)){
    writeBootScreenWithQr();
    bbepRefresh(&bbep, REFRESH_FULL);
    waitforrefresh(60);
    bbepSleep(&bbep, 1);
    delay(200);
    }
    pwrmgm(false);
    }
    else{
        writeSerial("No display found", true);
    }
}


int getplane() {
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    if (colorScheme == 0) return PLANE_0;
    if (colorScheme == 1 || colorScheme == 2) return PLANE_0;
    if (colorScheme == 5) return PLANE_1;
    return PLANE_1;
}

int getBitsPerPixel() {
    if (globalConfig.displays[0].color_scheme == 4) return 4;
    if (globalConfig.displays[0].color_scheme == 3) return 2;
    if (globalConfig.displays[0].color_scheme == 5) return 2;
    return 1;
}

float readBatteryVoltage() {
    if (globalConfig.power_option.battery_sense_pin == 0xFF) return -1.0;
    uint8_t sensePin = globalConfig.power_option.battery_sense_pin;
    uint8_t enablePin = globalConfig.power_option.battery_sense_enable_pin;
    uint16_t scalingFactor = globalConfig.power_option.voltage_scaling_factor;
    pinMode(sensePin, INPUT);
    if (enablePin != 0xFF) {
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, HIGH);
        delay(10);
    }
    const int numSamples = 10;
    uint32_t adcSum = 0;
    for (int i = 0; i < numSamples; i++) {
        adcSum += analogRead(sensePin);
        delay(2);
    }
    uint32_t adcAverage = adcSum / numSamples;
    if (enablePin != 0xFF) digitalWrite(enablePin, LOW);
    if (scalingFactor > 0) return (adcAverage * scalingFactor) / (100000.0);
    return -1.0;
}

float readChipTemperature() {
#ifdef TARGET_ESP32
    return temperatureRead();
#elif defined(TARGET_NRF)
    int32_t tempRaw = 0;
    uint32_t err_code = sd_temp_get(&tempRaw);
    if (err_code == 0) return tempRaw * 0.25f;
    return -999.0;
#else
    return -999.0;
#endif
}

void updatemsdata(){
    float batteryVoltage = readBatteryVoltage();
    float chipTemperature = readChipTemperature();
    uint16_t batteryVoltageMv = (uint16_t)(batteryVoltage * 1000);
    uint16_t batteryVoltage10mv = batteryVoltageMv / 10;
    if (batteryVoltage10mv > 511) batteryVoltage10mv = 511;
    int16_t tempEncoded = (int16_t)((chipTemperature + 40.0f) * 2.0f);
    if (tempEncoded < 0) tempEncoded = 0;
    else if (tempEncoded > 255) tempEncoded = 255;
    uint8_t temperatureByte = (uint8_t)tempEncoded;
    uint8_t batteryVoltageLowByte = (uint8_t)(batteryVoltage10mv & 0xFF);
    uint8_t statusByte = ((batteryVoltage10mv >> 8) & 0x01) |
                         ((rebootFlag & 0x01) << 1) |
                         ((connectionRequested & 0x01) << 2) |
                         ((mloopcounter & 0x0F) << 4);
    uint16_t msd_cid = 0x2446;
    memset(msd_payload, 0, 16);
    memcpy(msd_payload, (uint8_t*)&msd_cid, sizeof(msd_cid));
    memcpy(&msd_payload[2], dynamicreturndata, 11);
    msd_payload[13] = temperatureByte;
    msd_payload[14] = batteryVoltageLowByte;
    msd_payload[15] = statusByte;
#ifdef TARGET_NRF
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, msd_payload, 16);
    Bluefruit.Advertising.setInterval(32, 400);
    Bluefruit.Advertising.setFastTimeout(1);
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
#endif
#ifdef TARGET_ESP32
    if (advertisementData != nullptr) {
        String manufacturerDataStr;
        manufacturerDataStr.reserve(16);
        for (int i = 0; i < 16; i++) manufacturerDataStr += (char)msd_payload[i];
        advertisementData->setManufacturerData(manufacturerDataStr);
        BLEAdvertising *pAdvertising = (pServer != nullptr) ? pServer->getAdvertising() : BLEDevice::getAdvertising();
        if (pAdvertising != nullptr) {
            pAdvertising->stop();
            BLEAdvertisementData freshAdvertisementData;
            static String savedDeviceName = "";
            if (savedDeviceName.length() == 0) savedDeviceName = "OD" + getChipIdHex();
            freshAdvertisementData.setName(savedDeviceName);
            freshAdvertisementData.setFlags(0x06);
            freshAdvertisementData.setManufacturerData(manufacturerDataStr);
            *advertisementData = freshAdvertisementData;
            pAdvertising->setAdvertisementData(freshAdvertisementData);
            if (pService != nullptr) pAdvertising->addServiceUUID(pService->getUUID());
            pAdvertising->setScanResponse(false);
            pAdvertising->setMinPreferred(0x06);
            pAdvertising->setMinPreferred(0x12);
            pAdvertising->start();
        }
    }
#endif
    mloopcounter++;
    mloopcounter &= 0x0F;
}

void handleDirectWriteCompressedData(uint8_t* data, uint16_t len) {
    uint32_t newTotalSize = directWriteCompressedReceived + len;
    if (newTotalSize > (54 * 1024)) {
        cleanupDirectWriteState(true);
        uint8_t errorResponse[] = {0xFF, 0xFF};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    memcpy(directWriteCompressedBuffer + directWriteCompressedReceived, data, len);
    directWriteCompressedReceived += len;
    uint8_t ackResponse[] = {0x00, 0x71};
    sendResponse(ackResponse, sizeof(ackResponse));
}

void decompressDirectWriteData() {
    if (directWriteCompressedReceived == 0) return;
    struct uzlib_uncomp d;
    memset(&d, 0, sizeof(d));
    d.source = directWriteCompressedBuffer;
    d.source_limit = directWriteCompressedBuffer + directWriteCompressedReceived;
    d.source_read_cb = NULL;
    uzlib_init();
    int hdr = uzlib_zlib_parse_header(&d);
    if (hdr < 0) return;
    uint16_t window = 0x100 << hdr;
    if (window > (uint16_t)(32 * 1024)) window = (uint16_t)(32 * 1024);
    uzlib_uncompress_init(&d, dictionaryBuffer, window);
    int res;
    do {
        d.dest_start = decompressionChunk;
        d.dest = decompressionChunk;
        d.dest_limit = decompressionChunk + 4096;
        res = uzlib_uncompress(&d);
        size_t bytesOut = d.dest - d.dest_start;
        if (bytesOut > 0) {
            bbepWriteData(&bbep, decompressionChunk, bytesOut);
            directWriteBytesWritten += bytesOut;
        }
    } while (res == TINF_OK && directWriteBytesWritten < directWriteTotalBytes);
}

void cleanupDirectWriteState(bool refreshDisplay) {
    directWriteActive = false;
    directWriteCompressed = false;
    directWriteBitplanes = false;
    directWritePlane2 = false;
    directWriteBytesWritten = 0;
    directWriteCompressedReceived = 0;
    directWriteCompressedSize = 0;
    directWriteDecompressedTotal = 0;
    directWriteCompressedBuffer = nullptr;
    directWriteWidth = 0;
    directWriteHeight = 0;
    directWriteTotalBytes = 0;
    directWriteRefreshMode = 0;
    directWriteStartTime = 0;
    if (refreshDisplay && displayPowerState) {
        bbepSleep(&bbep, 1);
        delay(200);
    }
    displayPowerState = false;
    pwrmgm(false);
}

void handleDirectWriteStart(uint8_t* data, uint16_t len) {
    if (directWriteActive) cleanupDirectWriteState(false);
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    directWriteBitplanes = (colorScheme == 1 || colorScheme == 2);
    directWritePlane2 = false;
    directWriteCompressed = (len >= 4);
    directWriteWidth = globalConfig.displays[0].pixel_width;
    directWriteHeight = globalConfig.displays[0].pixel_height;
    if (directWriteBitplanes) directWriteTotalBytes = (directWriteWidth * directWriteHeight + 7) / 8;
    else {
        int bitsPerPixel = getBitsPerPixel();
        if (bitsPerPixel == 4) directWriteTotalBytes = (directWriteWidth * directWriteHeight + 1) / 2;
        else if (bitsPerPixel == 2) directWriteTotalBytes = (directWriteWidth * directWriteHeight + 3) / 4;
        else directWriteTotalBytes = (directWriteWidth * directWriteHeight + 7) / 8;
    }
    if (directWriteCompressed) {
        memcpy(&directWriteDecompressedTotal, data, 4);
        directWriteCompressedBuffer = compressedDataBuffer;
        directWriteCompressedSize = 0;
        directWriteCompressedReceived = 0;
        if (len > 4) {
            uint32_t compressedDataLen = len - 4;
            if (compressedDataLen > (54 * 1024)) {
                cleanupDirectWriteState(false);
                uint8_t errorResponse[] = {0xFF, 0xFF};
                sendResponse(errorResponse, sizeof(errorResponse));
                return;
            }
            memcpy(directWriteCompressedBuffer, data + 4, compressedDataLen);
            directWriteCompressedReceived = compressedDataLen;
        }
    }
    directWriteActive = true;
    directWriteBytesWritten = 0;
    directWriteStartTime = millis();
    if (displayPowerState) {
        pwrmgm(false);
        delay(100);
    }
    pwrmgm(true);
    bbepInitIO(&bbep, globalConfig.displays[0].dc_pin, globalConfig.displays[0].reset_pin, globalConfig.displays[0].busy_pin, globalConfig.displays[0].cs_pin, globalConfig.displays[0].data_pin, globalConfig.displays[0].clk_pin, 8000000);
    bbepWakeUp(&bbep);
    bbepSendCMDSequence(&bbep, bbep.pInitFull);
    bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
    bbepStartWrite(&bbep, directWriteBitplanes ? PLANE_0 : getplane());
    uint8_t ackResponse[] = {0x00, 0x70};
    sendResponse(ackResponse, sizeof(ackResponse));
}

void handleDirectWriteData(uint8_t* data, uint16_t len) {
    if (!directWriteActive || len == 0) return;
    if (directWriteCompressed) {
        handleDirectWriteCompressedData(data, len);
        return;
    }
    uint32_t remainingBytes = (directWriteBytesWritten < directWriteTotalBytes) ? (directWriteTotalBytes - directWriteBytesWritten) : 0;
    uint16_t bytesToWrite = (len > remainingBytes) ? remainingBytes : len;
    if (bytesToWrite > 0) {
        bbepWriteData(&bbep, data, bytesToWrite);
        directWriteBytesWritten += bytesToWrite;
    }
    if (directWriteBytesWritten >= directWriteTotalBytes) {
        handleDirectWriteEnd(nullptr, 0);
    } else {
        uint8_t ackResponse[] = {0x00, 0x71};
        sendResponse(ackResponse, sizeof(ackResponse));
    }
}

void handleDirectWriteEnd(uint8_t* data, uint16_t len) {
    if (!directWriteActive) return;
    if (directWriteCompressed && directWriteCompressedReceived > 0) decompressDirectWriteData();
    int refreshMode = REFRESH_FULL;
    if (data != nullptr && len >= 1 && data[0] == 1) refreshMode = REFRESH_FAST;
    uint8_t ackResponse[] = {0x00, 0x72};
    sendResponse(ackResponse, sizeof(ackResponse));
    delay(100);
    bbepRefresh(&bbep, refreshMode);
    bool refreshSuccess = waitforrefresh(60);
    bbepSleep(&bbep, 1);
    delay(200);
    cleanupDirectWriteState(false);
    if (refreshSuccess) {
        uint8_t refreshResponse[] = {0x00, 0x73};
        sendResponse(refreshResponse, sizeof(refreshResponse));
    } else {
        uint8_t timeoutResponse[] = {0x00, 0x74};
        sendResponse(timeoutResponse, sizeof(timeoutResponse));
    }
}
