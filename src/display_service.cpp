#include "display_service.h"

#include <Arduino.h>
#include <bb_epaper.h>
#include <string.h>
#include <Wire.h>
#include "structs.h"
#include "buzzer_control.h"
#include "sensor_sht40.h"
#include "sensor_bq27220.h"
#include "communication.h"
#include "encryption.h"
#include "boot_screen.h"
#include "touch_input.h"
#include "uzlib.h"
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
#include "display_seeed_gfx.h"
#endif

#ifdef TARGET_NRF
extern "C" {
#include "nrf_soc.h"
}
#include <bluefruit.h>
#include "ble_init.h"
#include "nrf.h"
#endif

#ifdef TARGET_ESP32
#include <BLEDevice.h>
#include "ble_init.h"
#include "wifi_service.h"
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
extern uint32_t directWriteCompressedReceived;
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
extern uint8_t decompressionChunk[OPENDISPLAY_DECOMPRESSION_CHUNK_SIZE];
volatile bool epdRefreshInProgress = false;

extern uint32_t displayed_etag;

static const uint8_t ERR_ETAG_MISMATCH = 0x01u;
static const uint8_t ERR_RECT_OOB = 0x03u;
static const uint8_t ERR_RECT_ALIGN = 0x04u;
static const uint8_t ERR_PARTIAL_FLAGS = 0x05u;
static const uint8_t ERR_PARTIAL_STREAM = 0x06u;
static const uint8_t ERR_PARTIAL_UNSUPPORTED = 0x07u;

static const uint8_t PARTIAL_FLAG_COMPRESSED = 0x01u;
static const uint8_t PARTIAL_ALLOWED_FLAGS = PARTIAL_FLAG_COMPRESSED;

struct PartialStreamContext {
    bool active;
    bool compressed;
    uint8_t flags;
    uint32_t new_etag;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    uint32_t expected_stream_size;
    uint32_t plane_size;
    uint32_t bytes_received;
    uint32_t bytes_written;
    uint8_t current_plane;
    uint32_t start_time;
};

#ifdef TARGET_ESP32
extern BLEAdvertisementData* advertisementData;
extern BLEServer* pServer;
extern BLEService* pService;
#endif

void pwrmgm(bool onoff);
String getChipIdHex();
void writeSerial(String message, bool newLine);
void flushLog();
void bbepInitIO(BBEPDISP *pBBEP, uint8_t u8DC, uint8_t u8RST, uint8_t u8BUSY, uint8_t u8CS, uint8_t u8MOSI, uint8_t u8SCK, uint32_t u32Speed);
void bbepWakeUp(BBEPDISP *pBBEP);
void bbepSendCMDSequence(BBEPDISP *pBBEP, const uint8_t *pSeq);
void bbepRefresh(BBEPDISP *pBBEP, int iMode);
void bbepSleep(BBEPDISP *pBBEP, int iMode);
void bbepSetAddrWindow(BBEPDISP *pBBEP, int x, int y, int cx, int cy);
void bbepStartWrite(BBEPDISP *pBBEP, int iPlane);
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen);
void bbepFill(BBEPDISP *pBBEP, unsigned char ucColor, int iPlane);
void bbepWriteCmd(BBEPDISP *pBBEP, uint8_t cmd);
void bbepCMD2(BBEPDISP *pBBEP, uint8_t cmd1, uint8_t cmd2);
void bbepWaitBusy(BBEPDISP *pBBEP);
bool bbepIsBusy(BBEPDISP *pBBEP);
void flashLed(uint8_t color, uint8_t brightness);
bool waitforrefresh(int timeout);

#ifdef TARGET_NRF
static bool nrfVbusPresent() {
    //ignored for now
    //return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
    return false;
}
#else
static bool nrfVbusPresent() { return true; }
#endif

static void epdBsPinLowIfNrf() {
#ifdef TARGET_NRF
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
#endif
}

// Battery boot: power-cycle the panel rail once. pwrmgm(true) already waits ~900 ms
// per enable; extra delays here are only for rail discharge between off/on.
static void prepareEpdRailForBoot() {
    epdBsPinLowIfNrf();
    pwrmgm(true);
#ifdef TARGET_NRF
    if (!nrfVbusPresent()) {
        delay(50);
        pwrmgm(false);
        delay(50);
        epdBsPinLowIfNrf();
        pwrmgm(true);
    }
#endif
}

static void initBbepPanelSession() {
    const DisplayConfig& d = globalConfig.displays[0];
    bbepInitIO(&bbep, d.dc_pin, d.reset_pin, d.busy_pin, d.cs_pin, d.data_pin, d.clk_pin, 8000000);
    bbepWakeUp(&bbep);
    bbepSendCMDSequence(&bbep, bbep.pInitFull);
    delay(200);
}

static bool refreshBootScreenFull() {
    if (!writeBootScreenWithQr()) {
        writeSerial("Boot screen render failed", true);
        return false;
    }
    writeSerial("EPD refresh: FULL (boot)", true);
    touchSuspendForEpdRefresh();
    bbepRefresh(&bbep, REFRESH_FULL);
    return waitforrefresh(60);
}

static void cleanup_partial_write_state(void);
static bool panel_skips_bbep_set_addr_window(void);
static void partial_set_addr_window(BBEPDISP *pBBEP, int x, int y, int cx, int cy);
static bool partial_consume_bytes(uint8_t* data, uint32_t len);
static void partial_prepare_panel_ram(void);
static bool partial_write_to_panel(int refreshMode);
static bool partial_write_stream_bytes(uint8_t* data, uint32_t len);
static bool zlib_stream_to_direct_write(const uint8_t* data, uint32_t len, bool final);
static bool zlib_stream_to_partial_write(const uint8_t* data, uint32_t len, bool final);
static uint32_t calc_controller_plane_bytes(uint16_t width, uint16_t height);
static uint32_t parse_be_u32(const uint8_t* data);
static void send_direct_write_nack(uint8_t opcode, uint8_t error, bool cleanupState);
static PartialStreamContext partialCtx = {};

void checkPartialWriteTimeout(void) {
    if (partialCtx.active && partialCtx.start_time > 0 &&
        (millis() - partialCtx.start_time) > 900000UL) {
        writeSerial("ERROR: Partial write timeout - cleaning up stuck state", true);
        cleanup_partial_write_state();
    }
}

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
#define FONT_BASE_WIDTH 8
#define FONT_BASE_HEIGHT 8
#define FONT_SMALL_THRESHOLD 264

extern const uint8_t writelineFont[] PROGMEM;
extern uint8_t staticWhiteRow[680];
extern uint8_t staticRowBuffer[BOOT_ROW_BUFFER_SIZE];
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
        case 0x0041: return EP_PANEL_UNDEFINED;
        // bb_epaper 2.1.9 does not define the 13.3" EP133 panel yet.
        case 0x0042: return EP_PANEL_UNDEFINED;
        case 0x0043: return EP154_200x200_4GRAY;
        case 0x0044: return EP42B_400x300_4GRAY;
        case 0x0045: return EP397_800x480;
        case 0x0046: return EP397_800x480_4GRAY;
        case 0x0047: return EP368_792x528;
        case 0x0048: return EP368_792x528_4GRAY;
        case 0x0049: return EP213ZZ_122x250;
        case 0x004A: return EP40_SPECTRA_400x600;
        case 0x004B: return EP27_176x264;
        case 0x004C: return EP27_176x264_4GRAY;
        default: return EP_PANEL_UNDEFINED;
    }
}

bool seeed_driver_used(void) {
#if !defined(TARGET_ESP32) || !defined(OPENDISPLAY_SEEED_GFX)
    return false;
#else
    if (globalConfig.display_count < 1) return false;
    const struct DisplayConfig& d = globalConfig.displays[0];
    if (d.panel_ic_type != PANEL_IC_SEEED_ED103TC2_1872X1404 &&
        d.panel_ic_type != PANEL_IC_SEEED_ED103TC2_1872X1404_4GRAY) return false;
    if (d.display_technology != 0 && d.display_technology != 1) return false;
    return true;
#endif
}

bool waitforrefresh(int timeout){
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) return seeed_gfx_wait_refresh(timeout);
#endif
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

#ifdef TARGET_ESP32
static bool s_wire_open_display_ready = false;
static int8_t s_wire_sda_pin = -1;
static int8_t s_wire_scl_pin = -1;
static uint32_t s_wire_clock_hz = 0;

static bool wireBeginForOpenDisplay(int sda, int scl, uint32_t hz) {
    // Do not pinMode() before begin on ESP32 — periman must hand pins to the I2C driver.
    if (Wire.begin(sda, scl, hz)) {
        Wire.setClock(hz);
        s_wire_sda_pin = (int8_t)sda;
        s_wire_scl_pin = (int8_t)scl;
        s_wire_clock_hz = hz;
        s_wire_open_display_ready = true;
        return true;
    }
    if (hz > 100000u && Wire.begin(sda, scl, 100000u)) {
        writeSerial("NOTE: I2C fallback to 100kHz (SDA=GPIO" + String(sda) + " SCL=GPIO" + String(scl) + ")", true);
        Wire.setClock(100000u);
        s_wire_sda_pin = (int8_t)sda;
        s_wire_scl_pin = (int8_t)scl;
        s_wire_clock_hz = 100000u;
        s_wire_open_display_ready = true;
        return true;
    }
    writeSerial("ERROR: Wire.begin failed (SDA=GPIO" + String(sda) + " SCL=GPIO" + String(scl) + ")", true);
    return false;
}
#endif

static bool i2cDataBusValid(uint8_t bus_id) {
    if (bus_id >= globalConfig.data_bus_count) {
        return false;
    }
    const struct DataBus& bus = globalConfig.data_buses[bus_id];
    return bus.bus_type == 0x01 && bus.pin_1 != 0xFF && bus.pin_2 != 0xFF;
}

bool openDisplayI2cBusConfigured(void) {
    for (uint8_t i = 0; i < globalConfig.data_bus_count; i++) {
        if (i2cDataBusValid(i)) {
            return true;
        }
    }
    return false;
}

void invalidateOpenDisplayWire(void) {
#ifdef TARGET_ESP32
    if (s_wire_open_display_ready) {
        Wire.end();
    }
    s_wire_open_display_ready = false;
#endif
}

bool initOrRestoreWireForBus(uint8_t bus_id) {
#ifdef TARGET_ESP32
    if (bus_id == 0xFF) {
        bus_id = 0;
    }
    if (!i2cDataBusValid(bus_id)) {
        return false;
    }
    const struct DataBus& bus = globalConfig.data_buses[bus_id];
    uint32_t hz = bus.bus_speed_hz ? bus.bus_speed_hz : 100000u;
    int sda = (int)bus.pin_2;
    int scl = (int)bus.pin_1;
    if (s_wire_open_display_ready && s_wire_sda_pin == sda && s_wire_scl_pin == scl) {
        return true;
    }
    if (s_wire_open_display_ready) {
        Wire.end();
        s_wire_open_display_ready = false;
    }
    if (!wireBeginForOpenDisplay(sda, scl, hz)) {
        s_wire_open_display_ready = false;
        return false;
    }
    return true;
#else
    (void)bus_id;
    initOrRestoreWireForOpenDisplay();
    return true;
#endif
}

void initOrRestoreWireForOpenDisplay(void) {
#ifdef TARGET_ESP32
    if (globalConfig.data_bus_count > 0 && i2cDataBusValid(0)) {
        (void)initOrRestoreWireForBus(0);
        return;
    }
    if (!s_wire_open_display_ready) {
        if (Wire.begin()) {
            s_wire_open_display_ready = true;
        }
    }
#else
    if (globalConfig.data_bus_count > 0) {
        const struct DataBus& bus = globalConfig.data_buses[0];
        if (bus.bus_type == 0x01 && bus.pin_1 != 0xFF && bus.pin_2 != 0xFF) {
            uint32_t hz = bus.bus_speed_hz ? bus.bus_speed_hz : 100000u;
            pinMode(bus.pin_1, INPUT);
            pinMode(bus.pin_2, INPUT);
            if (bus.pullups & 0x01) {
                pinMode(bus.pin_1, INPUT_PULLUP);
            }
            if (bus.pullups & 0x02) {
                pinMode(bus.pin_2, INPUT_PULLUP);
            }
        }
    }
    Wire.begin();
    if (globalConfig.data_bus_count > 0) {
        const struct DataBus& bus = globalConfig.data_buses[0];
        if (bus.bus_speed_hz > 0) {
            Wire.setClock(bus.bus_speed_hz);
        }
    }
#endif
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
            if(i == 0){
                #ifdef TARGET_ESP32
                initOrRestoreWireForOpenDisplay();
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
                Wire.begin(); // Uses default I2C pins
                Wire.setClock(busSpeed);
                writeSerial("NOTE: nRF52840 using default I2C pins (config pins: SCL=" + String(bus->pin_1) + ", SDA=" + String(bus->pin_2) + ")", true);
                #endif
                writeSerial("I2C bus " + String(i) + " initialized: SCL=pin" + String(bus->pin_1) + ", SDA=pin" + String(bus->pin_2) + ", Speed=" + String(busSpeed) + "Hz", true);
            } else {
                writeSerial("I2C bus " + String(i) + " configured (init on demand): SCL=pin" + String(bus->pin_1) +
                    ", SDA=pin" + String(bus->pin_2) + ", Speed=" + String(busSpeed) + "Hz", true);
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
    writeSerial("[initio] >> LEDs", true); flushLog();
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
#ifdef TARGET_NRF
                if (nrfVbusPresent())
#endif
                {
                    flashLed(0xE0, 15);
                    flashLed(0x1C, 15);
                    flashLed(0x03, 15);
                    flashLed(0xFF, 15);
                }
            }
        }
    }
    writeSerial("[initio] >> initPassiveBuzzers", true); flushLog();
    initPassiveBuzzers();
    writeSerial("[initio] >> pwr_pin", true); flushLog();
    if(globalConfig.system_config.pwr_pin != 0xFF){
    pinMode(globalConfig.system_config.pwr_pin, OUTPUT);
    digitalWrite(globalConfig.system_config.pwr_pin, LOW);
    }
    else{
        writeSerial("Power pin not set", true);
    }
    writeSerial("[initio] >> initDataBuses", true); flushLog();
    initDataBuses();
    writeSerial("[initio] >> initSensors", true); flushLog();
    initSensors();
    writeSerial("[initio] << done", true); flushLog();
}

void scanI2CDevices(){
    writeSerial("=== Scanning I2C Bus for Devices ===", true);
    initOrRestoreWireForOpenDisplay();
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
        if(sensor->sensor_type == SENSOR_TYPE_AXP2101){
            writeSerial("  Detected AXP2101 PMIC sensor", true);
        }
        else if(sensor->sensor_type == SENSOR_TYPE_TEMPERATURE){
            writeSerial("  Temperature sensor (initialization not implemented)", true);
        }
        else if(sensor->sensor_type == SENSOR_TYPE_HUMIDITY){
            writeSerial("  Humidity sensor (initialization not implemented)", true);
        }
        else if(sensor->sensor_type == SENSOR_TYPE_SHT40){
            writeSerial("  SHT40 (I2C + MSD slot)", true);
        }
        else if(sensor->sensor_type == SENSOR_TYPE_BQ27220){
            writeSerial("  BQ27220 fuel gauge (MSD voltage + optional dynamic SOC/status bytes)", true);
        }
        else{
            writeSerial("  Unknown sensor type 0x" + String(sensor->sensor_type, HEX), true);
        }
    }
    initSht40Sensors();
    initBq27220Sensors();
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
    if(!initOrRestoreWireForBus(busId)){
        writeSerial("ERROR: Failed to (re)init I2C bus " + String(busId) + " for AXP2101", true);
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
    uint8_t whiteCode = (colorScheme == COLOR_SCHEME_GRAY4) ? 0x03 : 0x01;
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
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        pwrmgm(true);
        writeSerial("Display: Seeed_GFX (panel_ic " + String(globalConfig.displays[0].panel_ic_type) + ", " +
                    String(globalConfig.displays[0].pixel_width) + "x" + String(globalConfig.displays[0].pixel_height) + ", " +
                    String(getBitsPerPixel()) + " bpp)", true);
        seeed_gfx_epaper_begin();
        if (opnd_seeed_tcon_busy_timeout_occurred()) {
            writeSerial("Seeed_GFX init failed (TCON busy timeout) — skipping boot refresh", true);
            pwrmgm(false);
            return;
        }
        writeSerial(String("Height: ") + String(globalConfig.displays[0].pixel_height), true);
        writeSerial(String("Width: ") + String(globalConfig.displays[0].pixel_width), true);
        if (! (globalConfig.displays[0].transmission_modes & TRANSMISSION_MODE_CLEAR_ON_BOOT)){
            writeBootScreenWithQr();
            writeSerial("EPD refresh: FULL (boot, Seeed)", true);
            touchSuspendForEpdRefresh();
            seeed_gfx_full_update();
            waitforrefresh(60);
            seeed_gfx_sleep_after_refresh();
            delay(200);
            pwrmgm(false);
            touchResumeAfterEpdRefresh();
        } else {
            pwrmgm(false);
        }
    } else
#endif
    {
        prepareEpdRailForBoot();
        memset(&bbep, 0, sizeof(BBEPDISP));
        int panelType = mapEpd(globalConfig.displays[0].panel_ic_type);
        bbepSetPanelType(&bbep, panelType);
        bbepSetRotation(&bbep, globalConfig.displays[0].rotation * 90);
        writeSerial(String("Height: ") + String(globalConfig.displays[0].pixel_height), true);
        writeSerial(String("Width: ") + String(globalConfig.displays[0].pixel_width), true);
        initBbepPanelSession();
        if (! (globalConfig.displays[0].transmission_modes & TRANSMISSION_MODE_CLEAR_ON_BOOT)){
            bool bootOk = refreshBootScreenFull();
            if (!bootOk && !nrfVbusPresent()) {
                writeSerial("Boot refresh failed on battery — re-powering panel and retrying", true);
                touchResumeAfterEpdRefresh();
                pwrmgm(false);
                delay(200);
                prepareEpdRailForBoot();
                initBbepPanelSession();
                bootOk = refreshBootScreenFull();
            }
            if (!bootOk) {
                writeSerial("Boot screen refresh did not complete", true);
            }
            bbepSleep(&bbep, 1);
            delay(200);
            pwrmgm(false);
            touchResumeAfterEpdRefresh();
        } else {
            pwrmgm(false);
        }
    }
    }
    else{
        writeSerial("No display found", true);
    }
}


int getplane() {
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    if (colorScheme == COLOR_SCHEME_MONO || colorScheme == COLOR_SCHEME_GRAY16) return PLANE_0;
    if (colorScheme == COLOR_SCHEME_BWR || colorScheme == COLOR_SCHEME_BWY) return PLANE_0;
    if (colorScheme == COLOR_SCHEME_GRAY4) return PLANE_1;
    return PLANE_1;
}

int getBitsPerPixel() {
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (globalConfig.display_count > 0 &&
        globalConfig.displays[0].panel_ic_type == PANEL_IC_SEEED_ED103TC2_1872X1404_4GRAY) {
        return 4;
    }
#endif
    if (globalConfig.displays[0].color_scheme == COLOR_SCHEME_BWGBRY) return 4;
    if (globalConfig.displays[0].color_scheme == COLOR_SCHEME_BWRY) return 2;
    if (globalConfig.displays[0].color_scheme == COLOR_SCHEME_GRAY4) return 2;
    return 1;
}

static float readBatteryVoltageUncached() {
    if (bq27220IsConfigured()) {
        float gaugeV = bq27220BatteryVoltageVolts();
        if (gaugeV >= 0.0f) {
            return gaugeV;
        }
    }
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
    if (enablePin != 0xFF) {
        digitalWrite(enablePin, LOW);
    }
    if (scalingFactor > 0) return (adcAverage * scalingFactor) / (100000.0);
    return -1.0;
}

static constexpr uint32_t kBatteryVoltageTtlMs = 30000u;
float readBatteryVoltage() {
    static uint32_t lastReadMs = 0;
    static float cachedVoltage = -1.0f;
    static bool haveReading = false;
    if (haveReading && (uint32_t)(millis() - lastReadMs) < kBatteryVoltageTtlMs) {
        return cachedVoltage;
    }
    cachedVoltage = readBatteryVoltageUncached();
    lastReadMs = millis();
    haveReading = true;
    return cachedVoltage;
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
    pollSht40SensorsForMsd();
    pollBq27220ForMsd();
    float batteryVoltage = readBatteryVoltage();
    float chipTemperature = readChipTemperature();
    uint16_t batteryVoltage10mv = 0;
    if (batteryVoltage >= 0.0f) {
        uint16_t batteryVoltageMv = (uint16_t)(batteryVoltage * 1000.0f);
        batteryVoltage10mv = batteryVoltageMv / 10;
        if (batteryVoltage10mv > 511) {
            batteryVoltage10mv = 511;
        }
    }
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
    static uint8_t prev_msd_payload_nrf[16] = {0xFF};
    if (memcmp(prev_msd_payload_nrf, msd_payload, 16) == 0) {
        mloopcounter++;
        mloopcounter &= 0x0F;
        return;
    }
    memcpy(prev_msd_payload_nrf, msd_payload, 16);
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, msd_payload, 16);
    ble_nrf_apply_adv_interval();
    Bluefruit.Advertising.setFastTimeout(1);
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
#endif
#ifdef TARGET_ESP32
    if (advertisementData != nullptr) {
        static uint8_t prev_msd_payload[16] = {0xFF};
        if (memcmp(prev_msd_payload, msd_payload, 16) == 0) {
            mloopcounter++;
            mloopcounter &= 0x0F;
            return;
        }
        memcpy(prev_msd_payload, msd_payload, 16);
        String manufacturerDataStr;
        manufacturerDataStr.reserve(16);
        for (int i = 0; i < 16; i++) manufacturerDataStr += (char)msd_payload[i];
        advertisementData->setManufacturerData(manufacturerDataStr);
        BLEAdvertising *pAdvertising = (pServer != nullptr) ? pServer->getAdvertising() : BLEDevice::getAdvertising();
        if (pAdvertising != nullptr) {
            if (pServer != nullptr && pServer->getConnectedCount() > 0) {
                *advertisementData = BLEAdvertisementData();
                advertisementData->setName("OD" + getChipIdHex());
                advertisementData->setFlags(0x06);
                advertisementData->setManufacturerData(manufacturerDataStr);
            } else {
                pAdvertising->stop();
                BLEAdvertisementData freshAdvertisementData;
                static String savedDeviceName = "";
                if (savedDeviceName.length() == 0) savedDeviceName = "OD" + getChipIdHex();
                freshAdvertisementData.setName(savedDeviceName);
                freshAdvertisementData.setFlags(0x06);
                freshAdvertisementData.setManufacturerData(manufacturerDataStr);
                *advertisementData = freshAdvertisementData;
                pAdvertising->setAdvertisementData(freshAdvertisementData);
                pAdvertising->setScanResponse(false);
                pAdvertising->setMinPreferred(0x06);
                pAdvertising->setMaxPreferred(0x12);
                delay(50);
                pAdvertising->start();
            }
        }
    }
    opendisplay_mdns_update_msd_txt();
#endif
    mloopcounter++;
    mloopcounter &= 0x0F;
}

// --- Quiet image-write logging ---------------------------------------------
// An image push arrives as a 0x70 start, many 0x71 data frames, and a 0x72 end.
// Logging every frame + its ack floods the UART (~1 MB of text for a 1.3 MB
// image) and, once the TX buffer fills, throttles the transfer itself. Instead
// we log the first frame in full, a 5%-step percentage meter thereafter, and
// the final frame + chunk total at completion. imageWriteLogQuiet{Cmd,Ack}()
// let communication.cpp suppress the per-frame command/ack spam accordingly.
static uint32_t imgLogTotalBytes;    // expected payload for this stream
static uint32_t imgLogChunks;        // 0x71 frames seen this stream
static uint8_t  imgLogLastStep;      // last 5% step printed (pct/5)
static uint16_t imgLogLastLen;       // length of most recent frame
static uint8_t  imgLogLastHead[16];  // first bytes of most recent frame
static uint8_t  imgLogLastHeadLen;   // valid bytes in imgLogLastHead
static uint32_t imgLogStartMs;       // millis() at stream start (for throughput)

static String imgLogHex(const uint8_t* buf, uint8_t n) {
    String s;
    for (uint8_t i = 0; i < n; i++) {
        if (i > 0) s += " ";
        if (buf[i] < 16) s += "0";
        s += String(buf[i], HEX);
    }
    return s;
}

static void imageWriteLogReset(void) {
    imgLogTotalBytes = 0;
    imgLogChunks = 0;
    imgLogLastStep = 0;
    imgLogLastLen = 0;
    imgLogLastHeadLen = 0;
    imgLogStartMs = 0;
}

static void imageWriteLogStart(uint32_t totalBytes) {
    imgLogTotalBytes = totalBytes;
    imgLogStartMs = millis();
    writeSerial("DW start: " + String(totalBytes) + " bytes expected", true);
}

static void imageWriteLogChunk(const uint8_t* data, uint16_t len) {
    imgLogChunks++;
    imgLogLastLen = len;
    imgLogLastHeadLen = (len < sizeof(imgLogLastHead)) ? (uint8_t)len : (uint8_t)sizeof(imgLogLastHead);
    memcpy(imgLogLastHead, data, imgLogLastHeadLen);
    if (imgLogChunks == 1) {
        writeSerial("DW frame 1: " + String(len) + " bytes: " + imgLogHex(imgLogLastHead, imgLogLastHeadLen), true);
        if (len > 0 && imgLogTotalBytes > 0) {
            uint32_t est = (imgLogTotalBytes + len - 1) / len;
            writeSerial("DW expecting ~" + String(est) + " chunks", true);
        }
    }
}

static void imageWriteLogProgress(uint32_t written, uint32_t total) {
    if (total == 0) return;
    uint32_t pct = (uint64_t)written * 100u / total;
    if (pct >= 100) return;                 // completion summary covers 100%
    uint8_t step = (uint8_t)(pct / 5u);
    if (step <= imgLogLastStep) return;
    imgLogLastStep = step;
    writeSerial("DW " + String(pct) + "% (" + String(imgLogChunks) + " chunks, " +
                String(written) + "/" + String(total) + " bytes)", true);
}

static void imageWriteLogFinish(uint32_t written, uint32_t total) {
    writeSerial("DW final frame " + String(imgLogChunks) + ": " + String(imgLogLastLen) +
                " bytes: " + imgLogHex(imgLogLastHead, imgLogLastHeadLen), true);
    uint32_t elapsedMs = millis() - imgLogStartMs;   // unsigned wrap-safe over one stream
    String rate = "n/a";
    if (elapsedMs > 0) rate = String((float)written / 1.024f / (float)elapsedMs, 1);  // bytes/ms /1.024 = KB/s
    writeSerial("DW complete: " + String(imgLogChunks) + " chunks, " +
                String(written) + "/" + String(total) + " bytes, " +
                String(elapsedMs / 1000.0f, 2) + " s, " + rate + " KB/s", true);
}

bool imageWriteLogQuietCmd(void) {
    return (directWriteActive || partialCtx.active) && imgLogChunks >= 1;
}

bool imageWriteLogQuietAck(void) {
    return (directWriteActive || partialCtx.active) && imgLogChunks >= 2;
}

// True when this raw frame is a mid-stream image-write data chunk (command
// header 0x0071, unencrypted) whose per-frame BLE-receive/queue logging should
// be suppressed. Lets the receive callback and queue drain in the other files
// silence their spam without duplicating the stream-state check.
bool imageWriteLogQuietFrame(const uint8_t* data, uint16_t len) {
    return len >= 2 && data[0] == 0x00 && data[1] == 0x71 && imageWriteLogQuietCmd();
}
// ---------------------------------------------------------------------------

void handleDirectWriteCompressedData(uint8_t* data, uint16_t len) {
    if (len > UINT32_MAX - directWriteCompressedReceived) {
        cleanupDirectWriteState(true);
        uint8_t errorResponse[] = {0xFF, 0x71};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    if (!zlib_stream_to_direct_write(data, len, false)) {
        cleanupDirectWriteState(true);
        uint8_t errorResponse[] = {0xFF, 0x71};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    directWriteCompressedReceived += len;
    imageWriteLogProgress(directWriteBytesWritten, directWriteTotalBytes);
    uint8_t ackResponse[] = {0x00, 0x71};
    sendResponse(ackResponse, sizeof(ackResponse));
}

// True when the active display uses the bb_epaper 4-gray scheme (two 1-bit
// controller planes). The Seeed driver path has its own 4bpp handling.
static inline bool directWriteIsGray4(void) {
    return (globalConfig.displays[0].color_scheme == COLOR_SCHEME_GRAY4)
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        && !seeed_driver_used()
#endif
        ;
}

// Two-plane uploads (4-gray and BWR/BWY) arrive as two pre-split, row-padded 1-bit
// controller planes concatenated (plane0 then plane1). 4-gray is already gray-coded
// host-side (py-opendisplay applies the panel's gray LUT, matching bbepSetPixel4Gray:
// plane0 <- stored bit0, plane1 <- stored bit1); BWR/BWY send plane0 = BW (palette
// index 1) then plane1 = accent (index 2). Both share the identical byte layout, so
// stream the bytes to the panel, switching from PLANE_0 to PLANE_1 at the single-plane
// boundary - no on-device de-interleave or 2bpp frame buffer. directWriteBytesWritten
// is the running total across both planes, so the compressed and uncompressed paths
// share this one plane-split implementation.
static void streamGray4Bytes(const uint8_t* buf, uint32_t len) {
    const uint32_t planeBytes = (((uint32_t)directWriteWidth + 7u) / 8u) * directWriteHeight;
    uint32_t off = 0;
    while (off < len && directWriteBytesWritten < 2u * planeBytes) {
        if (directWriteBytesWritten == 0u) {
            bbepSetAddrWindow(&bbep, 0, 0, directWriteWidth, directWriteHeight);
            bbepStartWrite(&bbep, PLANE_0);
        } else if (directWriteBytesWritten == planeBytes) {
            bbepSetAddrWindow(&bbep, 0, 0, directWriteWidth, directWriteHeight);
            bbepStartWrite(&bbep, PLANE_1);
        }
        const uint32_t limit = (directWriteBytesWritten < planeBytes) ? planeBytes : 2u * planeBytes;
        uint32_t take = len - off;
        if (directWriteBytesWritten + take > limit) take = limit - directWriteBytesWritten;
        bbepWriteData(&bbep, (uint8_t*)(buf + off), (int)take);
        off += take;
        directWriteBytesWritten += take;
    }
}

static bool directWriteTouchSuspended = false;

void cleanupDirectWriteState(bool refreshDisplay) {
    directWriteActive = false;
    directWriteCompressed = false;
    directWriteBitplanes = false;
    directWritePlane2 = false;
    directWriteBytesWritten = 0;
    directWriteCompressedReceived = 0;
    directWriteDecompressedTotal = 0;
    directWriteWidth = 0;
    directWriteHeight = 0;
    directWriteTotalBytes = 0;
    directWriteRefreshMode = 0;
    directWriteStartTime = 0;
    if (refreshDisplay && displayPowerState) {
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        if (seeed_driver_used()) {
            seeed_gfx_direct_sleep();
        } else
#endif
        {
            bbepSleep(&bbep, 1);
        }
        delay(200);
    }
    displayPowerState = false;
    pwrmgm(false);
    if (directWriteTouchSuspended) {
        touchResumeAfterEpdRefresh();
        directWriteTouchSuspended = false;
    }
}

void handleDirectWriteStart(uint8_t* data, uint16_t len) {
if (partialCtx.active) cleanup_partial_write_state();
    if (directWriteActive) {
        cleanupDirectWriteState(false);
    }
    imageWriteLogReset();
    touchSuspendForEpdRefresh();
    directWriteTouchSuspended = true;
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        seeed_gfx_prepare_hardware();
    }
#endif
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    directWriteBitplanes = (colorScheme == COLOR_SCHEME_BWR || colorScheme == COLOR_SCHEME_BWY);
    directWritePlane2 = false;
    directWriteCompressed = (len >= 4);
    directWriteWidth = globalConfig.displays[0].pixel_width;
    directWriteHeight = globalConfig.displays[0].pixel_height;
    uint32_t pixels = (uint32_t)directWriteWidth * (uint32_t)directWriteHeight;
    if (directWriteBitplanes) directWriteTotalBytes = 2u * (((uint32_t)directWriteWidth + 7u) / 8u) * directWriteHeight;
    else {
        // Panel RAM is row-padded: each row occupies ceil(w / pixelsPerByte) bytes, and the
        // Python sender row-pads every plane (np.packbits(axis=1)). Size FLAT and we under-count
        // on width-not-divisible-by-8 panels (e.g. 122-wide EP213), auto-completing before the
        // bottom rows are written. Size row-padded to match sender + the gray4/bitplane paths.
        uint32_t w = (uint32_t)directWriteWidth;
        uint32_t h = (uint32_t)directWriteHeight;
        int bitsPerPixel = getBitsPerPixel();
        if (bitsPerPixel == 4) directWriteTotalBytes = ((w + 1u) / 2u) * h;       // ceil(w/2) bytes/row
        else if (bitsPerPixel == 2) directWriteTotalBytes = ((w + 3u) / 4u) * h;  // ceil(w/4) bytes/row
        else directWriteTotalBytes = calc_controller_plane_bytes(directWriteWidth, directWriteHeight); // ceil(w/8) bytes/row
    }
    // 4-gray arrives as two concatenated 1bpp planes (plane0 ++ plane1), streamed to
    // PLANE_0/PLANE_1. Both compressed and uncompressed transports feed bytes through
    // streamGray4Bytes as chunks arrive.
    const bool gray4 = directWriteIsGray4();
    if (gray4) directWriteTotalBytes = 2u * (((uint32_t)directWriteWidth + 7u) / 8u) * directWriteHeight;
    if (directWriteCompressed) {
        memcpy(&directWriteDecompressedTotal, data, 4);
        if (directWriteDecompressedTotal != directWriteTotalBytes) {
            cleanupDirectWriteState(false);
            uint8_t errorResponse[] = {0xFF, 0x70};
            sendResponse(errorResponse, sizeof(errorResponse));
            return;
        }
    }
    directWriteActive = true;
    directWriteBytesWritten = 0;
    directWriteStartTime = millis();
    imageWriteLogStart(directWriteTotalBytes);
    if (displayPowerState) {
        pwrmgm(false);
        delay(50);
    }
    pwrmgm(true);
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        seeed_gfx_direct_write_reset();
    } else
#endif
    {
        bbepInitIO(&bbep, globalConfig.displays[0].dc_pin, globalConfig.displays[0].reset_pin, globalConfig.displays[0].busy_pin, globalConfig.displays[0].cs_pin, globalConfig.displays[0].data_pin, globalConfig.displays[0].clk_pin, 8000000);
        bbepWakeUp(&bbep);
        bbepSendCMDSequence(&bbep, bbep.pInitFull);
        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
        bbepStartWrite(&bbep, directWriteBitplanes ? PLANE_0 : getplane());
    }
    if (directWriteCompressed) {
        od_zlib_stream_reset(directWriteDecompressedTotal);
        if (len > 4) {
            uint32_t compressedDataLen = len - 4;
            if (!zlib_stream_to_direct_write(data + 4, compressedDataLen, false)) {
                cleanupDirectWriteState(false);
                uint8_t errorResponse[] = {0xFF, 0x70};
                sendResponse(errorResponse, sizeof(errorResponse));
                return;
            }
            directWriteCompressedReceived = compressedDataLen;
        }
    }
    uint8_t ackResponse[] = {0x00, 0x70};
    sendResponse(ackResponse, sizeof(ackResponse));
}

void handlePartialWriteStart(uint8_t* data, uint16_t len) {
    if (directWriteActive) cleanupDirectWriteState(false);
    if (partialCtx.active) cleanup_partial_write_state();
    imageWriteLogReset();

    if (len < 17) {
        send_direct_write_nack(0x76, ERR_PARTIAL_STREAM, false);
        return;
    }

    uint8_t flags     = data[0];
    uint32_t oldEtag  = parse_be_u32(data + 1);
    uint32_t newEtag  = parse_be_u32(data + 5);
    uint16_t rectX    = ((uint16_t)data[9]  << 8) | data[10];
    uint16_t rectY    = ((uint16_t)data[11] << 8) | data[12];
    uint16_t rectW    = ((uint16_t)data[13] << 8) | data[14];
    uint16_t rectH    = ((uint16_t)data[15] << 8) | data[16];

    if ((flags & ~PARTIAL_ALLOWED_FLAGS) != 0) {
        send_direct_write_nack(0x76, ERR_PARTIAL_FLAGS, false);
        return;
    }

    if (oldEtag == 0 || oldEtag != displayed_etag || newEtag == 0) {
        send_direct_write_nack(0x76, ERR_ETAG_MISMATCH, false);
        return;
    }

    uint16_t dispW = globalConfig.displays[0].pixel_width;
    uint16_t dispH = globalConfig.displays[0].pixel_height;
    if (getBitsPerPixel() != 1) {
        // bb_epaper partial refresh support is effectively non-existent for
        // 2bpp+ panels, and physical panels may not support that mode either.
        // This protocol uses two 1bpp controller planes as old/new image memory.
        send_direct_write_nack(0x76, ERR_PARTIAL_UNSUPPORTED, false);
        return;
    }

    if (rectW == 0 || rectH == 0 ||
        (uint32_t)rectX + rectW > dispW ||
        (uint32_t)rectY + rectH > dispH) {
        send_direct_write_nack(0x76, ERR_RECT_OOB, false);
        return;
    }

    if ((rectX & 7u) != 0 || (rectW & 7u) != 0) {
        send_direct_write_nack(0x76, ERR_RECT_ALIGN, false);
        return;
    }

    uint32_t planeBytes = calc_controller_plane_bytes(rectW, rectH);
    uint32_t expectedLogicalSize = planeBytes * 2u;

    if (expectedLogicalSize == 0) {
        uint8_t errResponse[] = {0xFF, 0x76};
        sendResponse(errResponse, sizeof(errResponse));
        return;
    }

    memset(&partialCtx, 0, sizeof(partialCtx));
    partialCtx.active = true;
    partialCtx.compressed = (flags & PARTIAL_FLAG_COMPRESSED) != 0;
    partialCtx.flags = flags;
    partialCtx.new_etag = newEtag;
    partialCtx.x = rectX;
    partialCtx.y = rectY;
    partialCtx.width = rectW;
    partialCtx.height = rectH;
    partialCtx.expected_stream_size = expectedLogicalSize;
    partialCtx.plane_size = planeBytes;
    partialCtx.current_plane = 0xFF;
    partialCtx.start_time = millis();
    imageWriteLogStart(expectedLogicalSize);

    partial_prepare_panel_ram();
    if (partialCtx.compressed) od_zlib_stream_reset(expectedLogicalSize);

    // Process optional initial stream bytes before ACK
    if (len > 17) {
        uint16_t initLen = len - 17;
        if (!partial_consume_bytes(data + 17, (uint32_t)initLen)) {
            send_direct_write_nack(0x76, ERR_PARTIAL_STREAM, true);
            return;
        }
    }

    uint8_t ackResponse[] = {0x00, 0x76};
    sendResponse(ackResponse, sizeof(ackResponse));
}

void handleDirectWriteData(uint8_t* data, uint16_t len) {
    if (partialCtx.active) {
        if (len == 0) return;
        imageWriteLogChunk(data, len);
        if (!partial_consume_bytes(data, (uint32_t)len)) {
            send_direct_write_nack(0x71, ERR_PARTIAL_STREAM, true);
            return;
        }
        imageWriteLogProgress(partialCtx.bytes_received, partialCtx.expected_stream_size);
        uint8_t ackResponse[] = {0x00, 0x71};
        sendResponse(ackResponse, sizeof(ackResponse));
        return;
    }
    if (!directWriteActive || len == 0) return;
    imageWriteLogChunk(data, len);
    if (directWriteCompressed) {
        handleDirectWriteCompressedData(data, len);
        return;
    }
    uint32_t remainingBytes = (directWriteBytesWritten < directWriteTotalBytes) ? (directWriteTotalBytes - directWriteBytesWritten) : 0;
    uint16_t bytesToWrite = (len > remainingBytes) ? remainingBytes : len;
    if (bytesToWrite > 0) {
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        if (seeed_driver_used()) {
            seeed_gfx_direct_write_chunk(data, bytesToWrite);
            directWriteBytesWritten += bytesToWrite;
        } else
#endif
        if (directWriteIsGray4() || directWriteBitplanes) {
            streamGray4Bytes(data, bytesToWrite);  // advances directWriteBytesWritten, splits planes
        } else {
            bbepWriteData(&bbep, data, bytesToWrite);
            directWriteBytesWritten += bytesToWrite;
        }
    }
    imageWriteLogProgress(directWriteBytesWritten, directWriteTotalBytes);
    if (directWriteBytesWritten >= directWriteTotalBytes) {
        handleDirectWriteEnd(nullptr, 0);
    } else {
        uint8_t ackResponse[] = {0x00, 0x71};
        sendResponse(ackResponse, sizeof(ackResponse));
    }
}

void handleDirectWriteEnd(uint8_t* data, uint16_t len) {
    if (partialCtx.active) {
        if (data != nullptr && len > 1) {
            send_direct_write_nack(0x72, ERR_PARTIAL_STREAM, true);
            return;
        }
        if (partialCtx.compressed) {
            if (partialCtx.bytes_received == 0 || !zlib_stream_to_partial_write(nullptr, 0, true)) {
                send_direct_write_nack(0x72, ERR_PARTIAL_STREAM, true);
                return;
            }
        } else if (partialCtx.bytes_written != partialCtx.expected_stream_size) {
            send_direct_write_nack(0x72, ERR_PARTIAL_STREAM, true);
            return;
        }
        imageWriteLogFinish(partialCtx.bytes_received, partialCtx.expected_stream_size);
        uint8_t ackResponse[] = {0x00, 0x72};
        sendResponse(ackResponse, sizeof(ackResponse));
        int refreshMode = REFRESH_PARTIAL;
        if (data != nullptr && len >= 1 && data[0] == REFRESH_FULL) refreshMode = REFRESH_FULL;
        else if (data != nullptr && len >= 1 && data[0] == REFRESH_FAST) refreshMode = REFRESH_FAST;
        bool refreshSuccess = partial_write_to_panel(refreshMode);
        if (refreshSuccess) {
            displayed_etag = partialCtx.new_etag;
            uint8_t validatedResponse[] = {0x00, 0x73};
            sendResponse(validatedResponse, sizeof(validatedResponse));
        } else {
            displayed_etag = 0;
            uint8_t timeoutResponse[] = {0x00, 0x74};
            sendResponse(timeoutResponse, sizeof(timeoutResponse));
        }
        cleanup_partial_write_state();
        return;
    }
    if (!directWriteActive) return;
    directWriteStartTime = 0;
    if (directWriteCompressed && !zlib_stream_to_direct_write(nullptr, 0, true)) {
        cleanupDirectWriteState(true);
        uint8_t errorResponse[] = {0xFF, 0x72};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    const bool gray4 = directWriteIsGray4();
    if (gray4 || directWriteBitplanes) {
        // Both planes must be present before refresh. Compressed and uncompressed
        // paths stream live as 0x71 chunks, so confirm the full two-plane payload
        // arrived before refreshing stale RAM or committing an etag.
        if (directWriteBytesWritten != directWriteTotalBytes) {
            cleanupDirectWriteState(false);
            uint8_t errorResponse[] = {0xFF, 0x72};
            sendResponse(errorResponse, sizeof(errorResponse));
            return;
        }
    }
    imageWriteLogFinish(directWriteBytesWritten, directWriteTotalBytes);
    int refreshMode = REFRESH_FULL;
    if (data != nullptr && len >= 1 && data[0] == 1) refreshMode = REFRESH_FAST;
    writeSerial("EPD refresh: ", false);
    writeSerial(refreshMode == REFRESH_FAST ? "FAST" : "FULL", false);
    writeSerial(" (mode=", false);
    writeSerial(String(refreshMode), false);
    writeSerial(", end payload ", false);
    if (data != nullptr && len > 0) {
        writeSerial("0x", false);
        writeSerial(String(data[0], HEX), false);
    } else {
        writeSerial("none (auto)", false);
    }
    writeSerial(")", true);
    uint8_t ackResponse[] = {0x00, 0x72};
    sendResponse(ackResponse, sizeof(ackResponse));
    delay(20);
    epdRefreshInProgress = true;
    bool refreshSuccess = false;
    uint32_t newEtag = 0;
    bool hasNewEtag = data != nullptr && len >= 5;
    if (hasNewEtag) newEtag = parse_be_u32(data + 1);
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        seeed_gfx_direct_refresh(refreshMode);
        refreshSuccess = waitforrefresh(60);
        seeed_gfx_direct_sleep();
    } else
#endif
    {
        bbepRefresh(&bbep, refreshMode);
        refreshSuccess = waitforrefresh(60);
        bbepSleep(&bbep, 1);
    }
    epdRefreshInProgress = false;
    delay(50);
    cleanupDirectWriteState(false);
#ifdef TARGET_ESP32
    esp32_restart_ble_advertising();
#endif
    if (refreshSuccess) {
        // A successful refresh changed the panel image. Commit the new etag
        // when the client supplied a valid one; otherwise clear the stale etag
        // (etag-less full upload / auto-complete) so a later partial update
        // gets a clean ETAG mismatch and falls back to a full upload instead
        // of diffing against the wrong, now-outdated base image.
        if (hasNewEtag && newEtag != 0) displayed_etag = newEtag;
        else displayed_etag = 0;
        uint8_t refreshResponse[] = {0x00, 0x73};
        sendResponse(refreshResponse, sizeof(refreshResponse));
    } else {
        if (hasNewEtag) displayed_etag = 0;
        uint8_t timeoutResponse[] = {0x00, 0x74};
        sendResponse(timeoutResponse, sizeof(timeoutResponse));
    }
}

static void cleanup_partial_write_state(void) {
    bool powerDown = partialCtx.active && displayPowerState;
    memset(&partialCtx, 0, sizeof(partialCtx));
    if (powerDown) {
        bbepSleep(&bbep, 1);
        delay(200);
        displayPowerState = false;
        pwrmgm(false);
    }
}

static bool panel_skips_bbep_set_addr_window(void) {
    return bbep.type == EP397_800x480 || bbep.type == EP397_800x480_4GRAY ||
           bbep.type == EP426_800x480 || bbep.type == EP426_800x480_4GRAY;
}

static bool panel_uses_pixel_ram_x(BBEPDISP *pBBEP) {
    return pBBEP->type == EP397_800x480 || pBBEP->type == EP397_800x480_4GRAY ||
           pBBEP->type == EP426_800x480 || pBBEP->type == EP426_800x480_4GRAY;
}

static bool panel_uses_ep397_y_decrement(BBEPDISP *pBBEP) {
    return pBBEP->type == EP397_800x480 || pBBEP->type == EP397_800x480_4GRAY;
}

static bool panel_uses_ep426_x_decrement(BBEPDISP *pBBEP) {
    return pBBEP->type == EP426_800x480 || pBBEP->type == EP426_800x480_4GRAY;
}

static bool panel_skips_reinit_on_partial_refresh(BBEPDISP *pBBEP) {
    return panel_uses_ep397_y_decrement(pBBEP) || panel_uses_ep426_x_decrement(pBBEP);
}

static void partial_set_ep397_ram_y(BBEPDISP *pBBEP, int ty, int cy) {
    uint8_t uc[4];
    int yLast = ty + cy - 1;
    int ramYStart = (pBBEP->native_height - 1) - ty;
    int ramYEnd = (pBBEP->native_height - 1) - yLast;

    bbepWriteCmd(pBBEP, SSD1608_SET_RAMYPOS);
    uc[0] = (uint8_t)(ramYStart & 0xff);
    uc[1] = (uint8_t)(ramYStart >> 8);
    uc[2] = (uint8_t)(ramYEnd & 0xff);
    uc[3] = (uint8_t)(ramYEnd >> 8);
    bbepWriteData(pBBEP, uc, 4);

    bbepWriteCmd(pBBEP, SSD1608_SET_RAMYCOUNT);
    uc[0] = (uint8_t)(ramYStart & 0xff);
    uc[1] = (uint8_t)(ramYStart >> 8);
    bbepWriteData(pBBEP, uc, 2);
}

static void partial_set_ep426_ram_y(BBEPDISP *pBBEP, int ty, int cy) {
    uint8_t uc[4];
    int yLast = ty + cy - 1;

    // Match epd426_init_* 0x45 wire order: Y start in bytes 0-1, Y end in bytes 2-3.
    bbepWriteCmd(pBBEP, SSD1608_SET_RAMYPOS);
    uc[0] = (uint8_t)ty;
    uc[1] = (uint8_t)(ty >> 8);
    uc[2] = (uint8_t)yLast;
    uc[3] = (uint8_t)(yLast >> 8);
    bbepWriteData(pBBEP, uc, 4);

    bbepWriteCmd(pBBEP, SSD1608_SET_RAMYCOUNT);
    uc[0] = (uint8_t)ty;
    uc[1] = (uint8_t)(ty >> 8);
    bbepWriteData(pBBEP, uc, 2);
}

static void partial_set_pixel_ram_x(BBEPDISP *pBBEP, int x, int cx) {
    uint8_t uc[4];
    int px0 = x;
    int px1 = x + cx - 1;
    if (panel_uses_ep426_x_decrement(pBBEP)) {
        px0 = (pBBEP->native_width - 1) - x;
        px1 = (pBBEP->native_width - 1) - (x + cx - 1);
    }

    bbepWriteCmd(pBBEP, SSD1608_SET_RAMXPOS);
    uc[0] = (uint8_t)(px0 & 0xff);
    uc[1] = (uint8_t)((px0 >> 8) & 0xff);
    uc[2] = (uint8_t)(px1 & 0xff);
    uc[3] = (uint8_t)(px1 >> 8);
    bbepWriteData(pBBEP, uc, 4);

    bbepWriteCmd(pBBEP, SSD1608_SET_RAMXCOUNT);
    uc[0] = (uint8_t)(px0 & 0xff);
    uc[1] = (uint8_t)(px0 >> 8);
    bbepWriteData(pBBEP, uc, 2);
}

static void partial_set_addr_window(BBEPDISP *pBBEP, int x, int y, int cx, int cy) {
    if (!panel_skips_bbep_set_addr_window()) {
        bbepSetAddrWindow(pBBEP, x, y, cx, cy);
        return;
    }
    if (!pBBEP) return;

    uint8_t uc[4];
    int ty = y;
    cx = (cx + 7) & 0xfff8;

    if (panel_uses_pixel_ram_x(pBBEP)) {
        partial_set_pixel_ram_x(pBBEP, x, cx);
    } else {
        int tx = x / 8;
        bbepWriteCmd(pBBEP, SSD1608_SET_RAMXPOS);
        uc[0] = (uint8_t)tx;
        uc[1] = (uint8_t)(tx + ((cx - 1) >> 3));
        bbepWriteData(pBBEP, uc, 2);
        bbepCMD2(pBBEP, SSD1608_SET_RAMXCOUNT, (uint8_t)tx);
    }

    if (panel_uses_ep426_x_decrement(pBBEP)) {
        partial_set_ep426_ram_y(pBBEP, ty, cy);
    } else if (panel_uses_ep397_y_decrement(pBBEP)) {
        partial_set_ep397_ram_y(pBBEP, ty, cy);
    } else {
        bbepWriteCmd(pBBEP, SSD1608_SET_RAMYPOS);
        uc[0] = (uint8_t)ty;
        uc[1] = (uint8_t)(ty >> 8);
        uc[2] = (uint8_t)(ty + cy - 1);
        uc[3] = (uint8_t)((ty + cy - 1) >> 8);
        bbepWriteData(pBBEP, uc, 4);
        uc[0] = (uint8_t)ty;
        uc[1] = (uint8_t)(ty >> 8);
        bbepWriteCmd(pBBEP, SSD1608_SET_RAMYCOUNT);
        bbepWriteData(pBBEP, uc, 2);
    }
    bbepWaitBusy(pBBEP);
}

static bool partial_consume_bytes(uint8_t* data, uint32_t len) {
    if (partialCtx.compressed) {
        if (len > UINT32_MAX - partialCtx.bytes_received) return false;
    } else {
        if (partialCtx.bytes_received > partialCtx.expected_stream_size ||
            len > partialCtx.expected_stream_size - partialCtx.bytes_received) {
            return false;
        }
    }
    partialCtx.bytes_received += len;
    if (partialCtx.compressed) return zlib_stream_to_partial_write(data, len, false);
    return partial_write_stream_bytes(data, len);
}

static bool zlib_stream_to_direct_write(const uint8_t* data, uint32_t len, bool final) {
    od_zlib_status_t status = od_zlib_stream_push(data, len, final);
    if (status == OD_ZLIB_STATUS_ERROR) {
        writeSerial(String("zlib stream error: ") + od_zlib_stream_error(), true);
        return false;
    }

    for (;;) {
        size_t bytesOut = 0;
        status = od_zlib_stream_poll(decompressionChunk, OPENDISPLAY_DECOMPRESSION_CHUNK_SIZE, &bytesOut);
        if (bytesOut > 0) {
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
            if (seeed_driver_used()) {
                seeed_gfx_direct_write_chunk(decompressionChunk, (uint32_t)bytesOut);
                directWriteBytesWritten += (uint32_t)bytesOut;
            } else
#endif
            if (directWriteIsGray4() || directWriteBitplanes) {
                uint32_t before = directWriteBytesWritten;
                streamGray4Bytes(decompressionChunk, (uint32_t)bytesOut);
                if (directWriteBytesWritten - before != (uint32_t)bytesOut) {
                    return false;
                }
            } else
            {
                bbepWriteData(&bbep, decompressionChunk, bytesOut);
                directWriteBytesWritten += (uint32_t)bytesOut;
            }
            if (directWriteBytesWritten > directWriteDecompressedTotal) {
                return false;
            }
        }
        if (status == OD_ZLIB_STATUS_OUTPUT_READY) continue;
        if (status == OD_ZLIB_STATUS_NEEDS_INPUT) return !final;
        if (status == OD_ZLIB_STATUS_DONE) {
            if (directWriteBytesWritten != directWriteDecompressedTotal) {
                return false;
            }
            return true;
        }
        writeSerial(String("zlib stream error: ") + od_zlib_stream_error(), true);
        return false;
    }
}

static bool zlib_stream_to_partial_write(const uint8_t* data, uint32_t len, bool final) {
    od_zlib_status_t status = od_zlib_stream_push(data, len, final);
    if (status == OD_ZLIB_STATUS_ERROR) {
        writeSerial(String("partial zlib stream error: ") + od_zlib_stream_error(), true);
        return false;
    }

    for (;;) {
        size_t bytesOut = 0;
        status = od_zlib_stream_poll(decompressionChunk, OPENDISPLAY_DECOMPRESSION_CHUNK_SIZE, &bytesOut);
        if (bytesOut > 0 && !partial_write_stream_bytes(decompressionChunk, (uint32_t)bytesOut)) return false;
        if (status == OD_ZLIB_STATUS_OUTPUT_READY) continue;
        if (status == OD_ZLIB_STATUS_NEEDS_INPUT) return !final;
        if (status == OD_ZLIB_STATUS_DONE) return partialCtx.bytes_written == partialCtx.expected_stream_size;
        writeSerial(String("partial zlib stream error: ") + od_zlib_stream_error(), true);
        return false;
    }
}

static bool partial_write_stream_bytes(uint8_t* data, uint32_t len) {
    uint32_t offset = 0;
    while (offset < len) {
        if (partialCtx.bytes_written >= partialCtx.expected_stream_size) return false;

        uint8_t targetPlane = partialCtx.bytes_written < partialCtx.plane_size ? PLANE_1 : PLANE_0;
        if (partialCtx.current_plane != targetPlane) {
            if (targetPlane == PLANE_0 && partialCtx.bytes_written != partialCtx.plane_size) return false;
            partial_set_addr_window(&bbep, partialCtx.x, partialCtx.y, partialCtx.width, partialCtx.height);
            bbepStartWrite(&bbep, targetPlane);
            partialCtx.current_plane = targetPlane;
        }

        uint32_t planeEnd = targetPlane == PLANE_1 ? partialCtx.plane_size : partialCtx.expected_stream_size;
        uint32_t chunk = planeEnd - partialCtx.bytes_written;
        if (chunk > len - offset) chunk = len - offset;
        bbepWriteData(&bbep, data + offset, (int)chunk);
        partialCtx.bytes_written += chunk;
        offset += chunk;
    }
    return true;
}

static bool partial_trigger_refresh(int refreshMode) {
    if (refreshMode < 0 || refreshMode > 3) refreshMode = REFRESH_PARTIAL;
    if (panel_skips_reinit_on_partial_refresh(&bbep)) {
        if (panel_uses_ep397_y_decrement(&bbep)) {
            static const uint8_t u8CMDz3[4] = {0xf7, 0xd7, 0xff, 0};
            bbepCMD2(&bbep, SSD1608_DISP_CTRL2, u8CMDz3[refreshMode]);
        } else {
            static const uint8_t u8CMD[4] = {0xf7, 0xc7, 0xff, 0xc0};
            bbepCMD2(&bbep, SSD1608_DISP_CTRL2, u8CMD[refreshMode]);
        }
        bbepWriteCmd(&bbep, SSD1608_MASTER_ACTIVATE);
        return waitforrefresh(60);
    }
    bbepRefresh(&bbep, refreshMode);
    return waitforrefresh(60);
}

static void partial_prepare_panel_ram(void) {
    writeSerial("EPD partial start: auto-fill panel RAM", true);
    if (!displayPowerState) {
        pwrmgm(true);
    }
    bbepInitIO(&bbep, globalConfig.displays[0].dc_pin, globalConfig.displays[0].reset_pin, globalConfig.displays[0].busy_pin, globalConfig.displays[0].cs_pin, globalConfig.displays[0].data_pin, globalConfig.displays[0].clk_pin, 8000000);
    bbepWakeUp(&bbep);
    const uint8_t* initSeq = bbep.pInitPart ? bbep.pInitPart : bbep.pInitFull;
    bbepSendCMDSequence(&bbep, initSeq);
    bbepFill(&bbep, BBEP_WHITE, PLANE_1);
    bbepFill(&bbep, BBEP_WHITE, PLANE_0);
}

static bool partial_write_to_panel(int refreshMode) {
    writeSerial("EPD refresh: PARTIAL (raw rect ", false);
    writeSerial(String(partialCtx.x), false);
    writeSerial(",", false);
    writeSerial(String(partialCtx.y), false);
    writeSerial(" ", false);
    writeSerial(String(partialCtx.width), false);
    writeSerial("x", false);
    writeSerial(String(partialCtx.height), false);
    writeSerial(")", true);

    if (partialCtx.bytes_written != partialCtx.expected_stream_size) return false;
    delay(20);
    epdRefreshInProgress = true;
    bool refreshSuccess = partial_trigger_refresh(refreshMode);
    epdRefreshInProgress = false;
    bbepSleep(&bbep, 1);
    delay(50);
    displayPowerState = false;
    pwrmgm(false);
    return refreshSuccess;
}

static uint32_t calc_controller_plane_bytes(uint16_t width, uint16_t height) {
    return ((uint32_t)(width + 7u) / 8u) * height;
}

static uint32_t parse_be_u32(const uint8_t* data) {
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8)  |  (uint32_t)data[3];
}

static void send_direct_write_nack(uint8_t opcode, uint8_t error, bool cleanupState) {
    displayed_etag = 0;
    if (cleanupState) {
        if (partialCtx.active) cleanup_partial_write_state();
        else cleanupDirectWriteState(false);
    }
    uint8_t errResponse[] = {0xFF, opcode, error, 0x00};
    sendResponse(errResponse, sizeof(errResponse));
}
