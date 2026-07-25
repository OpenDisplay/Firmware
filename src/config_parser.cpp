#include "config_parser.h"
#include "factory_config.h"
#include "structs.h"
#include "od_log.h"
#include "encryption_state.h"
#include "encryption.h"
#include "power_latch.h"
#include "wifi_service.h"  // OPENDISPLAY_HAS_WIFI + lanActivePort()/lanTlsEnabled()
#include <Arduino.h>
#include <string.h>

#ifdef TARGET_NRF
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#endif
#ifdef TARGET_ESP32
#include <LittleFS.h>
#include <WiFi.h>
#endif

#ifndef COMM_MODE_BLE
#define COMM_MODE_BLE (1 << 0)
#define COMM_MODE_OEPL (1 << 1)
#define COMM_MODE_WIFI (1 << 2)
#endif
#ifndef DEVICE_FLAG_PWR_PIN
#define DEVICE_FLAG_PWR_PIN (1 << 0)
#define DEVICE_FLAG_XIAOINIT (1 << 1)
#define DEVICE_FLAG_WS_PP_INIT (1 << 2)
#define DEVICE_FLAG_BATTERY_LATCH (1 << 3)
#define DEVICE_FLAG_PWR_LATCH_DFF (1 << 4)
#endif

// The parse-time dumps and printConfigSummary() are od_log_debug, which the
// default OD_LOG_LEVEL (INFO) compiles out entirely -- so they no longer cost
// serial time in the deep-sleep wake window and need no runtime quiet flag.

extern struct GlobalConfig globalConfig;
extern uint8_t activeLedInstance;
extern char wifiSsid[33];
extern char wifiPassword[33];
extern uint8_t wifiEncryptionType;
extern bool wifiConfigured;
#ifdef TARGET_ESP32
extern char wifiServerUrl[65];
extern uint16_t wifiServerPort;
// extern bool wifiServerConfigured;  // dead -- see the 0x26 wifi_config parse
extern bool wifiConnected;
extern bool wifiInitialized;
#endif

void xiaoinit();
void powerDownExternalFlashFromConfig(void);
void ws_pp_init();
extern bool encryptionInitialized;

bool initConfigStorage(){
    #ifdef TARGET_NRF
    if (!InternalFS.begin()) {
        od_log_error("ERROR: Failed to mount internal file system");
        return false;
    }
    return true;
    #endif
    #ifdef TARGET_ESP32
    if (!LittleFS.begin(true)) { // true = format on failure
        od_log_error("ERROR: Failed to mount LittleFS");
        return false;
    }
    return true;
    #endif
    return false; // Should never reach here
}

void formatConfigStorage(){
    #ifdef TARGET_NRF
    InternalFS.format();
    #endif
    #ifdef TARGET_ESP32
    LittleFS.format();
    #endif
}

// See getConfigScratch() in config_parser.h for the sharing contract. Replaces a
// per-consumer 4 KB buffer each in loadGlobalConfig (static), hasValidStoredConfig
// (static) and handleReadConfig (stack -- a 4 KB spike on the loop task).
static uint8_t configScratch[MAX_CONFIG_SIZE];

uint8_t* getConfigScratch(void) {
    return configScratch;
}

bool saveConfig(uint8_t* configData, uint32_t len){
    if (len > MAX_CONFIG_SIZE) {
        od_log_error("ERROR: Config data too large (%u bytes)", (unsigned)len);
        return false;
    }
    if (configData == nullptr) {
        return false;
    }
    // Header on the stack; the payload is written straight from the caller's
    // buffer. Two writes produce the same bytes the old single write did, and
    // the factory-provisioning caller passes a flash pointer, so this also drops
    // a 4 KB flash->RAM copy at first boot.
    config_header_t header;
    header.magic = CONFIG_STORAGE_MAGIC;
    header.version = CONFIG_STORAGE_VERSION;
    header.crc = calculateConfigCRC(configData, len);
    header.data_len = len;
    #ifdef TARGET_NRF
    if (InternalFS.exists(CONFIG_FILE_PATH)) {
        InternalFS.remove(CONFIG_FILE_PATH);
    }
    File file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_WRITE);
    #elif defined(TARGET_ESP32)
    if (LittleFS.exists(CONFIG_FILE_PATH)) {
        LittleFS.remove(CONFIG_FILE_PATH);
    }
    File file = LittleFS.open(CONFIG_FILE_PATH, FILE_WRITE);
    #endif
    if (!file) {
        od_log_error("ERROR: Failed to open config file for writing");
        #ifdef TARGET_NRF
        file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_WRITE);
        #elif defined(TARGET_ESP32)
        file = LittleFS.open(CONFIG_FILE_PATH, FILE_WRITE);
        #endif
        if (!file) {
            od_log_error("ERROR: Failed to open config file for writing with CREATE|WRITE");
        return false;
        }
    }
    const size_t totalSize = sizeof(config_header_t) + len;
    size_t bytesWritten = file.write((uint8_t*)&header, sizeof(header));
    if (bytesWritten == sizeof(header) && len > 0) {
        bytesWritten += file.write(configData, len);
    }
    file.close();
    if (bytesWritten != totalSize) {
        od_log_error("ERROR: Failed to write complete config data (expected %u, wrote %u)", (unsigned)totalSize, (unsigned)bytesWritten);
        return false;
    }
    return true;
}

bool clearStoredConfig(void) {
    #ifdef TARGET_NRF
    if (InternalFS.exists(CONFIG_FILE_PATH)) {
        if (!InternalFS.remove(CONFIG_FILE_PATH)) {
            od_log_error("ERROR: Failed to remove config file");
            return false;
        }
    }
    #elif defined(TARGET_ESP32)
    if (LittleFS.exists(CONFIG_FILE_PATH)) {
        if (!LittleFS.remove(CONFIG_FILE_PATH)) {
            od_log_error("ERROR: Failed to remove config file");
            return false;
        }
    }
    #endif
    memset(&globalConfig, 0, sizeof(globalConfig));
    memset(&securityConfig, 0, sizeof(securityConfig));
    wifiConfigured = false;
    wifiSsid[0] = '\0';
    wifiPassword[0] = '\0';
    wifiEncryptionType = 0;
    return true;
}

bool loadConfig(uint8_t* configData, uint32_t* len){
    #ifdef TARGET_NRF
    File file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_READ);
    #elif defined(TARGET_ESP32)
    File file = LittleFS.open(CONFIG_FILE_PATH, FILE_READ);
    #endif
    if (!file) {
        return false;
    }
    if (configData == nullptr || len == nullptr) {
        file.close();
        return false;
    }
    // Header staged on the stack, payload read straight into the caller's buffer.
    // On any failure below configData may hold unvalidated bytes; every caller
    // gates on the return value, so it is never read after a false return.
    config_header_t header;
    size_t bytesRead = file.read((uint8_t*)&header, sizeof(header));
    if (bytesRead != sizeof(header)) {
        od_log_error("ERROR: Failed to read config header (expected %u, got %u)", (unsigned)sizeof(header), (unsigned)bytesRead);
        file.close();
        return false;
    }
    if (header.magic != CONFIG_STORAGE_MAGIC) {
        od_log_error("ERROR: Invalid config magic number");
        file.close();
        return false;
    }
    if (header.data_len > MAX_CONFIG_SIZE) {
        od_log_error("ERROR: Config data too large");
        file.close();
        return false;
    }
    // Capacity is now checked BEFORE the read rather than after staging: the
    // read target is the caller's buffer, so this bound is what keeps a large
    // (but in-spec) config from overrunning a smaller caller buffer.
    if (header.data_len > *len) {
        od_log_error("ERROR: Config data larger than buffer");
        file.close();
        return false;
    }
    bytesRead = file.read(configData, header.data_len);
    file.close();
    if (bytesRead != header.data_len) {
        od_log_error("ERROR: Failed to read complete config data (expected %u, read %u)", (unsigned)header.data_len, (unsigned)bytesRead);
        return false;
    }
    uint32_t calculatedCRC = calculateConfigCRC(configData, header.data_len);
    if (header.crc != calculatedCRC) {
        od_log_error("ERROR: Config CRC mismatch");
        return false;
    }
    *len = header.data_len;
    return true;
}

bool hasValidStoredConfig(void) {
#ifdef TARGET_NRF
    if (!InternalFS.exists(CONFIG_FILE_PATH)) {
        return false;
    }
#elif defined(TARGET_ESP32)
    if (!LittleFS.exists(CONFIG_FILE_PATH)) {
        return false;
    }
#endif
    uint32_t len = MAX_CONFIG_SIZE;
    return loadConfig(getConfigScratch(), &len);
}

static uint16_t crc16_ccitt_feed(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)((uint16_t)b << 8);
    for (int j = 0; j < 8; j++) {
        if ((crc & 0x8000U) != 0U) {
            crc = (uint16_t)(((uint32_t)crc << 1) ^ 0x1021U);
        } else {
            crc = (uint16_t)((uint32_t)crc << 1);
        }
    }
    return crc;
}

// CRC-16/CCITT-FALSE over the toolbox-outer config container body (excludes the
// trailing 2 CRC bytes), with the first two (length) bytes forced to zero so the
// value is independent of the container length. Matches the canonical helper in
// OpenDisplay-Firmware_NRF/config_parser.c and OpenDisplay-Firmware_Silabs.
static uint16_t config_toolbox_outer_crc16(const uint8_t* data, uint32_t body_len) {
    uint16_t crc = 0xFFFFU;
    if (body_len < 2U) {
        for (uint32_t i = 0; i < body_len; i++) {
            crc = crc16_ccitt_feed(crc, data[i]);
        }
        return crc;
    }
    crc = crc16_ccitt_feed(crc, 0);
    crc = crc16_ccitt_feed(crc, 0);
    for (uint32_t i = 2U; i < body_len; i++) {
        crc = crc16_ccitt_feed(crc, data[i]);
    }
    return crc;
}

uint32_t calculateConfigCRC(uint8_t* data, uint32_t len){
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return ~crc;
}

bool loadGlobalConfig(){
    memset(&globalConfig, 0, sizeof(globalConfig));
    // Initialize security config defaults
    memset(&securityConfig, 0, sizeof(securityConfig));
    // Reset pin defaults to disabled (flag not set)
    wifiConfigured = false;
    wifiSsid[0] = '\0';
    wifiPassword[0] = '\0';
    wifiEncryptionType = 0;
    globalConfig.data_extended_loaded = false;
    uint8_t* configData = getConfigScratch();
    uint32_t configLen = MAX_CONFIG_SIZE;
    if (!loadConfig(configData, &configLen)) {
        globalConfig.loaded = false;
        return false;
    }
    if (configLen < 3) {
        od_log_error("ERROR: Config too short");
        globalConfig.loaded = false;
        return false;
    }
    uint32_t offset = 0;
    offset += 2;
    globalConfig.version = configData[offset++];
    globalConfig.minor_version = 0; // Not stored in current format
    while (offset < configLen - 2) { // -2 for CRC
        if (offset + 2 > configLen - 2) break;
        offset++;
        uint8_t packetId = configData[offset++];
        switch (packetId) {
            case 0x01: // system_config
                if (offset + sizeof(struct SystemConfig) <= configLen - 2) {
                    memcpy(&globalConfig.system_config, &configData[offset], sizeof(struct SystemConfig));
                    offset += sizeof(struct SystemConfig);
                } else {
                    od_log_error("ERROR: Not enough data for system_config");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x02: // manufacturer_data
                if (offset + sizeof(struct ManufacturerData) <= configLen - 2) {
                    memcpy(&globalConfig.manufacturer_data, &configData[offset], sizeof(struct ManufacturerData));
                    offset += sizeof(struct ManufacturerData);
                } else {
                    od_log_error("ERROR: Not enough data for manufacturer_data");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x04: // power_option
                if (offset + sizeof(struct PowerOption) <= configLen - 2) {
                    memcpy(&globalConfig.power_option, &configData[offset], sizeof(struct PowerOption));
                    offset += sizeof(struct PowerOption);
                } else {
                    od_log_error("ERROR: Not enough data for power_option");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x20: // display
                if (globalConfig.display_count < 4 && offset + sizeof(struct DisplayConfig) <= configLen - 2) {
                    memcpy(&globalConfig.displays[globalConfig.display_count], &configData[offset], sizeof(struct DisplayConfig));
                    offset += sizeof(struct DisplayConfig);
                    globalConfig.display_count++;
                } else if (globalConfig.display_count >= 4) {
                    od_log_warn("WARNING: Maximum display count reached, skipping");
                    offset += sizeof(struct DisplayConfig);
                } else {
                    od_log_error("ERROR: Not enough data for display");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x21: // led
                if (globalConfig.led_count < 4 && offset + sizeof(struct LedConfig) <= configLen - 2) {
                    memcpy(&globalConfig.leds[globalConfig.led_count], &configData[offset], sizeof(struct LedConfig));
                    offset += sizeof(struct LedConfig);
                    globalConfig.led_count++;
                    // Reset active LED instance to re-detect RGB LEDs after config change
                    activeLedInstance = 0xFF;
                } else if (globalConfig.led_count >= 4) {
                    od_log_warn("WARNING: Maximum LED count reached, skipping");
                    offset += sizeof(struct LedConfig);
                } else {
                    od_log_error("ERROR: Not enough data for LED");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x23: // sensor_data
                if (globalConfig.sensor_count < 4 && offset + sizeof(struct SensorData) <= configLen - 2) {
                    memcpy(&globalConfig.sensors[globalConfig.sensor_count], &configData[offset], sizeof(struct SensorData));
                    offset += sizeof(struct SensorData);
                    globalConfig.sensor_count++;
                } else if (globalConfig.sensor_count >= 4) {
                    od_log_warn("WARNING: Maximum sensor count reached, skipping");
                    offset += sizeof(struct SensorData);
                } else {
                    od_log_error("ERROR: Not enough data for sensor");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x24: // data_bus
                if (globalConfig.data_bus_count < 4 && offset + sizeof(struct DataBus) <= configLen - 2) {
                    memcpy(&globalConfig.data_buses[globalConfig.data_bus_count], &configData[offset], sizeof(struct DataBus));
                    offset += sizeof(struct DataBus);
                    globalConfig.data_bus_count++;
                } else if (globalConfig.data_bus_count >= 4) {
                    od_log_warn("WARNING: Maximum data_bus count reached, skipping");
                    offset += sizeof(struct DataBus);
                } else {
                    od_log_error("ERROR: Not enough data for data_bus");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x25: // binary_inputs
                if (globalConfig.binary_input_count < 4 && offset + sizeof(struct BinaryInputs) <= configLen - 2) {
                    memcpy(&globalConfig.binary_inputs[globalConfig.binary_input_count], &configData[offset], sizeof(struct BinaryInputs));
                    offset += sizeof(struct BinaryInputs);
                    globalConfig.binary_input_count++;
                } else if (globalConfig.binary_input_count >= 4) {
                    od_log_warn("WARNING: Maximum binary_input count reached, skipping");
                    offset += sizeof(struct BinaryInputs);
                } else {
                    od_log_error("ERROR: Not enough data for binary_input");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x28: // touch_controller
                if (globalConfig.touch_controller_count < 4 && offset + sizeof(struct TouchController) <= configLen - 2) {
                    memcpy(&globalConfig.touch_controllers[globalConfig.touch_controller_count], &configData[offset], sizeof(struct TouchController));
                    offset += sizeof(struct TouchController);
                    globalConfig.touch_controller_count++;
                } else if (globalConfig.touch_controller_count >= 4) {
                    od_log_warn("WARNING: Maximum touch_controller count reached, skipping");
                    offset += sizeof(struct TouchController);
                } else {
                    od_log_error("ERROR: Not enough data for touch_controller");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x29: // passive_buzzer
                if (globalConfig.passive_buzzer_count < 4 && offset + sizeof(struct BuzzerConfig) <= configLen - 2) {
                    memcpy(&globalConfig.passive_buzzers[globalConfig.passive_buzzer_count], &configData[offset], sizeof(struct BuzzerConfig));
                    offset += sizeof(struct BuzzerConfig);
                    globalConfig.passive_buzzer_count++;
                } else if (globalConfig.passive_buzzer_count >= 4) {
                    od_log_warn("WARNING: Maximum passive_buzzer count reached, skipping");
                    offset += sizeof(struct BuzzerConfig);
                } else {
                    od_log_error("ERROR: Not enough data for passive_buzzer");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x2C: // data_extended
                if (offset + sizeof(struct DataExtended) <= configLen - 2) {
                    memcpy(&globalConfig.data_extended, &configData[offset], sizeof(struct DataExtended));
                    offset += sizeof(struct DataExtended);
                    globalConfig.data_extended.manufacturer_name[31] = '\0';
                    globalConfig.data_extended.model_name[31] = '\0';
                    globalConfig.data_extended.serial_number[31] = '\0';
                    globalConfig.data_extended.friendly_name[31] = '\0';
                    globalConfig.data_extended.device_location[31] = '\0';
                    globalConfig.data_extended.device_id[31] = '\0';
                    globalConfig.data_extended.custom_string_1[31] = '\0';
                    globalConfig.data_extended.custom_string_2[31] = '\0';
                    globalConfig.data_extended.custom_string_3[31] = '\0';
                    globalConfig.data_extended_loaded = true;
                } else {
                    od_log_error("ERROR: Not enough data for data_extended");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x2B: // flash_config
                if (globalConfig.flash_config_count < 2 && offset + sizeof(struct FlashConfig) <= configLen - 2) {
                    memcpy(&globalConfig.flash_configs[globalConfig.flash_config_count], &configData[offset], sizeof(struct FlashConfig));
                    offset += sizeof(struct FlashConfig);
                    globalConfig.flash_config_count++;
                } else if (globalConfig.flash_config_count >= 2) {
                    od_log_warn("WARNING: Maximum flash_config count reached, skipping");
                    offset += sizeof(struct FlashConfig);
                } else {
                    od_log_error("ERROR: Not enough data for flash_config");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x26: // wifi_config (see struct WifiConfig)
                {
                    if (offset + sizeof(struct WifiConfig) > configLen - 2) {
                        od_log_error("ERROR: Not enough data for wifi_config");
                        globalConfig.loaded = false;
                        return false;
                    }
                    struct WifiConfig wc;
                    memcpy(&wc, &configData[offset], sizeof(wc));
                    offset += sizeof(wc);

                    memcpy(wifiSsid, wc.ssid, sizeof(wc.ssid));
                    wifiSsid[32] = '\0';
                    uint8_t ssidLen = 0;
                    while (ssidLen < 32 && wifiSsid[ssidLen] != '\0') ssidLen++;

                    memcpy(wifiPassword, wc.password, sizeof(wc.password));
                    wifiPassword[32] = '\0';
                    uint8_t passwordLen = 0;
                    while (passwordLen < 32 && wifiPassword[passwordLen] != '\0') passwordLen++;

                    wifiEncryptionType = wc.encryption_type;

#ifdef TARGET_ESP32
                    memcpy(wifiServerUrl, wc.server_host, 64);
                    wifiServerUrl[64] = '\0';

                    bool isStringFormat = false;
                    for (int i = 0; i < 64; i++) {
                        if (wifiServerUrl[i] == '\0') {
                            isStringFormat = true;
                            break;
                        }
                        if (i > 0 && wifiServerUrl[i] < 32 && wifiServerUrl[i] != '\0') {
                            break;
                        }
                    }

                    if (!isStringFormat && wifiServerUrl[4] == '\0' &&
                        (wifiServerUrl[0] != 0 || wifiServerUrl[1] != 0 ||
                         wifiServerUrl[2] != 0 || wifiServerUrl[3] != 0)) {
                        uint8_t ip[4];
                        ip[0] = wc.server_host[0];
                        ip[1] = wc.server_host[1];
                        ip[2] = wc.server_host[2];
                        ip[3] = wc.server_host[3];
                        snprintf(wifiServerUrl, 65, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
                        od_log_debug("Converted numeric IP to string: \"%s\"", wifiServerUrl);
                    } else if (!isStringFormat && wifiServerUrl[0] != '\0') {
                        uint32_t ipNum = (uint32_t)wc.server_host[0] |
                                        ((uint32_t)wc.server_host[1] << 8) |
                                        ((uint32_t)wc.server_host[2] << 16) |
                                        ((uint32_t)wc.server_host[3] << 24);
                        uint8_t ip[4];
                        ip[0] = (ipNum >> 24) & 0xFF;
                        ip[1] = (ipNum >> 16) & 0xFF;
                        ip[2] = (ipNum >> 8) & 0xFF;
                        ip[3] = ipNum & 0xFF;
                        snprintf(wifiServerUrl, 65, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
                        od_log_debug("Converted 32-bit integer to IP string: \"%s\"", wifiServerUrl);
                    }

                    // server_port is the one BIG-ENDIAN field in WifiConfig; read it byte-wise
                    // (former reserved[64]=MSB, reserved[65]=LSB).
                    wifiServerPort = (uint16_t)(((uint16_t)((const uint8_t*)&wc.server_port)[0] << 8) |
                                                ((const uint8_t*)&wc.server_port)[1]);
                    if (wifiServerPort == 0) {
                        wifiServerPort = 2446;
                    }

                    // wifiServerConfigured is dead: it was only ever read by the log
                    // lines that used to sit here. It described the old "tag pushes to
                    // an upload server" model, but the LAN transport inverted that --
                    // the device listens and the host connects to it, so server_host
                    // gates nothing. server_host stays part of the 0x26 wire format.
                    //
                    // Report the endpoint the LAN listener will actually bind, not just
                    // the raw config field: the TLS-PSK channel runs on server_port + 1
                    // and there is no config entry for it.
#ifdef OPENDISPLAY_HAS_WIFI
                    od_log_debug("LAN: %s on port %u (server_port %u)",
                                 lanTlsEnabled() ? "TLS-PSK" : "plaintext",
                                 (unsigned)lanActivePort(), (unsigned)wifiServerPort);
#else
                    od_log_debug("LAN: transport not compiled in (server_port %u)", (unsigned)wifiServerPort);
#endif
#endif
                    wifiConfigured = true;
                    od_log_info("=== WiFi Configuration Loaded ===");
                    // Do NOT log the SSID or password (credentials). Presence/length only.
                    od_log_debug("SSID: (set, %u chars)", ssidLen);
                    od_log_debug("Password: %s", passwordLen > 0 ? "(set)" : "(empty)");
                    const char* encTypeStr = "Unknown";
                    switch (wifiEncryptionType) {
                        case 0x00: encTypeStr = "None (Open)"; break;
                        case 0x01: encTypeStr = "WEP"; break;
                        case 0x02: encTypeStr = "WPA"; break;
                        case 0x03: encTypeStr = "WPA2"; break;
                        case 0x04: encTypeStr = "WPA3"; break;
                    }
                    od_log_debug("Encryption Type: 0x%02X (%s)", wifiEncryptionType, encTypeStr);
                    od_log_debug("SSID length: %u bytes", ssidLen);
                    od_log_debug("Password length: %u bytes", passwordLen);
                    od_log_debug("WiFi configured: true");
                }
                break;
            case 0x27: // security_config
                {
                    if (offset + sizeof(struct SecurityConfig) <= configLen - 2) {
                        memcpy(&securityConfig, &configData[offset], sizeof(struct SecurityConfig));
                        offset += sizeof(struct SecurityConfig);
                        // Check if key is all zeros (encryption disabled)
                        bool keyIsZero = true;
                        for (int i = 0; i < 16; i++) {
                            if (securityConfig.encryption_key[i] != 0) {
                                keyIsZero = false;
                                break;
                            }
                        }
                        if (keyIsZero) {
                            securityConfig.encryption_enabled = 0;
                            od_log_debug("Security config: Encryption disabled (key is all zeros)");
                        } else if (securityConfig.encryption_enabled) {
                            od_log_debug("Security config: Encryption enabled");
                            od_log_debug("Session timeout: %u seconds", securityConfig.session_timeout_seconds);
                        } else {
                            od_log_debug("Security config: Encryption disabled (flag set to 0)");
                        }
                        // Log security flags
                        if (securityConfig.flags & OD_SECURITY_FLAG_REWRITE_ALLOWED) {
                            od_log_debug("Security config: Rewrite allowed (unauthorized config writes permitted)");
                        }
                        if (securityConfig.flags & OD_SECURITY_FLAG_SHOW_KEY_ON_SCREEN) {
                            od_log_debug("Security config: Show key on screen enabled (future feature)");
                        }
                        if (securityConfig.flags & OD_SECURITY_FLAG_RESET_PIN_ENABLED) {
                            od_log_debug("Security config: Reset pin %u enabled (polarity: %s, pullup: %s, pulldown: %s)",
                                       securityConfig.reset_pin,
                                       (securityConfig.flags & OD_SECURITY_FLAG_RESET_PIN_POLARITY) ? "HIGH" : "LOW",
                                       (securityConfig.flags & OD_SECURITY_FLAG_RESET_PIN_PULLUP) ? "yes" : "no",
                                       (securityConfig.flags & OD_SECURITY_FLAG_RESET_PIN_PULLDOWN) ? "yes" : "no");
                        } else {
                            od_log_debug("Security config: Reset pin disabled");
                        }
                    } else {
                        od_log_error("ERROR: Not enough data for security_config");
                        globalConfig.loaded = false;
                        return false;
                    }
                }
                break;
            default:
                od_log_warn("WARNING: Unknown packet ID 0x%02X, skipping", packetId);
                offset = configLen - 2; // Skip to CRC
                break;
        }
    }
    if (offset < configLen - 2) {
        uint16_t crcGiven = configData[configLen - 2] | (configData[configLen - 1] << 8);
        // Advisory (warn-only) validation using CRC-16/CCITT to match the toolbox,
        // nRF and Silabs firmware. Not enforced: a mismatch logs a warning only.
        uint16_t crcCalculated = config_toolbox_outer_crc16(configData, configLen - 2);
        if (crcGiven != crcCalculated) {
            od_log_warn("WARNING: Config CRC mismatch (given: 0x%04X, calculated: 0x%04X)", crcGiven, crcCalculated);
        }
    }
    globalConfig.loaded = true;
    return true;
}

void printConfigSummary(){
    if (!globalConfig.loaded) {
        od_log_debug("Config not loaded");
        return;
    }
    od_log_debug("=== Configuration Summary ===");
    od_log_debug("Version: %u.%u", globalConfig.version, globalConfig.minor_version);
    od_log_debug("Loaded: %s", globalConfig.loaded ? "Yes" : "No");
    od_log_debug(" ");
    od_log_debug("--- System Configuration ---");
    od_log_debug("IC Type: 0x%04X", globalConfig.system_config.ic_type);
    od_log_debug("Communication Modes: 0x%02X", globalConfig.system_config.communication_modes);
    od_log_debug("  BLE: %s", (globalConfig.system_config.communication_modes & COMM_MODE_BLE) ? "enabled" : "disabled");
    od_log_debug("  OEPL: %s", (globalConfig.system_config.communication_modes & COMM_MODE_OEPL) ? "enabled" : "disabled");
    od_log_debug("  WiFi: %s", (globalConfig.system_config.communication_modes & COMM_MODE_WIFI) ? "enabled" : "disabled");
    #ifdef TARGET_ESP32
    if (globalConfig.system_config.communication_modes & COMM_MODE_WIFI) {
        if (wifiConfigured) {
            od_log_debug("  WiFi SSID: (configured)");  // credential; not logged verbatim
            if (wifiInitialized) {
                if (wifiConnected) {
                    String localIp = WiFi.localIP().toString();
                    od_log_debug("  WiFi Status: Connected (IP: %s)", localIp.c_str());
                } else {
                    od_log_debug("  WiFi Status: Disconnected");
                }
            } else {
                od_log_debug("  WiFi Status: Not initialized");
            }
        } else {
            od_log_debug("  WiFi Status: Configured but not loaded");
        }
    }
    #endif
    od_log_debug("Device Flags: 0x%02X", globalConfig.system_config.device_flags);
    od_log_debug("  PWR_PIN flag: %s", (globalConfig.system_config.device_flags & DEVICE_FLAG_PWR_PIN) ? "enabled" : "disabled");
    #ifdef TARGET_NRF
    od_log_debug("  XIAOINIT flag: %s", (globalConfig.system_config.device_flags & DEVICE_FLAG_XIAOINIT) ? "enabled" : "disabled");
    #endif
    od_log_debug("  WS_PP_INIT flag: %s", (globalConfig.system_config.device_flags & DEVICE_FLAG_WS_PP_INIT) ? "enabled" : "disabled");
    od_log_debug("  BATTERY_LATCH flag: %s", (globalConfig.system_config.device_flags & DEVICE_FLAG_BATTERY_LATCH) ? "enabled" : "disabled");
    od_log_debug("  PWR_LATCH_DFF flag: %s", (globalConfig.system_config.device_flags & DEVICE_FLAG_PWR_LATCH_DFF) ? "enabled" : "disabled");
    od_log_debug("Power Pin: %u", globalConfig.system_config.pwr_pin);
    od_log_debug("Power Pin 2: %u", globalConfig.system_config.pwr_pin_2);
    od_log_debug("Power Pin 3: %u", globalConfig.system_config.pwr_pin_3);
    od_log_debug(" ");
    od_log_debug("--- Manufacturer Data ---");
    od_log_debug("Manufacturer ID: 0x%04X", globalConfig.manufacturer_data.manufacturer_id);
    od_log_debug("Board Type: %u", globalConfig.manufacturer_data.board_type);
    od_log_debug("Board Revision: %u", globalConfig.manufacturer_data.board_revision);
    od_log_debug(" ");
    od_log_debug("--- Power Configuration ---");
    od_log_debug("Power Mode: %u", globalConfig.power_option.power_mode);
    od_log_debug("Battery Capacity: %u %u %u mAh",
               globalConfig.power_option.battery_capacity_mah[0],
               globalConfig.power_option.battery_capacity_mah[1],
               globalConfig.power_option.battery_capacity_mah[2]);
    od_log_debug("Awake Timeout: %u ms", globalConfig.power_option.sleep_timeout_ms);
    od_log_debug("Deep Sleep Time: %u seconds", globalConfig.power_option.deep_sleep_time_seconds);
    od_log_debug("Min Wake Time: %u seconds", globalConfig.power_option.min_wake_time_seconds);
    od_log_debug("TX Power: %u", globalConfig.power_option.tx_power);
    od_log_debug("Sleep Flags: 0x%02X", globalConfig.power_option.sleep_flags);
    od_log_debug("Button Wake: %s (sleep_flags bit0)", (globalConfig.power_option.sleep_flags & OD_SLEEP_FLAG_BUTTON_WAKE_DISABLE) ? "disabled" : "enabled");
    od_log_debug("Screen Timeout: %u s (EPD keep-alive; 0 = off immediately after refresh)", globalConfig.power_option.screen_timeout_seconds);
    od_log_debug("Battery Sense Pin: %u", globalConfig.power_option.battery_sense_pin);
    od_log_debug("Battery Sense Enable Pin: %u", globalConfig.power_option.battery_sense_enable_pin);
    od_log_debug("Battery Sense Flags: 0x%02X", globalConfig.power_option.battery_sense_flags);
    od_log_debug("  ENABLE_INVERTED: %s", (globalConfig.power_option.battery_sense_flags & OD_BATTERY_SENSE_FLAG_ENABLE_INVERTED) ? "yes" : "no");
    od_log_debug("Capacity Estimator: %u", globalConfig.power_option.capacity_estimator);
    od_log_debug("Voltage Scaling Factor: %u", globalConfig.power_option.voltage_scaling_factor);
    od_log_debug("Deep Sleep Current: %u uA", (unsigned)globalConfig.power_option.deep_sleep_current_ua);
    od_log_debug(" ");
    od_log_debug("--- Display Configurations (%u) ---", globalConfig.display_count);
    for (int i = 0; i < globalConfig.display_count; i++) {
        od_log_debug("Display %d:", i);
        od_log_debug("  Instance: %u", globalConfig.displays[i].instance_number);
        od_log_debug("  Technology: 0x%02X", globalConfig.displays[i].display_technology);
        od_log_debug("  Panel IC Type: 0x%04X", globalConfig.displays[i].panel_ic_type);
        od_log_debug("  Resolution: %ux%u", globalConfig.displays[i].pixel_width, globalConfig.displays[i].pixel_height);
        od_log_debug("  Size: %ux%u mm", globalConfig.displays[i].active_width_mm, globalConfig.displays[i].active_height_mm);
        od_log_debug("  Tag Type: 0x%04X", globalConfig.displays[i].legacy_tag_type);
        od_log_debug("  Rotation: %u degrees", (unsigned)(globalConfig.displays[i].rotation * 90));
        od_log_debug("  Reset Pin: %u", globalConfig.displays[i].reset_pin);
        od_log_debug("  Busy Pin: %u", globalConfig.displays[i].busy_pin);
        od_log_debug("  DC Pin: %u", globalConfig.displays[i].dc_pin);
        od_log_debug("  CS Pin: %u", globalConfig.displays[i].cs_pin);
        od_log_debug("  Data Pin: %u", globalConfig.displays[i].data_pin);
        od_log_debug("  Partial Update: %s", globalConfig.displays[i].partial_update_support ? "Yes" : "No");
        od_log_debug("  Color Scheme: 0x%02X", globalConfig.displays[i].color_scheme);
        od_log_debug("  Transmission Modes: 0x%02X", globalConfig.displays[i].transmission_modes);
        od_log_debug("    ZIPXL: %s", (globalConfig.displays[i].transmission_modes & OD_TRANSMISSION_MODE_STREAMING_DECOMPRESSION) ? "enabled" : "disabled");
        od_log_debug("    ZIP: %s", (globalConfig.displays[i].transmission_modes & OD_TRANSMISSION_MODE_ZIP) ? "enabled" : "disabled");
        od_log_debug("    G5: %s", (globalConfig.displays[i].transmission_modes & OD_TRANSMISSION_MODE_G5) ? "enabled" : "disabled");
        od_log_debug("    DIRECT_WRITE: %s", (globalConfig.displays[i].transmission_modes & OD_TRANSMISSION_MODE_DIRECT_WRITE) ? "enabled" : "disabled");
        od_log_debug("    CLEAR_ON_BOOT: %s", (globalConfig.displays[i].transmission_modes & OD_TRANSMISSION_MODE_CLEAR_ON_BOOT) ? "enabled" : "disabled");
        od_log_debug("  Full update energy (mC): %u", globalConfig.displays[i].full_update_mC);
        od_log_debug(" ");
    }
    od_log_debug("--- LED Configurations (%u) ---", globalConfig.led_count);
    for (int i = 0; i < globalConfig.led_count; i++) {
        od_log_debug("LED %d:", i);
        od_log_debug("  Instance: %u", globalConfig.leds[i].instance_number);
        od_log_debug("  Type: 0x%02X", globalConfig.leds[i].led_type);
        od_log_debug("  Pins: R=%u G=%u B=%u 4=%u",
                   globalConfig.leds[i].led_1_r,
                   globalConfig.leds[i].led_2_g,
                   globalConfig.leds[i].led_3_b,
                   globalConfig.leds[i].led_4);
        od_log_debug("  Flags: 0x%02X", globalConfig.leds[i].led_flags);
        od_log_debug(" ");
    }
    od_log_debug("--- Sensor Configurations (%u) ---", globalConfig.sensor_count);
    for (int i = 0; i < globalConfig.sensor_count; i++) {
        od_log_debug("Sensor %d:", i);
        od_log_debug("  Instance: %u", globalConfig.sensors[i].instance_number);
        od_log_debug("  Type: 0x%04X", globalConfig.sensors[i].sensor_type);
        od_log_debug("  Bus ID: %u", globalConfig.sensors[i].bus_id);
        od_log_debug("  I2C addr (7-bit) / MSD data start byte: %u / %u", globalConfig.sensors[i].i2c_addr_7bit, globalConfig.sensors[i].msd_data_start_byte);
        od_log_debug(" ");
    }
    od_log_debug("--- Data Bus Configurations (%u) ---", globalConfig.data_bus_count);
    for (int i = 0; i < globalConfig.data_bus_count; i++) {
        od_log_debug("Data Bus %d:", i);
        od_log_debug("  Instance: %u", globalConfig.data_buses[i].instance_number);
        od_log_debug("  Type: 0x%02X", globalConfig.data_buses[i].bus_type);
        od_log_debug("  Pins: 1=%u 2=%u 3=%u 4=%u 5=%u 6=%u 7=%u",
                   globalConfig.data_buses[i].pin_1,
                   globalConfig.data_buses[i].pin_2,
                   globalConfig.data_buses[i].pin_3,
                   globalConfig.data_buses[i].pin_4,
                   globalConfig.data_buses[i].pin_5,
                   globalConfig.data_buses[i].pin_6,
                   globalConfig.data_buses[i].pin_7);
        od_log_debug("  Speed: %u Hz", (unsigned)globalConfig.data_buses[i].bus_speed_hz);
        od_log_debug("  Flags: 0x%02X", globalConfig.data_buses[i].bus_flags);
        od_log_debug("  Pullups: 0x%02X", globalConfig.data_buses[i].pullups);
        od_log_debug("  Pulldowns: 0x%02X", globalConfig.data_buses[i].pulldowns);
        od_log_debug(" ");
    }
    od_log_debug("--- Binary Input Configurations (%u) ---", globalConfig.binary_input_count);
    for (int i = 0; i < globalConfig.binary_input_count; i++) {
        od_log_debug("Binary Input %d:", i);
        od_log_debug("  Instance: %u", globalConfig.binary_inputs[i].instance_number);
        od_log_debug("  Type: 0x%02X", globalConfig.binary_inputs[i].input_type);
        od_log_debug("  Display As: 0x%02X", globalConfig.binary_inputs[i].display_as);
        od_log_debug("  Pins: 1=%u 2=%u 3=%u 4=%u 5=%u 6=%u 7=%u 8=%u",
                   globalConfig.binary_inputs[i].input_pin_1,
                   globalConfig.binary_inputs[i].input_pin_2,
                   globalConfig.binary_inputs[i].input_pin_3,
                   globalConfig.binary_inputs[i].input_pin_4,
                   globalConfig.binary_inputs[i].input_pin_5,
                   globalConfig.binary_inputs[i].input_pin_6,
                   globalConfig.binary_inputs[i].input_pin_7,
                   globalConfig.binary_inputs[i].input_pin_8);
        od_log_debug("  Input Flags: 0x%02X", globalConfig.binary_inputs[i].pins_used);
        od_log_debug("  Invert: 0x%02X", globalConfig.binary_inputs[i].invert);
        od_log_debug("  Pullups: 0x%02X", globalConfig.binary_inputs[i].pullups);
        od_log_debug("  Pulldowns: 0x%02X", globalConfig.binary_inputs[i].pulldowns);
        if (globalConfig.binary_inputs[i].input_type == 3) {
            od_log_debug("  ADC Ladder: count=%u idBase=%u byteIdx=%u",
                        globalConfig.binary_inputs[i].reserved[0],
                        globalConfig.binary_inputs[i].reserved[1],
                        globalConfig.binary_inputs[i].button_data_byte_index);
        }
        od_log_debug(" ");
    }
    od_log_debug("--- Touch Controllers (%u) ---", globalConfig.touch_controller_count);
    for (int i = 0; i < globalConfig.touch_controller_count; i++) {
        od_log_debug("Touch %d:", i);
        od_log_debug("  Instance: %u", globalConfig.touch_controllers[i].instance_number);
        od_log_debug("  IC type: %u", globalConfig.touch_controllers[i].touch_ic_type);
        od_log_debug("  Bus ID: %u", globalConfig.touch_controllers[i].bus_id);
        od_log_debug("  I2C addr (7-bit): 0x%02X", globalConfig.touch_controllers[i].i2c_addr_7bit);
        od_log_debug("  INT/RST/EN pins: %u / %u / %u",
                    globalConfig.touch_controllers[i].int_pin,
                    globalConfig.touch_controllers[i].rst_pin,
                    globalConfig.touch_controllers[i].enable_pin);
        od_log_debug("  Display instance: %u", globalConfig.touch_controllers[i].display_instance);
        od_log_debug("  Flags: 0x%02X", globalConfig.touch_controllers[i].flags);
        od_log_debug("  Poll ms / MSD start byte: %u / %u", globalConfig.touch_controllers[i].poll_interval_ms, globalConfig.touch_controllers[i].touch_data_start_byte);
        od_log_debug(" ");
    }
    od_log_debug("--- Passive buzzers (%u) ---", globalConfig.passive_buzzer_count);
    for (int i = 0; i < globalConfig.passive_buzzer_count; i++) {
        od_log_debug("Buzzer %d:", i);
        od_log_debug("  Instance: %u", globalConfig.passive_buzzers[i].instance_number);
        od_log_debug("  Drive / enable pin: %u / %u", globalConfig.passive_buzzers[i].drive_pin, globalConfig.passive_buzzers[i].enable_pin);
        od_log_debug("  Flags: 0x%02X", globalConfig.passive_buzzers[i].flags);
        od_log_debug("  Duty %%: %u", globalConfig.passive_buzzers[i].duty_percent);
        od_log_debug(" ");
    }
    if (globalConfig.data_extended_loaded) {
        od_log_debug("--- Data Extended ---");
        od_log_debug("  manufacturer_name: %s", (char*)globalConfig.data_extended.manufacturer_name);
        od_log_debug("  model_name: %s",        (char*)globalConfig.data_extended.model_name);
        od_log_debug("  serial_number: %s",     (char*)globalConfig.data_extended.serial_number);
        od_log_debug("  friendly_name: %s",     (char*)globalConfig.data_extended.friendly_name);
        od_log_debug("  device_location: %s",   (char*)globalConfig.data_extended.device_location);
        od_log_debug("  device_id: %s",         (char*)globalConfig.data_extended.device_id);
        od_log_debug("  custom_string_1: %s",   (char*)globalConfig.data_extended.custom_string_1);
        od_log_debug("  custom_string_2: %s",   (char*)globalConfig.data_extended.custom_string_2);
        od_log_debug("  custom_string_3: %s",   (char*)globalConfig.data_extended.custom_string_3);
        od_log_debug(" ");
    }
    od_log_debug("=============================");
}

void full_config_init() {
    od_log_info("Initializing config storage...");
    if (!initConfigStorage()) {
        od_log_error("Config storage initialization failed");
        return;
    }
    od_log_info("Config storage initialized successfully");

#ifdef FACTORY_CLEAR_CONFIG_ON_BOOT
    od_log_info("Factory clear build: erasing stored config");
    clearStoredConfig();
    od_log_info("Config cleared; skipping load");
    return;
#endif

    od_log_info("Loading global configuration...");
    bool configLoaded = loadGlobalConfig();
    if (!configLoaded && tryProvisionFactoryEmbed()) {
        configLoaded = loadGlobalConfig();
    }
    if (configLoaded) {
        od_log_info("Global configuration loaded successfully");
        printConfigSummary();
        clearEncryptionSession();
        encryptionInitialized = true;
        checkResetPin();
#ifdef TARGET_NRF
        powerDownExternalFlashFromConfig();
        if (globalConfig.loaded && (globalConfig.system_config.device_flags & DEVICE_FLAG_XIAOINIT)) {
            od_log_info("Device flag DEVICE_FLAG_XIAOINIT is set, calling xiaoinit()...");
            xiaoinit();
            od_log_info("xiaoinit() completed");
        }
#endif
        if (globalConfig.loaded && (globalConfig.system_config.device_flags & DEVICE_FLAG_WS_PP_INIT)) {
            od_log_info("Device flag DEVICE_FLAG_WS_PP_INIT is set, calling ws_pp_init()...");
            ws_pp_init();
            od_log_info("ws_pp_init() completed");
        }
        // Must run after config load: latch pins/flag come from globalConfig.
        powerLatchBegin();
    } else {
        od_log_error("Global configuration load failed or no config found");
    }
}
