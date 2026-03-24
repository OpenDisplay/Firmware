#include "config_parser.h"
#include "structs.h"
#include "encryption_state.h"
#include "encryption.h"
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
#endif
#ifndef TRANSMISSION_MODE_RAW
#define TRANSMISSION_MODE_RAW (1 << 0)
#define TRANSMISSION_MODE_ZIP (1 << 1)
#define TRANSMISSION_MODE_G5 (1 << 2)
#define TRANSMISSION_MODE_DIRECT_WRITE (1 << 3)
#define TRANSMISSION_MODE_CLEAR_ON_BOOT (1 << 7)
#endif

void writeSerial(String message, bool newLine = true);

extern struct GlobalConfig globalConfig;
extern uint8_t activeLedInstance;
extern char wifiSsid[33];
extern char wifiPassword[33];
extern uint8_t wifiEncryptionType;
extern bool wifiConfigured;
#ifdef TARGET_ESP32
extern char wifiServerUrl[65];
extern uint16_t wifiServerPort;
extern bool wifiServerConfigured;
extern bool wifiConnected;
extern bool wifiInitialized;
#endif

void xiaoinit();
void ws_pp_init();

extern bool encryptionInitialized;

bool initConfigStorage(){
    #ifdef TARGET_NRF
    if (!InternalFS.begin()) {
        writeSerial("ERROR: Failed to mount internal file system");
        return false;
    }
    return true;
    #endif
    #ifdef TARGET_ESP32
    if (!LittleFS.begin(true)) { // true = format on failure
        writeSerial("ERROR: Failed to mount LittleFS");
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

bool saveConfig(uint8_t* configData, uint32_t len){
    if (len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Config data too large (" + String(len) + " bytes)");
        return false;
    }
    static config_storage_t config;
    config.magic = 0xDEADBEEF;
    config.version = 1;
    config.data_len = len;
    config.crc = calculateConfigCRC(configData, len);
    memcpy(config.data, configData, len);
    size_t headerSize = sizeof(config_storage_t) - MAX_CONFIG_SIZE; // Size without data array
    size_t totalSize = headerSize + len; // Header + actual data length
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
        writeSerial("ERROR: Failed to open config file for writing");
        #ifdef TARGET_NRF
        file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_WRITE);
        #elif defined(TARGET_ESP32)
        file = LittleFS.open(CONFIG_FILE_PATH, FILE_WRITE);
        #endif
        if (!file) {
            writeSerial("ERROR: Failed to open config file for writing with CREATE|WRITE");
        return false;
        }
    }
    size_t bytesWritten = file.write((uint8_t*)&config, totalSize);
    file.close();
    if (bytesWritten != totalSize) {
        writeSerial("ERROR: Failed to write complete config data (expected " + String(totalSize) + ", wrote " + String(bytesWritten) + ")");
        return false;
    }
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
    static config_storage_t config;
    static size_t bytesRead;
    static size_t headerSize = sizeof(config_storage_t) - MAX_CONFIG_SIZE; // Size without data array
    bytesRead = file.read((uint8_t*)&config, headerSize);
    if (bytesRead != headerSize) {
        writeSerial("ERROR: Failed to read config header (expected " + String(headerSize) + ", got " + String(bytesRead) + ")");
        file.close();
        return false;
    }
    if (config.magic != 0xDEADBEEF) {
        writeSerial("ERROR: Invalid config magic number");
        file.close();
        return false;
    }
    if (config.data_len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Config data too large");
        file.close();
        return false;
    }
    bytesRead = file.read(config.data, config.data_len);
    file.flush();
    file.close();
    if (bytesRead != config.data_len) {
        writeSerial("ERROR: Failed to read complete config data (expected " + String(config.data_len) + ", read " + String(bytesRead) + ")");
        return false;
    }
    uint32_t calculatedCRC = calculateConfigCRC(config.data, config.data_len);
    if (config.crc != calculatedCRC) {
        writeSerial("ERROR: Config CRC mismatch");
        return false;
    }
    if (config.data_len > *len) {
        writeSerial("ERROR: Config data larger than buffer");
        return false;
    }
    for (uint32_t i = 0; i < config.data_len && i < *len; i++) {
        configData[i] = config.data[i];
    }
    *len = config.data_len;
    return true;
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
    static uint8_t configData[MAX_CONFIG_SIZE];
    static uint32_t configLen = MAX_CONFIG_SIZE;
    if (!loadConfig(configData, &configLen)) {
        globalConfig.loaded = false;
        return false;
    }
    if (configLen < 3) {
        writeSerial("ERROR: Config too short");
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
                    writeSerial("ERROR: Not enough data for system_config");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x02: // manufacturer_data
                if (offset + sizeof(struct ManufacturerData) <= configLen - 2) {
                    memcpy(&globalConfig.manufacturer_data, &configData[offset], sizeof(struct ManufacturerData));
                    offset += sizeof(struct ManufacturerData);
                } else {
                    writeSerial("ERROR: Not enough data for manufacturer_data");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x04: // power_option
                if (offset + sizeof(struct PowerOption) <= configLen - 2) {
                    memcpy(&globalConfig.power_option, &configData[offset], sizeof(struct PowerOption));
                    offset += sizeof(struct PowerOption);
                } else {
                    writeSerial("ERROR: Not enough data for power_option");
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
                    writeSerial("WARNING: Maximum display count reached, skipping");
                    offset += sizeof(struct DisplayConfig);
                } else {
                    writeSerial("ERROR: Not enough data for display");
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
                    writeSerial("WARNING: Maximum LED count reached, skipping");
                    offset += sizeof(struct LedConfig);
                } else {
                    writeSerial("ERROR: Not enough data for LED");
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
                    writeSerial("WARNING: Maximum sensor count reached, skipping");
                    offset += sizeof(struct SensorData);
                } else {
                    writeSerial("ERROR: Not enough data for sensor");
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
                    writeSerial("WARNING: Maximum data_bus count reached, skipping");
                    offset += sizeof(struct DataBus);
                } else {
                    writeSerial("ERROR: Not enough data for data_bus");
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
                    writeSerial("WARNING: Maximum binary_input count reached, skipping");
                    offset += sizeof(struct BinaryInputs);
                } else {
                    writeSerial("ERROR: Not enough data for binary_input");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x26: // wifi_config
                {
                    const uint16_t WIFI_CONFIG_SIZE = 162;
                    if (offset + WIFI_CONFIG_SIZE <= configLen) {
                        memcpy(wifiSsid, &configData[offset], 32);
                        wifiSsid[32] = '\0';  // Ensure null termination
                        uint8_t ssidLen = 0;
                        while (ssidLen < 32 && wifiSsid[ssidLen] != '\0') ssidLen++;
                        offset += 32;
                        memcpy(wifiPassword, &configData[offset], 32);
                        wifiPassword[32] = '\0';  // Ensure null termination
                        uint8_t passwordLen = 0;
                        while (passwordLen < 32 && wifiPassword[passwordLen] != '\0') passwordLen++;
                        offset += 32;
                        wifiEncryptionType = configData[offset++];
                        #ifdef TARGET_ESP32
                        // Parse server configuration from reserved bytes
                        // First, read as string (like SSID)
                        memcpy(wifiServerUrl, &configData[offset], 64);
                        wifiServerUrl[64] = '\0';  // Ensure null termination
                        
                        // Check if it's stored as a string (has null terminator in first few bytes)
                        // or as a 4-byte IP address (numeric format from config tool)
                        bool isStringFormat = false;
                        for (int i = 0; i < 64; i++) {
                            if (wifiServerUrl[i] == '\0') {
                                isStringFormat = true;
                                break;
                            }
                            // If we find a non-printable character (except null), it's likely binary
                            if (i > 0 && wifiServerUrl[i] < 32 && wifiServerUrl[i] != '\0') {
                                break;
                            }
                        }
                        
                        // If first 4 bytes look like an IP address in numeric format (little-endian)
                        // and there's no null terminator in first 5 bytes, convert to IP string
                        // Check if bytes 0-3 are non-zero and byte 4 is null (indicating 4-byte format)
                        if (!isStringFormat && wifiServerUrl[4] == '\0' && 
                            (wifiServerUrl[0] != 0 || wifiServerUrl[1] != 0 || 
                             wifiServerUrl[2] != 0 || wifiServerUrl[3] != 0)) {
                            // The config tool stores IP as 32-bit integer in little-endian format
                            // Bytes are: [byte0][byte1][byte2][byte3] = IP address
                            // Convert 4-byte IP (stored as little-endian) to string format
                            uint8_t ip[4];
                            ip[0] = configData[offset];
                            ip[1] = configData[offset + 1];
                            ip[2] = configData[offset + 2];
                            ip[3] = configData[offset + 3];
                            snprintf(wifiServerUrl, 65, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
                            writeSerial("Converted numeric IP to string: \"" + String(wifiServerUrl) + "\"");
                        } else if (!isStringFormat && wifiServerUrl[0] != '\0') {
                            // Try to interpret as 32-bit integer (little-endian) and convert to IP
                            uint32_t ipNum = (uint32_t)configData[offset] | 
                                            ((uint32_t)configData[offset + 1] << 8) |
                                            ((uint32_t)configData[offset + 2] << 16) |
                                            ((uint32_t)configData[offset + 3] << 24);
                            // Convert to IP string (interpret as big-endian IP address)
                            uint8_t ip[4];
                            ip[0] = (ipNum >> 24) & 0xFF;
                            ip[1] = (ipNum >> 16) & 0xFF;
                            ip[2] = (ipNum >> 8) & 0xFF;
                            ip[3] = ipNum & 0xFF;
                            snprintf(wifiServerUrl, 65, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
                            writeSerial("Converted 32-bit integer to IP string: \"" + String(wifiServerUrl) + "\"");
                        }
                        
                        offset += 64;
                        // Read port (2 bytes, network byte order)
                        wifiServerPort = (configData[offset] << 8) | configData[offset + 1];
                        offset += 2;
                        
                        // Check if server is configured (URL not empty and not "0.0.0.0")
                        wifiServerConfigured = (wifiServerUrl[0] != '\0' && 
                                               strcmp(wifiServerUrl, "0.0.0.0") != 0);
                        if (wifiServerConfigured) {
                            writeSerial("Server configured: YES");
                            writeSerial("Server URL: \"" + String(wifiServerUrl) + "\"");
                            writeSerial("Server Port: " + String(wifiServerPort));
                        } else {
                            writeSerial("Server configured: NO");
                            if (wifiServerUrl[0] == '\0') {
                                writeSerial("Reason: URL is empty");
                            } else if (strcmp(wifiServerUrl, "0.0.0.0") == 0) {
                                writeSerial("Reason: URL is \"0.0.0.0\"");
                            }
                        }
                        offset += 29;  // Skip remaining reserved bytes
                        #else
                        offset += 95;  // Skip all reserved bytes on non-ESP32
                        #endif
                        wifiConfigured = true;
                        writeSerial("=== WiFi Configuration Loaded ===");
                        writeSerial("SSID: \"" + String(wifiSsid) + "\"");
                        if (passwordLen > 0) {
                            writeSerial("Password: \"" + String(wifiPassword) + "\"");
                        } else {
                            writeSerial("Password: (empty)");
                        }
                        String encTypeStr = "Unknown";
                        switch(wifiEncryptionType) {
                            case 0x00: encTypeStr = "None (Open)"; break;
                            case 0x01: encTypeStr = "WEP"; break;
                            case 0x02: encTypeStr = "WPA"; break;
                            case 0x03: encTypeStr = "WPA2"; break;
                            case 0x04: encTypeStr = "WPA3"; break;
                        }
                        writeSerial("Encryption Type: 0x" + String(wifiEncryptionType, HEX) + " (" + encTypeStr + ")");
                        writeSerial("SSID length: " + String(ssidLen) + " bytes");
                        writeSerial("Password length: " + String(passwordLen) + " bytes");
                        writeSerial("WiFi configured: true");
                    } else {
                        writeSerial("ERROR: Not enough data for wifi_config");
                        globalConfig.loaded = false;
                        return false;
                    }
                }
                break;
            case 0x27: // security_config
                {
                    if (offset + sizeof(struct SecurityConfig) <= configLen) {
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
                            writeSerial("Security config: Encryption disabled (key is all zeros)");
                        } else if (securityConfig.encryption_enabled) {
                            writeSerial("Security config: Encryption enabled");
                            writeSerial("Session timeout: " + String(securityConfig.session_timeout_seconds) + " seconds");
                        } else {
                            writeSerial("Security config: Encryption disabled (flag set to 0)");
                        }
                        // Log security flags
                        if (securityConfig.flags & SECURITY_FLAG_REWRITE_ALLOWED) {
                            writeSerial("Security config: Rewrite allowed (unauthorized config writes permitted)");
                        }
                        if (securityConfig.flags & SECURITY_FLAG_SHOW_KEY_ON_SCREEN) {
                            writeSerial("Security config: Show key on screen enabled (future feature)");
                        }
                        if (securityConfig.flags & SECURITY_FLAG_RESET_PIN_ENABLED) {
                            writeSerial("Security config: Reset pin " + String(securityConfig.reset_pin) + 
                                       " enabled (polarity: " + String((securityConfig.flags & SECURITY_FLAG_RESET_PIN_POLARITY) ? "HIGH" : "LOW") + 
                                       ", pullup: " + String((securityConfig.flags & SECURITY_FLAG_RESET_PIN_PULLUP) ? "yes" : "no") + 
                                       ", pulldown: " + String((securityConfig.flags & SECURITY_FLAG_RESET_PIN_PULLDOWN) ? "yes" : "no") + ")");
                        } else {
                            writeSerial("Security config: Reset pin disabled");
                        }
                    } else {
                        writeSerial("ERROR: Not enough data for security_config");
                        globalConfig.loaded = false;
                        return false;
                    }
                }
                break;
            default:
                writeSerial("WARNING: Unknown packet ID 0x" + String(packetId, HEX) + ", skipping");
                offset = configLen - 2; // Skip to CRC
                break;
        }
    }
    if (offset < configLen - 2) {
        uint16_t crcGiven = configData[configLen - 2] | (configData[configLen - 1] << 8);
        uint32_t crcCalculated32 = calculateConfigCRC(configData, configLen - 2);
        uint16_t crcCalculated = (uint16_t)(crcCalculated32 & 0xFFFF);  // Use lower 16 bits for backwards compatibility
        if (crcGiven != crcCalculated) {
            writeSerial("WARNING: Config CRC mismatch (given: 0x" + String(crcGiven, HEX) + 
                       ", calculated: 0x" + String(crcCalculated, HEX) + ")");
        }
    }
    globalConfig.loaded = true;
    return true;
}

void printConfigSummary(){
    if (!globalConfig.loaded) {
        writeSerial("Config not loaded");
        return;
    }
    writeSerial("=== Configuration Summary ===");
    writeSerial("Version: " + String(globalConfig.version) + "." + String(globalConfig.minor_version));
    writeSerial("Loaded: " + String(globalConfig.loaded ? "Yes" : "No"));
    writeSerial("");
    writeSerial("--- System Configuration ---");
    writeSerial("IC Type: 0x" + String(globalConfig.system_config.ic_type, HEX));
    writeSerial("Communication Modes: 0x" + String(globalConfig.system_config.communication_modes, HEX));
    writeSerial("  BLE: " + String((globalConfig.system_config.communication_modes & COMM_MODE_BLE) ? "enabled" : "disabled"));
    writeSerial("  OEPL: " + String((globalConfig.system_config.communication_modes & COMM_MODE_OEPL) ? "enabled" : "disabled"));
    writeSerial("  WiFi: " + String((globalConfig.system_config.communication_modes & COMM_MODE_WIFI) ? "enabled" : "disabled"));
    #ifdef TARGET_ESP32
    if (globalConfig.system_config.communication_modes & COMM_MODE_WIFI) {
        if (wifiConfigured) {
            writeSerial("  WiFi SSID: \"" + String(wifiSsid) + "\"");
            if (wifiInitialized) {
                if (wifiConnected) {
                    writeSerial("  WiFi Status: Connected (IP: " + WiFi.localIP().toString() + ")");
                } else {
                    writeSerial("  WiFi Status: Disconnected");
                }
            } else {
                writeSerial("  WiFi Status: Not initialized");
            }
        } else {
            writeSerial("  WiFi Status: Configured but not loaded");
        }
    }
    #endif
    writeSerial("Device Flags: 0x" + String(globalConfig.system_config.device_flags, HEX));
    writeSerial("  PWR_PIN flag: " + String((globalConfig.system_config.device_flags & DEVICE_FLAG_PWR_PIN) ? "enabled" : "disabled"));
    #ifdef TARGET_NRF
    writeSerial("  XIAOINIT flag: " + String((globalConfig.system_config.device_flags & DEVICE_FLAG_XIAOINIT) ? "enabled" : "disabled"));
    #endif
    writeSerial("  WS_PP_INIT flag: " + String((globalConfig.system_config.device_flags & DEVICE_FLAG_WS_PP_INIT) ? "enabled" : "disabled"));
    writeSerial("Power Pin: " + String(globalConfig.system_config.pwr_pin));
    writeSerial("");
    writeSerial("--- Manufacturer Data ---");
    writeSerial("Manufacturer ID: 0x" + String(globalConfig.manufacturer_data.manufacturer_id, HEX));
    writeSerial("Board Type: " + String(globalConfig.manufacturer_data.board_type));
    writeSerial("Board Revision: " + String(globalConfig.manufacturer_data.board_revision));
    writeSerial("");
    writeSerial("--- Power Configuration ---");
    writeSerial("Power Mode: " + String(globalConfig.power_option.power_mode));
    writeSerial("Battery Capacity: " + String(globalConfig.power_option.battery_capacity_mah[0]) + 
               " " + String(globalConfig.power_option.battery_capacity_mah[1]) + 
               " " + String(globalConfig.power_option.battery_capacity_mah[2]) + " mAh");
    writeSerial("Awake Timeout: " + String(globalConfig.power_option.sleep_timeout_ms) + " ms");
    writeSerial("Deep Sleep Time: " + String(globalConfig.power_option.deep_sleep_time_seconds) + " seconds");
    writeSerial("TX Power: " + String(globalConfig.power_option.tx_power));
    writeSerial("Sleep Flags: 0x" + String(globalConfig.power_option.sleep_flags, HEX));
    writeSerial("Battery Sense Pin: " + String(globalConfig.power_option.battery_sense_pin));
    writeSerial("Battery Sense Enable Pin: " + String(globalConfig.power_option.battery_sense_enable_pin));
    writeSerial("Battery Sense Flags: 0x" + String(globalConfig.power_option.battery_sense_flags, HEX));
    writeSerial("Capacity Estimator: " + String(globalConfig.power_option.capacity_estimator));
    writeSerial("Voltage Scaling Factor: " + String(globalConfig.power_option.voltage_scaling_factor));
    writeSerial("Deep Sleep Current: " + String(globalConfig.power_option.deep_sleep_current_ua) + " uA");
    writeSerial("");
    writeSerial("--- Display Configurations (" + String(globalConfig.display_count) + ") ---");
    for (int i = 0; i < globalConfig.display_count; i++) {
        writeSerial("Display " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.displays[i].instance_number));
        writeSerial("  Technology: 0x" + String(globalConfig.displays[i].display_technology, HEX));
        writeSerial("  Panel IC Type: 0x" + String(globalConfig.displays[i].panel_ic_type, HEX));
        writeSerial("  Resolution: " + String(globalConfig.displays[i].pixel_width) + "x" + String(globalConfig.displays[i].pixel_height));
        writeSerial("  Size: " + String(globalConfig.displays[i].active_width_mm) + "x" + String(globalConfig.displays[i].active_height_mm) + " mm");
        writeSerial("  Tag Type: 0x" + String(globalConfig.displays[i].tag_type, HEX));
        writeSerial("  Rotation: " + String(globalConfig.displays[i].rotation * 90) + " degrees");
        writeSerial("  Reset Pin: " + String(globalConfig.displays[i].reset_pin));
        writeSerial("  Busy Pin: " + String(globalConfig.displays[i].busy_pin));
        writeSerial("  DC Pin: " + String(globalConfig.displays[i].dc_pin));
        writeSerial("  CS Pin: " + String(globalConfig.displays[i].cs_pin));
        writeSerial("  Data Pin: " + String(globalConfig.displays[i].data_pin));
        writeSerial("  Partial Update: " + String(globalConfig.displays[i].partial_update_support ? "Yes" : "No"));
        writeSerial("  Color Scheme: 0x" + String(globalConfig.displays[i].color_scheme, HEX));
        writeSerial("  Transmission Modes: 0x" + String(globalConfig.displays[i].transmission_modes, HEX));
        writeSerial("    RAW: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_RAW) ? "enabled" : "disabled"));
        writeSerial("    ZIP: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_ZIP) ? "enabled" : "disabled"));
        writeSerial("    G5: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_G5) ? "enabled" : "disabled"));
        writeSerial("    DIRECT_WRITE: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_DIRECT_WRITE) ? "enabled" : "disabled"));
        writeSerial("    CLEAR_ON_BOOT: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_CLEAR_ON_BOOT) ? "enabled" : "disabled"));
        writeSerial("");
    }
    writeSerial("--- LED Configurations (" + String(globalConfig.led_count) + ") ---");
    for (int i = 0; i < globalConfig.led_count; i++) {
        writeSerial("LED " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.leds[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.leds[i].led_type, HEX));
        writeSerial("  Pins: R=" + String(globalConfig.leds[i].led_1_r) + 
                   " G=" + String(globalConfig.leds[i].led_2_g) + 
                   " B=" + String(globalConfig.leds[i].led_3_b) + 
                   " 4=" + String(globalConfig.leds[i].led_4));
        writeSerial("  Flags: 0x" + String(globalConfig.leds[i].led_flags, HEX));
        writeSerial("");
    }
    writeSerial("--- Sensor Configurations (" + String(globalConfig.sensor_count) + ") ---");
    for (int i = 0; i < globalConfig.sensor_count; i++) {
        writeSerial("Sensor " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.sensors[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.sensors[i].sensor_type, HEX));
        writeSerial("  Bus ID: " + String(globalConfig.sensors[i].bus_id));
        writeSerial("");
    }
    writeSerial("--- Data Bus Configurations (" + String(globalConfig.data_bus_count) + ") ---");
    for (int i = 0; i < globalConfig.data_bus_count; i++) {
        writeSerial("Data Bus " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.data_buses[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.data_buses[i].bus_type, HEX));
        writeSerial("  Pins: 1=" + String(globalConfig.data_buses[i].pin_1) + 
                   " 2=" + String(globalConfig.data_buses[i].pin_2) + 
                   " 3=" + String(globalConfig.data_buses[i].pin_3) + 
                   " 4=" + String(globalConfig.data_buses[i].pin_4) + 
                   " 5=" + String(globalConfig.data_buses[i].pin_5) + 
                   " 6=" + String(globalConfig.data_buses[i].pin_6) + 
                   " 7=" + String(globalConfig.data_buses[i].pin_7));
        writeSerial("  Speed: " + String(globalConfig.data_buses[i].bus_speed_hz) + " Hz");
        writeSerial("  Flags: 0x" + String(globalConfig.data_buses[i].bus_flags, HEX));
        writeSerial("  Pullups: 0x" + String(globalConfig.data_buses[i].pullups, HEX));
        writeSerial("  Pulldowns: 0x" + String(globalConfig.data_buses[i].pulldowns, HEX));
        writeSerial("");
    }
    writeSerial("--- Binary Input Configurations (" + String(globalConfig.binary_input_count) + ") ---");
    for (int i = 0; i < globalConfig.binary_input_count; i++) {
        writeSerial("Binary Input " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.binary_inputs[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.binary_inputs[i].input_type, HEX));
        writeSerial("  Display As: 0x" + String(globalConfig.binary_inputs[i].display_as, HEX));
        writeSerial("  Pins: 1=" + String(globalConfig.binary_inputs[i].reserved_pin_1) + 
                   " 2=" + String(globalConfig.binary_inputs[i].reserved_pin_2) + 
                   " 3=" + String(globalConfig.binary_inputs[i].reserved_pin_3) + 
                   " 4=" + String(globalConfig.binary_inputs[i].reserved_pin_4) + 
                   " 5=" + String(globalConfig.binary_inputs[i].reserved_pin_5) + 
                   " 6=" + String(globalConfig.binary_inputs[i].reserved_pin_6) + 
                   " 7=" + String(globalConfig.binary_inputs[i].reserved_pin_7) + 
                   " 8=" + String(globalConfig.binary_inputs[i].reserved_pin_8));
        writeSerial("  Input Flags: 0x" + String(globalConfig.binary_inputs[i].input_flags, HEX));
        writeSerial("  Invert: 0x" + String(globalConfig.binary_inputs[i].invert, HEX));
        writeSerial("  Pullups: 0x" + String(globalConfig.binary_inputs[i].pullups, HEX));
        writeSerial("  Pulldowns: 0x" + String(globalConfig.binary_inputs[i].pulldowns, HEX));
        writeSerial("");
    }
    writeSerial("=============================");
}

void full_config_init() {
    writeSerial("Initializing config storage...");
    if (initConfigStorage()) {
        writeSerial("Config storage initialized successfully");
    } else {
        writeSerial("Config storage initialization failed");
    }
    writeSerial("Loading global configuration...");
    if (loadGlobalConfig()) {
        writeSerial("Global configuration loaded successfully");
        printConfigSummary();
        clearEncryptionSession();
        encryptionInitialized = true;
        checkResetPin();
#ifdef TARGET_NRF
        if (globalConfig.loaded && (globalConfig.system_config.device_flags & DEVICE_FLAG_XIAOINIT)) {
            writeSerial("Device flag DEVICE_FLAG_XIAOINIT is set, calling xiaoinit()...");
            xiaoinit();
            writeSerial("xiaoinit() completed");
        }
#endif
        if (globalConfig.loaded && (globalConfig.system_config.device_flags & DEVICE_FLAG_WS_PP_INIT)) {
            writeSerial("Device flag DEVICE_FLAG_WS_PP_INIT is set, calling ws_pp_init()...");
            ws_pp_init();
            writeSerial("ws_pp_init() completed");
        }
    } else {
        writeSerial("Global configuration load failed or no config found");
    }
}
