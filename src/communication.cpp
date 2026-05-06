#include "communication.h"
#include "structs.h"
#include "config_parser.h"
#include "encryption.h"
#include "device_control.h"
#include "buzzer_control.h"
#include "display_service.h"

#include <Arduino.h>
#include <string.h>

#ifdef TARGET_ESP32
#include <BLEServer.h>
#include <WiFi.h>
#include "wifi_service.h"
extern BLEServer* pServer;
#endif

#ifdef TARGET_NRF
#include <bluefruit.h>
#endif

void writeSerial(String message, bool newLine = true);
bool isAuthenticated();

static void reloadConfigAfterSave(void) {
    if (!loadGlobalConfig()) {
        writeSerial("WARNING: Config was saved but reload from storage failed (see errors above). "
                    "Reboot may be required.");
        return;
    }
    writeSerial("Config reloaded from storage after save");
    clearEncryptionSession();
#ifdef TARGET_ESP32
    initWiFi();
#endif
}
bool encryptResponse(uint8_t* plaintext, uint16_t plaintext_len, uint8_t* ciphertext,
                    uint16_t* ciphertext_len, uint8_t* nonce, uint8_t* auth_tag);
bool isEncryptionEnabled();
void sendResponseUnencrypted(uint8_t* response, uint8_t len);
void secureEraseConfig();
extern struct SecurityConfig securityConfig;
typedef struct {
    bool active;
    uint32_t totalSize;
    uint32_t receivedSize;
    uint8_t buffer[4096];
    uint32_t expectedChunks;
    uint32_t receivedChunks;
} chunked_write_state_t;
extern chunked_write_state_t chunkedWriteState;
extern uint8_t configReadResponseBuffer[128];
extern uint8_t msd_payload[16];
extern struct GlobalConfig globalConfig;
String getChipIdHex();
float readBatteryVoltage();

#ifdef TARGET_ESP32
struct ResponseQueueItem {
    uint8_t data[512];
    uint16_t len;
    bool pending;
};
extern WiFiClient wifiClient;
extern bool wifiServerConnected;
extern ResponseQueueItem responseQueue[10];
extern uint8_t responseQueueHead;
extern uint8_t responseQueueTail;
static constexpr uint8_t RESPONSE_QUEUE_SIZE_LOCAL = 10;
static constexpr uint16_t MAX_RESPONSE_SIZE_LOCAL = 512;

static void send_wifi_lan_frame(const uint8_t* payload, uint16_t len) {
    if (!wifiServerConnected || !wifiClient.connected() || len == 0) {
        return;
    }
    uint8_t hdr[2] = { (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
    if (wifiClient.write(hdr, 2) != 2 || wifiClient.write(payload, len) != len) {
        writeSerial("ERROR: LAN response write incomplete", true);
    }
}

/** Mirror responses to BLE only when a central is connected; LAN already got send_wifi_lan_frame. */
static void esp32_queue_ble_notify_copy(const uint8_t* response, uint16_t len) {
    if (len > MAX_RESPONSE_SIZE_LOCAL) {
        writeSerial("ERROR: Response too large for queue (" + String(len) + " > " + String(MAX_RESPONSE_SIZE_LOCAL) + ")", true);
        return;
    }
    if (pServer == nullptr || pServer->getConnectedCount() == 0) {
        return;
    }
    uint8_t nextHead = (responseQueueHead + 1) % RESPONSE_QUEUE_SIZE_LOCAL;
    if (nextHead == responseQueueTail) {
        writeSerial("ERROR: Response queue full, dropping response", true);
        return;
    }
    memcpy(responseQueue[responseQueueHead].data, response, len);
    responseQueue[responseQueueHead].len = len;
    responseQueue[responseQueueHead].pending = true;
    responseQueueHead = nextHead;
    writeSerial("ESP32: Response queued (queue size: " + String((responseQueueHead - responseQueueTail + RESPONSE_QUEUE_SIZE_LOCAL) % RESPONSE_QUEUE_SIZE_LOCAL) + ")", true);
}
#endif

#ifdef TARGET_NRF
extern BLECharacteristic imageCharacteristic;
#endif

#ifndef BUILD_VERSION
#define BUILD_VERSION "0.0"
#endif
#ifndef SHA
#define SHA ""
#endif
#define STRINGIFY_LOCAL(x) #x
#define XSTRINGIFY_LOCAL(x) STRINGIFY_LOCAL(x)
#define SHA_STRING_LOCAL XSTRINGIFY_LOCAL(SHA)

static constexpr uint8_t FIRMWARE_SHA_HEX_BYTES = 40;
static const char kFirmwareShaPlaceholder[FIRMWARE_SHA_HEX_BYTES + 1] =
    "0000000000000000000000000000000000000000";

void sendResponseUnencrypted(uint8_t* response, uint8_t len) {
    writeSerial("Sending unencrypted response (error/status):", true);
    writeSerial("  Length: " + String(len) + " bytes", true);
    writeSerial("  Command: 0x" + String(response[0], HEX) + String(response[1], HEX), true);
    String hexDump = "  Full command: ";
    for (int i = 0; i < len && i < 32; i++) {
        if (i > 0) hexDump += " ";
        if (response[i] < 16) hexDump += "0";
        hexDump += String(response[i], HEX);
    }
    if (len > 32) hexDump += " ...";
    writeSerial(hexDump, true);
#ifdef TARGET_ESP32
    send_wifi_lan_frame(response, len);
    esp32_queue_ble_notify_copy(response, len);
#endif
#ifdef TARGET_NRF
    if (Bluefruit.connected() && imageCharacteristic.notifyEnabled()) {
        String nrfHexDump = "NRF: Sending unencrypted response: ";
        for (int i = 0; i < len && i < 32; i++) {
            if (i > 0) nrfHexDump += " ";
            if (response[i] < 16) nrfHexDump += "0";
            nrfHexDump += String(response[i], HEX);
        }
        if (len > 32) nrfHexDump += "...";
        writeSerial(nrfHexDump, true);
        writeSerial("NRF: BLE notification sent (" + String(len) + " bytes)", true);
        imageCharacteristic.notify(response, len);
    } else {
        writeSerial("ERROR: Cannot send BLE response - not connected or notifications not enabled", true);
        writeSerial("  Connected: " + String(Bluefruit.connected() ? "yes" : "no"), true);
        writeSerial("  Notify enabled: " + String(imageCharacteristic.notifyEnabled() ? "yes" : "no"), true);
    }
#endif
}

void sendResponse(uint8_t* response, uint8_t len) {
    static uint8_t encrypted_response[600];
    if (isAuthenticated() && len >= 2) {
        uint16_t command = (response[0] << 8) | response[1];
        uint8_t status = (len >= 3) ? response[2] : 0x00;
        // Encrypt all authenticated responses except auth/version handshakes and FE/FF status.
        // Direct-write / partial-write / LED acks must be encrypted too; LAN/BLE clients decrypt every response.
        if (command != 0x0050 && command != 0x0043 && status != 0xFE && status != 0xFF) {
            uint8_t nonce[16];
            uint8_t auth_tag[12];
            uint16_t encrypted_len = 0;
            if (encryptResponse(response, len, encrypted_response, &encrypted_len, nonce, auth_tag)) {
                writeSerial("Sending encrypted response:", true);
                writeSerial("  Original length: " + String(len) + " bytes", true);
                writeSerial("  Encrypted length: " + String(encrypted_len) + " bytes", true);
                response = encrypted_response;
                len = encrypted_len;
            } else {
                writeSerial("WARNING: Failed to encrypt response, sending unencrypted error response", true);
                uint8_t errorResponse[] = {0xFF, (uint8_t)(command & 0xFF), 0x00};
                response = errorResponse;
                len = sizeof(errorResponse);
            }
        } else {
            writeSerial("Sending unencrypted response (authentication/firmware version/error)", true);
        }
    }

    writeSerial("Sending response:", true);
    writeSerial("  Length: " + String(len) + " bytes", true);
    writeSerial("  Command: 0x" + String(response[0], HEX) + String(response[1], HEX), true);
    String hexDump = "  Full command: ";
    for (int i = 0; i < len && i < 32; i++) {
        if (i > 0) hexDump += " ";
        if (response[i] < 16) hexDump += "0";
        hexDump += String(response[i], HEX);
    }
    if (len > 32) hexDump += " ...";
    writeSerial(hexDump, true);
#ifdef TARGET_ESP32
    send_wifi_lan_frame(response, len);
    esp32_queue_ble_notify_copy(response, len);
#endif
#ifdef TARGET_NRF
    if (Bluefruit.connected() && imageCharacteristic.notifyEnabled()) {
        String nrfHexDump = "NRF: Sending response: ";
        for (int i = 0; i < len && i < 32; i++) {
            if (i > 0) nrfHexDump += " ";
            if (response[i] < 16) nrfHexDump += "0";
            nrfHexDump += String(response[i], HEX);
        }
        if (len > 32) nrfHexDump += "...";
        writeSerial(nrfHexDump, true);
        writeSerial("NRF: BLE notification sent (" + String(len) + " bytes)", true);
        imageCharacteristic.notify(response, len);
        delay(20);
    } else {
        writeSerial("ERROR: Cannot send BLE response - not connected or notifications not enabled", true);
        writeSerial("  Connected: " + String(Bluefruit.connected() ? "yes" : "no"), true);
        writeSerial("  Notify enabled: " + String(imageCharacteristic.notifyEnabled() ? "yes" : "no"), true);
    }
#endif
}

void handleReadMSD() {
    writeSerial("=== READ MSD COMMAND (0x0044) ===", true);
    uint8_t response[2 + 16];
    uint16_t responseLen = 0;
    response[responseLen++] = 0x00;
    response[responseLen++] = 0x44;
    memcpy(&response[responseLen], msd_payload, sizeof(msd_payload));
    responseLen += sizeof(msd_payload);
    sendResponse(response, responseLen);
    writeSerial("MSD read response sent (" + String(responseLen) + " bytes)", true);
}

uint16_t calculateCRC16CCITT(uint8_t* data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
            crc &= 0xFFFF;
        }
    }
    return crc;
}

uint8_t getFirmwareMajor() {
    String version = String(BUILD_VERSION);
    version.trim();
    if (version.length() == 0) {
        return 0;
    }
    int dotIndex = version.indexOf('.');
    if (dotIndex > 0) {
        return version.substring(0, dotIndex).toInt();
    }
    return 0;
}

uint8_t getFirmwareMinor() {
    String version = String(BUILD_VERSION);
    version.trim();
    if (version.length() == 0) {
        return 0;
    }
    int dotIndex = version.indexOf('.');
    if (dotIndex > 0 && dotIndex < (int)(version.length() - 1)) {
        return version.substring(dotIndex + 1).toInt();
    }
    return 0;
}

const char* getFirmwareShaString() {
    return SHA_STRING_LOCAL;
}

void handleFirmwareVersion() {
    writeSerial("Building Firmware Version response...", true);
    uint8_t major = getFirmwareMajor();
    uint8_t minor = getFirmwareMinor();
    String shaStr = String(getFirmwareShaString());
    if (shaStr.length() >= 2 && shaStr.charAt(0) == '"' && shaStr.charAt(shaStr.length() - 1) == '"') {
        shaStr = shaStr.substring(1, shaStr.length() - 1);
    }
    shaStr.trim();
    const bool noShaCompiled = (shaStr.length() == 0 || shaStr == "\"\"");
    if (noShaCompiled) {
        shaStr = kFirmwareShaPlaceholder;
    }
    if (String(BUILD_VERSION).length() == 0) {
        major = 0;
        minor = 0;
    }
    writeSerial("Firmware version: " + String(major) + "." + String(minor), true);
    writeSerial("SHA: " + shaStr, true);
    uint8_t shaLen = shaStr.length();
    if (shaLen > 40) shaLen = 40;
    uint8_t response[2 + 1 + 1 + 1 + 40];
    uint16_t offset = 0;
    response[offset++] = 0x00;
    response[offset++] = 0x43;
    response[offset++] = major;
    response[offset++] = minor;
    response[offset++] = shaLen;
    for (uint8_t i = 0; i < shaLen && i < 40; i++) {
        response[offset++] = shaStr.charAt(i);
    }
    sendResponse(response, offset);
    writeSerial("Firmware version response sent", true);
}

void handleReadConfig() {
    uint8_t configData[4096];
    uint32_t configLen = 4096;
    if (loadConfig(configData, &configLen)) {
        writeSerial("Sending config data in chunks...", true);
        uint32_t remaining = configLen;
        uint32_t offset = 0;
        uint16_t chunkNumber = 0;
        const uint16_t maxChunks = 10;
        while (remaining > 0 && chunkNumber < maxChunks) {
            uint16_t responseLen = 0;
            configReadResponseBuffer[responseLen++] = 0x00;
            configReadResponseBuffer[responseLen++] = 0x40;
            configReadResponseBuffer[responseLen++] = chunkNumber & 0xFF;
            configReadResponseBuffer[responseLen++] = (chunkNumber >> 8) & 0xFF;
            if (chunkNumber == 0) {
                configReadResponseBuffer[responseLen++] = configLen & 0xFF;
                configReadResponseBuffer[responseLen++] = (configLen >> 8) & 0xFF;
            }
            uint16_t maxDataSize = 100 - responseLen;
            uint16_t chunkSize = (remaining < maxDataSize) ? remaining : maxDataSize;
            if (chunkSize == 0) break;
            memcpy(configReadResponseBuffer + responseLen, configData + offset, chunkSize);
            responseLen += chunkSize;
            if (responseLen > 100 || responseLen == 0) break;
            sendResponse(configReadResponseBuffer, responseLen);
            offset += chunkSize;
            remaining -= chunkSize;
            chunkNumber++;
#ifdef TARGET_ESP32
            delay(1);
#else
            delay(50);
#endif
        }
    } else {
        uint8_t errorResponse[] = {0xFF, 0x40, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
    }
}

void handleWriteConfig(uint8_t* data, uint16_t len) {
    if (len == 0) return;
    if (isEncryptionEnabled() && !isAuthenticated()) {
        bool rewriteAllowed = (securityConfig.flags & (1 << 0)) != 0;
        if (!rewriteAllowed) {
            uint8_t response[] = {0x00, (uint8_t)(0x0041 & 0xFF), 0xFE};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }
        secureEraseConfig();
    }
    if (len > 200) {
        chunkedWriteState.active = true;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.expectedChunks = 0;
        chunkedWriteState.receivedChunks = 0;
        if (len >= 202) {
            chunkedWriteState.totalSize = data[0] | (data[1] << 8);
            chunkedWriteState.expectedChunks = (chunkedWriteState.totalSize + 200 - 1) / 200;
            uint16_t chunkDataSize = ((len - 2) < 200) ? (len - 2) : 200;
            memcpy(chunkedWriteState.buffer, data + 2, chunkDataSize);
            chunkedWriteState.receivedSize = chunkDataSize;
            chunkedWriteState.receivedChunks = 1;
        } else {
            uint16_t chunkSize = (len < 200) ? len : 200;
            chunkedWriteState.totalSize = len;
            chunkedWriteState.expectedChunks = 1;
            memcpy(chunkedWriteState.buffer, data, chunkSize);
            chunkedWriteState.receivedSize = chunkSize;
            chunkedWriteState.receivedChunks = 1;
        }
        uint8_t ackResponse[] = {0x00, 0x41, 0x00, 0x00};
        sendResponse(ackResponse, sizeof(ackResponse));
        return;
    }
    uint8_t responseOk[] = {0x00, 0x41, 0x00, 0x00};
    uint8_t responseErr[] = {0xFF, 0x41, 0x00, 0x00};
    bool ok = saveConfig(data, len);
    if (ok) {
        reloadConfigAfterSave();
    }
    sendResponse(ok ? responseOk : responseErr, 4);
}

void handleWriteConfigChunk(uint8_t* data, uint16_t len) {
    if (!chunkedWriteState.active) {
        uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    if (chunkedWriteState.receivedChunks == 1 && isEncryptionEnabled() && !isAuthenticated()) {
        bool rewriteAllowed = (securityConfig.flags & (1 << 0)) != 0;
        if (!rewriteAllowed) {
            chunkedWriteState.active = false;
            uint8_t response[] = {0x00, (uint8_t)(0x0042 & 0xFF), 0xFE};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }
        secureEraseConfig();
    }
    if (len == 0 || len > 200 || chunkedWriteState.receivedSize + len > 4096 || chunkedWriteState.receivedChunks >= 20) {
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    memcpy(chunkedWriteState.buffer + chunkedWriteState.receivedSize, data, len);
    chunkedWriteState.receivedSize += len;
    chunkedWriteState.receivedChunks++;
    if (chunkedWriteState.receivedChunks >= chunkedWriteState.expectedChunks) {
        uint8_t ok[] = {0x00, 0x42, 0x00, 0x00};
        uint8_t err[] = {0xFF, 0x42, 0x00, 0x00};
        bool saved = saveConfig(chunkedWriteState.buffer, chunkedWriteState.receivedSize);
        if (saved) {
            reloadConfigAfterSave();
        }
        sendResponse(saved ? ok : err, 4);
        chunkedWriteState.active = false;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.receivedChunks = 0;
    } else {
        uint8_t ackResponse[] = {0x00, 0x42, 0x00, 0x00};
        sendResponse(ackResponse, sizeof(ackResponse));
    }
}

#ifdef TARGET_NRF
typedef uint16_t BLEConnHandle;
typedef BLECharacteristic* BLECharPtr;
#else
typedef void* BLEConnHandle;
typedef void* BLECharPtr;
#endif

void imageDataWritten(BLEConnHandle conn_hdl, BLECharPtr chr, uint8_t* data, uint16_t len) {
    (void)conn_hdl;
    (void)chr;
    if (len < 2) {
        writeSerial("ERROR: Command too short (" + String(len) + " bytes)");
        return;
    }

    uint16_t command = (data[0] << 8) | data[1];
    writeSerial("Processing command: 0x" + String(command, HEX));

    if (command == 0x0050) {
        writeSerial("=== AUTHENTICATE COMMAND (0x0050) ===");
        handleAuthenticate(data + 2, len - 2);
        return;
    }

    if (command == 0x0043) {
        writeSerial("=== FIRMWARE VERSION COMMAND (0x0043) ===");
        handleFirmwareVersion();
        return;
    }

    if (isEncryptionEnabled()) {
        if (!isAuthenticated()) {
            writeSerial("ERROR: Command requires authentication (encryption enabled)");
            uint8_t response[] = {0x00, (uint8_t)(command & 0xFF), 0xFE};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }

        if (len < 2 + 16 + 12) {
            writeSerial("ERROR: Unencrypted command received when encryption is enabled");
            uint8_t response[] = {0x00, (uint8_t)(command & 0xFF), 0xFE};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }

        uint8_t nonce_full[16];
        uint8_t auth_tag[12];
        static uint8_t plaintext[512];
        uint16_t plaintext_len = 0;

        memcpy(nonce_full, data + 2, 16);
        memcpy(auth_tag, data + len - 12, 12);

        uint16_t encrypted_data_len = len - 2 - 16 - 12;

        static char data_buf[256];
        snprintf(data_buf, sizeof(data_buf), "Encrypted command: len=%u, command=0x%04X, encrypted_data_len=%u",
                 (unsigned int)len, (unsigned int)command, (unsigned int)encrypted_data_len);
        writeSerial(data_buf);
        static char nonce_buf[64];
        snprintf(nonce_buf, sizeof(nonce_buf), "Full nonce: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                 nonce_full[0], nonce_full[1], nonce_full[2], nonce_full[3],
                 nonce_full[4], nonce_full[5], nonce_full[6], nonce_full[7],
                 nonce_full[8], nonce_full[9], nonce_full[10], nonce_full[11],
                 nonce_full[12], nonce_full[13], nonce_full[14], nonce_full[15]);
        writeSerial(nonce_buf);

        if (!decryptCommand(data + 2 + 16, encrypted_data_len, plaintext, &plaintext_len, nonce_full, auth_tag, command)) {
            writeSerial("ERROR: Decryption failed");
            uint8_t response[] = {0x00, (uint8_t)(command & 0xFF), 0xFF};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }

        static uint8_t decrypted_data[512];
        decrypted_data[0] = data[0];
        decrypted_data[1] = data[1];
        memcpy(decrypted_data + 2, plaintext, plaintext_len);
        len = 2 + plaintext_len;
        data = decrypted_data;
    }

    switch (command) {
        case 0x0040:
            writeSerial("=== READ CONFIG COMMAND (0x0040) ===");
            writeSerial("Command received at time: " + String(millis()));
            handleReadConfig();
            writeSerial("Returned from handleReadConfig");
            break;
        case 0x0041:
            writeSerial("=== WRITE CONFIG COMMAND (0x0041) ===");
            handleWriteConfig(data + 2, len - 2);
            break;
        case 0x0042:
            writeSerial("=== WRITE CONFIG CHUNK COMMAND (0x0042) ===");
            handleWriteConfigChunk(data + 2, len - 2);
            break;
        case 0x000F:
            writeSerial("=== Reboot COMMAND (0x000F) ===");
            delay(100);
            reboot();
            break;
        case 0x0043:
            writeSerial("=== FIRMWARE VERSION COMMAND (0x0043) ===");
            handleFirmwareVersion();
            break;
        case 0x0044:
            writeSerial("=== READ MSD COMMAND (0x0044) ===");
            handleReadMSD();
            break;
        case 0x0070:
            writeSerial("=== DIRECT WRITE START COMMAND (0x0070) ===");
            handleDirectWriteStart(data + 2, len - 2);
            break;
        case 0x0071:
            handleDirectWriteData(data + 2, len - 2);
            break;
        case 0x0072:
            writeSerial("=== DIRECT WRITE END COMMAND (0x0072) ===");
            handleDirectWriteEnd(data + 2, len - 2);
            break;
        case 0x0076:
            handlePartialWriteStart(data + 2, len - 2);
            break;
        case 0x0073:
            writeSerial("=== LED ACTIVATE COMMAND (0x0073) ===");
            handleLedActivate(data + 2, len - 2);
            break;
        case 0x0075:
            writeSerial("=== BUZZER ACTIVATE COMMAND (0x0075) ===");
            handleBuzzerActivate(data + 2, len - 2);
            break;
        case 0x0051:
            writeSerial("=== ENTER DFU MODE COMMAND (0x0051) ===");
            enterDFUMode();
            break;
        default:
            writeSerial("ERROR: Unknown command: 0x" + String(command, HEX));
            writeSerial("Expected: 0x0011 (read config), 0x0064 (image info), 0x0065 (block data), or 0x0003 (finalize)");
            break;
    }
    writeSerial("Command processing completed successfully");
}
