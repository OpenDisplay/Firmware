#include "communication.h"
#include "structs.h"
#include "config_parser.h"
#include "encryption.h"
#include "device_control.h"
#include "buzzer_control.h"
#include "display_service.h"
#include "od_log.h"

#include <Arduino.h>
#include <string.h>

#ifdef TARGET_ESP32
#include "ble_init.h"   // NimBLE-Arduino + BLE* aliases
#include <WiFi.h>
#include "wifi_service.h"
extern BLEServer* pServer;
#endif

#ifdef TARGET_NRF
#include <bluefruit.h>
#endif

bool isAuthenticated();
extern struct GlobalConfig globalConfig;

// F4 -- command origin marker. The shared dispatcher (imageDataWritten) and the
// response senders read this to (a) BYPASS the app-layer AES-CCM envelope for
// TLS-secured LAN frames (SECTION 9 rule 4: origin-gated decrypt) and (b) route
// each response back over ONLY its origin transport (no BLE/LAN dual-delivery).
// Everything runs on the single loop() task (BLE queue drain + handleWiFiServer),
// so this plain global needs no locking: it is set immediately before each
// imageDataWritten() call and restored to ORIGIN_BLE after. On non-LAN builds it
// stays ORIGIN_BLE for the whole lifetime (LAN never sets it).
// enum CommandOrigin lives in communication.h so display_service.cpp / main.cpp can
// name the values instead of comparing against a bare 0.
volatile uint8_t g_commandOrigin = ORIGIN_BLE;

// Transport tag for the RX banner and TX dump. Three transports share this
// dispatcher (nRF BLE, ESP32 BLE via commandQueue, ESP32 LAN), and without a tag
// the log cannot show which one a frame took -- in particular whether a frame used
// the TLS CCM-bypass path. Accurate at every call site below because the LAN
// listener sets g_commandOrigin immediately around its dispatch. Always "BLE" on
// nRF and on ESP32 builds without the LAN transport.
uint8_t commandOrigin(void) { return g_commandOrigin; }

static const char* originTag(void) {
    switch (g_commandOrigin) {
        case ORIGIN_LAN_TLS:   return "LAN-TLS";
        case ORIGIN_LAN_PLAIN: return "LAN";
        default:               return "BLE";
    }
}

static void reloadConfigAfterSave(void) {
    if (!loadGlobalConfig()) {
        od_log_warn("WARNING: Config was saved but reload from storage failed (see errors above). "
                    "Reboot may be required.");
        return;
    }
    od_log_info("Config reloaded from storage after save");
    // Live-disable takes effect now: with keep-alive off, drop a still-warm panel
    // here instead of waiting out the stale (<=30 s) deadline armed by the last push.
    if (globalConfig.power_option.screen_timeout_seconds == 0 && epdSessionIsWarm()) {
        epdSessionForceOff();
    }
    clearEncryptionSession();
#ifdef OPENDISPLAY_HAS_WIFI
    // Non-blocking: a config write arrives over a live BLE link, and the blocking
    // form stalls the loop task for up to 36 s of connect retries, freezing BLE
    // command processing. handleWiFiServer() starts the LAN server on association.
    initWiFi(false);
#endif
}
bool encryptResponse(uint8_t* plaintext, uint16_t plaintext_len, uint8_t* ciphertext,
                    uint16_t* ciphertext_len, uint8_t* nonce, uint8_t* auth_tag);
bool isEncryptionEnabled();
void sendResponseUnencrypted(uint8_t* response, uint16_t len);
void secureEraseConfig();
extern struct SecurityConfig securityConfig;
// chunked_write_state_t comes from config_parser.h; this file used to redefine it
// with a hardcoded 4096 in place of MAX_CONFIG_SIZE.
extern chunked_write_state_t chunkedWriteState;
extern uint8_t configReadResponseBuffer[128];
extern uint8_t msd_payload[16];
String getChipIdHex();
float readBatteryVoltage();

#ifdef TARGET_ESP32
// ResponseQueueItem / RESPONSE_QUEUE_SIZE / MAX_RESPONSE_SIZE come from structs.h.
// This file previously redeclared the struct with a hardcoded 512 and kept private
// *_LOCAL copies of both sizes, so the guard in esp32_queue_ble_notify_copy() and the
// slot it protects were sized in two different files and had to be edited in lockstep.
extern WiFiClient wifiClient;
extern bool wifiServerConnected;
extern ResponseQueueItem responseQueue[RESPONSE_QUEUE_SIZE];
extern uint8_t responseQueueHead;
extern uint8_t responseQueueTail;
// Drains the response ring to BLE (defined in main.cpp). handleReadConfig() calls
// this between chunks so a multi-chunk config read never overflows the ring.
extern void flushResponseQueueToBle();

/** Mirror responses to BLE only when a central is connected; LAN responses go via opendisplay_lan_send_frame. */
static void esp32_queue_ble_notify_copy(const uint8_t* response, uint16_t len, bool quiet = false) {
    if (len > MAX_RESPONSE_SIZE) {
        od_log_error("ERROR: Response too large for queue (%u > %u)", len, MAX_RESPONSE_SIZE);
        return;
    }
    if (pServer == nullptr || pServer->getConnectedCount() == 0) {
        return;
    }
    uint8_t nextHead = (responseQueueHead + 1) % RESPONSE_QUEUE_SIZE;
    if (nextHead == responseQueueTail) {
        od_log_error("ERROR: Response queue full, dropping response");
        return;
    }
    memcpy(responseQueue[responseQueueHead].data, response, len);
    responseQueue[responseQueueHead].len = len;
    responseQueue[responseQueueHead].pending = true;
    responseQueueHead = nextHead;
    // The "[BLE][Q:n] TX ..." line above already reports every response and its
    // queue depth, so the nominal enqueue (depth 1, drained next loop pass) is
    // pure noise. Log only when a backlog is forming and the 10-slot ring is at
    // risk of dropping responses.
    const uint8_t depth = (responseQueueHead - responseQueueTail + RESPONSE_QUEUE_SIZE) % RESPONSE_QUEUE_SIZE;
    if (!quiet && depth >= 2) od_log_debug("BLE: response queue backlog (queue size: %u)", depth);
}
#endif

#ifdef TARGET_NRF
extern BLECharacteristic imageCharacteristic;
#endif

#ifndef BUILD_VERSION
#define BUILD_VERSION "1.0.0"
#endif
#ifndef SHA
#define SHA ""
#endif
#define STRINGIFY_LOCAL(x) #x
#define XSTRINGIFY_LOCAL(x) STRINGIFY_LOCAL(x)
#define SHA_STRING_LOCAL XSTRINGIFY_LOCAL(SHA)
// Always stringify so -DBUILD_VERSION=2.24.0 works: three-part tags are not
// valid C numeric literals (two-part tags like 2.23 accidentally compiled as floats).
#define BUILD_VERSION_STRING_LOCAL XSTRINGIFY_LOCAL(BUILD_VERSION)

static constexpr uint8_t FIRMWARE_SHA_HEX_BYTES = 40;
static const char kFirmwareShaPlaceholder[FIRMWARE_SHA_HEX_BYTES + 1] =
    "0000000000000000000000000000000000000000";

// BUILD_VERSION is major.minor or major.minor.patch (optional leading 'v').
// Two-part tags imply patch 0. Component index: 0=major, 1=minor, 2=patch.
static uint8_t parseFirmwareVersionComponent(unsigned index) {
    const char* v = BUILD_VERSION_STRING_LOCAL;
    if (v == nullptr || v[0] == '\0') {
        return 0;
    }
    // XSTRINGIFY of a quoted macro yields "\"1.0.0\""; of an unquoted
    // 2.24.0 yields "2.24.0". Strip one leading quote when present.
    if (v[0] == '"') {
        v++;
    }
    while (*v == ' ' || *v == 'v' || *v == 'V') {
        v++;
    }
    for (unsigned i = 0; i < index; i++) {
        while (*v >= '0' && *v <= '9') {
            v++;
        }
        if (*v != '.') {
            return 0;
        }
        v++;
    }
    if (*v < '0' || *v > '9') {
        return 0;
    }
    unsigned n = 0;
    while (*v >= '0' && *v <= '9') {
        n = n * 10U + (unsigned)(*v - '0');
        if (n > 255U) {
            return 255;
        }
        v++;
    }
    return (uint8_t)n;
}

// Builds "<label><space-separated %02X bytes, up to 32><' ...' if truncated>" into buf.
static void buildHexDump(char* buf, size_t bufSize, const char* label, const uint8_t* data, uint16_t len) {
    int pos = snprintf(buf, bufSize, "%s", label);
    if (pos < 0) {
        pos = 0;
        buf[0] = '\0';
    }
    int dumpLen = (len < 32) ? len : 32;
    for (int i = 0; i < dumpLen && pos < (int)bufSize; i++) {
        int n = snprintf(buf + pos, bufSize - pos, i > 0 ? " %02X" : "%02X", data[i]);
        if (n < 0) {
            break;
        }
        pos += n;
    }
    if (len > 32 && pos >= 0 && pos < (int)bufSize) {
        snprintf(buf + pos, bufSize - pos, " ...");
    }
}

void sendResponseUnencrypted(uint8_t* response, uint16_t len) {
    od_log_debug("Sending unencrypted response (error/status):");
    od_log_debug("  Length: %u bytes", len);
    od_log_debug("  Command: 0x%02X%02X", response[0], response[1]);
    char hexDump[160];
    buildHexDump(hexDump, sizeof(hexDump), "  Full command: ", response, len);
    od_log_debug("%s", hexDump);
#ifdef TARGET_ESP32
    // F4 de-fan-out: reply over the origin transport only.
    if (g_commandOrigin == ORIGIN_BLE) {
        esp32_queue_ble_notify_copy(response, len);
    } else {
#ifdef OPENDISPLAY_HAS_WIFI
        opendisplay_lan_send_frame(response, len);
#endif
    }
#endif
#ifdef TARGET_NRF
    if (Bluefruit.connected() && imageCharacteristic.notifyEnabled()) {
        char nrfHexDump[160] = {0};
        buildHexDump(nrfHexDump, sizeof(nrfHexDump), "NRF: Sending unencrypted response: ", response, len);
        od_log_debug("%s", nrfHexDump);
        od_log_debug("NRF: BLE notification sent (%u bytes)", len);
        imageCharacteristic.notify(response, len);
    } else {
        od_log_error("ERROR: Cannot send BLE response - not connected or notifications not enabled");
        od_log_error("  Connected: %s", Bluefruit.connected() ? "yes" : "no");
        od_log_error("  Notify enabled: %s", imageCharacteristic.notifyEnabled() ? "yes" : "no");
    }
#endif
}

void sendResponse(uint8_t* response, uint16_t len) {
    static uint8_t encrypted_response[600];
    uint8_t errorResponse[3];
    // Suppress the 4-line dump for the per-frame 0x0071 image-write ack once the
    // stream is past its first chunk (chunk 1's ack still logs). Computed before
    // `response` is swapped to the encrypted buffer. Errors/NACKs start with 0xFF
    // and never match, so they always log.
    // Also suppress the 7-byte PIPE ACK {00 81 highest_seen mask:4} mid-stream.
    // Length test uses the plaintext ACK (encryption happens after this check).
    const bool quietAck = (len == 2 && response[0] == 0x00 && response[1] == 0x71 && imageWriteLogQuietAck())
                       || (len == 7 && response[0] == 0x00 && response[1] == 0x81 && imageWriteLogQuietAck());
    // TLS-origin responses are already secured by the TLS record layer; never wrap
    // them in the app-layer CCM envelope (no double-encrypt; SECTION 9 rule 4).
    if (isAuthenticated() && len >= 2 && g_commandOrigin != ORIGIN_LAN_TLS) {
        uint16_t command = (response[0] << 8) | response[1];
        // The 7-byte PIPE data ACK {0x00,0x81,highest_seen,mask:4} carries a rolling
        // seq at byte[2]; a highest_seen of 0xFE/0xFF (any image >= 255 chunks) must
        // not trip the unencrypted-status heuristic below — pipe ACKs encrypt
        // normally when authenticated (plan 1.6). Other pipe shapes never collide:
        // 0x80 response byte[2] = ver (0x01), pipe NACK byte[2] = err (0x01-0x04),
        // 0x82 acks are 2 bytes (status defaults to 0x00).
        const bool pipeDataAck = (len == 7 && response[0] == 0x00 && response[1] == 0x81);
        uint8_t status = (len >= 3 && !pipeDataAck) ? response[2] : 0x00;
        // Encrypt all authenticated responses except auth/version handshakes and FE/FF status.
        // Direct-write / partial-write / LED acks must be encrypted too; LAN/BLE clients decrypt every response.
        if (command != CMD_AUTHENTICATE && command != CMD_FIRMWARE_VERSION && status != RESP_AUTH_REQUIRED && status != RESP_NACK) {
            uint8_t nonce[ENCRYPTION_NONCE_SIZE];
            uint8_t auth_tag[ENCRYPTION_TAG_SIZE];
            uint16_t encrypted_len = 0;
            if (encryptResponse(response, len, encrypted_response, &encrypted_len, nonce, auth_tag)) {
                if (!quietAck) {
                    od_log_debug("[%s] Sending encrypted response:", originTag());
                    od_log_debug("  Original length: %u bytes", len);
                    od_log_debug("  Encrypted length: %u bytes", encrypted_len);
                }
                response = encrypted_response;
                len = encrypted_len;
            } else {
                od_log_warn("WARNING: Failed to encrypt response, sending unencrypted error response");
                errorResponse[0] = RESP_NACK;
                errorResponse[1] = (uint8_t)(command & 0xFF);
                errorResponse[2] = 0x00;
                response = errorResponse;
                len = sizeof(errorResponse);
            }
        } else if (!quietAck) {
            od_log_debug("[%s] Sending unencrypted response (authentication/firmware version/error)", originTag());
        }
    }

    if (!quietAck) {
        // One-line TX log: opcode, length, and up to 32 payload bytes (the opcode
        // is also the first two bytes of the dump). Replaces the old 4-line block.
        uint16_t cmd = (response[0] << 8) | response[1];
        char line[160];
        int pos;
#ifdef TARGET_ESP32
        // Queue depth *before* this response is enqueued, so a healthy path reads
        // [BLE][Q:0] and a rising Q flags the drain falling behind the producer.
        if (g_commandOrigin == ORIGIN_BLE) {
            const uint8_t qdepth = (responseQueueHead - responseQueueTail + RESPONSE_QUEUE_SIZE) % RESPONSE_QUEUE_SIZE;
            pos = snprintf(line, sizeof(line), "[%s][Q:%u] TX 0x%04X (%u B):",
                           originTag(), (unsigned)qdepth, cmd, (unsigned)len);
        } else
#endif
        pos = snprintf(line, sizeof(line), "[%s] TX 0x%04X (%u B):", originTag(), cmd, (unsigned)len);
        if (pos < 0) {
            pos = 0;
            line[0] = '\0';
        }
        int dumpLen = (len < 32) ? len : 32;
        for (int i = 0; i < dumpLen && pos < (int)sizeof(line); i++) {
            int n = snprintf(line + pos, sizeof(line) - pos, " %02X", response[i]);
            if (n < 0) {
                break;
            }
            pos += n;
        }
        if (len > 32 && pos >= 0 && pos < (int)sizeof(line)) {
            snprintf(line + pos, sizeof(line) - pos, " ...");
        }
        od_log_debug("%s", line);
    }
#ifdef TARGET_ESP32
    // F4 de-fan-out: reply over the origin transport only.
    if (g_commandOrigin == ORIGIN_BLE) {
        esp32_queue_ble_notify_copy(response, len, quietAck);
    } else {
#ifdef OPENDISPLAY_HAS_WIFI
        opendisplay_lan_send_frame(response, len);
#endif
    }
#endif
#ifdef TARGET_NRF
    if (Bluefruit.connected() && imageCharacteristic.notifyEnabled()) {
        if (!quietAck) {
            char nrfHexDump[160] = {0};
            buildHexDump(nrfHexDump, sizeof(nrfHexDump), "NRF: Sending response: ", response, len);
            od_log_debug("%s", nrfHexDump);
            od_log_debug("NRF: BLE notification sent (%u bytes)", len);
        }
        // Bounded retry only when the SoftDevice TX queue is full (notify()==false).
        // Replaces an unconditional delay(20): pays latency only on backpressure and
        // speeds the legacy per-chunk path ~20 ms/chunk.
        bool notified = imageCharacteristic.notify(response, len);
        for (uint8_t attempt = 0; !notified && attempt < 4; ++attempt) {
            delay(5);
            notified = imageCharacteristic.notify(response, len);
        }
    } else {
        od_log_error("ERROR: Cannot send BLE response - not connected or notifications not enabled");
        od_log_error("  Connected: %s", Bluefruit.connected() ? "yes" : "no");
        od_log_error("  Notify enabled: %s", imageCharacteristic.notifyEnabled() ? "yes" : "no");
    }
#endif
}

void handleReadMSD() {
    uint8_t response[2 + 16];
    uint16_t responseLen = 0;
    response[responseLen++] = RESP_ACK;
    response[responseLen++] = RESP_MSD_READ;
    memcpy(&response[responseLen], msd_payload, sizeof(msd_payload));
    responseLen += sizeof(msd_payload);
    sendResponse(response, responseLen);
    od_log_debug("MSD read response sent (%u bytes)", responseLen);
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
    return parseFirmwareVersionComponent(0);
}

uint8_t getFirmwareMinor() {
    return parseFirmwareVersionComponent(1);
}

uint8_t getFirmwarePatch() {
    return parseFirmwareVersionComponent(2);
}

const char* getFirmwareShaString() {
    return SHA_STRING_LOCAL;
}

void handleFirmwareVersion() {
    uint8_t major = getFirmwareMajor();
    uint8_t minor = getFirmwareMinor();
    uint8_t patch = getFirmwarePatch();
    String shaStr = String(getFirmwareShaString());
    if (shaStr.length() >= 2 && shaStr.charAt(0) == '"' && shaStr.charAt(shaStr.length() - 1) == '"') {
        shaStr = shaStr.substring(1, shaStr.length() - 1);
    }
    shaStr.trim();
    const bool noShaCompiled = (shaStr.length() == 0 || shaStr == "\"\"");
    if (noShaCompiled) {
        shaStr = kFirmwareShaPlaceholder;
    }
    od_log_info("Firmware version: %u.%u.%u", major, minor, patch);
    od_log_info("SHA: %s", shaStr.c_str());
    uint8_t shaLen = shaStr.length();
    if (shaLen > 40) shaLen = 40;
    // [ACK][0x43][major][minor][shaLen][sha…][patch] — patch is trailing so
    // old hosts that stop after SHA keep working.
    uint8_t response[2 + 1 + 1 + 1 + 40 + 1];
    uint16_t offset = 0;
    response[offset++] = RESP_ACK;
    response[offset++] = RESP_FIRMWARE_VERSION;
    response[offset++] = major;
    response[offset++] = minor;
    response[offset++] = shaLen;
    for (uint8_t i = 0; i < shaLen && i < 40; i++) {
        response[offset++] = shaStr.charAt(i);
    }
    response[offset++] = patch;
    sendResponse(response, offset);
}

void handleReadConfig() {
    // Shared scratch rather than a 4 KB stack array: this runs on the loop task,
    // where a 4 KB frame is a real overflow risk. Nothing below re-enters a config
    // path, so no other consumer can claim the scratch while we hold it.
    uint8_t* configData = getConfigScratch();
    uint32_t configLen = MAX_CONFIG_SIZE;
    if (loadConfig(configData, &configLen)) {
        uint32_t remaining = configLen;
        uint32_t offset = 0;
        uint16_t chunkNumber = 0;
        // Cover the full MAX_CONFIG_SIZE. Worst-case per-chunk payload is 94 B
        // (chunk 0 also carries the 2-byte total-length header; later chunks
        // carry 96), so ceil(MAX_CONFIG_SIZE / 94) chunks always sends it all.
        const uint16_t maxChunks = (MAX_CONFIG_SIZE + 93) / 94;
        while (remaining > 0 && chunkNumber < maxChunks) {
            uint16_t responseLen = 0;
            configReadResponseBuffer[responseLen++] = RESP_ACK;
            configReadResponseBuffer[responseLen++] = RESP_CONFIG_READ;
            configReadResponseBuffer[responseLen++] = chunkNumber & 0xFF;
            configReadResponseBuffer[responseLen++] = (chunkNumber >> 8) & 0xFF;
            if (chunkNumber == 0) {
                configReadResponseBuffer[responseLen++] = configLen & 0xFF;
                configReadResponseBuffer[responseLen++] = (configLen >> 8) & 0xFF;
            }
            uint16_t maxDataSize = MAX_RESPONSE_DATA_SIZE - responseLen;
            uint16_t chunkSize = (remaining < maxDataSize) ? remaining : maxDataSize;
            if (chunkSize == 0) break;
            memcpy(configReadResponseBuffer + responseLen, configData + offset, chunkSize);
            responseLen += chunkSize;
            if (responseLen > MAX_RESPONSE_DATA_SIZE || responseLen == 0) break;
            sendResponse(configReadResponseBuffer, responseLen);
            offset += chunkSize;
            remaining -= chunkSize;
            chunkNumber++;
#ifdef TARGET_ESP32
            // Drain THIS chunk to BLE before enqueuing the next: this handler runs
            // synchronously on the loop task, so the ring's only drainer cannot run
            // until we return. Without this, chunk 9+ overflows the 10-slot ring and
            // is silently dropped, truncating configs > ~864 B on read-back.
            flushResponseQueueToBle();
#else
            delay(50);
#endif
        }
    } else {
        uint8_t errorResponse[] = {RESP_NACK, RESP_CONFIG_READ, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
    }
}

void handleWriteConfig(uint8_t* data, uint16_t len) {
    if (len == 0) return;
    if (isEncryptionEnabled() && !isAuthenticated()) {
        bool rewriteAllowed = (securityConfig.flags & (1 << 0)) != 0;
        if (!rewriteAllowed) {
            uint8_t response[] = {RESP_ACK, (uint8_t)(CMD_CONFIG_WRITE & 0xFF), RESP_AUTH_REQUIRED};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }
        secureEraseConfig();
    }
    if (len > CONFIG_CHUNK_SIZE) {
        chunkedWriteState.active = true;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.expectedChunks = 0;
        chunkedWriteState.receivedChunks = 0;
        if (len >= CONFIG_CHUNK_SIZE_WITH_PREFIX) {
            chunkedWriteState.totalSize = data[0] | (data[1] << 8);
            chunkedWriteState.expectedChunks = (chunkedWriteState.totalSize + CONFIG_CHUNK_SIZE - 1) / CONFIG_CHUNK_SIZE;
            uint16_t chunkDataSize = ((len - 2) < CONFIG_CHUNK_SIZE) ? (len - 2) : CONFIG_CHUNK_SIZE;
            memcpy(chunkedWriteState.buffer, data + 2, chunkDataSize);
            chunkedWriteState.receivedSize = chunkDataSize;
            chunkedWriteState.receivedChunks = 1;
        } else {
            uint16_t chunkSize = (len < CONFIG_CHUNK_SIZE) ? len : CONFIG_CHUNK_SIZE;
            chunkedWriteState.totalSize = len;
            chunkedWriteState.expectedChunks = 1;
            memcpy(chunkedWriteState.buffer, data, chunkSize);
            chunkedWriteState.receivedSize = chunkSize;
            chunkedWriteState.receivedChunks = 1;
        }
        uint8_t ackResponse[] = {RESP_ACK, RESP_CONFIG_WRITE, 0x00, 0x00};
        sendResponse(ackResponse, sizeof(ackResponse));
        return;
    }
    uint8_t responseOk[] = {RESP_ACK, RESP_CONFIG_WRITE, 0x00, 0x00};
    uint8_t responseErr[] = {RESP_NACK, RESP_CONFIG_WRITE, 0x00, 0x00};
    bool ok = saveConfig(data, len);
    if (ok) {
        reloadConfigAfterSave();
    }
    sendResponse(ok ? responseOk : responseErr, 4);
}

void handleClearConfig(void) {
    uint8_t responseOk[] = {RESP_ACK, RESP_CONFIG_CLEAR, 0x00, 0x00};
    uint8_t responseErr[] = {RESP_NACK, RESP_CONFIG_CLEAR, 0x00, 0x00};

    if (!clearStoredConfig()) {
        sendResponse(responseErr, sizeof(responseErr));
        return;
    }

    od_log_info("Stored config cleared");
    sendResponse(responseOk, sizeof(responseOk));
}

void handleWriteConfigChunk(uint8_t* data, uint16_t len) {
    if (!chunkedWriteState.active) {
        uint8_t errorResponse[] = {RESP_NACK, RESP_CONFIG_CHUNK, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    if (chunkedWriteState.receivedChunks == 1 && isEncryptionEnabled() && !isAuthenticated()) {
        bool rewriteAllowed = (securityConfig.flags & (1 << 0)) != 0;
        if (!rewriteAllowed) {
            chunkedWriteState.active = false;
            uint8_t response[] = {RESP_ACK, (uint8_t)(CMD_CONFIG_CHUNK & 0xFF), RESP_AUTH_REQUIRED};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }
        secureEraseConfig();
    }
    if (len == 0 || len > CONFIG_CHUNK_SIZE || chunkedWriteState.receivedSize + len > MAX_CONFIG_SIZE || chunkedWriteState.receivedChunks >= MAX_CONFIG_CHUNKS) {
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {RESP_NACK, RESP_CONFIG_CHUNK, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    memcpy(chunkedWriteState.buffer + chunkedWriteState.receivedSize, data, len);
    chunkedWriteState.receivedSize += len;
    chunkedWriteState.receivedChunks++;
    if (chunkedWriteState.receivedChunks >= chunkedWriteState.expectedChunks) {
        uint8_t ok[] = {RESP_ACK, RESP_CONFIG_CHUNK, 0x00, 0x00};
        uint8_t err[] = {RESP_NACK, RESP_CONFIG_CHUNK, 0x00, 0x00};
        bool saved = saveConfig(chunkedWriteState.buffer, chunkedWriteState.receivedSize);
        if (saved) {
            reloadConfigAfterSave();
        }
        sendResponse(saved ? ok : err, 4);
        chunkedWriteState.active = false;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.receivedChunks = 0;
    } else {
        uint8_t ackResponse[] = {RESP_ACK, RESP_CONFIG_CHUNK, 0x00, 0x00};
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

// Human-readable name for a command opcode, used for the single dispatch banner
// emitted by imageDataWritten() (the shared command handler for nRF, ESP32 BLE,
// and the ESP32 LAN transport). Returns nullptr for opcodes not dispatched here
// (incl. CMD_NFC_ENDPOINT 0x0083, which this Firmware does not implement on any
// target) — the switch default logs those as unknown. Single source of truth for
// the banner text: keep in sync with the dispatch switch below; individual
// cases/handlers must NOT log their own "=== ... COMMAND ... ===" banner.
static const char* commandName(uint16_t cmd) {
    switch (cmd) {
        case CMD_REBOOT:              return "REBOOT";              // 0x000F
        case CMD_CONFIG_READ:         return "READ CONFIG";         // 0x0040
        case CMD_CONFIG_WRITE:        return "WRITE CONFIG";        // 0x0041
        case CMD_CONFIG_CHUNK:        return "WRITE CONFIG CHUNK";  // 0x0042
        case CMD_FIRMWARE_VERSION:    return "FIRMWARE VERSION";    // 0x0043
        case CMD_READ_MSD:            return "READ MSD";            // 0x0044
        case CMD_CONFIG_CLEAR:        return "CLEAR CONFIG";        // 0x0045
        case CMD_AUTHENTICATE:        return "AUTHENTICATE";        // 0x0050
        case CMD_ENTER_DFU:           return "ENTER DFU MODE";      // 0x0051
        case CMD_POWER_OFF:           return "POWER OFF";           // 0x0052
        case CMD_DEEP_SLEEP:          return "DEEP SLEEP";          // 0x0053
        case CMD_DIRECT_WRITE_START:  return "DIRECT WRITE START";  // 0x0070
        case CMD_DIRECT_WRITE_DATA:   return "DIRECT WRITE DATA";   // 0x0071
        case CMD_DIRECT_WRITE_END:    return "DIRECT WRITE END";    // 0x0072
        case CMD_LED_ACTIVATE:        return "LED ACTIVATE";        // 0x0073
        case CMD_LED_STOP:            return "LED STOP";            // 0x0075
        case CMD_PARTIAL_WRITE_START: return "PARTIAL WRITE START"; // 0x0076
        case CMD_BUZZER:              return "BUZZER ACTIVATE";     // 0x0077
        case CMD_PIPE_WRITE_START:    return "PIPE WRITE START";    // 0x0080
        case CMD_PIPE_WRITE_DATA:     return "PIPE WRITE DATA";     // 0x0081
        case CMD_PIPE_WRITE_END:      return "PIPE WRITE END";      // 0x0082
        default:                      return nullptr;
    }
}

void imageDataWritten(BLEConnHandle conn_hdl, BLECharPtr chr, uint8_t* data, uint16_t len) {
    (void)conn_hdl;
    (void)chr;
    if (len < 2) {
        od_log_error("ERROR: Command too short (%u bytes)", len);
        return;
    }

    uint16_t command = (data[0] << 8) | data[1];
    // Silence the per-frame command spam for image-write data (0x0071) once the
    // stream is past its first chunk; the display handler's 5% meter reports it.
    const bool quietCmd = (command == CMD_DIRECT_WRITE_DATA || command == CMD_PIPE_WRITE_DATA) && imageWriteLogQuietCmd();
    // Single per-command banner for the whole dispatch. Named via commandName();
    // unknown opcodes (nullptr) get no banner here and fall to the switch default's
    // "Unknown command" error. Cases and handlers must not log their own banner.
    if (!quietCmd) {
        const char* name = commandName(command);
        if (name != nullptr) {
            od_log_info("=== [%s] %s COMMAND (0x%04X) ===", originTag(), name, command);
        }
    }

    // AUTHENTICATE and FIRMWARE_VERSION are handled before the encryption gate
    // (they are the handshake). The banner is already emitted above via commandName().
    if (command == CMD_AUTHENTICATE) {
        handleAuthenticate(data + 2, len - 2);
        return;
    }

    if (command == CMD_FIRMWARE_VERSION) {
        handleFirmwareVersion();
        return;
    }

    // SECTION 9 rule 4 (origin-gated decrypt): a frame arriving on the TLS-PSK LAN
    // channel is already confidential + authenticated by TLS, so the app-layer
    // AES-CCM envelope MUST NOT be required/applied — dispatch its plaintext
    // command directly. BLE and plaintext-LAN frames still honor the CCM gate.
    if (isEncryptionEnabled() && g_commandOrigin != ORIGIN_LAN_TLS) {
        if (!isAuthenticated()) {
            od_log_error("ERROR: [%s] Command requires authentication (encryption enabled)", originTag());
            uint8_t response[] = {RESP_ACK, (uint8_t)(command & 0xFF), RESP_AUTH_REQUIRED};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }

        if (len < BLE_CMD_HEADER_SIZE + ENCRYPTION_NONCE_SIZE + ENCRYPTION_TAG_SIZE) {
            od_log_error("ERROR: [%s] Unencrypted command received when encryption is enabled", originTag());
            uint8_t response[] = {RESP_ACK, (uint8_t)(command & 0xFF), RESP_AUTH_REQUIRED};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }

        uint8_t nonce_full[ENCRYPTION_NONCE_SIZE];
        uint8_t auth_tag[ENCRYPTION_TAG_SIZE];
        static uint8_t plaintext[512];
        uint16_t plaintext_len = 0;

        memcpy(nonce_full, data + BLE_CMD_HEADER_SIZE, ENCRYPTION_NONCE_SIZE);
        memcpy(auth_tag, data + len - ENCRYPTION_TAG_SIZE, ENCRYPTION_TAG_SIZE);

        uint16_t encrypted_data_len = len - BLE_CMD_HEADER_SIZE - ENCRYPTION_NONCE_SIZE - ENCRYPTION_TAG_SIZE;

        if (!quietCmd) {
            od_log_debug("Encrypted command: len=%u, command=0x%04X, encrypted_data_len=%u",
                         (unsigned int)len, (unsigned int)command, (unsigned int)encrypted_data_len);
            od_log_debug("Full nonce: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                         nonce_full[0], nonce_full[1], nonce_full[2], nonce_full[3],
                         nonce_full[4], nonce_full[5], nonce_full[6], nonce_full[7],
                         nonce_full[8], nonce_full[9], nonce_full[10], nonce_full[11],
                         nonce_full[12], nonce_full[13], nonce_full[14], nonce_full[15]);
        }

        if (!decryptCommand(data + BLE_CMD_HEADER_SIZE + ENCRYPTION_NONCE_SIZE, encrypted_data_len, plaintext, &plaintext_len, nonce_full, auth_tag, command)) {
            od_log_error("ERROR: Decryption failed");
            uint8_t response[] = {RESP_ACK, (uint8_t)(command & 0xFF), RESP_NACK};
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

    // The per-command banner is logged once above (commandName()); cases below do
    // NOT log their own "=== ... COMMAND ... ===". CMD_AUTHENTICATE and
    // CMD_FIRMWARE_VERSION are handled by the early returns above and so are absent
    // here. CMD_NFC_ENDPOINT (0x0083) is intentionally not handled by this Firmware
    // (any target) — it falls to default as an unknown command.
    switch (command) {
        case CMD_REBOOT:              // 0x000F
            delay(100);
            reboot();
            break;
        case CMD_CONFIG_READ:         // 0x0040
            handleReadConfig();
            break;
        case CMD_CONFIG_WRITE:        // 0x0041
            handleWriteConfig(data + 2, len - 2);
            break;
        case CMD_CONFIG_CHUNK:        // 0x0042
            handleWriteConfigChunk(data + 2, len - 2);
            break;
        case CMD_READ_MSD:            // 0x0044
            handleReadMSD();
            break;
        case CMD_CONFIG_CLEAR:        // 0x0045
            handleClearConfig();
            break;
        case CMD_ENTER_DFU:           // 0x0051
            enterDFUMode();
            break;
        case CMD_POWER_OFF:           // 0x0052
            handlePowerOffCommand(data + 2, len - 2);
            break;
        case CMD_DEEP_SLEEP:          // 0x0053
            handleDeepSleepCommand(data + 2, len - 2);
            break;
        case CMD_DIRECT_WRITE_START:  // 0x0070
            handleDirectWriteStart(data + 2, len - 2);
            break;
        case CMD_DIRECT_WRITE_DATA:   // 0x0071
            handleDirectWriteData(data + 2, len - 2);
            break;
        case CMD_DIRECT_WRITE_END:    // 0x0072
            handleDirectWriteEnd(data + 2, len - 2);
            break;
        case CMD_LED_ACTIVATE:        // 0x0073
            handleLedActivate(data + 2, len - 2);
            break;
        case CMD_LED_STOP:            // 0x0075
            handleLedStop(data + 2, len - 2);
            break;
        case CMD_PARTIAL_WRITE_START: // 0x0076
            handlePartialWriteStart(data + 2, len - 2);
            break;
        case CMD_BUZZER:              // 0x0077
            handleBuzzerActivate(data + 2, len - 2);
            break;
        case CMD_PIPE_WRITE_START:    // 0x0080
            handlePipeWriteStart(data + 2, len - 2);
            break;
        case CMD_PIPE_WRITE_DATA:     // 0x0081
            // The replay counter (verifyNonceReplay) already advanced at decrypt time,
            // above this switch, for every 0x0081 frame — including ones the handler
            // then queues or discards — so drops/dupes never desync it and the counter
            // delta stays within in-flight <= W <= 32 <= the +-32 replay window.
            handlePipeWriteData(data + 2, len - 2);
            break;
        case CMD_PIPE_WRITE_END:      // 0x0082
            handlePipeWriteEnd(data + 2, len - 2);
            break;
        default:
            od_log_error("ERROR: Unknown command: 0x%04X", command);
            break;
    }
}
