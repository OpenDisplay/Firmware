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

#include "ble_transport.h"
#include "command_queue.h"

#ifdef TARGET_ESP32
// wifi_service.h only, deliberately: this file talks to the LAN transport
// through opendisplay_lan_send_frame() and needs no WiFi stack types. <WiFi.h>
// was here solely for two extern declarations that nothing used.
#include "wifi_service.h"
#endif

#include "link_owner.h"
#include "session_guard.h"

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

// Instance identity of the frame currently being dispatched -- the packed owner
// word (link_owner.h) of the connection that WROTE it, not merely its transport.
//
// g_commandOrigin says BLE-or-LAN and nothing more, which is not enough to decide
// whether a frame still belongs to the live session: BLE conn handles are reused,
// so a frame queued by a dead instance is indistinguishable from the new owner's by
// transport alone. serviceBleRx() sets this from the frame's own tag (which
// CommandQueueItem carries from the write callback) and the LAN listener sets it
// from the LAN owner's identity, both immediately before dispatch. Same
// single-loop-task argument as g_commandOrigin, so no locking.
volatile uint32_t g_commandInstance = 0;

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

// --- auth-abuse drop (CONNECTION_POLICY R3 / freeze-hardening Phase 4) ---------
//
// Count CONSECUTIVE commands answered RESP_AUTH_REQUIRED and drop the link at the
// threshold, so a session that cannot authenticate stops holding the exclusive slot
// while it retries.
//
// This is an OPTIMISATION, not a hole-closer -- the distinction matters for how
// hard it should try. Phase 3 narrowed the activity clock so handshake/discovery
// opcodes and pre-auth commands no longer stamp it, which means such a peer already
// ages normally and the idle timeout reclaims the slot at OD_BLE_IDLE_TIMEOUT_MS.
// What this adds is speed and a reason: roughly one exchange instead of 120 s, and
// an explicit final RESP_AUTH_REQUIRED before a deliberate drop rather than a
// silent timeout. So it may fail safe (never dropping) without reopening anything.
#ifndef OD_AUTH_ABUSE_THRESHOLD
// CLIENT BEHAVIOUR THIS ASSUMES: py-opendisplay authenticates in ONE exchange, so a
// legitimate client never reaches 2, let alone 10.
//
// Chosen deliberately BELOW py-opendisplay's 16-frame pipe window, which is the one
// case where a well-behaved client trips it: if its session dies mid-upload, every
// in-flight frame bounces. Dropping at 10 rather than waiting out all 16 is the
// right outcome -- the session is already dead, every one of those frames is doomed,
// and the drop tells the client immediately instead of after a full window of
// pointless round trips. Recorded because an earlier prototype inherited this
// threshold by accident rather than deciding it.
#define OD_AUTH_ABUSE_THRESHOLD 10
#endif
#ifndef OD_AUTH_ABUSE_FLUSH_MS
// Hard bound on the whole best-effort delivery attempt below. On expiry the drop
// happens regardless, so a client that has stopped reading cannot keep the abuser
// attached by refusing to drain.
#define OD_AUTH_ABUSE_FLUSH_MS 500
#endif
#ifndef OD_AUTH_ABUSE_DWELL_FALLBACK_MS
// Used when the negotiated connection interval is not yet known. The central
// chooses that interval and this firmware requests none, so there is no constant to
// hard-code -- see BleTransport::connIntervalMs().
#define OD_AUTH_ABUSE_DWELL_FALLBACK_MS 50
#endif

// Set by any RESP_AUTH_REQUIRED answer for the frame being dispatched, on EVERY
// transport. Read once after the dispatch switch to decide whether the frame was
// activity. Loop-task-only, like g_commandOrigin.
static bool     s_frameRejected = false;
static uint8_t  s_authRejectRun = 0;        // consecutive RESP_AUTH_REQUIRED answers
static bool     s_authAbuseDropPending = false;
static uint32_t s_authAbuseDeadlineMs = 0;  // hard bound on the delivery attempt
static uint32_t s_authAbuseDwellUntil = 0;  // set once TX has drained; 0 = not yet

// Called at every site that answers RESP_AUTH_REQUIRED.
//
// BLE ONLY, and the origin gate is not decoration: the same auth gate is reachable
// from plaintext LAN, and counting those would let LAN traffic drop a BLE client.
// TLS-LAN never reaches the gate at all (the transport is the authentication).
static void noteAuthRejected(void) {
    // Mark the frame first, for every origin. The COUNTER is BLE-only (see below),
    // but "this frame was refused, so it is not activity" is transport-independent
    // -- and getting that wrong on LAN is exactly how a TLS client could hold the
    // slot forever.
    s_frameRejected = true;
    if (g_commandOrigin != ORIGIN_BLE) return;
    if (s_authAbuseDropPending) return;              // already decided
    if (s_authRejectRun < 255) s_authRejectRun++;
    if (s_authRejectRun < OD_AUTH_ABUSE_THRESHOLD) return;
    // The offender is the frame's own instance, taken from its queue tag -- not
    // "whichever peer the stack lists first", which is how an earlier prototype
    // misidentified it before frames carried identity.
    if (!linkIsOwnerWord(g_commandInstance)) return; // not the owner: nothing to drop
    od_log_warn("Auth abuse: %u consecutive unauthenticated commands - dropping link",
                (unsigned)s_authRejectRun);
    s_authAbuseDropPending = true;
    s_authAbuseDeadlineMs = millis() + OD_AUTH_ABUSE_FLUSH_MS;
    s_authAbuseDwellUntil = 0;
}

void resetAuthAbuseCounter(void) {
    s_authRejectRun = 0;
    s_authAbuseDropPending = false;
    s_authAbuseDeadlineMs = 0;
    s_authAbuseDwellUntil = 0;
}

void serviceBleAuthAbuseDisconnect(void) {
    if (!s_authAbuseDropPending) return;
    // Never mid-refresh: loop() is blocked throughout one, and the abort is
    // loop-task-only by contract.
    if (epdRefreshInProgress) return;

    const LinkId owner = linkOwnerId();
    if (owner.who != OWNER_BLE) {
        // The link went away, or LAN took the slot, while we were draining. Nothing
        // to drop -- and dropping on a stale identity is exactly what the epoch
        // exists to prevent.
        resetAuthAbuseCounter();
        return;
    }

    // BEST EFFORT, and deliberately not more than that. An empty TX ring proves the
    // stack ACCEPTED the notification, not that it went on air: the ring advances
    // when notify() returns true, and a BLE notification is unacknowledged. Without
    // an indication -- a wire change this plan forbids -- there is no delivery
    // signal to wait on, so this drains, dwells about one connection interval to
    // give the radio a chance to send, and then drops.
    serviceBleTx();
    const bool expired = (int32_t)(millis() - s_authAbuseDeadlineMs) >= 0;
    if (!bleTxQueuePending() && s_authAbuseDwellUntil == 0) {
        uint16_t intervalMs = ble.connIntervalMs(owner.handle);
        if (intervalMs == 0) intervalMs = OD_AUTH_ABUSE_DWELL_FALLBACK_MS;
        const uint32_t dwellEnd = millis() + intervalMs + 5u;   // +margin
        // Never past the hard deadline: a drain landing just before it yields a
        // short or zero dwell, which is the expiry case behaving as specified
        // rather than a contradiction.
        s_authAbuseDwellUntil =
            ((int32_t)(dwellEnd - s_authAbuseDeadlineMs) > 0) ? s_authAbuseDeadlineMs : dwellEnd;
    }
    const bool dwelled = (s_authAbuseDwellUntil != 0) &&
                         ((int32_t)(millis() - s_authAbuseDwellUntil) >= 0);
    if (!expired && !dwelled) return;   // keep draining next pass

    // One more pass if RX still holds frames. serviceBleRx() drains once per pass,
    // early, while this runs late -- so a frame that arrived on the callback task in
    // between is still queued, and the abort's ring reset would discard it unread.
    // That frame may be the client's authentication, which would cancel this drop
    // entirely. Bounded by the same hard deadline, so a client that keeps the ring
    // permanently non-empty cannot defer the drop indefinitely.
    if (!expired && bleRxQueuePending()) return;

    resetAuthAbuseCounter();
    // dropLink=true. The abort's own step 10 is the R3a bounded wait for link-down
    // before its step 11 releases -- two bounded waits in sequence, composing rather
    // than conflicting: this one runs BEFORE the abort precisely because the abort
    // deliberately skips the client NACK when dropping, and asking one routine to
    // both hold the link open for a response and tear it down is contradictory.
    abortToKnownState("auth abuse", true, owner);
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

/** Mirror responses to BLE only when a central is connected; LAN responses go via opendisplay_lan_send_frame. */
static void queueBleNotifyCopy(const uint8_t* response, uint16_t len) {
    // Nothing to queue against with no central attached; serviceBleTx() would
    // discard it on the next pass anyway.
    if (!ble.isConnected()) {
        return;
    }
    // No log here. logTxFrame() reports every response with its PRE-enqueue depth,
    // so the backlog this used to announce at "depth >= 2 after push" is the same
    // event as "[Q:1] or higher" on the line immediately above -- it only ever
    // restated the number. bleTxQueuePush logs the failures (oversize / ring full).
    (void)bleTxQueuePush(response, len);
}

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

// The single TX log line. Every response leaving this file goes through here, so a
// response can never reach the drain side without having been logged at its source
// -- sendResponseUnencrypted() previously had no TX line at all, which is why its
// responses showed up only as an anonymous queue depth from serviceBleTx().
//
// The depth is read BEFORE the response is enqueued, so a healthy path reads
// [BLE][Q:0] and a rising Q flags the drain falling behind the producer. LAN
// responses bypass the ring entirely (opendisplay_lan_send_frame), so they carry
// no depth -- the ORIGIN_BLE test below is deliberately the SAME predicate that
// routes the frame in both senders, so [Q:n] appears exactly when the frame really
// enters the ring. Keep the two in step or the depth becomes a lie.
//
// Not gated on TARGET_ESP32: both targets have queued their BLE responses through
// this ring since Phase 3 (see the de-fan-out comment in sendResponse). nRF needs
// the number more than ESP32 does -- loop() runs there at TASK_PRIO_LOW and is
// starved by the Bluefruit tasks, which is exactly when the drain falls behind.
// `encrypted` selects ETX vs UTX, replacing the former three-line "Sending encrypted
// response: / Original length: / Encrypted length:" block -- on the line that already
// names the opcode, the length and the bytes. Both states are spelled out rather than
// letting absence mean plaintext, so a frame that should have been wrapped and was
// not is visible instead of merely unremarked. Mirrors ERX/URX on the receive side.
static void logTxFrame(const uint8_t* frame, uint16_t len, bool encrypted = false) {
    const uint16_t cmd = (len >= 2) ? (uint16_t)((frame[0] << 8) | frame[1]) : frame[0];
    char label[64];
    // Folded into the direction token rather than trailing after the length: ETX/UTX
    // is fixed-width and sits up front where a capture is scanned, whereas a token
    // after a variable-width byte count never lands in the same column twice.
    const char* dir = encrypted ? "ETX" : "UTX";
    if (g_commandOrigin == ORIGIN_BLE) {
        snprintf(label, sizeof(label), "[%s][Q:%u] %s 0x%04X (%u B): ",
                 originTag(), (unsigned)bleTxQueueDepth(), dir, cmd, (unsigned)len);
    } else {
        snprintf(label, sizeof(label), "[%s] %s 0x%04X (%u B): ", originTag(), dir, cmd, (unsigned)len);
    }
    // Label (~50 B) plus 32 bytes of hex (96 B) plus the truncation marker; the old
    // 160-byte buffer here was close enough to truncating to be worth the margin.
    char line[192];
    od_log_hex_line(line, sizeof(line), label, frame, len);
    od_log_debug("%s", line);
}

void sendResponseUnencrypted(uint8_t* response, uint16_t len) {
    logTxFrame(response, len);
    // F4 de-fan-out: reply over the origin transport only. One path for both
    // targets as of Phase 3 (see sendResponse).
    if (g_commandOrigin == ORIGIN_BLE) {
        queueBleNotifyCopy(response, len);
    } else {
#ifdef OPENDISPLAY_HAS_WIFI
        opendisplay_lan_send_frame(response, len);
#endif
    }
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
    // Set only where the CCM envelope is actually applied below, so the log reports
    // what left the device rather than what policy intended. Every skip path -- TLS
    // origin, unauthenticated, handshake opcode, FE/FF status, and encryptResponse()
    // itself failing -- leaves it false and the frame is reported "plain".
    bool wasEncrypted = false;
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
                response = encrypted_response;
                len = encrypted_len;
                wasEncrypted = true;
            } else {
                od_log_warn("WARNING: Failed to encrypt response, sending unencrypted error response");
                errorResponse[0] = RESP_NACK;
                errorResponse[1] = (uint8_t)(command & 0xFF);
                errorResponse[2] = 0x00;
                response = errorResponse;
                len = sizeof(errorResponse);
            }
        }
    }

    // Logged here, after the encryption swap, so the dump is the bytes actually sent,
    // the depth is the pre-enqueue one, and the enc/plain token is the outcome rather
    // than the intent.
    if (!quietAck) logTxFrame(response, len, wasEncrypted);
    // F4 de-fan-out: reply over the origin transport only. One path for both
    // targets as of Phase 3: nRF used to notify() inline here with a blocking
    // delay(5) x 4 retry, which was only safe because it ran on the same task as
    // dispatch. Now that both targets dispatch from loop(), both queue and let
    // serviceBleTx() apply the non-blocking "retry next pass" backpressure rule.
    if (g_commandOrigin == ORIGIN_BLE) {
        queueBleNotifyCopy(response, len);
    } else {
#ifdef OPENDISPLAY_HAS_WIFI
        opendisplay_lan_send_frame(response, len);
#endif
    }
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
            // Drain THIS chunk to BLE before enqueuing the next: this handler runs
            // synchronously on the loop task, so the ring's only drainer cannot run
            // until we return. Without this, chunk 9+ overflows the 10-slot ring and
            // is silently dropped, truncating configs > ~864 B on read-back.
            //
            // nRF used to sit in delay(50) here instead, because it notified
            // inline from the BLE callback task and only needed to pace the
            // SoftDevice. It now shares the ring, so it needs the same flush --
            // and drops 50 ms per chunk.
            serviceBleTx();
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
            noteAuthRejected();
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
            resetChunkedWriteState();
            noteAuthRejected();
            uint8_t response[] = {RESP_ACK, (uint8_t)(CMD_CONFIG_CHUNK & 0xFF), RESP_AUTH_REQUIRED};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }
        secureEraseConfig();
    }
    if (len == 0 || len > CONFIG_CHUNK_SIZE || chunkedWriteState.receivedSize + len > MAX_CONFIG_SIZE || chunkedWriteState.receivedChunks >= MAX_CONFIG_CHUNKS) {
        resetChunkedWriteState();
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
        resetChunkedWriteState();
    } else {
        uint8_t ackResponse[] = {RESP_ACK, RESP_CONFIG_CHUNK, 0x00, 0x00};
        sendResponse(ackResponse, sizeof(ackResponse));
    }
}

// BLEConnHandle / BLECharPtr and the imageDataWritten declaration come from
// communication.h -- one declaration shared by all three callers.

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
    // Carries no encryption token: the ERX/URX line from bleRxQueuePush() already
    // reports it for this frame, and stating it twice is how the two spellings drift.
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
            noteAuthRejected();
            uint8_t response[] = {RESP_ACK, (uint8_t)(command & 0xFF), RESP_AUTH_REQUIRED};
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }

        if (len < BLE_CMD_HEADER_SIZE + ENCRYPTION_NONCE_SIZE + ENCRYPTION_TAG_SIZE) {
            od_log_error("ERROR: [%s] Unencrypted command received when encryption is enabled", originTag());
            noteAuthRejected();
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

        // The nonce is no longer dumped per frame: the banner above already reports
        // this frame as "enc", and on ESP32 the transport's RX line dumps the first
        // 32 bytes -- of which 2..17 ARE the nonce -- so it restated bytes already on
        // screen. It moves to the failure path below, where it is the only thing that
        // separates a replay-window jump from nonce reuse from a wrong session key,
        // and where nRF (which has no RX hex line at all) would otherwise be blind.
        if (!decryptCommand(data + BLE_CMD_HEADER_SIZE + ENCRYPTION_NONCE_SIZE, encrypted_data_len, plaintext, &plaintext_len, nonce_full, auth_tag, command)) {
            // 16 bytes render as 47 chars + NUL, an exact fit in 48; sized past that
            // so a future ENCRYPTION_NONCE_SIZE bump truncates nothing.
            char nonceHex[64];
            od_log_hex_line(nonceHex, sizeof(nonceHex), "", nonce_full, ENCRYPTION_NONCE_SIZE);
            od_log_error("ERROR: Decryption failed (0x%04X, %u B payload, nonce %s)",
                         (unsigned)command, (unsigned)encrypted_data_len, nonceHex);
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

    // Cleared before dispatch, inspected after it: the handlers themselves can
    // still refuse this frame, so acceptance is not knowable until they return.
    s_frameRejected = false;

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

    // R4 ACTIVITY, decided HERE -- after dispatch, on the OUTCOME rather than on a
    // prediction of it. This is the third and final position for this test, and the
    // two earlier ones were both wrong in the same way:
    //
    //  - At the top, gated on isAuthenticated(): an authenticated client sending a
    //    too-short plaintext frame stamped here, then got RESP_AUTH_REQUIRED from
    //    the length check below it.
    //  - Just before the switch: TLS-LAN frames bypass the CCM gate and reach
    //    dispatch, but handleWriteConfig() and the chunk handler apply their OWN
    //    app-layer auth check and can still answer RESP_AUTH_REQUIRED -- so a TLS
    //    client repeating CMD_CONFIG_WRITE stamped the clock on every rejected
    //    attempt and held the slot indefinitely.
    //
    // Both are the same mistake at different depths: anything that predicts
    // acceptance is wrong at whatever layer rejects next. Reading s_frameRejected
    // after the handler has run is the only position with nothing below it.
    //
    // Unknown opcodes do not stamp (commandName() is null for them), and the two
    // handshake/discovery opcodes return from their own early branches and never
    // reach here -- so "handshake and discovery are not activity" holds
    // structurally rather than by a test that could drift.
    if (!s_frameRejected && commandName(command) != nullptr &&
        linkIsOwnerWord(g_commandInstance)) {
        linkStampOwnerCommand();
        // A fully accepted command means this client is working normally, so it
        // clears the auth-abuse state ENTIRELY -- including a drop already pending.
        // Clearing only the run would let a client that recovered mid-flush (say it
        // re-authenticated after its session expired under a 16-frame pipe burst)
        // still be dropped by a decision taken moments earlier, which is the worst
        // outcome for a mechanism whose whole value is reacting quickly.
        resetAuthAbuseCounter();
    }
}
