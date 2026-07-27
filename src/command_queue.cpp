// Storage and access for the BLE command rings. Previously defined in main.h, a
// single-inclusion globals header included only by main.cpp -- the rings are
// shared between main.cpp, communication.cpp and the transport implementations,
// so they belong in a translation unit of their own.
//
// Phase 2 removed the TARGET_ESP32 guard: both targets now compile the rings.
// nRF carries them unused until Phase 3 moves its dispatch to loop().

#include <stdio.h>
#include <string.h>

#include "command_queue.h"
#include "ble_transport.h"
#include "encryption.h"   // isEncryptionEnabled(), for the ERX/URX token
#include "od_log.h"

// Defined in display_service.cpp. True for a mid-stream image-write data frame
// (0x0071 / 0x0081) whose per-frame RX logging should be suppressed.
bool imageWriteLogQuietFrame(const uint8_t* data, uint16_t len);

// --- RX ----------------------------------------------------------------------
static CommandQueueItem s_rx[COMMAND_QUEUE_SIZE];
static volatile uint8_t s_rxHead = 0;
static volatile uint8_t s_rxTail = 0;

// The single RX log line, and the single place the three push failures are named.
//
// It lives here, not in either transport, because this is the one function both
// stack callbacks call -- ESP32's onWrite() and nRF's onWriteCb(). A copy in each
// callback is how the two targets drifted in the first place: ESP32 had a hex line
// and separate "too large" / "empty" warnings, nRF had neither and reported all
// three failures as "queue full", pointing at ring depth for what was actually a
// malformed frame.
//
// Logged at ARRIVAL, on the stack callback task, so the timestamp is when the radio
// delivered the frame rather than when loop() got to it -- that ordering is the
// point of the line, and it cannot be had from the consumer side. The cost is real
// and deliberate: od_log ends in a blocking serial write (~9 ms for a full hex line
// at 115200), so this delays the NimBLE host / SoftDevice callback task. It is
// compiled out entirely at the default INFO level; only the -debug envs pay it, and
// only for frames the quiet predicate does not suppress.
//
// Depth is the PRE-push count, matching logTxFrame()'s pre-enqueue depth, so a
// healthy path reads [BLE][Q:0] and a rising Q means arrivals are outrunning
// loop()'s drain. RX is BLE-only by construction -- LAN frames reach the dispatcher
// without touching this ring -- so the tag is a literal, not originTag().
bool bleRxQueuePush(const uint8_t* data, uint16_t len) {
    if (len == 0) {
        od_log_warn("WARNING: Empty BLE frame received, dropping");
        return false;
    }
    if (len > MAX_COMMAND_SIZE) {
        od_log_warn("WARNING: Command too large for queue (%u > %u), dropping",
                    (unsigned)len, (unsigned)MAX_COMMAND_SIZE);
        return false;
    }
    // Publish head with RELEASE after the payload is fully written so the
    // consumer never observes a slot before its bytes land.
    uint8_t head = __atomic_load_n(&s_rxHead, __ATOMIC_RELAXED);
    uint8_t tail = __atomic_load_n(&s_rxTail, __ATOMIC_ACQUIRE);
    uint8_t nextHead = (head + 1) % COMMAND_QUEUE_SIZE;
    if (nextHead == tail) {
        od_log_error("ERROR: Command queue full, dropping command (%u slots)",
                     (unsigned)COMMAND_QUEUE_SIZE);
        return false;
    }
    if (!imageWriteLogQuietFrame(data, len)) {
        const uint16_t cmd = (len >= 2) ? (uint16_t)((data[0] << 8) | data[1]) : data[0];
        const uint8_t depth = (uint8_t)((head - tail + COMMAND_QUEUE_SIZE) % COMMAND_QUEUE_SIZE);
        // ERX / URX: does this frame carry the app-layer CCM envelope? Mirrors the
        // gate in imageDataWritten() -- the two handshake opcodes are dispatched
        // before it, and a frame too short to hold nonce+tag cannot be wrapped. The
        // ORIGIN_LAN_TLS term of that gate is omitted deliberately: this ring is BLE
        // only, LAN frames never reach it. Anything URX while encryption is on is
        // rejected by the dispatcher, so the token is the frame's form, not intent.
        const bool encrypted = isEncryptionEnabled() &&
                               cmd != CMD_AUTHENTICATE && cmd != CMD_FIRMWARE_VERSION &&
                               len >= BLE_CMD_HEADER_SIZE + ENCRYPTION_NONCE_SIZE + ENCRYPTION_TAG_SIZE;
        char label[48];
        snprintf(label, sizeof(label), "[BLE][Q:%u] %s 0x%04X (%u B): ",
                 (unsigned)depth, encrypted ? "ERX" : "URX", cmd, (unsigned)len);
        char line[192];
        od_log_hex_line(line, sizeof(line), label, data, len);
        od_log_debug("%s", line);
    }
    memcpy(s_rx[head].data, data, len);
    s_rx[head].len = len;
    s_rx[head].pending = true;
    __atomic_store_n(&s_rxHead, nextHead, __ATOMIC_RELEASE);
    return true;
}

CommandQueueItem* bleRxQueuePeek(void) {
    // ACQUIRE the head so the payload the producer wrote before its RELEASE
    // store is visible to us.
    uint8_t tail = __atomic_load_n(&s_rxTail, __ATOMIC_RELAXED);
    uint8_t head = __atomic_load_n(&s_rxHead, __ATOMIC_ACQUIRE);
    if (tail == head) return nullptr;
    return &s_rx[tail];
}

void bleRxQueueConsume(void) {
    uint8_t tail = __atomic_load_n(&s_rxTail, __ATOMIC_RELAXED);
    s_rx[tail].pending = false;
    __atomic_store_n(&s_rxTail, (uint8_t)((tail + 1) % COMMAND_QUEUE_SIZE), __ATOMIC_RELEASE);
}

uint8_t bleRxQueueDiscardAll(void) {
    // ACQUIRE the head for the same reason peek does, then jump the tail to it in
    // one RELEASE store. Safe against a concurrent producer: it only ever advances
    // the head, so a frame pushed after this load simply survives to the next pass
    // rather than being lost or double-counted.
    uint8_t tail = __atomic_load_n(&s_rxTail, __ATOMIC_RELAXED);
    uint8_t head = __atomic_load_n(&s_rxHead, __ATOMIC_ACQUIRE);
    if (tail == head) return 0;
    const uint8_t dropped = (uint8_t)((head - tail + COMMAND_QUEUE_SIZE) % COMMAND_QUEUE_SIZE);
    for (uint8_t i = tail; i != head; i = (uint8_t)((i + 1) % COMMAND_QUEUE_SIZE)) {
        s_rx[i].pending = false;
    }
    __atomic_store_n(&s_rxTail, head, __ATOMIC_RELEASE);
    return dropped;
}

uint8_t bleRxQueueHead(void) {
    return __atomic_load_n(&s_rxHead, __ATOMIC_RELAXED);
}

uint8_t bleRxQueueDepth(void) {
    // RELAXED both: a snapshot for logging, not a synchronisation point. The
    // producer may push concurrently, in which case this simply reads one frame
    // stale -- which is the correct answer for "how deep was it a moment ago".
    const uint8_t head = __atomic_load_n(&s_rxHead, __ATOMIC_RELAXED);
    const uint8_t tail = __atomic_load_n(&s_rxTail, __ATOMIC_RELAXED);
    return (uint8_t)((head - tail + COMMAND_QUEUE_SIZE) % COMMAND_QUEUE_SIZE);
}

bool bleRxQueuePending(void) {
    return __atomic_load_n(&s_rxTail, __ATOMIC_RELAXED) !=
           __atomic_load_n(&s_rxHead, __ATOMIC_RELAXED);
}

// --- TX ----------------------------------------------------------------------
static ResponseQueueItem s_tx[RESPONSE_QUEUE_SIZE];
static uint8_t s_txHead = 0;
static uint8_t s_txTail = 0;

bool bleTxQueuePush(const uint8_t* data, uint16_t len) {
    if (len > MAX_RESPONSE_SIZE) {
        od_log_error("ERROR: Response too large for queue (%u > %u)", len, MAX_RESPONSE_SIZE);
        return false;
    }
    uint8_t nextHead = (s_txHead + 1) % RESPONSE_QUEUE_SIZE;
    if (nextHead == s_txTail) {
        od_log_error("ERROR: Response queue full, dropping response");
        return false;
    }
    memcpy(s_tx[s_txHead].data, data, len);
    s_tx[s_txHead].len = len;
    s_tx[s_txHead].pending = true;
    s_txHead = nextHead;
    return true;
}

uint8_t bleTxQueueDepth(void) {
    return (uint8_t)((s_txHead - s_txTail + RESPONSE_QUEUE_SIZE) % RESPONSE_QUEUE_SIZE);
}

uint8_t bleTxQueueHead(void) {
    return s_txHead;
}

bool bleTxQueuePending(void) {
    return s_txTail != s_txHead;
}

void serviceBleTx(void) {
    if (s_txTail == s_txHead) return;
    if (ble.notifyReady()) {
        uint8_t drained = 0;
        while (s_txTail != s_txHead && drained < 16) {
            // false means backpressure (NimBLE mbuf exhaustion): stop draining and
            // leave the entry queued to retry next pass rather than advancing past
            // a dropped ACK, which would stall the pipe window. See
            // BleTransport::notify().
            if (!ble.notify(s_tx[s_txTail].data, s_tx[s_txTail].len)) {
                break;
            }
            s_tx[s_txTail].pending = false;
            s_txTail = (s_txTail + 1) % RESPONSE_QUEUE_SIZE;
            drained++;
        }
    } else if (ble.isConnected()) {
        // Connected but CCCD not enabled yet -- keep responses queued
    } else {
        while (s_txTail != s_txHead) {
            s_tx[s_txTail].pending = false;
            s_txTail = (s_txTail + 1) % RESPONSE_QUEUE_SIZE;
        }
    }
}
