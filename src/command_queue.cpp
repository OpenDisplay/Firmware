// Storage and access for the BLE command rings. Previously defined in main.h, a
// single-inclusion globals header included only by main.cpp -- the rings are
// shared between main.cpp, communication.cpp and the transport implementations,
// so they belong in a translation unit of their own.
//
// Phase 2 removed the TARGET_ESP32 guard: both targets now compile the rings.
// nRF carries them unused until Phase 3 moves its dispatch to loop().

#include <string.h>

#include "command_queue.h"
#include "ble_transport.h"
#include "od_log.h"

// --- RX ----------------------------------------------------------------------
static CommandQueueItem s_rx[COMMAND_QUEUE_SIZE];
static volatile uint8_t s_rxHead = 0;
static volatile uint8_t s_rxTail = 0;

bool bleRxQueuePush(const uint8_t* data, uint16_t len) {
    if (len == 0 || len > MAX_COMMAND_SIZE) return false;
    // Publish head with RELEASE after the payload is fully written so the
    // consumer never observes a slot before its bytes land.
    uint8_t head = __atomic_load_n(&s_rxHead, __ATOMIC_RELAXED);
    uint8_t tail = __atomic_load_n(&s_rxTail, __ATOMIC_ACQUIRE);
    uint8_t nextHead = (head + 1) % COMMAND_QUEUE_SIZE;
    if (nextHead == tail) {
        return false;
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
