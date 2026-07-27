#ifndef COMMAND_QUEUE_H
#define COMMAND_QUEUE_H

#include <stdint.h>
#include "opendisplay_protocol.h"   // OD_BLE_MAX_FRAME

// The two BLE command rings, declared together because they are two halves of
// one thing: frames in from the stack callback, responses out to it.
//
// Portable by construction -- no Bluefruit, no NimBLE, no Arduino -- and
// compiled on every target as of Phase 2. nRF carries the storage but does not
// yet use it: its write callback still dispatches inline on the SoftDevice
// callback task, and its responses still go straight out through
// BleTransport::notify(). Phase 3 makes nRF a producer on the RX ring and moves
// its dispatch to loop(), at which point bleServiceTx() drives both targets.
// See docs/PLAN_BLE_TRANSPORT_ABSTRACTION_2026-07-27.md.
//
// Deliberately not in structs.h: that header is the config-packet / wire-protocol
// hub pulled into every translation unit, and these rings are neither.
// Deliberately not on the BleTransport class either: this is buffering, not link
// state.

// --- RX: BLE stack callback (producer) -> loop() (consumer) ------------------
// PIPE_WRITE ingest sizing: 33 slots hold a full W=32 in-flight window + END
// across a 60 s Spectra SPI stall (loop blocked in bbepWriteData).
// OD_BLE_MAX_FRAME (256) covers pipe <=244, legacy <=232, HA <=244; the GATT
// layer rejects anything larger with ATT 0x0D rather than the app dropping it
// silently.
//
// BLE_RX_QUEUE_SLOTS is an escape hatch, not an expected fallback: nRF RAM
// headroom was confirmed sufficient for the full depth (2026-07-27), so no env
// overrides it today. Mirrors the PIPE_SMALL_DRAM_WINDOW precedent in structs.h.
// Shrinking it below PIPE_MAX_W + 1 caps the PIPE_WRITE window and costs
// throughput -- a deliberate trade, never a link-time discovery.
#ifndef BLE_RX_QUEUE_SLOTS
#define BLE_RX_QUEUE_SLOTS 33
#endif
#define COMMAND_QUEUE_SIZE BLE_RX_QUEUE_SLOTS
#define MAX_COMMAND_SIZE   OD_BLE_MAX_FRAME

struct CommandQueueItem {
    uint8_t data[MAX_COMMAND_SIZE];
    uint16_t len;
    bool pending;
};

// SPSC. Push runs on the stack callback task; peek/consume run on loop(). The
// producer publishes head with RELEASE after the payload lands, so the consumer
// never observes a slot before its bytes are visible.
//
// peek/consume rather than the copying pop the plan sketched: the consumer is
// the only party that may touch a slot until it advances the tail, so handing
// out a pointer is safe and avoids a 256-byte stack buffer plus one memcpy per
// frame -- up to 33 of them per loop pass during a pipe transfer. The peeked
// slot is deliberately mutable: the dispatcher decrypts in place, exactly as it
// did when it was handed the raw ring slot.
bool bleRxQueuePush(const uint8_t* data, uint16_t len);   // false = ring full
CommandQueueItem* bleRxQueuePeek(void);                   // nullptr = empty
void bleRxQueueConsume(void);                             // advance past the peeked slot
uint8_t bleRxQueueHead(void);                             // producer-side, for pollActivity()
bool bleRxQueuePending(void);                             // unconsumed frames waiting

// --- TX: command handlers (producer) -> loop() flush (consumer) --------------
// One definition of the struct, in one place: communication.cpp used to carry
// its own copy plus a MAX_RESPONSE_SIZE_LOCAL constant, so the bound checked
// before the memcpy in the enqueue path lived in a different file from the slot
// it guarded -- two definitions of one type (an ODR violation) that had to be
// edited in lockstep or the guard would admit a response larger than the slot.
#define RESPONSE_QUEUE_SIZE 10
#define MAX_RESPONSE_SIZE   OD_BLE_MAX_FRAME

struct ResponseQueueItem {
    uint8_t data[MAX_RESPONSE_SIZE];
    uint16_t len;
    bool pending;
};

// Both ends run on loop() today, so no atomics here.
bool bleTxQueuePush(const uint8_t* data, uint16_t len);   // false = too large or full
uint8_t bleTxQueueDepth(void);
uint8_t bleTxQueueHead(void);                             // producer-side, for pollActivity()
bool bleTxQueuePending(void);

// Drain queued responses to BLE notifications. Called once per loop() pass and --
// critically -- between commands inside the bounded command drain: at small
// negotiated ack_every (N_eff 1-2) a 33-command drain can emit up to ~32 pipe
// ACKs, which would overflow the 10-slot response ring (dropping the NEWEST
// entry) and leave only stale ACKs, lagging window refunds and collapsing
// throughput. handleReadConfig() calls it between config chunks for the same
// reason.
void bleServiceTx(void);

#endif  // COMMAND_QUEUE_H
