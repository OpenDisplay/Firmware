#ifndef COMMAND_QUEUE_H
#define COMMAND_QUEUE_H

#include <stdint.h>
#include "opendisplay_protocol.h"   // OD_BLE_MAX_FRAME
#include "structs.h"                // PIPE_MAX_W (shrunk by PIPE_SMALL_DRAM_WINDOW)

// The two BLE command rings, declared together because they are two halves of
// one thing: frames in from the stack callback, responses out to it.
//
// Portable by construction -- no Bluefruit, no NimBLE, no Arduino -- and
// compiled on every target as of Phase 2. nRF carries the storage but does not
// yet use it: its write callback still dispatches inline on the SoftDevice
// callback task, and its responses still go straight out through
// BleTransport::notify(). Phase 3 makes nRF a producer on the RX ring and moves
// its dispatch to loop(), at which point serviceBleTx() drives both targets.
// See docs/PLAN_BLE_TRANSPORT_ABSTRACTION_2026-07-27.md.
//
// Deliberately not in structs.h: that header is the config-packet / wire-protocol
// hub pulled into every translation unit, and these rings are neither.
// Deliberately not on the BleTransport class either: this is buffering, not link
// state.

// --- RX: BLE stack callback (producer) -> loop() (consumer) ------------------
// DEPTH IS DERIVED FROM THE PIPE WINDOW, not hardcoded. The worst case this ring
// must absorb is a 60 s Spectra SPI stall (loop blocked inside bbepWriteData)
// while a client streams a PIPE_WRITE transfer:
//
//   * up to PIPE_MAX_W unacknowledged DATA (0x0081) frames -- the sender's
//     window rule bounds these, and
//   * the END (0x0082) that closes the transfer. END carries no seq, so it sits
//     OUTSIDE the sliding window's sequence space and the window rule does not
//     bound it: a sender that pipelines its tail can have END in flight on top
//     of a full window.
//
//   => frames in flight  <=  PIPE_MAX_W + 1
//
// The ring reserves one slot to tell full from empty (see bleRxQueuePush), so
// usable capacity is SLOTS - 1. Hence SLOTS = PIPE_MAX_W + 2.
//
// This previously read 33, which gave 32 usable and was one short of its own
// stated design point ("a full W=32 window plus END") -- the END would be
// rejected at exactly the stall the number was chosen for. 33 is correct for
// PIPE_REORDER_SLOTS, where W + 1 makes seq % SLOTS collision-free; that is
// modular-indexing arithmetic, not queue capacity, and the value looks to have
// been carried across.
//
// Deriving it also right-sizes env:esp32-N4, which sets PIPE_SMALL_DRAM_WINDOW
// (PIPE_MAX_W 16) yet carried a ring sized for a 32-frame window it can never
// receive.
//
// OD_BLE_MAX_FRAME (256) covers pipe <=244, legacy <=232, HA <=244; the GATT
// layer rejects anything larger with ATT 0x0D rather than the app dropping it
// silently.
//
// BLE_RX_QUEUE_SLOTS remains overridable as an escape hatch, not an expected
// fallback: nRF RAM headroom was confirmed sufficient (2026-07-27) and no env
// overrides it. Overriding it BELOW the derived value caps the effective
// PIPE_WRITE window and costs throughput -- a deliberate trade, never a
// link-time discovery. The assert below makes that trade explicit rather than
// silent.
#ifndef BLE_RX_QUEUE_SLOTS
#define BLE_RX_QUEUE_SLOTS (PIPE_MAX_W + 2)
#endif
#define COMMAND_QUEUE_SIZE BLE_RX_QUEUE_SLOTS
#define MAX_COMMAND_SIZE   OD_BLE_MAX_FRAME

static_assert(COMMAND_QUEUE_SIZE - 1 >= PIPE_MAX_W + 1,
              "RX ring too small for a full PIPE_WRITE window plus END "
              "(usable capacity is COMMAND_QUEUE_SIZE - 1)");

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

// Discard every unconsumed frame. Consumer-side: call only from the loop task,
// and only when the frames are known to be worthless -- i.e. the client that sent
// them is gone. Returns how many were dropped, for the log.
uint8_t bleRxQueueDiscardAll(void);

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
void serviceBleTx(void);

#endif  // COMMAND_QUEUE_H
