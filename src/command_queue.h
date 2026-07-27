#ifndef COMMAND_QUEUE_H
#define COMMAND_QUEUE_H

#include <stdint.h>
#include "opendisplay_protocol.h"   // OD_BLE_MAX_FRAME

// The two BLE command rings, declared together because they are two halves of
// one thing: frames in from the stack callback, responses out to it. Storage
// lives in command_queue.cpp.
//
// Portable by construction -- no Bluefruit, no NimBLE, no Arduino -- so both
// ble_transport_*.cpp and the application can include it.
//
// Deliberately not in structs.h: that header is the config-packet / wire-protocol
// hub pulled into every translation unit, and these rings are neither.
// Deliberately not on the BleTransport class either: this is buffering, not link
// state.
//
// Phase 2 moves both rings out of the TARGET_ESP32 guard and adds push/pop
// accessors so nRF shares them; Phase 3 makes the nRF write callback a producer
// on the RX ring. See docs/PLAN_BLE_TRANSPORT_ABSTRACTION_2026-07-27.md.

#ifdef TARGET_ESP32

// --- RX: BLE stack callback (producer) -> loop() (consumer) ------------------
// PIPE_WRITE ingest sizing: 33 slots hold a full W=32 in-flight window + END
// across a 60 s Spectra SPI stall (loop blocked in bbepWriteData).
// OD_BLE_MAX_FRAME (256) covers pipe <=244, legacy <=232, HA <=244; the GATT
// layer rejects anything larger with ATT 0x0D rather than the app dropping it
// silently.
#define COMMAND_QUEUE_SIZE 33
#define MAX_COMMAND_SIZE   OD_BLE_MAX_FRAME

struct CommandQueueItem {
    uint8_t data[MAX_COMMAND_SIZE];
    uint16_t len;
    bool pending;
};

// SPSC: the stack callback publishes head with RELEASE after the payload lands;
// loop() ACQUIREs it and RELEASEs the tail after consuming.
extern CommandQueueItem commandQueue[COMMAND_QUEUE_SIZE];
extern volatile uint8_t commandQueueHead;
extern volatile uint8_t commandQueueTail;

// --- TX: command handlers (producer) -> loop() flush (consumer) --------------
// One definition of the struct, in one place: communication.cpp used to carry
// its own copy plus a MAX_RESPONSE_SIZE_LOCAL constant, so the bound checked
// before the memcpy in esp32_queue_ble_notify_copy() lived in a different file
// from the slot it guarded -- two definitions of one type (an ODR violation)
// that had to be edited in lockstep or the guard would admit a response larger
// than the slot.
#define RESPONSE_QUEUE_SIZE 10
#define MAX_RESPONSE_SIZE   OD_BLE_MAX_FRAME

struct ResponseQueueItem {
    uint8_t data[MAX_RESPONSE_SIZE];
    uint16_t len;
    bool pending;
};

// Single-task: both ends run on loop(), so no atomics here.
extern ResponseQueueItem responseQueue[RESPONSE_QUEUE_SIZE];
extern uint8_t responseQueueHead;
extern uint8_t responseQueueTail;

#endif  // TARGET_ESP32
#endif  // COMMAND_QUEUE_H
