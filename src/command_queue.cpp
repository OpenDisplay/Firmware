// Storage for the BLE command rings. Previously defined in main.h, which is a
// single-inclusion globals header included only by main.cpp -- the rings are
// shared between main.cpp, communication.cpp and ble_transport_esp32.cpp, so
// they belong in a translation unit of their own rather than in one consumer's
// private header.
//
// The whole file is gated on TARGET_ESP32 to match command_queue.h; Phase 2
// removes the guard so nRF shares the same rings.
#ifdef TARGET_ESP32

#include "command_queue.h"

CommandQueueItem commandQueue[COMMAND_QUEUE_SIZE];
volatile uint8_t commandQueueHead = 0;
volatile uint8_t commandQueueTail = 0;

ResponseQueueItem responseQueue[RESPONSE_QUEUE_SIZE];
uint8_t responseQueueHead = 0;
uint8_t responseQueueTail = 0;

#endif  // TARGET_ESP32
