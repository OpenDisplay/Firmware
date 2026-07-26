#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>

void sendResponseUnencrypted(uint8_t* response, uint16_t len);
void sendResponse(uint8_t* response, uint16_t len);
uint16_t calculateCRC16CCITT(uint8_t* data, uint32_t len);
uint8_t getFirmwareMajor();
uint8_t getFirmwareMinor();
uint8_t getFirmwarePatch();
const char* getFirmwareShaString();
void handleFirmwareVersion();
void handleReadMSD();
void handleReadConfig();
void handleWriteConfig(uint8_t* data, uint16_t len);
void handleWriteConfigChunk(uint8_t* data, uint16_t len);

// Transport a command arrived on. Set by the LAN listener around each dispatch and
// ORIGIN_BLE at all other times. Multi-frame transfers use it to reject frames from
// a transport that does not own the in-flight session, and to scope transport-only
// behaviour (LAN power-save suspension) to the transport that opened the session.
// Values are part of no wire format -- they are firmware-local bookkeeping.
enum CommandOrigin { ORIGIN_BLE = 0, ORIGIN_LAN_PLAIN = 1, ORIGIN_LAN_TLS = 2 };

/// Origin of the command currently being dispatched (a CommandOrigin value).
uint8_t commandOrigin(void);

// ------------------------------------------- unauthenticated-command guard ---
// Guard against a client that keeps sending gated commands into a dead session.
// Rationale and the choice of threshold are at the implementation in
// communication.cpp.
//
// Declared HERE, not in ble_init.h, because loop() calls into the guard on both
// targets. ble_init.h includes NimBLEDevice.h on ESP32, so including it from
// shared code drags the whole NimBLE surface into translation units that must
// also compile for the nRF SoftDevice. This header stays free of any BLE-stack
// include -- keep it that way; everything stack-specific is #ifdef'd inside
// communication.cpp, which already has both stacks available.

// Answers a gated frame with RESP_AUTH_REQUIRED (0xFE) and, on BLE, counts it
// toward the disconnect threshold. Every 0xFE the encryption gate emits must go
// through here rather than building the response inline, or the count is wrong.
void rejectUnauthenticated(uint16_t command);

// Clears the consecutive-rejection count. Called whenever a frame clears the
// encryption gate, and on a successful authentication.
void resetAuthGateRejects(void);

// Performs a drop requested by the guard. Serviced from loop() on both targets;
// no-op unless one is pending.
void serviceBleAuthAbuseDisconnect(void);

#endif
