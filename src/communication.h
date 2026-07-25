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

#endif
