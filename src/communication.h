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

// The shared command dispatcher, serving nRF BLE, ESP32 BLE and the ESP32 LAN
// transport. Both leading parameters are unused by the dispatch logic, so they
// are opaque on every target and this declaration needs no BLE stack types --
// which is what lets three callers in three files share one declaration instead
// of each re-typedef'ing their own. On nRF the Bluefruit write callback has a
// BLECharacteristic*-shaped signature; ble_transport_nrf.cpp adapts it to this.
typedef uint16_t BLEConnHandle;
typedef void*    BLECharPtr;
void imageDataWritten(BLEConnHandle conn_hdl, BLECharPtr chr, uint8_t* data, uint16_t len);

// Transport a command arrived on. Set by the LAN listener around each dispatch and
// ORIGIN_BLE at all other times. Multi-frame transfers use it to reject frames from
// a transport that does not own the in-flight session, and to scope transport-only
// behaviour (LAN power-save suspension) to the transport that opened the session.
// Values are part of no wire format -- they are firmware-local bookkeeping.
enum CommandOrigin { ORIGIN_BLE = 0, ORIGIN_LAN_PLAIN = 1, ORIGIN_LAN_TLS = 2 };

/// Origin of the command currently being dispatched (a CommandOrigin value).
uint8_t commandOrigin(void);

/**
 * Instance identity (packed owner word) of the frame being dispatched. Set by each
 * transport immediately before it calls imageDataWritten(): BLE from the frame's own
 * queue tag, LAN from the LAN owner's identity. Compared against the live owner word
 * so a frame from a departed instance neither executes nor stamps the activity clock.
 */
extern volatile uint32_t g_commandInstance;

/**
 * Drop a BLE link that has answered OD_AUTH_ABUSE_THRESHOLD consecutive commands
 * with RESP_AUTH_REQUIRED. Loop-serviced, both targets, and it must run on the loop
 * task: it ends in abortToKnownState().
 *
 * Best-effort delivery of the final RESP_AUTH_REQUIRED before the drop -- it drains
 * TX and dwells about one connection interval, both inside a hard bound. An empty
 * ring proves stack acceptance of an unacknowledged notification, not receipt, so a
 * deadline-truncated attempt may forfeit it by design.
 */
void serviceBleAuthAbuseDisconnect(void);

/** Clear the consecutive-rejection run. Called by abortToKnownState() so every
 *  session end resets it -- otherwise a new client inherits its predecessor's
 *  rejections, which is a defect an earlier prototype shipped with. */
void resetAuthAbuseCounter(void);

// --- deferred work, serviced by loop() ---------------------------------------
// Implemented in main.cpp, which owns loop() and the flags behind these. They
// are requests, not commands: the work happens on a later pass, and main.cpp
// decides when it is safe (never mid-EPD-refresh, never while the owning
// transport is still live).
//
// Declared here rather than in ble_transport.h because they are application
// policy, not link state -- and because neither is BLE-specific: the cleanup
// request is raised by the LAN transport on a LAN disconnect.

/// Abort any in-flight transfer and tear down the panel session. Raised on a
/// BLE or LAN disconnect; honoured once no refresh is in flight and the
/// transport that OWNS the transfer is confirmed gone.
void requestTransferSessionCleanup(void);

/// Re-arm BLE advertising when it is safe to. A no-op on targets whose stack
/// re-arms itself (see BleTransport::restartsAdvertisingOnDisconnect), so
/// callers need no target guard.
void requestAdvertisingRestart(void);

#endif
