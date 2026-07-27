#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include <stdint.h>

// Portable BLE link abstraction. Deliberately includes NO stack headers, so any
// translation unit can use it without dragging in Bluefruit or NimBLE types.
//
// Exactly one implementation is linked per build (ble_transport_nrf.cpp or
// ble_transport_esp32.cpp), so this is a plain class rather than an abstract
// base: virtual dispatch would cost a vtable and indirect calls for zero
// benefit, and application code would still only ever see one type.
//
// Threading, as of Phase 1: the callback contract is NOT yet symmetric. ESP32
// stack callbacks are flag-only (they copy into the RX ring and set a flag);
// nRF still dispatches commands inline on the SoftDevice callback task and runs
// the app connect/disconnect hooks there. Phase 3 makes nRF match ESP32. Until
// then the asymmetry lives entirely inside the two implementation files -- see
// docs/PLAN_BLE_TRANSPORT_ABSTRACTION_2026-07-27.md.
class BleTransport {
public:
    // --- lifecycle ---
    // begin()/startAdvertising() are separate calls so each target keeps its own
    // init ordering: nRF must bring the SoftDevice up BEFORE display/SPI and only
    // advertise after the boot screen, whereas ESP32 inits BLE after the display.
    bool begin(const char* deviceName);
    void startAdvertising();
    // Unconditional: brings advertising back up now. Deferral policy (mid-EPD
    // refresh, still connected, stack not up) belongs to the caller, not here.
    void restartAdvertising();
    void stopAdvertising();
    void end();                    // full teardown; no-op on nRF

    // --- state ---
    bool    isReady() const;       // stack initialised and usable
    uint8_t connectedCount() const;
    bool    isConnected() const { return connectedCount() > 0; }
    bool    notifyReady() const;   // connected AND the client has subscribed (CCCD)

    // --- data out ---
    // false means backpressure ("retry next pass"), not a hard failure: the
    // caller must leave the entry queued and not advance its tail.
    bool notify(const uint8_t* data, uint16_t len);

    // --- advertising payload ---
    // Pushes a new manufacturer-specific-data payload into the advertisement.
    // Each implementation keeps its own restart semantics (see the .cpp).
    void setManufacturerData(const uint8_t* msd, uint8_t len);

    // --- link policy (no-op where the stack does not support it) ---
    void requestFastLink();        // nRF: 2M PHY + 251-octet DLE on the live link
    void boostAdvertising();       // nRF: temporary fast advertising interval
    void tick();                   // periodic housekeeping (advertising interval restore)

    // --- events: consume-once, polled from loop(). No app-facing callbacks. ---
    bool takeConnectedEvent();
    bool takeDisconnectedEvent();

    // --- identity ---
    const char* addressString();   // advertised BLE address, lowercase "aa:bb:.."
};

extern BleTransport ble;

// Application hooks invoked by the transport on connect/disconnect. Defined in
// device_control.cpp. On nRF these still run on the SoftDevice callback task
// (Phase 1 preserves today's behaviour); on ESP32 the transport records an event
// instead and loop() drives the equivalent work.
void bleAppOnConnect();
void bleAppOnDisconnect(uint8_t reason);

#ifdef TARGET_ESP32
// Deferred work, owned by the APPLICATION rather than by BleTransport: each flag
// encodes a loop()-serviced policy decision (tear the session down, refresh the
// advertisement, re-arm the radio) that the transport has no business making.
// loop() raises them from takeConnectedEvent()/takeDisconnectedEvent().
// bleDisconnectCleanupPending is also raised by the LAN transport
// (wifi_service.cpp), which is exactly why it cannot be a BLE transport event.
extern volatile bool bleDisconnectCleanupPending;
extern volatile bool bleRestartAdvertisingPending;
extern volatile bool msdUpdatePending;
#endif

#endif  // BLE_TRANSPORT_H
