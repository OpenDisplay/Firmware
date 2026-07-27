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

    // True where the stack re-arms advertising by itself after a disconnect
    // (nRF: Bluefruit.Advertising.restartOnDisconnect(true)). Where it is false
    // the application must schedule the restart itself. A genuine capability
    // difference, stated as a query so callers need no target #ifdef.
    bool restartsAdvertisingOnDisconnect() const;

    // --- link policy (no-op where the stack does not support it) ---
    void requestFastLink();        // nRF: 2M PHY + 251-octet DLE on the live link
    void boostAdvertising();       // nRF: temporary fast advertising interval
    void tick();                   // periodic housekeeping (advertising interval restore)

    // --- events: consume-once, polled from loop(). No app-facing callbacks. ---
    bool takeConnectedEvent();
    // Optionally reports the stack's disconnect reason code, which is otherwise
    // lost now that the callback no longer runs application code inline.
    bool takeDisconnectedEvent(uint8_t* reason = nullptr);

    // --- identity ---
    const char* addressString();   // advertised BLE address, lowercase "aa:bb:.."
};

extern BleTransport ble;

// The loop()-serviced deferred-work flags that used to be declared here have
// moved: they encode application policy, not link state, so exporting them from
// the transport seam was backwards. The two that other translation units need to
// raise are now requestTransferSessionCleanup() / requestAdvertisingRestart() in
// communication.h; the flags themselves are private to main.cpp.

#endif  // BLE_TRANSPORT_H
