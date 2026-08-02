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
// Threading: the callback contract is now SYMMETRIC. On both targets a stack
// callback may copy bytes into the RX ring, publish its own connection-instance
// metadata, attempt the one ownership claim CAS, and set an event flag -- nothing
// else. Command dispatch, decrypt, EPD streaming, notify() and the connect/
// disconnect application work all run on the loop() task. (An earlier revision of
// this note said nRF still dispatched inline and that a later phase would fix it;
// that landed with the loop/BLE unification.)
//
// The claim CAS is the one addition to the historical "copy and flag" rule, and it
// is deliberate: ownership must be decided at the earliest transport hook, because
// the write filter below has to be able to answer "is this the owner?" long before
// any loop pass runs -- during a refresh, up to ~16 s before one.
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
    // The stack's TOTAL peer count. Note what this is NOT: a test for whether one
    // particular link is up. CONNECTION_POLICY R1 permits a refused contender to be
    // transiently attached, so dropping the owner takes this 2->1, never to 0 --
    // which is why the R3a wait polls instanceLive() per handle instead. Keep using
    // this only for "is anything connected at all".
    uint8_t connectedCount() const;
    bool    isConnected() const { return connectedCount() > 0; }
    bool    notifyReady() const;   // owner connected AND subscribed (CCCD), per-instance

    // --- connection instances (CONNECTION_POLICY R2/R3 requirement 5) ---
    // A fixed per-handle table, sized by the connection cap (3 on ESP32, 1 on nRF),
    // holding metadata only -- never frames. Callbacks write their own handle's
    // entry; the loop scans it. That inversion is what makes lost edges stop
    // mattering: state is bounded by the connection cap rather than by event rate,
    // so there is nothing to overflow and no eviction policy to specify.
    //
    // Liveness IS the packed (handle, epoch) identity word -- all-zero means empty --
    // so an entry can never present a live identity with a stale state, and the R3a
    // wait can read identity and liveness in one atomic load.
    bool     instanceLive(uint16_t handle, uint16_t epoch) const;
    uint8_t  liveInstanceCount() const;
    // Packed identity word of the i'th live instance, 0 when the slot is empty.
    // Phase 3's admission scan walks these to find contenders (any live instance
    // that is not the owner); Phase 2 only needs it for diagnostics and the wait.
    uint32_t instanceWordAt(uint8_t index) const;
    // 0 while this entry's ownership claim is still in flight; otherwise the
    // IDENTITY WORD the claim was decided for. Published with release ordering after
    // the claim CAS.
    //
    // Why the identity and not a bool: the loop-side scan reads the entry word, the
    // disposition and the owner word separately and cannot get them atomically. A
    // bare flag lets it pair one entry's identity with another's disposition after a
    // slot is retired and reused (ABA). Requiring decidedWord == the entry word
    // proves the disposition belongs to THIS instance.
    //
    // The distinction is load-bearing either way: refusing an in-flight instance can
    // disconnect the connection that is winning the slot, while never refusing one
    // leaves a decided loser attached forever -- on nRF, holding the only link.
    uint32_t instanceClaimDecidedWordAt(uint8_t index) const;
    static uint8_t instanceCapacity();

    // --- link drop (CONNECTION_POLICY R3a) ---
    // Requests termination of ONE link. Returns false only on a genuine failure to
    // ask; a true return means "requested", NOT "down" -- BLE disconnect is
    // asynchronous, so callers that need the link actually gone must wait on
    // instanceLive() (bleDropAndWait() in session_guard.cpp does exactly that).
    //
    // No `reason` parameter, deliberately. A host-initiated disconnect must send a
    // Core-Spec-legal HCI reason; 0x13 (REMOTE_USER_TERMINATED) is the only value
    // this firmware wants, it is what NimBLE defaults to, and it is the only value
    // Bluefruit can send at all (BLEConnection::disconnect() hardcodes it, with no
    // reason argument to pass). Note 0x09 (CONN_LIMIT) is NOT legal here: the
    // controller silently rejects it and the peer stays connected while the code
    // looks like it worked.
    //
    // Loop task only. A callback that severs its own link mid-dispatch is exactly
    // the class of bug the unified-loop work removed.
    // Takes the full instance identity, not just a handle: the transport
    // re-validates that (handle, epoch) is still the live instance immediately
    // before asking the stack, so a caller acting on a slightly stale scan cannot
    // disconnect whoever inherited the numeric handle in the meantime.
    //
    // RESIDUAL, stated rather than implied: this narrows that window to a few
    // instructions but cannot close it, because the stack API is handle-addressed
    // and the host task can retire and reassign a handle at any point. Closing it
    // fully would need the validate-and-disconnect pair to run on the host task
    // itself. The exposure is a spuriously dropped client that reconnects -- not
    // stranded ownership.
    bool disconnect(uint16_t handle, uint16_t epoch);

    // Negotiated connection interval in ms for `handle`, 0 when unknown. The
    // central chooses it; this firmware requests none. Phase 4's TX-flush dwell
    // sizes itself on this rather than on a constant.
    uint16_t connIntervalMs(uint16_t handle) const;

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

    // --- events: polled from loop(), consumed once by the take*() pair below.
    // No app-facing callbacks. ---
    // Non-destructive peek. Cooperative waits need to return to loop() on an event
    // without taking it away from serviceBleEvents(), which remains the single
    // consumer -- so event ordering, the disconnect RX-boundary capture and the
    // deferred-cleanup flags are all unaffected by anything that polls this.
    //
    // The backing flags are plain volatile, and the existing take/clear protocol
    // has a known pre-existing coalescing weakness: a second same-type event
    // arriving inside the check-then-clear window is lost. This peek neither
    // introduces nor worsens that; fixing it is a separate change.
    bool eventPending() const;
    // Reports the connecting instance's identity (packed word, link_owner.h) so the
    // loop can act on a specific newcomer. Under requirement 5 this is a hint: the
    // instance table, not the event, is the mechanism -- a connect that coalesces
    // away still leaves a live table entry.
    bool takeConnectedEvent(uint32_t* instanceWord = nullptr);
    // Optionally reports the departing instance's identity and the stack's
    // disconnect reason, which is otherwise lost now that the callback no longer
    // runs application code inline.
    //
    // `reason` is uint16_t because NimBLE's is not a byte: it uses two ranges, HCI
    // reasons wrapped as BLE_HS_ERR_HCI_BASE + code (0x200 + code) and host-layer
    // BLE_HS_E* codes in 1..31. The old uint8_t truncation kept only the low byte,
    // so an HCI reason survived by luck (0x213 & 0xFF == 0x13) while BLE_HS_ENOTCONN
    // (7) read back as the unrelated HCI "memory capacity exceeded". nRF stores a
    // raw HCI byte with no wrapping and is unaffected.
    //
    // The rxBoundary out-param is GONE: the RX-boundary mechanism it fed is retired
    // in favour of per-frame identity tags (CommandQueueItem::tag).
    bool takeDisconnectedEvent(uint16_t* reason = nullptr, uint32_t* instanceWord = nullptr);

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
