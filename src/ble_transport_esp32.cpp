// ESP32 (NimBLE-Arduino) implementation of BleTransport.
//
// The whole file is gated on TARGET_ESP32, so an nRF build compiles it to an
// empty translation unit -- no build_src_filter changes needed across the CI
// environments. Every NimBLE object lives here as file-static state; nothing
// outside this file names a NimBLE type.
//
// Threading contract, identical on both targets: a NimBLE host-task callback may
// copy bytes into the RX ring, publish its own instance-table entry, attempt the
// single ownership claim CAS, and set an event flag. Everything else -- dispatch,
// decrypt, EPD streaming, notify(), the connect/disconnect application work --
// runs on the loop() task.
//
// The claim is the one thing beyond "copy and flag", and it belongs here because
// the write filter must be able to test ownership before any loop pass runs.
#ifdef TARGET_ESP32

#include <Arduino.h>
#include <string.h>

#include "ble_transport.h"
#include "ble_transport_esp32.h"
#include "command_queue.h"
#include "link_owner.h"
#include "structs.h"
#include "od_log.h"

String getChipIdHex();

BleTransport ble;

// --- stack objects (were globals in main.h) ---------------------------------
static BLEServer*         s_server = nullptr;
static BLEService*        s_service = nullptr;
static BLECharacteristic* s_txCharacteristic = nullptr;
static BLEAdvertisementData s_advertisementData;

static volatile bool s_connectedEvent = false;
static volatile bool s_disconnectedEvent = false;
// NimBLE's reason is an int spanning two ranges (HCI wrapped at 0x200+, host-layer
// 1..31), so a uint8_t here silently aliased them onto each other. See
// takeDisconnectedEvent().
static volatile uint16_t s_disconnectReason = 0;
static volatile uint32_t s_connectedWord = 0;      // identity of the last connect
static volatile uint32_t s_disconnectedWord = 0;   // identity of the last disconnect

// --- the instance table (CONNECTION_POLICY R3 requirement 5) -----------------
// Sized by the connection cap. CONFIG_BT_NIMBLE_MAX_CONNECTIONS is 3 in the
// precompiled sdkconfig.h for S3/C3/C6 and absent for classic ESP32 (NimBLE's own
// #ifndef default is also 3), and a -D override is inert because the precompiled
// header wins -- which is exactly why exclusivity must be enforced in firmware
// rather than by config.
//
// Each entry's packed identity word IS its liveness: all-zero means empty. There is
// no separate `state` field, so a reader can never see a live identity with a stale
// state. Written on the NimBLE host task, read on the loop task; the word is the
// synchronisation point, and `reason`/`subscribed` are only read once it says down
// or under owner comparison.
#ifndef OD_BLE_MAX_INSTANCES
#define OD_BLE_MAX_INSTANCES 3
#endif

struct BleInstance {
    volatile uint32_t word;         // packed (OWNER_BLE, handle, epoch); 0 = empty
    // Per-link CCCD state (requirement 2). Written on the NimBLE host task by
    // onSubscribe, read on the loop task by notifyReady, so every access goes
    // through __atomic_*: `volatile` orders nothing and does not make a concurrent
    // read/write anything but a data race in C++.
    volatile uint8_t  subscribed;
    // Claim disposition, published with RELEASE after the CAS resolves: 0 while the
    // claim is in flight, otherwise the IDENTITY WORD the claim was decided for.
    //
    // It carries the identity rather than a bare flag because the loop-side scan
    // reads several fields and cannot get them atomically. A boolean lets two
    // distinct hazards through: the scan could pair entry w1 with a disposition that
    // actually belongs to w2 after the slot was retired and reused (ABA), and it
    // could not tell "resolved for THIS instance" from "resolved for whoever holds
    // this slot now". Matching decidedWord against the entry word proves both.
    //
    // The distinction itself is load-bearing: refusing an in-flight instance can
    // disconnect the connection that is winning the slot, while never refusing one
    // leaves a decided loser attached forever -- on nRF, holding the only link.
    volatile uint32_t decidedWord;
};
static BleInstance s_instances[OD_BLE_MAX_INSTANCES];

// Search by handle rather than indexing by it. NimBLE allocates from 0 upward in
// practice, so direct indexing usually works, but a 3-entry linear search costs the
// same at this size and cannot be broken by a stack change that hands out sparse
// handles.
static int instanceIndexOf(uint16_t handle) {
    for (int i = 0; i < OD_BLE_MAX_INSTANCES; i++) {
        const uint32_t w = __atomic_load_n(&s_instances[i].word, __ATOMIC_ACQUIRE);
        if (w != 0 && linkUnpackWord(w).handle == handle) return i;
    }
    return -1;
}

static uint32_t instancePublish(uint16_t handle, uint16_t epoch) {
    const uint32_t w = linkPackWord(OWNER_BLE, handle, epoch);
    for (int i = 0; i < OD_BLE_MAX_INSTANCES; i++) {
        uint32_t expected = 0;
        if (__atomic_compare_exchange_n(&s_instances[i].word, &expected, w,
                                        false, __ATOMIC_RELEASE, __ATOMIC_RELAXED)) {
            __atomic_store_n(&s_instances[i].subscribed, (uint8_t)0, __ATOMIC_RELAXED);
            __atomic_store_n(&s_instances[i].decidedWord, (uint32_t)0, __ATOMIC_RELEASE);
            return w;
        }
    }
    // Cannot happen while the table is sized at the connection cap; if the stack
    // ever exceeds it, the link is unrepresentable and therefore unserviceable.
    od_log_error("ERROR: BLE instance table full, handle %u unrepresentable", (unsigned)handle);
    return 0;
}

static void instanceRetire(uint16_t handle) {
    const int i = instanceIndexOf(handle);
    if (i < 0) return;
    __atomic_store_n(&s_instances[i].subscribed, (uint8_t)0, __ATOMIC_RELAXED);
    __atomic_store_n(&s_instances[i].decidedWord, (uint32_t)0, __ATOMIC_RELAXED);
    __atomic_store_n(&s_instances[i].word, (uint32_t)0, __ATOMIC_RELEASE);
}

static void instancesClear() {
    for (int i = 0; i < OD_BLE_MAX_INSTANCES; i++) {
        __atomic_store_n(&s_instances[i].subscribed, (uint8_t)0, __ATOMIC_RELAXED);
        __atomic_store_n(&s_instances[i].decidedWord, (uint32_t)0, __ATOMIC_RELAXED);
        __atomic_store_n(&s_instances[i].word, (uint32_t)0, __ATOMIC_RELEASE);
    }
}

static void clearHandles() {
    s_server = nullptr;
    s_service = nullptr;
    s_txCharacteristic = nullptr;
    instancesClear();
}

// --- link diagnostics (implementation-private) ------------------------------
static const char* phyName(uint8_t phy) {
    switch (phy) {
        case BLE_GAP_LE_PHY_1M:    return "1M";
        case BLE_GAP_LE_PHY_2M:    return "2M";
        case BLE_GAP_LE_PHY_CODED: return "Coded";
        default:                   return "?";
    }
}

// Report the whole negotiated picture on every event rather than one field per
// callback, so a single log line is enough to judge the link. Logged at INFO:
// this is the answer to "did the 2M PHY / big MTU actually get granted", which
// is worth having in a default-level capture from the bench.
//
// DLE (max LL PDU octets) is absent: NimBLEConnInfo exposes MTU and connection
// interval but not the negotiated data length, unlike Bluefruit's
// BLEConnection::getDataLength(). Not an oversight -- there is no accessor.
static void logNegotiatedLink(NimBLEConnInfo& info, const char* trigger) {
    uint8_t txPhy = 0;
    uint8_t rxPhy = 0;
    if (s_server != nullptr) {
        s_server->getPhy(info.getConnHandle(), &txPhy, &rxPhy);
    }
    od_log_info("[LINK negotiated after %s] PHY tx=%s rx=%s  ATT_MTU=%u  connInterval=%.2f ms",
                trigger, phyName(txPhy), phyName(rxPhy),
                (unsigned)info.getMTU(), info.getConnInterval() * 1.25f);
}

// --- stack callbacks (NimBLE host task -- flag-only) ------------------------
class OdServerCallbacks : public BLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        (void)pServer;
        const uint16_t handle = connInfo.getConnHandle();
        // ORDER IS NORMATIVE (R2): allocate the epoch, publish the table entry,
        // THEN attempt the claim -- so a successful claim never names an instance
        // the loop cannot yet see.
        //
        // The epoch is allocated for EVERY instance, admitted or not. Allocating on
        // successful claim instead (an earlier draft) would leave a refused
        // contender with no epoch, making 7a row 4 -- a contender that reuses the
        // incumbent's handle after a stale link -- indistinguishable from the
        // incumbent. Identity is what the admission decision is MADE on, so it must
        // precede the decision.
        const uint16_t epoch = linkNextEpoch();
        const uint32_t word = instancePublish(handle, epoch);
        // The claim itself, here rather than on the loop task: this is the earliest
        // transport hook, and R7d makes it the authoritative arbitration point. A
        // CAS win IS admission; a loss makes this instance a contender, whose writes
        // the filters below drop from its very first frame. Phase 3 adds the policy
        // that actively disconnects it.
        const LinkId id = { OWNER_BLE, handle, epoch };
        const bool admitted = (word != 0) && linkClaim(id);
        // The claim has resolved; publish that fact so the loop-side refusal scan
        // can tell this instance from one whose CAS has not run yet.
        {
            const int di = instanceIndexOf(handle);
            // Publish the identity the claim resolved FOR, not merely that it
            // resolved. NimBLE serialises host callbacks, so this handle cannot be
            // retired and reused underneath us here.
            if (di >= 0) __atomic_store_n(&s_instances[di].decidedWord, word, __ATOMIC_RELEASE);
        }
        od_log_info("=== BLE CLIENT CONNECTED (ESP32) h=%u e=%u %s ===",
                    (unsigned)handle, (unsigned)epoch,
                    admitted ? "[owner]" : "[contender - refused service]");
        // Payload before flag, flag RELEASE-stored, so a consumer that sees the flag
        // is guaranteed to see this word. A plain store here against the consumer's
        // atomic exchange would be a data race with nothing for its acquire to pair
        // against.
        __atomic_store_n(&s_connectedWord, word, __ATOMIC_RELAXED);
        // Flag-only beyond the claim. The app work a connect implies (rebootFlag
        // reset, updatemsdata() -- which polls I2C and mutates the shared
        // advertisement vector that loop() also drives on a 60 s cadence) would
        // corrupt the heap if run here on the NimBLE host task. loop() consumes the
        // event instead. The claim is exempt because it is one CAS on one word.
        __atomic_store_n(&s_connectedEvent, true, __ATOMIC_RELEASE);
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        (void)pServer;
        const uint16_t handle = connInfo.getConnHandle();
        const int idx = instanceIndexOf(handle);
        const uint32_t word = (idx >= 0) ? __atomic_load_n(&s_instances[idx].word, __ATOMIC_ACQUIRE) : 0;
        od_log_info("=== BLE CLIENT DISCONNECTED (ESP32) h=%u reason=0x%03X ===",
                    (unsigned)handle, (unsigned)reason);
        // Full width: NimBLE's reason spans two ranges and truncation aliases them.
        __atomic_store_n(&s_disconnectReason, (uint16_t)reason, __ATOMIC_RELAXED);
        __atomic_store_n(&s_disconnectedWord, word, __ATOMIC_RELAXED);
        // Retiring the entry is what makes the link's death observable per handle --
        // the predicate bleDropAndWait() polls, and the comparison that lets the loop
        // notice a departed owner even when every event edge was lost.
        //
        // No RX-boundary capture here any more: frames carry their writer's identity
        // (CommandQueueItem::tag), so a departed session's frames self-discard at
        // dispatch instead of needing a boundary that handle reuse could destroy.
        instanceRetire(handle);
        // The token is deliberately NOT released here -- see the matching note in
        // ble_transport_nrf.cpp. Releasing on this callback would admit a new owner
        // while the departed session's state is still live, and RX/TX run before
        // the deferred cleanup. Release stays the abort's last step (R3a); a
        // reconnecting client that loses its claim is reaped by contender refusal.
        // Flag-only. The session teardown this implies (EPD force-off with
        // SPI.end()/rail cut, partial + pipe cleanup) is heavyweight,
        // state-mutating work that races loop()'s SPI streaming and pipe-frame
        // processing. loop() consumes the event and applies its own deferral
        // policy (see serviceBleDisconnectCleanup in main.cpp).
        __atomic_store_n(&s_disconnectedEvent, true, __ATOMIC_RELEASE);
    }
    // Negotiation completes asynchronously, after requestFastLink() returns, so
    // these are the only points where the granted values are knowable. Both are
    // log-only -- no state is touched, so the callback contract holds.
    void onPhyUpdate(NimBLEConnInfo& connInfo, uint8_t txPhy, uint8_t rxPhy) override {
        (void)txPhy;   // logNegotiatedLink re-reads both via getPhy() for one
        (void)rxPhy;   // consistent source, and reports MTU/interval alongside
        logNegotiatedLink(connInfo, "PHY update");
    }
    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        (void)MTU;     // read back through connInfo.getMTU() with the rest
        logNegotiatedLink(connInfo, "MTU exchange");
    }
};

class OdCharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
    // Requirement 2: per-link subscribe state. This used to (void) its connInfo and
    // write one global, so a contender's subscribe cleared or overwrote the
    // incumbent's apparent notify-readiness and stalled its TX.
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
        (void)pCharacteristic;
        const uint16_t handle = connInfo.getConnHandle();
        const int idx = instanceIndexOf(handle);
        if (idx < 0) return;
        const bool on = (subValue & 0x0001) != 0;
        __atomic_store_n(&s_instances[idx].subscribed, (uint8_t)(on ? 1 : 0), __ATOMIC_RELAXED);
        od_log_info("BLE notify subscription h=%u: %s", (unsigned)handle, on ? "enabled" : "disabled");
    }
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        const uint16_t handle = connInfo.getConnHandle();
        // Requirement 1: drop a non-owner's write BEFORE it reaches the RX ring.
        // This must happen here, not on the loop task -- during a ~16 s refresh no
        // loop-side decision runs at all, which is long enough for a gatecrasher to
        // inject a full transfer's worth of commands into the incumbent's stream.
        //
        // One atomic load of the owner word, compared against this instance's
        // identity. That comparison is only possible because the token is a single
        // published word; a loop-task-only owner would give this callback nothing to
        // test against.
        const int idx = instanceIndexOf(handle);
        const uint32_t word = (idx >= 0) ? __atomic_load_n(&s_instances[idx].word, __ATOMIC_ACQUIRE) : 0;
        if (word == 0 || word != linkOwnerWord()) {
            od_log_debug("Dropped write from non-owner h=%u", (unsigned)handle);
            return;
        }
        // Keep the raw NimBLEAttValue: converting to Arduino String uses the
        // C-string (strlen) constructor, which truncates at the first 0x00 byte.
        // Pipe-write frames start with 0x00 (00 70 / 00 71 / 00 81), so String()
        // would report length 0. .length()/.c_str() on NimBLEAttValue preserve
        // the binary payload.
        NimBLEAttValue value = pCharacteristic->getValue();
        // Copy-and-enqueue is all this callback may do; loop() dispatches.
        // bleRxQueuePush() owns the arrival log and every drop reason (empty, too
        // large, ring full) so this callback and nRF's onWriteCb() cannot report the
        // same frame differently. Add no logging here.
        //
        // The frame carries `word` as its tag: the dispatcher re-checks it against
        // the live owner word, so a frame that was legitimate on arrival but whose
        // session ended before it was drained never executes in the next session.
        (void)bleRxQueuePush((const uint8_t*)value.c_str(), value.length(), word);
    }
};

// Static callback objects (no dynamic allocation).
static OdServerCallbacks         s_serverCallbacks;
static OdCharacteristicCallbacks s_charCallbacks;

// --- BleTransport ------------------------------------------------------------
bool BleTransport::begin(const char* deviceName) {
    clearHandles();
    od_log_info("=== Initializing ESP32 BLE ===");
    od_log_info("Device name will be: %s", deviceName);
    BLEDevice::init(deviceName);
    // Preferred only: the central drives the exchange and may settle lower.
    od_log_info("Setting preferred BLE ATT MTU to %u...", (unsigned)OD_BLE_PREFERRED_ATT_MTU);
    BLEDevice::setMTU(OD_BLE_PREFERRED_ATT_MTU);
    s_server = BLEDevice::createServer();
    if (s_server == nullptr) {
        od_log_error("ERROR: Failed to create BLE server");
        return false;
    }
    // deleteCallbacks=false: s_serverCallbacks is a static object; NimBLE must
    // not delete it on deinit()/replacement.
    s_server->setCallbacks(&s_serverCallbacks, false);
    od_log_info("Server callbacks configured");
    BLEUUID serviceUUID("00002446-0000-1000-8000-00805F9B34FB");
    s_service = s_server->createService(serviceUUID);
    if (s_service == nullptr) {
        od_log_error("ERROR: Failed to create BLE service");
        return false;
    }
    od_log_info("BLE service 0x2446 created successfully");
    BLEUUID charUUID("00002446-0000-1000-8000-00805F9B34FB");
    // Declaring max_len (rather than inheriting NimBLE's 512 default) makes the
    // GATT layer reject an oversize write with ATT 0x0D instead of letting it
    // reach onWrite() and be dropped silently by the MAX_COMMAND_SIZE check.
    s_txCharacteristic = s_service->createCharacteristic(
        charUUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::NOTIFY |
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::WRITE_NR,
        OD_BLE_MAX_FRAME
    );
    if (s_txCharacteristic == nullptr) {
        od_log_error("ERROR: Failed to create BLE characteristic");
        return false;
    }
    od_log_info("Characteristic created with properties: READ, NOTIFY, WRITE, WRITE_NR");
    // NimBLE auto-adds the 0x2902 CCCD for NOTIFY characteristics; no BLE2902 needed.
    s_txCharacteristic->setCallbacks(&s_charCallbacks);
    // NimBLE starts services automatically when the server starts advertising; no
    // explicit start() (deprecated no-op in NimBLE 2.x).
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    if (pAdvertising == nullptr) {
        od_log_error("ERROR: Failed to get advertising object");
        return false;
    }
    pAdvertising->addServiceUUID(serviceUUID);
    od_log_info("Service UUID added to advertising");
    s_advertisementData.setName(deviceName);
    s_advertisementData.setFlags(0x06);
    od_log_info("Device name added to advertising");
    return true;
}

void BleTransport::startAdvertising() {
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    if (pAdvertising == nullptr || s_server == nullptr) {
        return;
    }
    // setAdvertisementData() must be the LAST advertising-data call before
    // start(): NimBLE's enableScanResponse()/setPreferredParams()/etc. reset the
    // internal "custom data set" flag, which would make start() discard this
    // payload and re-advertise the piecemeal builder (no manufacturer data / no
    // name). Scan response is off by default in NimBLE 2.x, so no
    // enableScanResponse() needed.
    pAdvertising->setAdvertisementData(s_advertisementData);
    s_server->getAdvertising()->start();
    od_log_info("=== BLE advertising started successfully ===");
}

void BleTransport::restartAdvertising() {
    // Unconditional by contract: the caller owns the "still connected / mid-EPD
    // refresh / stack not up" deferral policy (see serviceBleAdvertisingRestart
    // in main.cpp). The delay mirrors the historical sequence.
    delay(100);
    BLEDevice::startAdvertising();
    od_log_info("BLE advertising restarted");
}

void BleTransport::stopAdvertising() {
    if (s_server == nullptr) return;
    BLEAdvertising* pAdvertising = s_server->getAdvertising();
    if (pAdvertising != nullptr) {
        pAdvertising->stop();
        od_log_info("BLE advertising stopped");
    }
}

void BleTransport::end() {
    BLEDevice::deinit(true);   // clearAll: disables + releases the BT controller
    clearHandles();
}

bool BleTransport::isReady() const {
    return s_server != nullptr;
}

uint8_t BleTransport::connectedCount() const {
    return (s_server != nullptr) ? (uint8_t)s_server->getConnectedCount() : 0;
}

bool BleTransport::notifyReady() const {
    if (s_txCharacteristic == nullptr || s_server == nullptr || s_server->getConnectedCount() == 0) {
        return false;
    }
    // The OWNER's subscription, not "whoever subscribed last". NimBLE auto-creates
    // the 0x2902 CCCD; onSubscribe tracks each client's toggle per instance.
    const uint32_t owner = linkOwnerWord();
    if (owner == 0) return false;
    const LinkId id = linkUnpackWord(owner);
    if (id.who != OWNER_BLE) return false;
    const int idx = instanceIndexOf(id.handle);
    if (idx < 0) return false;
    if (__atomic_load_n(&s_instances[idx].word, __ATOMIC_ACQUIRE) != owner) return false;
    return __atomic_load_n(&s_instances[idx].subscribed, __ATOMIC_RELAXED) != 0;
}

bool BleTransport::notify(const uint8_t* data, uint16_t len) {
    if (s_txCharacteristic == nullptr) return false;
    // Requirement 3, and a LIVE LEAK FIX rather than a new feature. This used to
    // call notify(data, len) -- the two-argument overload, whose third parameter
    // defaults to BLE_HS_CONN_HANDLE_NONE, documented as "send to ALL subscribed
    // clients". A second central that connected and subscribed therefore received
    // every response the incumbent was sent, authentication traffic included,
    // before loop() ran at all and with no policy decision having been made.
    //
    // Targeting the owner's handle closes it. With no owner there is nobody to
    // notify, and returning false leaves the entry queued rather than dropping it.
    const uint32_t owner = linkOwnerWord();
    if (owner == 0) return false;
    const LinkId id = linkUnpackWord(owner);
    if (id.who != OWNER_BLE) return false;
    // Re-validate the handle against the table immediately before sending, rather
    // than trusting a readiness check that ran earlier in the pass. Between
    // notifyReady() and here the host task can retire the owner and hand the SAME
    // numeric handle to a contender -- and a bare handle carries no epoch, so
    // NimBLE would happily deliver the departed owner's queued response to the
    // newcomer, reopening exactly the leak this function exists to close. The
    // full-word comparison catches it because the epoch differs.
    //
    // RESIDUAL, stated rather than papered over: this narrows the window to the few
    // instructions between the check and NimBLE's send, but cannot close it. The
    // stack can retire a link and reassign its numeric handle at any point on the
    // host task, and notify(handle) addresses whatever connection the stack
    // currently calls `handle` -- there is no epoch to pass it. Closing it fully
    // needs the send to be serialised with the host's connection lifecycle, which
    // this API does not offer. What IS closed is the systematic leak: the previous
    // two-argument call broadcast every response to ALL subscribed clients, for the
    // whole life of a contender's subscription.
    const int idx = instanceIndexOf(id.handle);
    if (idx < 0) return false;
    if (__atomic_load_n(&s_instances[idx].word, __ATOMIC_ACQUIRE) != owner) return false;
    // notify(data,len,handle) copies the payload into an mbuf immediately, so a
    // concurrent client WRITE_NR on this shared RX/TX characteristic cannot
    // corrupt the outgoing frame (as setValue()+notify() could, since the no-arg
    // notify sends whatever value is currently stored). On mbuf exhaustion this
    // returns false -- backpressure, not failure.
    return s_txCharacteristic->notify(data, len, id.handle);
}

bool BleTransport::instanceLive(uint16_t handle, uint16_t epoch) const {
    const int idx = instanceIndexOf(handle);
    if (idx < 0) return false;
    return __atomic_load_n(&s_instances[idx].word, __ATOMIC_ACQUIRE) ==
           linkPackWord(OWNER_BLE, handle, epoch);
}

uint8_t BleTransport::liveInstanceCount() const {
    uint8_t n = 0;
    for (int i = 0; i < OD_BLE_MAX_INSTANCES; i++) {
        if (__atomic_load_n(&s_instances[i].word, __ATOMIC_ACQUIRE) != 0) n++;
    }
    return n;
}

uint32_t BleTransport::instanceWordAt(uint8_t index) const {
    if (index >= OD_BLE_MAX_INSTANCES) return 0;
    return __atomic_load_n(&s_instances[index].word, __ATOMIC_ACQUIRE);
}

uint32_t BleTransport::instanceClaimDecidedWordAt(uint8_t index) const {
    if (index >= OD_BLE_MAX_INSTANCES) return 0;
    return __atomic_load_n(&s_instances[index].decidedWord, __ATOMIC_ACQUIRE);
}

uint8_t BleTransport::instanceCapacity() { return OD_BLE_MAX_INSTANCES; }

bool BleTransport::disconnect(uint16_t handle, uint16_t epoch) {
    if (s_server == nullptr) return false;
    // Re-validate the identity as late as possible: the caller's decision to drop
    // this link may have been made a few loads ago, and a numeric handle alone does
    // not identify a connection over time.
    if (!instanceLive(handle, epoch)) return true;   // already gone, or reassigned
    // BLE_ERR_REM_USER_CONN_TERM (0x13) is what NimBLE defaults to and the only
    // reason this seam ever sends; see the header for why 0x09 must not be used.
    const bool ok = s_server->disconnect(handle, BLE_ERR_REM_USER_CONN_TERM);
    if (!ok) {
        // NimBLE already treats "the link is gone" as success (it returns true for
        // BLE_HS_ENOTCONN / BLE_HS_EALREADY / UNK_CONN_ID), so a false here is a
        // genuine failure to ask, not a benign race with a client that left first.
        od_log_warn("WARNING: BLE disconnect request failed for handle %u", (unsigned)handle);
    }
    return ok;
}

uint16_t BleTransport::connIntervalMs(uint16_t handle) const {
    if (s_server == nullptr) return 0;
    NimBLEConnInfo info = s_server->getPeerInfoByHandle(handle);
    const uint16_t units = info.getConnInterval();   // 1.25 ms units
    if (units == 0) return 0;
    return (uint16_t)((units * 5 + 3) / 4);          // ceil(units * 1.25)
}

void BleTransport::setManufacturerData(const uint8_t* msd, uint8_t len) {
    s_advertisementData.setManufacturerData(msd, len);
    BLEAdvertising* pAdvertising = (s_server != nullptr) ? s_server->getAdvertising()
                                                         : BLEDevice::getAdvertising();
    if (pAdvertising == nullptr || connectedCount() > 0) {
        // Only rebuild+restart advertising while disconnected. The former
        // connected branch rebuilt the advertisement data but never pushed it via
        // setAdvertisementData(), so it was dead work -- dropped.
        return;
    }
    pAdvertising->stop();
    BLEAdvertisementData fresh;
    static String savedDeviceName = "";
    if (savedDeviceName.length() == 0) savedDeviceName = "OD" + getChipIdHex();
    fresh.setName(savedDeviceName.c_str());
    fresh.setFlags(0x06);
    fresh.setManufacturerData(msd, len);
    s_advertisementData = fresh;
    // setAdvertisementData() must be the last data call before start():
    // enableScanResponse()/setPreferredParams() reset NimBLE's custom-data flag
    // and would make start() drop this manufacturer-data payload.
    pAdvertising->setAdvertisementData(fresh);
    delay(50);
    pAdvertising->start();
}

// Match nRF's link tuning: 2M PHY + 251-octet DLE. Like nRF, the peripheral only
// auto-accepts what the central asks for, so without this the link stays at
// 1M / 27 octets whenever the phone does not request better.
//
// Called from loop() when the connect event is consumed, not from the connect
// callback -- these are host-stack calls, which the callback contract excludes.
void BleTransport::requestFastLink() {
    // Tune the OWNER's link. This used to read the single s_connHandle scalar,
    // which the newest connect overwrote -- so with a contender attached, link
    // tuning targeted the wrong link (one of the shared-scalar defects R3 names).
    if (s_server == nullptr) return;
    const uint32_t owner = linkOwnerWord();
    if (owner == 0) return;
    const LinkId id = linkUnpackWord(owner);
    if (id.who != OWNER_BLE) return;
    // 2 Mbps both directions. phyOptions applies only to the CODED PHY, so 0.
    // The peer may decline and stay at 1M -- not an error.
    if (!s_server->updatePhy(id.handle, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0)) {
        od_log_warn("2M PHY request rejected (staying at 1M)");
    }
    // 251-octet Link-Layer PDUs (max DLE); NimBLE derives the PHY-appropriate
    // on-air duration itself, so there is no time parameter to pass.
    s_server->setDataLen(id.handle, 251);
    od_log_debug("Requested fast link: 2M PHY + 251-octet DLE");
    // No negotiated-parameter logging here yet, unlike nRF: that would need an
    // equivalent of nRF's delayed one-shot, since negotiation completes after
    // this returns. NimBLEServer::getPhy() is the hook if it is wanted.
}

void BleTransport::boostAdvertising() {
    // No-op: the temporary fast-advertising interval is nRF-only today.
}

void BleTransport::tick() {
    // No-op: nothing periodic to restore, since boostAdvertising() is a no-op.
}

bool BleTransport::eventPending() const {
    // RELAXED: a non-destructive peek used to decide whether to return to loop(),
    // never to establish ordering. Reading one pass stale is harmless -- the next
    // pass sees it.
    return __atomic_load_n(&s_connectedEvent, __ATOMIC_RELAXED) ||
           __atomic_load_n(&s_disconnectedEvent, __ATOMIC_RELAXED);
}

bool BleTransport::takeConnectedEvent(uint32_t* instanceWord) {
    // Atomic exchange, not check-then-clear. The old form could lose an event that
    // arrived inside the gap, and -- now that the flag carries an identity payload
    // -- could also pair one event's flag with another's word. ACQUIRE pairs with
    // the callback's RELEASE store of the flag, which it makes after writing the
    // payload, so a true return guarantees the payload below is the one that flag
    // was raised for.
    if (!__atomic_exchange_n(&s_connectedEvent, false, __ATOMIC_ACQUIRE)) return false;
    if (instanceWord != nullptr) {
        *instanceWord = __atomic_load_n(&s_connectedWord, __ATOMIC_RELAXED);
    }
    return true;
}

bool BleTransport::takeDisconnectedEvent(uint16_t* reason, uint32_t* instanceWord) {
    // See takeConnectedEvent(): atomic exchange so the flag and its payload cannot
    // be torn apart, and so an event landing in the old check-then-clear window is
    // not silently dropped.
    if (!__atomic_exchange_n(&s_disconnectedEvent, false, __ATOMIC_ACQUIRE)) return false;
    if (reason != nullptr) *reason = __atomic_load_n(&s_disconnectReason, __ATOMIC_RELAXED);
    if (instanceWord != nullptr) {
        *instanceWord = __atomic_load_n(&s_disconnectedWord, __ATOMIC_RELAXED);
    }
    return true;
}

bool BleTransport::restartsAdvertisingOnDisconnect() const {
    // NimBLE does not re-advertise by itself here; the application schedules it
    // via requestAdvertisingRestart() so the restart can be held off while an
    // EPD refresh is mid-flight.
    return false;
}

const char* BleTransport::addressString() {
    // The advertised BLE address, lowercase colon-separated (SECTION 9 rule 6,
    // key `mac`). HARDWARE VALIDATION REQUIRED: confirm NimBLEDevice::getAddress()
    // == the advertised AdvA HA sees (public vs static-random) on BOTH S3 and C6.
    static String cached;
    cached = String(NimBLEDevice::getAddress().toString().c_str());
    cached.toLowerCase();
    return cached.c_str();
}

#endif  // TARGET_ESP32
