// ESP32 (NimBLE-Arduino) implementation of BleTransport.
//
// The whole file is gated on TARGET_ESP32, so an nRF build compiles it to an
// empty translation unit -- no build_src_filter changes needed across the CI
// environments. Every NimBLE object lives here as file-static state; nothing
// outside this file names a NimBLE type.
//
// Threading contract (already holds here, and is what Phase 3 brings to nRF):
// NimBLE host-task callbacks do exactly two things -- copy bytes into the RX
// ring, and set a flag. Everything else runs on the loop() task.
#ifdef TARGET_ESP32

#include <Arduino.h>
#include <string.h>

#include "ble_transport.h"
#include "ble_transport_esp32.h"
#include "command_queue.h"
#include "structs.h"
#include "od_log.h"

String getChipIdHex();

BleTransport ble;

// --- stack objects (were globals in main.h) ---------------------------------
static BLEServer*         s_server = nullptr;
static BLEService*        s_service = nullptr;
static BLECharacteristic* s_txCharacteristic = nullptr;
static BLEAdvertisementData s_advertisementData;

static volatile bool s_notifySubscribed = false;
static volatile bool s_connectedEvent = false;
static volatile bool s_disconnectedEvent = false;
static volatile uint8_t s_disconnectReason = 0;
// RX ring head at the instant the link dropped: the boundary between the departed
// client's queued frames and anything the next client pushes. Captured in the
// disconnect callback because that is the only moment it is knowable -- by the time
// loop() services the event, a reconnect may already have queued frames of its own.
static volatile uint8_t s_rxBoundaryAtDisconnect = 0;
static volatile uint16_t s_connHandle = BLE_HS_CONN_HANDLE_NONE;

static void clearHandles() {
    s_server = nullptr;
    s_service = nullptr;
    s_txCharacteristic = nullptr;
    s_notifySubscribed = false;
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
        od_log_info("=== BLE CLIENT CONNECTED (ESP32) ===");
        s_notifySubscribed = false;
        // Captured here because it is the only place NimBLE hands it to us; the
        // link tuning that consumes it runs later, on the loop task.
        s_connHandle = connInfo.getConnHandle();
        // Flag-only. The app work this implies (rebootFlag reset, updatemsdata()
        // -- which polls I2C and mutates the shared advertisement vector that
        // loop() also drives on a 60 s cadence) would corrupt the heap if run
        // here on the NimBLE host task. loop() consumes the event instead.
        s_connectedEvent = true;
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        (void)pServer;
        (void)connInfo;
        od_log_info("=== BLE CLIENT DISCONNECTED (ESP32) ===");
        s_notifySubscribed = false;
        s_disconnectReason = (uint8_t)reason;
        s_connHandle = BLE_HS_CONN_HANDLE_NONE;
        // Producer-task read of the producer-owned head: no synchronisation needed,
        // and within the copy-and-flag contract. Nothing this client can still send
        // exists past this point -- the link is gone -- so this is exactly the last
        // frame of the departed session.
        s_rxBoundaryAtDisconnect = bleRxQueueHead();
        // Flag-only. The session teardown this implies (EPD force-off with
        // SPI.end()/rail cut, partial + pipe cleanup) is heavyweight,
        // state-mutating work that races loop()'s SPI streaming and pipe-frame
        // processing. loop() consumes the event and applies its own deferral
        // policy (see serviceBleDisconnectCleanup in main.cpp).
        s_disconnectedEvent = true;
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
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
        (void)pCharacteristic;
        (void)connInfo;
        s_notifySubscribed = (subValue & 0x0001) != 0;
        od_log_info("BLE notify subscription: %s", s_notifySubscribed ? "enabled" : "disabled");
    }
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        (void)connInfo;
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
        (void)bleRxQueuePush((const uint8_t*)value.c_str(), value.length());
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
    // NimBLE auto-creates the 0x2902 CCCD; onSubscribe tracks the client's toggle.
    return s_notifySubscribed;
}

bool BleTransport::notify(const uint8_t* data, uint16_t len) {
    if (s_txCharacteristic == nullptr) return false;
    // notify(data,len) copies the payload into an mbuf immediately, so a
    // concurrent client WRITE_NR on this shared RX/TX characteristic cannot
    // corrupt the outgoing frame (as setValue()+notify() could, since the no-arg
    // notify sends whatever value is currently stored). On mbuf exhaustion this
    // returns false -- backpressure, not failure.
    return s_txCharacteristic->notify(data, len);
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
    if (s_server == nullptr || s_connHandle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    // 2 Mbps both directions. phyOptions applies only to the CODED PHY, so 0.
    // The peer may decline and stay at 1M -- not an error.
    if (!s_server->updatePhy(s_connHandle, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0)) {
        od_log_warn("2M PHY request rejected (staying at 1M)");
    }
    // 251-octet Link-Layer PDUs (max DLE); NimBLE derives the PHY-appropriate
    // on-air duration itself, so there is no time parameter to pass.
    s_server->setDataLen(s_connHandle, 251);
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

bool BleTransport::takeConnectedEvent() {
    if (!s_connectedEvent) return false;
    s_connectedEvent = false;
    return true;
}

bool BleTransport::takeDisconnectedEvent(uint8_t* reason, uint8_t* rxBoundary) {
    if (!s_disconnectedEvent) return false;
    s_disconnectedEvent = false;
    if (reason != nullptr) *reason = s_disconnectReason;
    if (rxBoundary != nullptr) *rxBoundary = s_rxBoundaryAtDisconnect;
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
