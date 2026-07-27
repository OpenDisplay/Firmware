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
// Defined in display_service.cpp. True for a mid-stream image-write data frame
// (0x0071) whose per-frame receive/queue logging should be suppressed.
bool imageWriteLogQuietFrame(const uint8_t* data, uint16_t len);

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

static void clearHandles() {
    s_server = nullptr;
    s_service = nullptr;
    s_txCharacteristic = nullptr;
    s_notifySubscribed = false;
}

// --- stack callbacks (NimBLE host task -- flag-only) ------------------------
class OdServerCallbacks : public BLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        (void)pServer;
        (void)connInfo;
        od_log_info("=== BLE CLIENT CONNECTED (ESP32) ===");
        s_notifySubscribed = false;
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
        // Flag-only. The session teardown this implies (EPD force-off with
        // SPI.end()/rail cut, partial + pipe cleanup) is heavyweight,
        // state-mutating work that races loop()'s SPI streaming and pipe-frame
        // processing. loop() consumes the event and applies its own deferral
        // policy (see serviceBleDisconnectCleanup in main.cpp).
        s_disconnectedEvent = true;
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
        const bool quiet = imageWriteLogQuietFrame((const uint8_t*)value.c_str(), value.length());
        if (value.length() > 0 && value.length() <= MAX_COMMAND_SIZE) {
            uint8_t* data = (uint8_t*)value.c_str();
            uint16_t len = value.length();
            if (!quiet) {
                // One-line RX log, mirroring the "[BLE] TX ..." response log.
                // This callback is the BLE write path only, so the tag is always
                // [BLE]; LAN frames are identified by the dispatch banner in
                // imageDataWritten(), which reads g_commandOrigin.
                uint16_t cmd = (len >= 2) ? ((data[0] << 8) | data[1]) : data[0];
                char line[160] = {0};
                int pos = snprintf(line, sizeof(line), "BLE: RX 0x%04X (%u B):", cmd, (unsigned)len);
                if (pos < 0) {
                    pos = 0;
                    line[0] = '\0';
                }
                int dumpLen = (len < 32) ? len : 32;
                for (int i = 0; i < dumpLen && pos < (int)sizeof(line); i++) {
                    int n = snprintf(line + pos, sizeof(line) - pos, " %02X", data[i]);
                    if (n < 0) {
                        break;
                    }
                    pos += n;
                }
                if (len > 32 && pos >= 0 && pos < (int)sizeof(line)) {
                    snprintf(line + pos, sizeof(line) - pos, " ...");
                }
                od_log_debug("%s", line);
            }
            // Copy-and-enqueue is all this callback may do; loop() dispatches.
            if (!bleRxQueuePush(data, len)) {
                od_log_error("ERROR: Command queue full, dropping command");
            }
        } else if (value.length() > MAX_COMMAND_SIZE) {
            od_log_warn("WARNING: Command too large, dropping");
        } else {
            od_log_warn("WARNING: Empty data received");
        }
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

void BleTransport::requestFastLink() {
    // No-op: NimBLE has no link tuning here today. nRF requests 2M PHY +
    // 251-octet DLE; giving ESP32 the same is a one-file change now that the seam
    // exists, but it is deliberately separate work so it can be measured alone
    // (open question 2 in the plan).
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

bool BleTransport::takeDisconnectedEvent(uint8_t* reason) {
    if (!s_disconnectedEvent) return false;
    s_disconnectedEvent = false;
    if (reason != nullptr) *reason = s_disconnectReason;
    return true;
}

bool BleTransport::restartsAdvertisingOnDisconnect() const {
    // NimBLE does not re-advertise by itself here; the application schedules it
    // via bleRestartAdvertisingPending so the restart can be held off while an
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
