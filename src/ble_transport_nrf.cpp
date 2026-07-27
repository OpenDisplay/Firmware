// nRF52840 (Adafruit Bluefruit / SoftDevice S140) implementation of BleTransport.
//
// The whole file is gated on TARGET_NRF, so an ESP32 build compiles it to an
// empty translation unit -- no build_src_filter changes needed across the CI
// environments. Every Bluefruit object lives here as file-static state; nothing
// outside this file names a Bluefruit type.
#ifdef TARGET_NRF

#include "ble_transport.h"
#include "ble_transport_nrf.h"
#include "communication.h"   // imageDataWritten + its opaque parameter typedefs
#include "structs.h"
#include "encryption.h"
#include "od_log.h"

extern "C" {
#include "nrf_soc.h"
}

extern struct GlobalConfig globalConfig;
String getChipIdHex();

BleTransport ble;

// --- stack objects (were globals in main.h) ---------------------------------
static BLEDfu s_dfu;
static BLEService s_imageService("2446");
// max_len is a GATT-declared attribute length the SoftDevice reserves for real
// (vloc = BLE_GATTS_VLOC_STACK), so 512 cost 512 B of attribute table while the
// link caps at ATT MTU 247 (payload 244) -- half of it was unreachable.
static BLECharacteristic s_imageCharacteristic(
    "2446", BLEWrite | BLEWriteWithoutResponse | BLENotify, OD_BLE_MAX_FRAME);

static bool     s_begun = false;
static uint16_t s_connHandle = BLE_CONN_HANDLE_INVALID;
static volatile bool s_connectedEvent = false;
static volatile bool s_disconnectedEvent = false;

// --- advertising interval policy --------------------------------------------
static uint32_t s_advBoostUntil = 0;

static constexpr uint16_t NRF_ADV_INTERVAL_MIN = 256;   // 160 ms
static constexpr uint16_t NRF_ADV_INTERVAL_MAX = 1600;  // 1000 ms
static constexpr uint16_t NRF_ADV_BOOST_MIN = 32;       // 20 ms
static constexpr uint16_t NRF_ADV_BOOST_MAX = 48;       // 30 ms
static constexpr uint32_t NRF_ADV_BOOST_MS = 3000;

static void applyAdvInterval() {
    if (s_advBoostUntil != 0 && millis() < s_advBoostUntil) {
        Bluefruit.Advertising.setInterval(NRF_ADV_BOOST_MIN, NRF_ADV_BOOST_MAX);
    } else {
        s_advBoostUntil = 0;
        Bluefruit.Advertising.setInterval(NRF_ADV_INTERVAL_MIN, NRF_ADV_INTERVAL_MAX);
    }
}

// --- link-layer diagnostics (implementation-private) ------------------------
// DLE (Data Length Extension) sets the max Link-Layer PDU payload: 27 octets by
// default, up to 251 once negotiated. The nRF peripheral only auto-accepts the
// central's request, which arrives AFTER the connect callback, so we log twice:
// once at connect (baseline) and once ~2.5 s later (negotiated).
static void logLinkParams(uint16_t conn_handle, const char* phase) {
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    if (conn == nullptr) {
        od_log_debug("[LINK %s] no connection (handle %u)", phase, conn_handle);
        return;
    }
    uint8_t  phy = conn->getPHY();
    uint16_t mtu = conn->getMtu();                // ATT MTU (23 default; 247 cap here)
    uint16_t dle = conn->getDataLength();         // LL PDU payload octets (27 default; 251 max)
    uint16_t ci  = conn->getConnectionInterval(); // units of 1.25 ms
    const char* phyStr = (phy == BLE_GAP_PHY_2MBPS) ? "2M" :
                         (phy == BLE_GAP_PHY_1MBPS) ? "1M" : "?";
    od_log_debug("[LINK %s] PHY=%s  ATT_MTU=%u  DLE=%u octets  connInterval=%.2f ms",
                 phase, phyStr, mtu, dle, ci * 1.25f);
}

// One-shot timer (armed only on connect -- no per-loop polling). Fires once on
// the FreeRTOS timer task after the central finishes negotiation. This is a
// third execution context; it only logs, which is why it is safe here.
static SoftwareTimer s_linkDiagTimer;
static uint16_t      s_linkDiagConn = BLE_CONN_HANDLE_INVALID;

static void linkDiagCallback(TimerHandle_t /*xTimer*/) {
    if (Bluefruit.connected()) {
        logLinkParams(s_linkDiagConn, "negotiated");
    }
}

static void armLinkDiag(uint16_t conn_handle) {
    s_linkDiagConn = conn_handle;
    static bool created = false;
    if (!created) {
        // Create the one-shot (repeating=false) on the first connection only.
        s_linkDiagTimer.begin(500, linkDiagCallback, NULL, false);
        created = true;
    }
    s_linkDiagTimer.reset();   // start/restart the one-shot; fires ~2.5 s later
}

// --- stack callbacks ---------------------------------------------------------
// Phase 1 preserves today's nRF threading: the app hook runs inline on the
// SoftDevice callback task, exactly as connect_callback did before. Phase 3
// converts these to flag-only, at which point the event flags below become the
// sole output and bleAppOnConnect/Disconnect move to loop().
static void onConnectCb(uint16_t conn_handle) {
    s_connHandle = conn_handle;
    s_connectedEvent = true;
    bleAppOnConnect();
    logLinkParams(conn_handle, "at connect");  // baseline (pre-negotiation)
    ble.requestFastLink();                     // request 2M PHY + 251-octet DLE
    armLinkDiag(conn_handle);                  // re-log once negotiation settles
}

// Adapter: Bluefruit's write_callback_t is BLECharacteristic*-shaped, whereas the
// shared dispatcher takes an opaque pointer (it ignores both leading arguments on
// every target). Adapting here is what keeps Bluefruit types out of
// communication.cpp. Still runs on the SoftDevice callback task in Phase 1;
// Phase 3 replaces the body with a push onto the shared RX ring.
static void onWriteCb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    imageDataWritten(conn_hdl, (BLECharPtr)chr, data, len);
}

static void onDisconnectCb(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    s_connHandle = BLE_CONN_HANDLE_INVALID;
    s_disconnectedEvent = true;
    bleAppOnDisconnect(reason);
}

// --- BleTransport ------------------------------------------------------------
bool BleTransport::begin(const char* deviceName) {
    Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.autoConnLed(false);
    Bluefruit.setTxPower(globalConfig.power_option.tx_power);
    Bluefruit.begin(1, 0);
    od_log_info("BLE initialized successfully");
    od_log_info("Setting up BLE service 0x2446...");
    s_imageService.begin();
    od_log_info("BLE service started");
    s_imageCharacteristic.setWriteCallback(onWriteCb);
    od_log_info("BLE write callback set");
    s_imageCharacteristic.begin();
    od_log_info("BLE characteristic started");
    // Register the DFU service LAST so its presence/absence (it is only added when
    // encryption is disabled) never shifts the handles of s_imageCharacteristic and
    // its CCCD. GATT handles are assigned in begin() order; keeping the app
    // characteristic ahead of the conditional DFU service keeps its handles stable
    // across encryption on/off, so a client's cached CCCD handle stays valid and
    // notify setup won't fail with ATT "Invalid handle". Must stay after
    // Bluefruit.begin() (SoftDevice up first).
    if (!isEncryptionEnabled()) {
        s_dfu.begin();
        od_log_info("BLE DFU initialized successfully (encryption disabled)");
    } else {
        od_log_info("BLE DFU service NOT initialized (encryption enabled - use CMD_ENTER_DFU)");
    }
    Bluefruit.Periph.setConnectCallback(onConnectCb);
    Bluefruit.Periph.setDisconnectCallback(onDisconnectCb);
    od_log_info("BLE callbacks registered");
    Bluefruit.setName(deviceName);
    od_log_info("Device name set to: %s", deviceName);
    od_log_info("Configuring power management...");
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    od_log_info("Power management configured");
    s_begun = true;
    return true;
}

void BleTransport::startAdvertising() {
    od_log_info("Configuring BLE advertising...");
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    // Deliberately kept inside this sequence rather than hoisted to the caller:
    // updatemsdata() lands in setManufacturerData() below, which itself sets
    // setFastTimeout(1). Calling it after the setFastTimeout(10) line would leave
    // the fast-advertising window at 1 s instead of 10 s -- a real behaviour
    // change. Phase 1 keeps the historical order byte-for-byte.
    updatemsdata();
    Bluefruit.Advertising.restartOnDisconnect(true);
    applyAdvInterval();
    Bluefruit.Advertising.setFastTimeout(10);
    od_log_info("Starting BLE advertising...");
    Bluefruit.Advertising.start(0);
}

void BleTransport::restartAdvertising() {
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
}

void BleTransport::stopAdvertising() {
    Bluefruit.Advertising.stop();
}

void BleTransport::end() {
    // No-op: the SoftDevice stays up for the life of the nRF firmware. Only the
    // ESP32 tears the controller down (deep sleep / pre-restart).
}

bool BleTransport::isReady() const {
    return s_begun;
}

uint8_t BleTransport::connectedCount() const {
    // Bluefruit.connected() returns the number of active connections; the
    // peripheral is configured for a single link (Bluefruit.begin(1, 0)).
    return (uint8_t)Bluefruit.connected();
}

bool BleTransport::notifyReady() const {
    return Bluefruit.connected() && s_imageCharacteristic.notifyEnabled();
}

bool BleTransport::notify(const uint8_t* data, uint16_t len) {
    return s_imageCharacteristic.notify(data, len);
}

void BleTransport::setManufacturerData(const uint8_t* msd, uint8_t len) {
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, msd, len);
    applyAdvInterval();
    Bluefruit.Advertising.setFastTimeout(1);
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
}

// Proactively upgrade the link for throughput: the nRF peripheral only
// auto-accepts the central's PHY/DLE requests, so if the phone never asks we
// stay at 1M / 27 octets. Both requests are no-ops if the peer already
// negotiated the same or better.
void BleTransport::requestFastLink() {
    BLEConnection* conn = Bluefruit.Connection(s_connHandle);
    if (conn == nullptr) return;

    // 2 Mbps PHY (tx + rx). Peer may decline and stay at 1M.
    conn->requestPHY(BLE_GAP_PHY_2MBPS);

    // 251-octet Link-Layer PDUs (max DLE). AUTO time lets the controller derive
    // the PHY-appropriate on-air duration.
    ble_gap_data_length_params_t dl;
    dl.max_tx_octets  = 251;
    dl.max_rx_octets  = 251;
    dl.max_tx_time_us = BLE_GAP_DATA_LENGTH_AUTO;
    dl.max_rx_time_us = BLE_GAP_DATA_LENGTH_AUTO;
    ble_gap_data_length_limitation_t limit = { 0, 0, 0 };
    if (!conn->requestDataLengthUpdate(&dl, &limit)) {
        od_log_warn("DLE 251 request rejected (tx_lim=%u rx_lim=%u time_lim_us=%u)",
                    limit.tx_payload_limited_octets, limit.rx_payload_limited_octets,
                    limit.tx_rx_time_limited_us);
    }
    od_log_debug("Requested fast link: 2M PHY + 251-octet DLE");
}

void BleTransport::boostAdvertising() {
    s_advBoostUntil = millis() + NRF_ADV_BOOST_MS;
}

void BleTransport::tick() {
    static bool was_boosted = false;
    const bool boosting = (s_advBoostUntil != 0 && millis() < s_advBoostUntil);
    if (boosting) {
        was_boosted = true;
        return;
    }
    if (!was_boosted || !Bluefruit.Advertising.isRunning()) {
        was_boosted = false;
        s_advBoostUntil = 0;
        return;
    }
    was_boosted = false;
    s_advBoostUntil = 0;
    Bluefruit.Advertising.setInterval(NRF_ADV_INTERVAL_MIN, NRF_ADV_INTERVAL_MAX);
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
}

bool BleTransport::takeConnectedEvent() {
    if (!s_connectedEvent) return false;
    s_connectedEvent = false;
    return true;
}

bool BleTransport::takeDisconnectedEvent() {
    if (!s_disconnectedEvent) return false;
    s_disconnectedEvent = false;
    return true;
}

const char* BleTransport::addressString() {
    // No nRF caller today: the only consumer is wifi_service.cpp, which is
    // ESP32-only (OPENDISPLAY_HAS_WIFI). Returning an empty string rather than
    // guessing at a Bluefruit address API that has never been exercised here --
    // implement and bench-verify against the advertised AdvA before any nRF use.
    return "";
}

#endif  // TARGET_NRF
