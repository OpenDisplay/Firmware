#include "ble_init.h"
#include "structs.h"
#include "encryption.h"
#include "display_service.h"

#ifdef TARGET_NRF
#include <bluefruit.h>
extern "C" {
#include "nrf_soc.h"
}
extern BLEDfu bledfu;
extern BLEService imageService;
extern BLECharacteristic imageCharacteristic;
extern struct GlobalConfig globalConfig;
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void imageDataWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
String getChipIdHex();
void writeSerial(String message, bool newLine = true);
#endif

#ifdef TARGET_ESP32
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#if defined(CONFIG_BLUEDROID_ENABLED)
#include <BLE2902.h>
#endif

String getChipIdHex();
void writeSerial(String message, bool newLine = true);
#include "esp32_ble_callbacks.h"

extern struct GlobalConfig globalConfig;
extern BLEServer* pServer;
extern BLEService* pService;
extern BLECharacteristic* pTxCharacteristic;
extern BLECharacteristic* pRxCharacteristic;
extern BLEAdvertisementData* advertisementData;
extern MyBLEServerCallbacks staticServerCallbacks;
extern MyBLECharacteristicCallbacks staticCharCallbacks;
#endif

#ifdef TARGET_NRF
static uint32_t s_nrf_adv_boost_until = 0;

static constexpr uint16_t NRF_ADV_INTERVAL_MIN = 256;   // 160 ms
static constexpr uint16_t NRF_ADV_INTERVAL_MAX = 1600;  // 1000 ms
static constexpr uint16_t NRF_ADV_BOOST_MIN = 32;         // 20 ms
static constexpr uint16_t NRF_ADV_BOOST_MAX = 48;         // 30 ms
static constexpr uint32_t NRF_ADV_BOOST_MS = 3000;

void ble_nrf_boost_advertising(void) {
    s_nrf_adv_boost_until = millis() + NRF_ADV_BOOST_MS;
}

void ble_nrf_apply_adv_interval(void) {
    if (s_nrf_adv_boost_until != 0 && millis() < s_nrf_adv_boost_until) {
        Bluefruit.Advertising.setInterval(NRF_ADV_BOOST_MIN, NRF_ADV_BOOST_MAX);
    } else {
        s_nrf_adv_boost_until = 0;
        Bluefruit.Advertising.setInterval(NRF_ADV_INTERVAL_MIN, NRF_ADV_INTERVAL_MAX);
    }
}

void ble_nrf_advertising_tick(void) {
    static bool was_boosted = false;
    const bool boosting = (s_nrf_adv_boost_until != 0 && millis() < s_nrf_adv_boost_until);
    if (boosting) {
        was_boosted = true;
        return;
    }
    if (!was_boosted || !Bluefruit.Advertising.isRunning()) {
        was_boosted = false;
        s_nrf_adv_boost_until = 0;
        return;
    }
    was_boosted = false;
    s_nrf_adv_boost_until = 0;
    Bluefruit.Advertising.setInterval(NRF_ADV_INTERVAL_MIN, NRF_ADV_INTERVAL_MAX);
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
}

// --- Link-layer diagnostics -------------------------------------------------
// DLE (Data Length Extension) sets the max Link-Layer PDU payload: 27 octets by
// default, up to 251 once negotiated. The nRF peripheral only auto-accepts the
// central's request, which arrives AFTER connect_callback, so we log twice: once
// at connect (baseline) and once ~2.5 s later (negotiated).
void ble_nrf_log_link_params(uint16_t conn_handle, const char* phase) {
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    if (conn == nullptr) {
        writeSerial(String("[LINK ") + phase + "] no connection (handle " + String(conn_handle) + ")");
        return;
    }
    uint8_t  phy = conn->getPHY();
    uint16_t mtu = conn->getMtu();                // ATT MTU (23 default; 247 cap here)
    uint16_t dle = conn->getDataLength();         // LL PDU payload octets (27 default; 251 max)
    uint16_t ci  = conn->getConnectionInterval(); // units of 1.25 ms
    const char* phyStr = (phy == BLE_GAP_PHY_2MBPS) ? "2M" :
                         (phy == BLE_GAP_PHY_1MBPS) ? "1M" : "?";
    writeSerial(String("[LINK ") + phase + "] PHY=" + phyStr +
                "  ATT_MTU=" + String(mtu) +
                "  DLE=" + String(dle) + " octets" +
                "  connInterval=" + String(ci * 1.25f, 2) + " ms");
}

// One-shot timer (armed only in connect_callback — no per-loop polling). Fires
// once on the FreeRTOS timer task after the central finishes negotiation.
static SoftwareTimer s_link_diag_timer;
static uint16_t      s_link_diag_conn = BLE_CONN_HANDLE_INVALID;

static void ble_nrf_link_diag_cb(TimerHandle_t /*xTimer*/) {
    if (Bluefruit.connected()) {
        ble_nrf_log_link_params(s_link_diag_conn, "negotiated");
    }
}

// Proactively upgrade the link for throughput: the nRF peripheral only auto-accepts
// the central's PHY/DLE requests, so if the phone never asks we stay at 1M / 27 octets.
// Requesting here (both are no-ops if the peer already negotiated the same or better).
void ble_nrf_request_fast_link(uint16_t conn_handle) {
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    if (conn == nullptr) return;

    // 2 Mbps PHY (tx + rx). Peer may decline and stay at 1M.
    conn->requestPHY(BLE_GAP_PHY_2MBPS);

    // 251-octet Link-Layer PDUs (max DLE). AUTO time lets the controller derive the
    // PHY-appropriate on-air duration.
    ble_gap_data_length_params_t dl;
    dl.max_tx_octets  = 251;
    dl.max_rx_octets  = 251;
    dl.max_tx_time_us = BLE_GAP_DATA_LENGTH_AUTO;
    dl.max_rx_time_us = BLE_GAP_DATA_LENGTH_AUTO;
    ble_gap_data_length_limitation_t limit = { 0, 0, 0 };
    if (!conn->requestDataLengthUpdate(&dl, &limit)) {
        writeSerial("DLE 251 request rejected (tx_lim=" + String(limit.tx_payload_limited_octets) +
                    " rx_lim=" + String(limit.rx_payload_limited_octets) +
                    " time_lim_us=" + String(limit.tx_rx_time_limited_us) + ")");
    }
    writeSerial("Requested fast link: 2M PHY + 251-octet DLE");
}

void ble_nrf_arm_link_diag(uint16_t conn_handle) {
    s_link_diag_conn = conn_handle;
    static bool created = false;
    if (!created) {
        // Create the one-shot (repeating=false) on the first connection only.
        s_link_diag_timer.begin(500, ble_nrf_link_diag_cb, NULL, false);
        created = true;
    }
    s_link_diag_timer.reset();   // start/restart the one-shot from now; fires ~2.5 s later
}

void ble_nrf_stack_init() {
    Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.autoConnLed(false);
    Bluefruit.setTxPower(globalConfig.power_option.tx_power);
    Bluefruit.begin(1, 0);
    if (!isEncryptionEnabled()) {
        bledfu.begin();
        writeSerial("BLE DFU initialized successfully (encryption disabled)");
    } else {
        writeSerial("BLE DFU service NOT initialized (encryption enabled - use CMD_ENTER_DFU)");
    }
    writeSerial("BLE initialized successfully");
    writeSerial("Setting up BLE service 0x2446...");
    imageService.begin();
    writeSerial("BLE service started");
    imageCharacteristic.setWriteCallback(imageDataWritten);
    writeSerial("BLE write callback set");
    imageCharacteristic.begin();
    writeSerial("BLE characteristic started");
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
    writeSerial("BLE callbacks registered");
    String deviceName = "OD" + getChipIdHex();
    Bluefruit.setName(deviceName.c_str());
    writeSerial("Device name set to: " + deviceName);
    writeSerial("Configuring power management...");
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    writeSerial("Power management configured");
}

void ble_nrf_advertising_start() {
    writeSerial("Configuring BLE advertising...");
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    updatemsdata();
    Bluefruit.Advertising.restartOnDisconnect(true);
    ble_nrf_apply_adv_interval();
    Bluefruit.Advertising.setFastTimeout(10);
    writeSerial("Starting BLE advertising...");
    Bluefruit.Advertising.start(0);
}
#endif

#ifdef TARGET_ESP32
void ble_init() {
    ble_init_esp32(true);
}
#endif

#ifdef TARGET_ESP32
volatile bool bleRestartAdvertisingPending = false;
volatile bool esp32BleNotifySubscribed = false;
#if defined(CONFIG_BLUEDROID_ENABLED)
static BLE2902* s_bleNotifyCccd = nullptr;
#endif

void esp32_ble_clear_handles(void) {
    pServer = nullptr;
    pService = nullptr;
    pTxCharacteristic = nullptr;
    pRxCharacteristic = nullptr;
    esp32BleNotifySubscribed = false;
}

bool esp32_ble_notify_enabled(void) {
    if (pTxCharacteristic == nullptr || pServer == nullptr || pServer->getConnectedCount() == 0) {
        return false;
    }
#if defined(CONFIG_NIMBLE_ENABLED)
    return esp32BleNotifySubscribed;
#else
    BLE2902* cccd = (BLE2902*)pTxCharacteristic->getDescriptorByUUID((uint16_t)0x2902);
    return cccd != nullptr && cccd->getNotifications();
#endif
}

void esp32_restart_ble_advertising(void) {
    if (pServer == nullptr) {
        bleRestartAdvertisingPending = true;
        return;
    }
    if (pServer->getConnectedCount() > 0) {
        bleRestartAdvertisingPending = false;
        return;
    }
    if (epdRefreshInProgress) {
        bleRestartAdvertisingPending = true;
        return;
    }
    bleRestartAdvertisingPending = false;
    delay(100);
    BLEDevice::startAdvertising();
    updatemsdata();
    writeSerial("BLE advertising restarted");
}

void ble_init_esp32(bool update_manufacturer_data) {
    esp32_ble_clear_handles();
#if defined(CONFIG_BLUEDROID_ENABLED)
    if (s_bleNotifyCccd != nullptr) {
        delete s_bleNotifyCccd;
        s_bleNotifyCccd = nullptr;
    }
#endif
    writeSerial("=== Initializing ESP32 BLE ===");
    String deviceName = "OD" + getChipIdHex();
    writeSerial("Device name will be: " + deviceName);
    BLEDevice::init(deviceName.c_str());
    writeSerial("Setting BLE MTU to 512...");
    BLEDevice::setMTU(512);
    pServer = BLEDevice::createServer();
    if (pServer == nullptr) {
        writeSerial("ERROR: Failed to create BLE server");
        return;
    }
    pServer->setCallbacks(&staticServerCallbacks);
    writeSerial("Server callbacks configured");
    BLEUUID serviceUUID("00002446-0000-1000-8000-00805F9B34FB");
    pService = pServer->createService(serviceUUID);
    if (pService == nullptr) {
        writeSerial("ERROR: Failed to create BLE service");
        return;
    }
    writeSerial("BLE service 0x2446 created successfully");
    BLEUUID charUUID("00002446-0000-1000-8000-00805F9B34FB");
    pTxCharacteristic = pService->createCharacteristic(
        charUUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR
    );
    if (pTxCharacteristic == nullptr) {
        writeSerial("ERROR: Failed to create BLE characteristic");
        return;
    }
    writeSerial("Characteristic created with properties: READ, NOTIFY, WRITE, WRITE_NR");
#if defined(CONFIG_BLUEDROID_ENABLED)
    s_bleNotifyCccd = new BLE2902();
    pTxCharacteristic->addDescriptor(s_bleNotifyCccd);
    writeSerial("CCCD (0x2902) descriptor added for notifications");
#endif
    pTxCharacteristic->setCallbacks(&staticCharCallbacks);
    pRxCharacteristic = pTxCharacteristic;
    pService->start();
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    if (pAdvertising == nullptr) {
        writeSerial("ERROR: Failed to get advertising object");
        return;
    }
    pAdvertising->addServiceUUID(serviceUUID);
    writeSerial("Service UUID added to advertising");
    advertisementData->setName(deviceName);
    advertisementData->setFlags(0x06);
    writeSerial("Device name added to advertising");
    if (update_manufacturer_data) {
        updatemsdata();
    }
    pAdvertising->setAdvertisementData(*advertisementData);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0006);
    pAdvertising->setMaxPreferred(0x0012);
    writeSerial("Advertising intervals set");
    pServer->getAdvertising()->setMinPreferred(0x06);
    pServer->getAdvertising()->setMaxPreferred(0x12);
    pServer->getAdvertising()->start();
    writeSerial("=== BLE advertising started successfully ===");
    writeSerial("Device ready: " + deviceName);
    writeSerial("Waiting for BLE connections...");
}
#endif
