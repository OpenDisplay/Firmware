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
    Bluefruit.Advertising.setInterval(256, 1600);
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
void ble_init_esp32(bool update_manufacturer_data) {
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
    pAdvertising->setMinPreferred(0x0012);
    writeSerial("Advertising intervals set");
    pServer->getAdvertising()->setMinPreferred(0x06);
    pServer->getAdvertising()->setMinPreferred(0x12);
    pServer->getAdvertising()->start();
    writeSerial("=== BLE advertising started successfully ===");
    writeSerial("Device ready: " + deviceName);
    writeSerial("Waiting for BLE connections...");
}
#endif
