#ifndef BLE_INIT_H
#define BLE_INIT_H

#include <stdint.h>

#ifdef TARGET_NRF
void ble_nrf_stack_init();
void ble_nrf_advertising_start();
void ble_nrf_boost_advertising(void);
void ble_nrf_apply_adv_interval(void);
void ble_nrf_advertising_tick(void);
// Link diagnostics: log negotiated PHY / ATT MTU / DLE (LL PDU octets) / conn interval.
void ble_nrf_log_link_params(uint16_t conn_handle, const char* phase);
void ble_nrf_arm_link_diag(uint16_t conn_handle);   // one-shot: re-log ~2.5 s after connect
void ble_nrf_request_fast_link(uint16_t conn_handle); // request 2M PHY + 251-octet DLE (max throughput)
#endif
#ifdef TARGET_ESP32
// ESP32 BLE is backed by NimBLE-Arduino (h2zero). The aliases below let the rest
// of the firmware keep the historical BLE* spellings so only the genuinely
// changed NimBLE 2.x call sites need edits.
#include <NimBLEDevice.h>
using BLEDevice                  = NimBLEDevice;
using BLEServer                  = NimBLEServer;
using BLEService                 = NimBLEService;
using BLECharacteristic          = NimBLECharacteristic;
using BLEAdvertising             = NimBLEAdvertising;
using BLEAdvertisementData       = NimBLEAdvertisementData;
using BLEServerCallbacks         = NimBLEServerCallbacks;
using BLECharacteristicCallbacks = NimBLECharacteristicCallbacks;
using BLEUUID                    = NimBLEUUID;

void ble_init();
void ble_init_esp32(bool update_manufacturer_data = true);
void esp32_restart_ble_advertising(void);
void esp32_ble_clear_handles(void);
bool esp32_ble_notify_enabled(void);
extern volatile bool bleRestartAdvertisingPending;
extern volatile bool esp32BleNotifySubscribed;
#endif

#endif
