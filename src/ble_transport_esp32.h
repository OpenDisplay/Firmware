#ifndef BLE_TRANSPORT_ESP32_H
#define BLE_TRANSPORT_ESP32_H

// NimBLE-facing declarations for the ESP32 BleTransport implementation.
// Included ONLY from ble_transport_esp32.cpp, inside that file's TARGET_ESP32
// gate. This is what stops NimBLE types leaking: the BLE* aliases below used to
// live in ble_init.h, which six translation units included (main.h,
// communication.cpp, display_service.cpp, device_control.cpp, wifi_service.cpp,
// esp32_ble_callbacks.h). Application code now includes ble_transport.h only.
#ifdef TARGET_ESP32

#include <NimBLEDevice.h>

// ESP32 BLE is backed by NimBLE-Arduino (h2zero). The aliases keep the
// historical BLE* spellings inside this implementation.
using BLEDevice                  = NimBLEDevice;
using BLEServer                  = NimBLEServer;
using BLEService                 = NimBLEService;
using BLECharacteristic          = NimBLECharacteristic;
using BLEAdvertising             = NimBLEAdvertising;
using BLEAdvertisementData       = NimBLEAdvertisementData;
using BLEServerCallbacks         = NimBLEServerCallbacks;
using BLECharacteristicCallbacks = NimBLECharacteristicCallbacks;
using BLEUUID                    = NimBLEUUID;

#endif  // TARGET_ESP32
#endif  // BLE_TRANSPORT_ESP32_H
