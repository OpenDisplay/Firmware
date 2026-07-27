#ifndef BLE_TRANSPORT_NRF_H
#define BLE_TRANSPORT_NRF_H

// Bluefruit-facing declarations for the nRF BleTransport implementation.
// Included ONLY from ble_transport_nrf.cpp, inside that file's TARGET_NRF gate --
// application code includes ble_transport.h and nothing else, so Bluefruit types
// never leak past this implementation.
#ifdef TARGET_NRF

#include <bluefruit.h>

// Defined in display_service.cpp. Called from startAdvertising() to keep the
// historical advertising bring-up sequence byte-for-byte (see the .cpp).
void updatemsdata();

#endif  // TARGET_NRF
#endif  // BLE_TRANSPORT_NRF_H
