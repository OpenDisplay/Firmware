#ifndef BLE_INIT_H
#define BLE_INIT_H

#ifdef TARGET_NRF
void ble_nrf_stack_init();
void ble_nrf_advertising_start();
#endif
#ifdef TARGET_ESP32
void ble_init();
void ble_init_esp32(bool update_manufacturer_data = true);
#endif

#endif
