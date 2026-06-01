#ifndef BLE_INIT_H
#define BLE_INIT_H

#ifdef TARGET_NRF
void ble_nrf_stack_init();
void ble_nrf_advertising_start();
#endif
#ifdef TARGET_ESP32
void ble_init();
void ble_init_esp32(bool update_manufacturer_data = true);
void esp32_restart_ble_advertising(void);
void esp32_ble_clear_handles(void);
bool esp32_ble_notify_enabled(void);
extern volatile bool bleRestartAdvertisingPending;
extern volatile bool esp32BleNotifySubscribed;
#endif

#endif
