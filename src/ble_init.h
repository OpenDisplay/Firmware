#ifndef BLE_INIT_H
#define BLE_INIT_H

#ifdef TARGET_NRF
void ble_nrf_stack_init();
void ble_nrf_advertising_start();
void ble_nrf_boost_advertising(void);
void ble_nrf_apply_adv_interval(void);
void ble_nrf_advertising_tick(void);
#endif
#ifdef TARGET_ESP32
void ble_init();
void ble_init_esp32(bool update_manufacturer_data = true);
void esp32_restart_ble_advertising(void);
void esp32_ble_clear_handles(void);
bool esp32_ble_notify_enabled(void);
extern volatile bool bleRestartAdvertisingPending;
extern volatile bool bleConnectMsdUpdatePending;
extern volatile bool bleDirectWriteCleanupPending;
extern volatile bool esp32BleNotifySubscribed;
#endif

#endif
