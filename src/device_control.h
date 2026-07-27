#ifndef DEVICE_CONTROL_H
#define DEVICE_CONTROL_H

#include <Arduino.h>

// BLE connect/disconnect application hooks, invoked by BleTransport.
// Declared in ble_transport.h; defined here in device_control.cpp.
void reboot();
void processButtonEvents();
void flashLed(uint8_t color, uint8_t brightness);
void processLedFlash();
void initButtons();
void handleLedActivate(uint8_t* data, uint16_t len);
void handleLedStop(uint8_t* data, uint16_t len);
void enterDFUMode();
void handleDeepSleepCommand(const uint8_t* payload, uint16_t payloadLen);
void handlePowerOffCommand(const uint8_t* payload, uint16_t payloadLen);

#endif
