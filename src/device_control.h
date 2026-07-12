#ifndef DEVICE_CONTROL_H
#define DEVICE_CONTROL_H

#include <Arduino.h>

void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void reboot();
void processButtonEvents();
void flashLed(uint8_t color, uint8_t brightness);
void processLedFlash();
void initButtons();
void handleLedActivate(uint8_t* data, uint16_t len);
void handleLedStop(uint8_t* data, uint16_t len);
void enterDFUMode();
void handleDeepSleepCommand(const uint8_t* payload, uint16_t payloadLen);

#endif
