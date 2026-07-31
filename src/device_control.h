#ifndef DEVICE_CONTROL_H
#define DEVICE_CONTROL_H

#include <Arduino.h>

void reboot();
void processButtonEvents();
void flashLed(uint8_t color, uint8_t brightness);
void processLedFlash();
void initButtons();
void handleLedActivate(uint8_t* data, uint16_t len);
void handleLedStop(uint8_t* data, uint16_t len);
/**
 * Stop LED playback immediately. DEEP SLEEP ONLY -- see buzzerStopForSleep() for
 * why this must not be called from abortToKnownState().
 */
void ledStopForSleep(void);
void enterDFUMode();
void handleDeepSleepCommand(const uint8_t* payload, uint16_t payloadLen);
void handlePowerOffCommand(const uint8_t* payload, uint16_t payloadLen);

#endif
