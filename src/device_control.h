#ifndef DEVICE_CONTROL_H
#define DEVICE_CONTROL_H

#include <Arduino.h>

void reboot();
void processButtonEvents();
void buttonPressFeedback(void);
#if defined(TARGET_ESP32)
void buttonWakeDeliverSyntheticClick(void);
#endif
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
#ifdef TARGET_ESP32
// force: sleep even with a client connected (explicit host request 0x0053).
// overrideSleepSeconds: nonzero replaces deep_sleep_time_seconds for this one
// sleep cycle (0x0053 duration payload); never changes sleep eligibility.
// lowBattery: low-battery callers only; skips the deep_sleep_time_seconds == 0
// guard and arms the charge-pin wake source (see armButtonWakeSources()).
void enterDeepSleep(bool force = false, uint16_t overrideSleepSeconds = 0, bool lowBattery = false);
#endif

#endif
