#ifndef BATTERY_LOW_H
#define BATTERY_LOW_H

#include <stdint.h>

// Deep-sleep duration used whenever a low-battery trip puts the device to
// sleep with no charger-wake GPIO to rely on. This is the only chance to
// notice a charger plug-in on a board without one (e.g. E1003), so it
// trades a small amount of extra wake-check energy for faster recovery.
constexpr uint16_t kLowBatterySleepSeconds = 1800;

bool isBatteryLow(void);
bool batteryLowStateActive(void);
void batteryLowPoll(void);
bool lowBatteryBootCheck(void);
bool batteryLowLoopCheck(void);

#endif
