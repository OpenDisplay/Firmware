#ifndef BATTERY_LOW_H
#define BATTERY_LOW_H

bool isBatteryLow(void);
bool batteryLowStateActive(void);
void batteryLowPoll(void);
bool lowBatteryBootCheck(void);

#endif
