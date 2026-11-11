#ifndef SENSOR_BQ27220_H
#define SENSOR_BQ27220_H

#include <stdint.h>

void initBq27220Sensors(void);
void pollBq27220ForMsd(void);
void initChargerGpio(void);
bool bq27220IsConfigured(void);
float bq27220BatteryVoltageVolts(void);
uint8_t bq27220Soc(void);
bool bq27220GaugeHealthy(void);
bool chargerIsCharging(void);

#endif
