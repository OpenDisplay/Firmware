#ifndef SENSOR_BQ27220_H
#define SENSOR_BQ27220_H

void initBq27220Sensors(void);
void pollBq27220ForMsd(void);
void initChargerGpio(void);
bool bq27220IsConfigured(void);
float bq27220BatteryVoltageVolts(void);

#endif
