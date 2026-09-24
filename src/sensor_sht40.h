#ifndef SENSOR_SHT40_H
#define SENSOR_SHT40_H

#include <stdint.h>

void initSht40Sensors(void);
void pollSht40SensorsForMsd(void);
void writeTempHumidityMsdInvalid(uint8_t start);
void writeTempHumidityMsd(uint8_t start, int16_t temp_centi, uint16_t rh_centi);

#endif
