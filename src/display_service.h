#ifndef DISPLAY_SERVICE_H
#define DISPLAY_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

uint32_t max_compressed_image_rx_bytes(uint8_t transmission_modes);

bool seeed_driver_used(void);
int mapEpd(int id);
bool waitforrefresh(int timeout);
float readBatteryVoltage();
float readChipTemperature();
void updatemsdata();
void initio();
void initDataBuses();
/** Re-apply I2C from data_bus[0] when set; else Wire.begin(). Call before TCON/touch on shared bus. */
void initOrRestoreWireForOpenDisplay(void);
void scanI2CDevices();
void initSensors();
void initAXP2101(uint8_t busId);
void readAXP2101Data();
void powerDownAXP2101();
void initDisplay();
void writeTextAndFill(const char* text);
void handleDirectWriteStart(uint8_t* data, uint16_t len);
void handleDirectWriteData(uint8_t* data, uint16_t len);
void handleDirectWriteCompressedData(uint8_t* data, uint16_t len);
void decompressDirectWriteData();
void cleanupDirectWriteState(bool refreshDisplay);
void handleDirectWriteEnd(uint8_t* data, uint16_t len);
void handlePartialWriteStart(uint8_t* data, uint16_t len);
void handlePartialWriteData(uint8_t* data, uint16_t len);
int getplane();
int getBitsPerPixel();

#endif
