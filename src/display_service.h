#ifndef DISPLAY_SERVICE_H
#define DISPLAY_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

#define OPENDISPLAY_DECOMPRESSION_CHUNK_SIZE 256

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
/** Call after Wire.end() so the next touch/sensor access re-inits the bus. */
void invalidateOpenDisplayWire(void);
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
void cleanupDirectWriteState(bool refreshDisplay);
void handleDirectWriteEnd(uint8_t* data, uint16_t len);
extern volatile bool epdRefreshInProgress;
void handlePartialWriteStart(uint8_t* data, uint16_t len);
int getplane();
int getBitsPerPixel();

#endif
