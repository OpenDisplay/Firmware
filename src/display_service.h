#ifndef DISPLAY_SERVICE_H
#define DISPLAY_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

#define OPENDISPLAY_DECOMPRESSION_CHUNK_SIZE 2048

// EPD panel power state machine states. The enum type + keep-alive constant live
// in this shared header (included by both main.cpp via main.h and by
// display_service.cpp) so the type is visible to every TU. The single
// source-of-truth VARIABLES (pwrmgmState / pwrmgmOffDeadlineMs / pwrmgmLock) are
// DEFINED in main.h next to displayPowerState and externed in display_service.cpp.
// PWR_OFF must be 0: BSS-zero after boot / ESP32 deep-sleep wake == rail off.
enum PwrMgmState : uint8_t { PWR_OFF = 0, PWR_WARM = 1, PWR_ACTIVE = 2 };
#define EPD_KEEPALIVE_MAX_S 30   // hard cap on power_option.screen_timeout_seconds (clamped, not rejected)

// EPD panel power session (keep-alive) cross-TU API. Acquire/Release are
// file-static in display_service.cpp (they own the ACTIVE<->WARM transitions and
// need panel-init knowledge); these are the public entry points.
void epdSessionForceOff(void);   // power the panel fully down now (idempotent)
void epdSessionTick(void);       // millis()-poll from loop()/idleDelay(): expire keep-alive
bool epdSessionIsWarm(void);     // true when the panel is powered-idle (PWR_WARM)

bool seeed_driver_used(void);
/** True when displays[0] is the E1004 13.3" T133A01 dual-controller panel. */
bool e1004_panel_used(void);
uint8_t e1004_cs2_pin(void);
int mapEpd(int id);
bool waitforrefresh(int timeout);
float readBatteryVoltage();
float readChipTemperature();
void updatemsdata();
void initio();
void initDataBuses();
/** True when data_bus[0] is a configured I2C bus (pin_1/pin_2 not 0xFF). */
bool openDisplayI2cBusConfigured(void);
/** Re-apply I2C from data_bus[0] when set; else Wire.begin(). Call before TCON/touch on shared bus. */
void initOrRestoreWireForOpenDisplay(void);
/** Select data_buses[bus_id] on Wire (bus_id 0xFF → 0). Switches pins when multiple I2C buses are configured. */
bool initOrRestoreWireForBus(uint8_t bus_id);
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
// Consumes one direct-write compressed payload into the panel controller. Returns
// false on overflow/decompress failure; the CALLER owns ACK/NACK emission.
bool handleDirectWriteCompressedData(uint8_t* data, uint16_t len);
void cleanupDirectWriteState(bool refreshDisplay);
// PIPE_WRITE (0x0080-0x0082) sliding-window handlers + state reset.
void handlePipeWriteStart(uint8_t* data, uint16_t len);
void handlePipeWriteData(uint8_t* data, uint16_t len);
void handlePipeWriteEnd(uint8_t* data, uint16_t len);
void resetPipeWriteState(void);
// True while a PIPE_WRITE stream is active (mid-transfer log suppression, resets).
bool pipeWriteActive(void);
void handleDirectWriteEnd(uint8_t* data, uint16_t len);
// True while an image push is mid-stream and the per-frame command/ack logging
// should be suppressed (chunk 1 still logs in full; the meter covers the rest).
bool imageWriteLogQuietCmd(void);
bool imageWriteLogQuietAck(void);
bool imageWriteLogQuietFrame(const uint8_t* data, uint16_t len);
extern volatile bool epdRefreshInProgress;
void handlePartialWriteStart(uint8_t* data, uint16_t len);
void checkPartialWriteTimeout(void);
void cleanupPartialWriteOnDisconnect(void);
int getplane();
int getBitsPerPixel();

#endif
