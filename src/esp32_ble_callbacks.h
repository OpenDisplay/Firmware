#ifndef ESP32_BLE_CALLBACKS_H
#define ESP32_BLE_CALLBACKS_H

#ifdef TARGET_ESP32

#include <Arduino.h>
#include <string.h>
#include "ble_init.h"   // NimBLE-Arduino + BLE* aliases
#include "od_log.h"

// Defined in display_service.cpp. True for a mid-stream image-write data frame
// (0x0071) whose per-frame receive/queue logging should be suppressed.
bool imageWriteLogQuietFrame(const uint8_t* data, uint16_t len);

// Kept in sync with main.h (included first, so its values win). PIPE_WRITE ingest:
// 33 slots hold a full W=32 window + END across a 60 s Spectra SPI stall; 256 covers
// pipe <=244 / legacy <=232 / HA <=244.
#ifndef COMMAND_QUEUE_SIZE
#define COMMAND_QUEUE_SIZE 33
#endif
#ifndef MAX_COMMAND_SIZE
#define MAX_COMMAND_SIZE 256
#endif

struct CommandQueueItem {
    uint8_t data[MAX_COMMAND_SIZE];
    uint16_t len;
    bool pending;
};

extern CommandQueueItem commandQueue[COMMAND_QUEUE_SIZE];
extern volatile uint8_t commandQueueHead;
extern volatile uint8_t commandQueueTail;
extern uint8_t rebootFlag;
extern volatile bool esp32BleNotifySubscribed;

void updatemsdata();
void cleanupDirectWriteState(bool refreshDisplay);
void cleanupPartialWriteOnDisconnect(void);
void resetPipeWriteState(void);
void touchResumeAfterEpdRefresh(void);
extern volatile bool epdRefreshInProgress;
extern bool directWriteActive;

class MyBLEServerCallbacks : public BLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        (void)pServer;
        (void)connInfo;
        od_log_info("=== BLE CLIENT CONNECTED (ESP32) ===");
        rebootFlag = 0;
        esp32BleNotifySubscribed = false;
        // Flag-only: updatemsdata() polls I2C and mutates the shared advertisement
        // vector, which loop() also drives (60 s cadence) — running it inline on the
        // NimBLE host task would corrupt the heap. Serviced from loop() instead.
        msdUpdatePending = true;
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        (void)pServer;
        (void)connInfo;
        (void)reason;
        od_log_info("=== BLE CLIENT DISCONNECTED (ESP32) ===");
        esp32BleNotifySubscribed = false;
        // Flag-only: the session teardown below (EPD force-off with SPI.end()/rail
        // cut, partial + pipe cleanup) is heavyweight, state-mutating work that races
        // loop()'s SPI streaming and pipe-frame processing on the Arduino task. Defer
        // it — and the epdRefreshInProgress deferral — to loop() where it is
        // single-task-safe (see serviceBleDisconnectCleanup in main.cpp).
        bleDisconnectCleanupPending = true;
        bleRestartAdvertisingPending = true;
    }
};

class MyBLECharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
        (void)pCharacteristic;
        (void)connInfo;
        esp32BleNotifySubscribed = (subValue & 0x0001) != 0;
        od_log_info("BLE notify subscription: %s", esp32BleNotifySubscribed ? "enabled" : "disabled");
    }
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        (void)connInfo;
        // Keep the raw NimBLEAttValue: converting to Arduino String uses the C-string
        // (strlen) constructor, which truncates at the first 0x00 byte. Pipe-write
        // frames start with 0x00 (00 70 / 00 71 / 00 81), so String() would report
        // length 0. .length()/.c_str() on NimBLEAttValue preserve the binary payload.
        NimBLEAttValue value = pCharacteristic->getValue();
        const bool quiet = imageWriteLogQuietFrame((const uint8_t*)value.c_str(), value.length());
        if (value.length() > 0 && value.length() <= MAX_COMMAND_SIZE) {
            uint8_t* data = (uint8_t*)value.c_str();
            uint16_t len = value.length();
            if (!quiet) {
                // One-line RX log, mirroring the "[BLE] TX ..." response log. This
                // callback is the BLE write path only, so the tag is always [BLE];
                // LAN frames are identified by the dispatch banner in
                // imageDataWritten(), which reads g_commandOrigin.
                uint16_t cmd = (len >= 2) ? ((data[0] << 8) | data[1]) : data[0];
                char line[160] = {0};
                int pos = snprintf(line, sizeof(line), "BLE: RX 0x%04X (%u B):", cmd, (unsigned)len);
                if (pos < 0) {
                    pos = 0;
                    line[0] = '\0';
                }
                int dumpLen = (len < 32) ? len : 32;
                for (int i = 0; i < dumpLen && pos < (int)sizeof(line); i++) {
                    int n = snprintf(line + pos, sizeof(line) - pos, " %02X", data[i]);
                    if (n < 0) {
                        break;
                    }
                    pos += n;
                }
                if (len > 32 && pos >= 0 && pos < (int)sizeof(line)) {
                    snprintf(line + pos, sizeof(line) - pos, " ...");
                }
                od_log_debug("%s", line);
            }
            // SPSC ring: publish head with RELEASE after the payload is fully written
            // so the consumer (main loop) never observes a slot before its bytes land.
            uint8_t head = __atomic_load_n(&commandQueueHead, __ATOMIC_RELAXED);
            uint8_t tail = __atomic_load_n(&commandQueueTail, __ATOMIC_ACQUIRE);
            uint8_t nextHead = (head + 1) % COMMAND_QUEUE_SIZE;
            if (nextHead != tail) {
                memcpy(commandQueue[head].data, data, len);
                commandQueue[head].len = len;
                commandQueue[head].pending = true;
                __atomic_store_n(&commandQueueHead, nextHead, __ATOMIC_RELEASE);
            } else {
                od_log_error("ERROR: Command queue full, dropping command");
            }
        } else if (value.length() > MAX_COMMAND_SIZE) {
            od_log_warn("WARNING: Command too large, dropping");
        } else {
            od_log_warn("WARNING: Empty data received");
        }
    }
};

#endif

#endif
