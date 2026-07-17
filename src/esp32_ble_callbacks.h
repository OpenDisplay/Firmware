#ifndef ESP32_BLE_CALLBACKS_H
#define ESP32_BLE_CALLBACKS_H

#ifdef TARGET_ESP32

#include <Arduino.h>
#include <string.h>
#include "ble_init.h"   // NimBLE-Arduino + BLE* aliases

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
        writeSerial("=== BLE CLIENT CONNECTED (ESP32) ===");
        rebootFlag = 0;
        esp32BleNotifySubscribed = false;
        updatemsdata();
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        (void)pServer;
        (void)connInfo;
        (void)reason;
        writeSerial("=== BLE CLIENT DISCONNECTED (ESP32) ===");
        esp32BleNotifySubscribed = false;
        if (epdRefreshInProgress) {
            writeSerial("EPD refresh in progress — deferring cleanup/advertising to main loop");
        } else {
            // ACTIVE-only-teardown invariant: a WARM (post-successful-refresh) panel
            // SURVIVES disconnect and keeps its keep-alive window, so the cleanups
            // below no-op on power when WARM and only tear down a mid-transfer
            // (PWR_ACTIVE) session. No logic change needed for keep-alive.
            if (directWriteActive) cleanupDirectWriteState(true);
            // Partial sessions (0x76 or pipe-partial) power the panel without
            // setting directWriteActive; release it here instead of waiting on
            // the 15-min partial watchdog.
            cleanupPartialWriteOnDisconnect();
        }
        resetPipeWriteState();   // clear any pipe transfer + reorder queue on disconnect
        bleRestartAdvertisingPending = true;
    }
};

class MyBLECharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
        (void)pCharacteristic;
        (void)connInfo;
        esp32BleNotifySubscribed = (subValue & 0x0001) != 0;
        writeSerial("BLE notify subscription: " + String(esp32BleNotifySubscribed ? "enabled" : "disabled"));
    }
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        (void)connInfo;
        // Keep the raw NimBLEAttValue: converting to Arduino String uses the C-string
        // (strlen) constructor, which truncates at the first 0x00 byte. Pipe-write
        // frames start with 0x00 (00 70 / 00 71 / 00 81), so String() would report
        // length 0. .length()/.c_str() on NimBLEAttValue preserve the binary payload.
        NimBLEAttValue value = pCharacteristic->getValue();
        const bool quiet = imageWriteLogQuietFrame((const uint8_t*)value.c_str(), value.length());
        if (!quiet) {
            writeSerial("=== BLE WRITE RECEIVED (ESP32) ===");
            writeSerial("Received data length: " + String(value.length()) + " bytes");
        }
        if (value.length() > 0 && value.length() <= MAX_COMMAND_SIZE) {
            uint8_t* data = (uint8_t*)value.c_str();
            uint16_t len = value.length();
            if (!quiet) {
                String hexDump = "Data: ";
                for (int i = 0; i < len && i < 16; i++) {
                    if (data[i] < 16) hexDump += "0";
                    hexDump += String(data[i], HEX) + " ";
                }
                writeSerial(hexDump);
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
                if (!quiet) writeSerial("ESP32: Command queued for processing");
            } else {
                writeSerial("ERROR: Command queue full, dropping command");
            }
        } else if (value.length() > MAX_COMMAND_SIZE) {
            writeSerial("WARNING: Command too large, dropping");
        } else {
            writeSerial("WARNING: Empty data received");
        }
    }
};

#endif

#endif
