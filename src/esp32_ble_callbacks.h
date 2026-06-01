#ifndef ESP32_BLE_CALLBACKS_H
#define ESP32_BLE_CALLBACKS_H

#ifdef TARGET_ESP32

#include <Arduino.h>
#include <BLECharacteristic.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <string.h>
#include "ble_init.h"

#ifndef COMMAND_QUEUE_SIZE
#define COMMAND_QUEUE_SIZE 5
#endif
#ifndef MAX_COMMAND_SIZE
#define MAX_COMMAND_SIZE 512
#endif

struct CommandQueueItem {
    uint8_t data[MAX_COMMAND_SIZE];
    uint16_t len;
    bool pending;
};

extern CommandQueueItem commandQueue[COMMAND_QUEUE_SIZE];
extern uint8_t commandQueueHead;
extern uint8_t commandQueueTail;
extern uint8_t rebootFlag;
extern volatile bool esp32BleNotifySubscribed;

void updatemsdata();
void cleanupDirectWriteState(bool refreshDisplay);
void touchResumeAfterEpdRefresh(void);
extern volatile bool epdRefreshInProgress;
extern bool directWriteActive;

class MyBLEServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        (void)pServer;
        writeSerial("=== BLE CLIENT CONNECTED (ESP32) ===");
        rebootFlag = 0;
        esp32BleNotifySubscribed = false;
        updatemsdata();
    }
    void onDisconnect(BLEServer* pServer) {
        (void)pServer;
        writeSerial("=== BLE CLIENT DISCONNECTED (ESP32) ===");
        esp32BleNotifySubscribed = false;
        if (epdRefreshInProgress) {
            writeSerial("EPD refresh in progress — deferring cleanup/advertising to main loop");
        } else if (directWriteActive) {
            cleanupDirectWriteState(true);
            touchResumeAfterEpdRefresh();
        }
        bleRestartAdvertisingPending = true;
    }
};

class MyBLECharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
#if defined(CONFIG_NIMBLE_ENABLED)
    void onSubscribe(BLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
        (void)pCharacteristic;
        (void)desc;
        esp32BleNotifySubscribed = (subValue & 0x0001) != 0;
        writeSerial("BLE notify subscription: " + String(esp32BleNotifySubscribed ? "enabled" : "disabled"));
    }
#endif
    void onWrite(BLECharacteristic* pCharacteristic) {
        writeSerial("=== BLE WRITE RECEIVED (ESP32) ===");
        String value = pCharacteristic->getValue();
        writeSerial("Received data length: " + String(value.length()) + " bytes");
        if (value.length() > 0 && value.length() <= MAX_COMMAND_SIZE) {
            uint8_t* data = (uint8_t*)value.c_str();
            uint16_t len = value.length();
            String hexDump = "Data: ";
            for (int i = 0; i < len && i < 16; i++) {
                if (data[i] < 16) hexDump += "0";
                hexDump += String(data[i], HEX) + " ";
            }
            writeSerial(hexDump);
            uint8_t nextHead = (commandQueueHead + 1) % COMMAND_QUEUE_SIZE;
            if (nextHead != commandQueueTail) {
                memcpy(commandQueue[commandQueueHead].data, data, len);
                commandQueue[commandQueueHead].len = len;
                commandQueue[commandQueueHead].pending = true;
                commandQueueHead = nextHead;
                writeSerial("ESP32: Command queued for processing");
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
