#include "main.h"
#ifdef TARGET_ESP32
#include <esp_heap_caps.h>
#endif
#include "boot_screen.h"
#include "communication.h"
#include "device_control.h"
#include "display_service.h"
#include "touch_input.h"
#include "encryption.h"

#if defined(TARGET_ESP32) && defined(OPENDISPLAY_LOG_UART)
#include <HardwareSerial.h>
#ifndef OPENDISPLAY_LOG_UART_RX
#define OPENDISPLAY_LOG_UART_RX 44
#endif
#ifndef OPENDISPLAY_LOG_UART_TX
#define OPENDISPLAY_LOG_UART_TX 43
#endif
static HardwareSerial LogSerialPort(1);
#endif

#if defined(TARGET_NRF)
static uint8_t s_compressedDataStorage[MAX_COMPRESSED_BUFFER_BYTES];
uint8_t* compressedDataBuffer = s_compressedDataStorage;
#elif defined(TARGET_ESP32) && defined(TARGET_LARGE_MEMORY) && defined(BOARD_HAS_PSRAM)
uint8_t* compressedDataBuffer = nullptr;
#else
static uint8_t s_compressedDataStorage[MAX_COMPRESSED_BUFFER_BYTES];
uint8_t* compressedDataBuffer = s_compressedDataStorage;
#endif

void allocCompressedDataBuffer(void) {
#if defined(TARGET_ESP32) && defined(TARGET_LARGE_MEMORY) && defined(BOARD_HAS_PSRAM)
    if (compressedDataBuffer) return;
    compressedDataBuffer = (uint8_t*)heap_caps_malloc(MAX_COMPRESSED_BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!compressedDataBuffer) {
        compressedDataBuffer = (uint8_t*)heap_caps_malloc(MAX_COMPRESSED_BUFFER_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
#endif
}

void setup() {
    #if defined(TARGET_ESP32) && defined(OPENDISPLAY_LOG_UART)
    LogSerialPort.begin(115200, SERIAL_8N1, OPENDISPLAY_LOG_UART_RX, OPENDISPLAY_LOG_UART_TX);
    delay(100);
    #elif !defined(DISABLE_USB_SERIAL)
    Serial.begin(115200);
    delay(100);
    #endif
    allocCompressedDataBuffer();
    writeSerial("=== FIRMWARE INFO ===");
    writeSerial("Firmware Version: " + String(getFirmwareMajor()) + "." + String(getFirmwareMinor()));
    const char* shaCStr = SHA_STRING;
    String shaStr = String(shaCStr);
    if (shaStr.length() >= 2 && shaStr.charAt(0) == '"' && shaStr.charAt(shaStr.length() - 1) == '"') {
        shaStr = shaStr.substring(1, shaStr.length() - 1);
    }
    if (shaStr.length() > 0 && shaStr != "\"\"" && shaStr != "") {
        writeSerial("Git SHA: " + shaStr);
    } else {
        writeSerial("Git SHA: (not set)");
    }
    #ifdef TARGET_ESP32
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool is_deep_sleep_wake = (wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED);
    if (is_deep_sleep_wake) {
        woke_from_deep_sleep = true;
        deep_sleep_count++;
        writeSerial("=== WOKE FROM DEEP SLEEP ===");
        writeSerial("Wake-up reason: " + String(wakeup_reason));
        writeSerial("Deep sleep count: " + String(deep_sleep_count));
        minimalSetup();
        return;
    } else {
        woke_from_deep_sleep = false;
        writeSerial("=== NORMAL BOOT ===");
    }
    #endif
    writeSerial("Starting setup...");
    full_config_init();
    initio();
#ifdef TARGET_NRF
    // SoftDevice must start before display/SPI; advertising starts after boot screen.
    ble_nrf_stack_init();
#endif
    initDisplay();
    writeSerial("Display initialized");
#ifdef TARGET_ESP32
    // Full BLE after display: ESP32 queues commands for loop() until setup returns.
    ble_init();
#elif defined(TARGET_NRF)
    ble_nrf_advertising_start();
#endif
    #ifdef TARGET_ESP32
    initWiFi();
    #endif
    updatemsdata();
    initButtons();
    initTouchInput();
    writeSerial("=== Setup completed successfully ===");
}

void loop() {
    #ifdef TARGET_ESP32
    handleWiFiServer();
    static uint32_t lastWiFiCheck = 0;
    if (wifiInitialized && (millis() - lastWiFiCheck > 10000)) {
        lastWiFiCheck = millis();
        if (WiFi.status() != WL_CONNECTED && wifiConnected) {
            writeSerial("WiFi connection lost (status: " + String(WiFi.status()) + ")");
            wifiConnected = false;
            if (wifiServerConnected) {
                disconnectWiFiServer();
            }
        } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
            writeSerial("WiFi reconnected (IP: " + WiFi.localIP().toString() + ")");
            wifiConnected = true;
            restartWiFiLanAfterReconnect();
        }
    }
    if (woke_from_deep_sleep && advertising_timeout_active) {
        if (pServer && pServer->getConnectedCount() > 0) {
            writeSerial("BLE connection established - switching to full mode");
            advertising_timeout_active = false;
            fullSetupAfterConnection();
            woke_from_deep_sleep = false;
            return;
        }
        uint32_t advertising_duration = millis() - advertising_start_time;
        uint32_t advertising_timeout_ms = globalConfig.power_option.sleep_timeout_ms;
        if (advertising_timeout_ms == 0) {
            advertising_timeout_ms = 10000;
        }
        if (advertising_duration >= advertising_timeout_ms) {
            writeSerial("BLE advertising timeout (" + String(advertising_duration) + " ms) - no connection, returning to deep sleep");
            advertising_timeout_active = false;
            enterDeepSleep();
            return;
        }
        return;
    }
    if (commandQueueTail != commandQueueHead) {
        writeSerial("ESP32: Processing queued command (" + String(commandQueue[commandQueueTail].len) + " bytes)");
        imageDataWritten(NULL, NULL, commandQueue[commandQueueTail].data, commandQueue[commandQueueTail].len);
        commandQueue[commandQueueTail].pending = false;
        commandQueueTail = (commandQueueTail + 1) % COMMAND_QUEUE_SIZE;
        writeSerial("Command processed");
    }
    if (pTxCharacteristic && pServer && pServer->getConnectedCount() > 0) {
        uint8_t bleDrain = 0;
        while (responseQueueTail != responseQueueHead && bleDrain < 16) {
            writeSerial("ESP32: Sending queued response (" + String(responseQueue[responseQueueTail].len) + " bytes)");
            pTxCharacteristic->setValue(responseQueue[responseQueueTail].data, responseQueue[responseQueueTail].len);
            pTxCharacteristic->notify();
            responseQueue[responseQueueTail].pending = false;
            responseQueueTail = (responseQueueTail + 1) % RESPONSE_QUEUE_SIZE;
            writeSerial("Response sent successfully");
            bleDrain++;
        }
    }
    if (directWriteActive && directWriteStartTime > 0) {
        uint32_t directWriteDuration = millis() - directWriteStartTime;
        if (directWriteDuration > 900000UL) {  // 15 minute timeout (upload + refresh window)
            writeSerial("ERROR: Direct write timeout (" + String(directWriteDuration) + " ms) - cleaning up stuck state");
            cleanupDirectWriteState(true);
        }
    }
    #ifdef TARGET_ESP32
    const bool wifiLanSession = wifiInitialized && wifiServerConnected && wifiClient.connected();
    #else
    const bool wifiLanSession = false;
    #endif
    bool bleActive = (commandQueueTail != commandQueueHead) ||
                     (responseQueueTail != responseQueueHead) ||
                     (pServer && pServer->getConnectedCount() > 0) ||
                     wifiLanSession;
    if (bleActive) {
        processButtonEvents();
        processTouchInput();
        delay(1);
    } else {
        if (!woke_from_deep_sleep && deep_sleep_count == 0 && globalConfig.power_option.power_mode == 1) {
            if (!firstBootDelayInitialized) {
                firstBootDelayInitialized = true;
                firstBootDelayStart = millis();
                processButtonEvents();
                writeSerial("First boot: waiting 60s before entering deep sleep");
            }
            uint32_t elapsed = millis() - firstBootDelayStart;
            if (elapsed < 60000) {
                idleDelay(5);
                return;
            }
            writeSerial("First boot delay elapsed, deep sleep permitted");
        }
        if(globalConfig.power_option.deep_sleep_time_seconds > 0 && globalConfig.power_option.power_mode == 1){
            enterDeepSleep();
        }
        else{
            idleDelay(2000);
        }
        updatemsdata();
        processButtonEvents();
        processTouchInput();
        if(!bleActive)writeSerial("Loop end: " + String(millis() / 100));
    }
    #else
    if(globalConfig.power_option.sleep_timeout_ms > 0){
        idleDelay(globalConfig.power_option.sleep_timeout_ms);
        updatemsdata();
    }
    else{
        idleDelay(500);
    }
    processButtonEvents();
    processTouchInput();
    writeSerial("Loop end: " + String(millis() / 100));
    #endif
}

// Button/LED runtime moved to device_control.cpp

void idleDelay(uint32_t delayMs) {
    const uint32_t CHECK_INTERVAL_MS = 100;
    uint32_t remainingDelay = delayMs;
    while (remainingDelay > 0) {
        processButtonEvents();
        processTouchInput();
        uint32_t chunkDelay = (remainingDelay > CHECK_INTERVAL_MS) ? CHECK_INTERVAL_MS : remainingDelay;
        delay(chunkDelay);
        remainingDelay -= chunkDelay;
    }
}

#ifdef TARGET_NRF
uint8_t pinToButtonIndex[64] = {0xFF};  // Map pin number to button index (max 64 pins)
#endif

#ifdef TARGET_ESP32
void minimalSetup() {
    allocCompressedDataBuffer();
    writeSerial("=== Minimal Setup (Deep Sleep Wake) ===");
    full_config_init();
    initio();
    ble_init_esp32(true); // Update manufacturer data
    initWiFi(false);
    writeSerial("=== BLE advertising started (minimal mode) ===");
    writeSerial("Advertising for 10 seconds, waiting for connection...");
    advertising_timeout_active = true;
    advertising_start_time = millis();
}

void fullSetupAfterConnection() {
    writeSerial("=== Full Setup After Connection ===");
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (globalConfig.display_count > 0 && seeed_driver_used()) {
        writeSerial("Panel: Seeed ED103 (bb_epaper not used)", true);
        writeSerial("=== Full setup completed ===");
        return;
    }
#endif
    if (globalConfig.display_count > 0) {
        memset(&bbep, 0, sizeof(BBEPDISP));
        int panelType = mapEpd(globalConfig.displays[0].panel_ic_type);
        writeSerial("Panel type: " + String(panelType));
        bbepSetPanelType(&bbep, panelType);
    }
    writeSerial("=== Full setup completed ===");
}

void enterDeepSleep() {
    if (globalConfig.power_option.power_mode != 1) {
        writeSerial("Skipping deep sleep - not battery powered (power_mode: " + String(globalConfig.power_option.power_mode) + ")");
        delay(2000);
        return;
    }
    if (globalConfig.power_option.deep_sleep_time_seconds == 0) {
        writeSerial("Skipping deep sleep - deep_sleep_time_seconds is 0");
        delay(2000);
        return;
    }
    writeSerial("Entering deep sleep for " + String(globalConfig.power_option.deep_sleep_time_seconds) + " seconds");
    woke_from_deep_sleep = true; // Will be true on next boot
    if (pServer != nullptr) {
        BLEAdvertising *pAdvertising = pServer->getAdvertising();
        if (pAdvertising != nullptr) {
            pAdvertising->stop();
            writeSerial("BLE advertising stopped");
        }
    }
    BLEDevice::deinit(true);
    writeSerial("BLE deinitialized");
    uint64_t sleep_timeout_us = (uint64_t)globalConfig.power_option.deep_sleep_time_seconds * 1000000ULL;
    esp_sleep_enable_timer_wakeup(sleep_timeout_us);
    writeSerial("Entering deep sleep...");
    delay(100); // Brief delay to ensure serial output is sent
    esp_deep_sleep_start();
}
#endif

void pwrmgm(bool onoff){
    if(globalConfig.display_count == 0){
        writeSerial("No display configured");
        return;
    }
    displayPowerState = onoff;
    uint8_t axp2101_bus_id = 0xFF;
    bool axp2101_found = false;
    for(uint8_t i = 0; i < globalConfig.sensor_count; i++){
        if(globalConfig.sensors[i].sensor_type == SENSOR_TYPE_AXP2101){
            axp2101_bus_id = globalConfig.sensors[i].bus_id;
            axp2101_found = true;
            break;
        }
    }
    if(axp2101_found){
        if(onoff){
        writeSerial("Powering up AXP2101 PMIC...");
            initAXP2101(axp2101_bus_id);
        }
        else{
            writeSerial("Powering down AXP2101 PMIC...");
            powerDownAXP2101();
            Wire.end();
            pinMode(47, OUTPUT);
            digitalWrite(47, HIGH);
            pinMode(48, OUTPUT);
            digitalWrite(48, HIGH);
        }
    }
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    const bool seeed_driver_spi = seeed_driver_used();
#else
    const bool seeed_driver_spi = false;
#endif
    if (!seeed_driver_spi) {
        if(onoff){
            pinMode(globalConfig.displays[0].reset_pin, OUTPUT);
            pinMode(globalConfig.displays[0].cs_pin, OUTPUT);
            if (globalConfig.displays[0].dc_pin != 0xFF) {
                pinMode(globalConfig.displays[0].dc_pin, OUTPUT);
            }
            pinMode(globalConfig.displays[0].clk_pin, OUTPUT);
            pinMode(globalConfig.displays[0].data_pin, OUTPUT);
            delay(200);
        }
        else{
            SPI.end();
            Wire.end();
            pinMode(globalConfig.displays[0].reset_pin, INPUT);
            pinMode(globalConfig.displays[0].cs_pin, INPUT);
            if (globalConfig.displays[0].dc_pin != 0xFF) {
                pinMode(globalConfig.displays[0].dc_pin, INPUT);
            }
            pinMode(globalConfig.displays[0].clk_pin, INPUT);
            pinMode(globalConfig.displays[0].data_pin, INPUT);
        }
    } else {
        if (onoff) {
            delay(200);
        } else {
            SPI.end();
            Wire.end();
        }
    }
    if(globalConfig.system_config.pwr_pin != 0xFF){
    if(onoff){
        digitalWrite(globalConfig.system_config.pwr_pin, HIGH);
        delay(200);
    }
    else{
        digitalWrite(globalConfig.system_config.pwr_pin, LOW);
    }
    }
    else{
        writeSerial("Power pin not set");
       }
}

void writeSerial(String message, bool newLine){
    #if defined(TARGET_ESP32) && defined(OPENDISPLAY_LOG_UART)
    if (newLine) LogSerialPort.println(message);
    else LogSerialPort.print(message);
    #elif !defined(DISABLE_USB_SERIAL)
    if (newLine == true) Serial.println(message);
    else Serial.print(message);
    #endif
}

void xiaoinit(){
    powerDownExternalFlash(20,24,21,25,22,23);
    pinMode(31, INPUT);
    pinMode(14, INPUT);
    pinMode(13, OUTPUT);  //that actually does something
    digitalWrite(13, LOW);
    pinMode(17, INPUT);
    //buttons
    pinMode(15, INPUT);
    pinMode(3, INPUT);
    pinMode(28, INPUT);
}

void ws_pp_init(){
    writeSerial("===  Photo Printer Initialization ===");
    pinMode(21, OUTPUT);
    digitalWrite(21, HIGH);
    pinMode(1, INPUT);
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);
    pinMode(6, INPUT);
    pinMode(7, LOW);
    digitalWrite(7, LOW);
    pinMode(14, INPUT);
    pinMode(15, INPUT);
    pinMode(16, INPUT);
    pinMode(17, INPUT);
    pinMode(18, INPUT);
    pinMode(38, OUTPUT);
    digitalWrite(38, HIGH);
    pinMode(39, OUTPUT);
    digitalWrite(39, HIGH);
    pinMode(40, OUTPUT);
    digitalWrite(40, HIGH);
    pinMode(41, OUTPUT);
    digitalWrite(41, HIGH);
    pinMode(42, OUTPUT);
    digitalWrite(42, HIGH);
    pinMode(45, OUTPUT);
    digitalWrite(45, HIGH);
    writeSerial("Photo Printer initialized");
}

bool powerDownExternalFlash(uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin, uint8_t csPin, uint8_t wpPin, uint8_t holdPin) {
    #ifdef TARGET_NRF
    auto spiTransfer = [&](uint8_t data) -> uint8_t {
        uint8_t result = 0;
        for (int i = 7; i >= 0; i--) {
            digitalWrite(mosiPin, (data >> i) & 1);
            digitalWrite(sckPin, LOW);
            delayMicroseconds(1);
            result |= (digitalRead(misoPin) << i);
            digitalWrite(sckPin, HIGH);
            delayMicroseconds(1);
        }
        return result;
    };
    writeSerial("=== External Flash Power-Down ===");
    writeSerial("Pin configuration: MOSI=" + String(mosiPin) + " MISO=" + String(misoPin) + " SCK=" + String(sckPin) + " CS=" + String(csPin) + " WP=" + String(wpPin) + " HOLD=" + String(holdPin));
    writeSerial("Configuring SPI pins...");
    pinMode(mosiPin, OUTPUT);
    pinMode(misoPin, INPUT);
    pinMode(sckPin, OUTPUT);
    pinMode(csPin, OUTPUT);
    pinMode(wpPin, OUTPUT);
    pinMode(holdPin, OUTPUT);
    writeSerial("SPI pins configured");
    digitalWrite(sckPin, HIGH);  // Clock idle high (SPI mode 0)
    digitalWrite(csPin, HIGH);   // CS inactive
    digitalWrite(wpPin, HIGH);   // WP disabled (active-low)
    digitalWrite(holdPin, HIGH); // HOLD disabled (active-low)
    writeSerial("Control pins set: CS=HIGH, WP=HIGH (disabled), HOLD=HIGH (disabled), SCK=HIGH (idle)");
    delay(1);
    writeSerial("Attempting to wake flash from deep power-down (command 0xAB)...");
    digitalWrite(csPin, LOW);
    spiTransfer(0xAB);
    digitalWrite(csPin, HIGH);
    delay(10); // Wait for flash to wake up (typically 3-35us, using 10ms for safety)
    writeSerial("Wake-up command sent, waiting 10ms...");
    writeSerial("Reading JEDEC ID before power-down...");
    digitalWrite(csPin, LOW);
    spiTransfer(0x9F); // JEDEC ID command
    uint8_t jedecId[3];
    for (int i = 0; i < 3; i++) {
        jedecId[i] = spiTransfer(0x00);
    }
    digitalWrite(csPin, HIGH);
    String jedecIdStr = "0x";
    for (int i = 0; i < 3; i++) {
        if (jedecId[i] < 16) jedecIdStr += "0";
        jedecIdStr += String(jedecId[i], HEX);
    }
    jedecIdStr.toUpperCase();
    writeSerial("JEDEC ID before: " + jedecIdStr + " (Manufacturer=0x" + String(jedecId[0], HEX) + ", MemoryType=0x" + String(jedecId[1], HEX) + ", Capacity=0x" + String(jedecId[2], HEX) + ")");
    delay(1);
    writeSerial("Sending deep power-down command (0xB9)...");
    digitalWrite(csPin, LOW);
    spiTransfer(0xB9);
    digitalWrite(csPin, HIGH);
    if(false){
    writeSerial("Deep power-down command sent, waiting 10ms...");
    delay(10); // Wait for command to complete
    writeSerial("Reading JEDEC ID after power-down command...");
    digitalWrite(csPin, LOW);
    spiTransfer(0x9F);
    uint8_t jedecIdAfter[3];
    for (int i = 0; i < 3; i++) {
        jedecIdAfter[i] = spiTransfer(0x00);
    }
    digitalWrite(csPin, HIGH);
    String jedecIdAfterStr = "0x";
    for (int i = 0; i < 3; i++) {
        if (jedecIdAfter[i] < 16) jedecIdAfterStr += "0";
        jedecIdAfterStr += String(jedecIdAfter[i], HEX);
    }
    jedecIdAfterStr.toUpperCase();
    writeSerial("JEDEC ID after: " + jedecIdAfterStr + " (byte[0]=0x" + String(jedecIdAfter[0], HEX) + ", byte[1]=0x" + String(jedecIdAfter[1], HEX) + ", byte[2]=0x" + String(jedecIdAfter[2], HEX) + ")");
    }
    digitalWrite(csPin, HIGH);
    pinMode(wpPin, INPUT);
    pinMode(holdPin, INPUT);
    pinMode(mosiPin, INPUT);
    pinMode(misoPin, INPUT);
    pinMode(sckPin, INPUT);    
    #else
    writeSerial("External flash power-down not implemented for ESP32");
    return false;
    #endif
    return false;
}