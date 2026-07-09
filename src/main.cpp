#include "main.h"
#include "boot_screen.h"
#include "communication.h"
#include "device_control.h"
#include "display_service.h"
#include "power_latch.h"
#include "touch_input.h"
#include "encryption.h"
#include "ble_init.h"

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

#ifdef TARGET_ESP32
// Distinguishes a hidden mid-cycle reset (PANIC/WDT/BROWNOUT/SW) from a real
// power-on or deep-sleep wake; any reset here clears the wake cause, so the
// next boot takes the NORMAL BOOT branch and redraws the boot screen.
static const char* resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}
#endif

void setup() {
    #if defined(TARGET_ESP32) && defined(OPENDISPLAY_LOG_UART)
    LogSerialPort.begin(115200, SERIAL_8N1, OPENDISPLAY_LOG_UART_RX, OPENDISPLAY_LOG_UART_TX);
    delay(100);
    #elif !defined(DISABLE_USB_SERIAL)
    Serial.begin(115200);
    delay(100);
    #endif
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
    // Set only by the ESP32 wake-cause check below; NRF has no deep-sleep wake path.
    bool is_deep_sleep_wake = false;
    #ifdef TARGET_ESP32
    esp_reset_reason_t reset_reason = esp_reset_reason();
    writeSerial("Reset reason: " + String(resetReasonName(reset_reason)) + " (" + String((int)reset_reason) + ")");
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    is_deep_sleep_wake = (wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED);
    if (is_deep_sleep_wake) {
        woke_from_deep_sleep = true;
        deep_sleep_count++;
        writeSerial("=== WOKE FROM DEEP SLEEP ===");
        writeSerial("Wake-up reason: " + String(wakeup_reason));
        writeSerial("Deep sleep count: " + String(deep_sleep_count));
    } else {
        woke_from_deep_sleep = false;
        writeSerial("=== NORMAL BOOT ===");
        // RTC memory survives soft resets (panic/WDT/esp_restart) but not power
        // loss: a non-zero count on a NORMAL BOOT means a hidden mid-cycle reset.
        writeSerial("Deep sleep count (RTC): " + String(deep_sleep_count));
    }
    #endif
    writeSerial("Starting setup...");
    if (is_deep_sleep_wake) { writeSerial("[wake] >> full_config_init"); flushLog(); }
    full_config_init();
    if (is_deep_sleep_wake) { writeSerial("[wake] << full_config_init >> initio"); flushLog(); }
    initio();
#ifdef TARGET_NRF
    // SoftDevice must start before display/SPI; advertising starts after boot screen.
    ble_nrf_stack_init();
#endif
    if (!is_deep_sleep_wake) {
        // Wake keeps the panel image; skipping initDisplay() (EPD rail power +
        // full refresh) is the wake path's main energy saving.
        initDisplay();
        writeSerial("Display initialized");
    }
#ifdef TARGET_ESP32
    // Full BLE after display: ESP32 queues commands for loop() until setup returns.
    if (is_deep_sleep_wake) { writeSerial("[wake] >> ble_init"); flushLog(); }
    ble_init();
    if (is_deep_sleep_wake) { writeSerial("[wake] << ble_init"); flushLog(); }
#elif defined(TARGET_NRF)
    ble_nrf_advertising_start();
#endif
    #ifdef TARGET_ESP32
    if (!is_deep_sleep_wake) {
        initWiFi(false);  // wake: WiFi stays deferred to fullSetupAfterConnection()
    }
    #endif
    updatemsdata();
    if (is_deep_sleep_wake) { writeSerial("[wake] >> initButtons"); flushLog(); }
    initButtons();
    if (is_deep_sleep_wake) { writeSerial("[wake] >> initTouchInput"); flushLog(); }
    initTouchInput();
    #ifdef TARGET_ESP32
    if (is_deep_sleep_wake) {
        // Arm the awake window LAST so buttons/GT911 bring-up doesn't shrink the
        // host's connection window. Without this, loop() falls into the idle
        // branch and re-enters deep sleep almost immediately.
        writeSerial("Advertising for " + String(globalConfig.power_option.sleep_timeout_ms) + " ms (sleep_timeout_ms), waiting for connection...");
        advertising_timeout_active = true;
        advertising_start_time = millis();
    }
    #endif
    writeSerial("=== Setup completed successfully ===");
}

void loop() {
    processLedFlash();
    #ifdef TARGET_ESP32
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
        delay(50);
        return;
    }
    if (commandQueueTail != commandQueueHead) {
        const bool quietCmd = imageWriteLogQuietFrame(commandQueue[commandQueueTail].data, commandQueue[commandQueueTail].len);
        if (!quietCmd) writeSerial("ESP32: Processing queued command (" + String(commandQueue[commandQueueTail].len) + " bytes)");
        imageDataWritten(NULL, NULL, commandQueue[commandQueueTail].data, commandQueue[commandQueueTail].len);
        commandQueue[commandQueueTail].pending = false;
        commandQueueTail = (commandQueueTail + 1) % COMMAND_QUEUE_SIZE;
        if (!quietCmd) writeSerial("Command processed");
    }
    if (responseQueueTail != responseQueueHead) {
        if (esp32_ble_notify_enabled()) {
            uint8_t bleDrain = 0;
            while (responseQueueTail != responseQueueHead && bleDrain < 16) {
                const bool quietAck = imageWriteLogQuietFrame(responseQueue[responseQueueTail].data, responseQueue[responseQueueTail].len);
                if (!quietAck) writeSerial("ESP32: Sending queued response (" + String(responseQueue[responseQueueTail].len) + " bytes)");
                pTxCharacteristic->setValue(responseQueue[responseQueueTail].data, responseQueue[responseQueueTail].len);
                pTxCharacteristic->notify();
                responseQueue[responseQueueTail].pending = false;
                responseQueueTail = (responseQueueTail + 1) % RESPONSE_QUEUE_SIZE;
                if (!quietAck) writeSerial("Response sent successfully");
                bleDrain++;
            }
        } else if (pServer && pServer->getConnectedCount() > 0) {
            // Connected but CCCD not enabled yet — keep responses queued
        } else {
            while (responseQueueTail != responseQueueHead) {
                responseQueue[responseQueueTail].pending = false;
                responseQueueTail = (responseQueueTail + 1) % RESPONSE_QUEUE_SIZE;
            }
        }
    }
    if (bleRestartAdvertisingPending) {
        esp32_restart_ble_advertising();
    }
    if (directWriteActive && directWriteStartTime > 0) {
        uint32_t directWriteDuration = millis() - directWriteStartTime;
        if (directWriteDuration > 900000UL) {  // 15 minute timeout (upload + refresh window)
            writeSerial("ERROR: Direct write timeout (" + String(directWriteDuration) + " ms) - cleaning up stuck state");
            cleanupDirectWriteState(true);
        }
    }
    checkPartialWriteTimeout();
    // WiFi handling runs after BLE queue processing to avoid blocking
    // BLE command responses (moved from top of loop in v1.6 fix).
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
    #ifdef TARGET_ESP32
    const bool wifiLanSession = wifiInitialized && wifiServerConnected && wifiClient.connected();
    #else
    const bool wifiLanSession = false;
    #endif
    bool bleActive = (commandQueueTail != commandQueueHead) ||
                     (responseQueueTail != responseQueueHead) ||
                     (pServer && pServer->getConnectedCount() > 0) ||
                     bleRestartAdvertisingPending ||
                     epdRefreshInProgress ||
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
                writeSerial("First boot: waiting 2 minutes before entering deep sleep");
            }
            uint32_t elapsed = millis() - firstBootDelayStart;
            if (elapsed < FIRST_BOOT_DEEP_SLEEP_DELAY_MS) {
                idleDelay(5);
                return;
            }
            if (!firstBootDelayElapsed) {
                firstBootDelayElapsed = true;
                writeSerial("First boot delay elapsed, deep sleep permitted");
            }
        }
        if(globalConfig.power_option.deep_sleep_time_seconds > 0 && globalConfig.power_option.power_mode == 1){
            enterDeepSleep();
        }
        else{
            idleDelay(2000);
        }
        static uint32_t lastMsdUpdate = 0;
        if (millis() - lastMsdUpdate >= 60000) {
            lastMsdUpdate = millis();
            updatemsdata();
        }
        processButtonEvents();
        processTouchInput();
    }
    #else
    if(globalConfig.power_option.sleep_timeout_ms > 0){
        idleDelay(globalConfig.power_option.sleep_timeout_ms);
        updatemsdata();
    }
    else{
        idleDelay(500);
    }
    ble_nrf_advertising_tick();
    processButtonEvents();
    processTouchInput();
    #endif
}

// Button/LED runtime moved to device_control.cpp

void idleDelay(uint32_t delayMs) {
    const uint32_t CHECK_INTERVAL_MS = 100;
    uint32_t remainingDelay = delayMs;
    while (remainingDelay > 0) {
#ifdef TARGET_NRF
        ble_nrf_advertising_tick();
#endif
        processButtonEvents();
        processTouchInput();
        processLedFlash();
        uint32_t chunkDelay = (remainingDelay > CHECK_INTERVAL_MS) ? CHECK_INTERVAL_MS : remainingDelay;
        delay(chunkDelay);
        remainingDelay -= chunkDelay;
    }
}


#ifdef TARGET_ESP32
void fullSetupAfterConnection() {
    writeSerial("=== Full Setup After Connection ===");
    initWiFi(false);
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
        bbepSetRotation(&bbep, globalConfig.displays[0].rotation * 90);
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
    delay(200);
    BLEDevice::deinit(true);
    esp32_ble_clear_handles();
    delay(100);
    writeSerial("BLE deinitialized");
    uint64_t sleep_timeout_us = (uint64_t)globalConfig.power_option.deep_sleep_time_seconds * 1000000ULL;
    esp_sleep_enable_timer_wakeup(sleep_timeout_us);
    writeSerial("Entering deep sleep...");
    delay(100); // Brief delay to ensure serial output is sent
    powerLatchHoldForSleep();
    esp_deep_sleep_start();
}
#endif

// Panel rail is cut after this — drive control lines LOW; BUSY stays an input.
static void configureDisplayPinsLowPower() {
    const DisplayConfig& d = globalConfig.displays[0];
    const uint8_t pins[] = {
        d.cs_pin, d.clk_pin, d.data_pin, d.dc_pin, d.reset_pin,
    };
    for (uint8_t pin : pins) {
        if (pin == 0xFF) continue;
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    if (d.busy_pin != 0xFF) {
        pinMode(d.busy_pin, INPUT);
    }

    if (!(globalConfig.system_config.device_flags &
          (DEVICE_FLAG_BATTERY_LATCH | DEVICE_FLAG_PWR_LATCH_DFF))) {
        const uint8_t auxPins[] = {
            globalConfig.system_config.pwr_pin_2,
            globalConfig.system_config.pwr_pin_3,
        };
        for (uint8_t pin : auxPins) {
            if (pin == 0xFF || pin == 0) continue;
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
    }
}

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
            invalidateOpenDisplayWire();
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
    const DisplayConfig& disp = globalConfig.displays[0];
    if (onoff) {
        if (globalConfig.system_config.pwr_pin != 0xFF) {
            digitalWrite(globalConfig.system_config.pwr_pin, HIGH);
            delay(800);
        } else {
            writeSerial("Power pin not set");
        }
        if (!seeed_driver_spi) {
            if (disp.reset_pin != 0xFF) {
                pinMode(disp.reset_pin, OUTPUT);
                digitalWrite(disp.reset_pin, HIGH);
            }
            if (disp.cs_pin != 0xFF) {
                pinMode(disp.cs_pin, OUTPUT);
                digitalWrite(disp.cs_pin, HIGH);
            }
            if (disp.dc_pin != 0xFF) {
                pinMode(disp.dc_pin, OUTPUT);
                digitalWrite(disp.dc_pin, LOW);
            }
            if (disp.clk_pin != 0xFF) {
                pinMode(disp.clk_pin, OUTPUT);
                digitalWrite(disp.clk_pin, LOW);
            }
            if (disp.data_pin != 0xFF) {
                pinMode(disp.data_pin, OUTPUT);
                digitalWrite(disp.data_pin, LOW);
            }
            if (disp.busy_pin != 0xFF) {
                pinMode(disp.busy_pin, INPUT);
            }
            delay(100);
        } else {
            if (disp.reset_pin != 0xFF) {
                pinMode(disp.reset_pin, OUTPUT);
                digitalWrite(disp.reset_pin, HIGH);
            }
            delay(200);
        }
        initOrRestoreWireForOpenDisplay();
    } else {
        if (!seeed_driver_spi) {
            SPI.end();
        }
        // Keep I2C alive when sensors/touch use data_bus[0] (e.g. reTerminal MISC_I2C on GPIO0/1).
        if (!openDisplayI2cBusConfigured()) {
            Wire.end();
            invalidateOpenDisplayWire();
        }
        if (globalConfig.system_config.pwr_pin != 0xFF) {
            configureDisplayPinsLowPower();
            digitalWrite(globalConfig.system_config.pwr_pin, LOW);
        }
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

// Blocks until the log UART has physically drained. The IDF panic handler only
// flushes the console UART, so without this the last lines before a crash are
// lost on the OPENDISPLAY_LOG_UART port — call after a marker to guarantee it
// reaches the wire before a suspect call can fault.
void flushLog(){
    #if defined(TARGET_ESP32) && defined(OPENDISPLAY_LOG_UART)
    LogSerialPort.flush();
    #elif !defined(DISABLE_USB_SERIAL)
    Serial.flush();
    #endif
    #ifdef TARGET_ESP32
    delay(5);
    #endif
}

void xiaoinit(){
    powerDownExternalFlash(20,24,21,25,22,23);
    //pinMode(31, INPUT);
    //pinMode(14, INPUT);
    pinMode(13, OUTPUT);  //that actually does something
    digitalWrite(13, LOW);
    //pinMode(17, INPUT);
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

#ifdef TARGET_NRF
void powerDownExternalFlashFromConfig(void) {
    if (!globalConfig.loaded || globalConfig.flash_config_count == 0) {
        return;
    }
    const FlashConfig* flashCfg = nullptr;
    for (uint8_t i = 0; i < globalConfig.flash_config_count; i++) {
        if ((globalConfig.flash_configs[i].flags & FLASH_CONFIG_FLAG_ENABLED) != 0) {
            flashCfg = &globalConfig.flash_configs[i];
            break;
        }
    }
    if (flashCfg == nullptr) {
        return;
    }
    const uint8_t mosiPin = flashCfg->mosi_pin;
    const uint8_t sckPin = flashCfg->sck_pin;
    const uint8_t csPin = flashCfg->cs_pin;
    if (mosiPin == 0xFF || sckPin == 0xFF || csPin == 0xFF) {
        writeSerial("Flash config: invalid MOSI/SCK/CS pins", true);
        return;
    }
    writeSerial("Flash config: deep sleep MOSI=" + String(mosiPin) + " SCK=" + String(sckPin) + " CS=" + String(csPin), true);

    pinMode(mosiPin, OUTPUT);
    pinMode(sckPin, OUTPUT);
    pinMode(csPin, OUTPUT);
    digitalWrite(sckPin, LOW);
    digitalWrite(csPin, LOW);

    uint8_t cmd = 0xB9;
    for (uint8_t bit = 0; bit < 8; bit++) {
        digitalWrite(mosiPin, (cmd & 0x80) ? HIGH : LOW);
        cmd <<= 1;
        delayMicroseconds(1);
        digitalWrite(sckPin, HIGH);
        delayMicroseconds(1);
        digitalWrite(sckPin, LOW);
    }
    digitalWrite(csPin, HIGH);
    delayMicroseconds(30);

    // Park like powerDownExternalFlash: CLK/MOSI LOW, CS HIGH (deselected, deep sleep).
    pinMode(mosiPin, OUTPUT);
    digitalWrite(mosiPin, LOW);
    pinMode(sckPin, OUTPUT);
    digitalWrite(sckPin, LOW);
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
}
#else
void powerDownExternalFlashFromConfig(void) {}
#endif

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
    // CS/WP/HOLD are active-low: keep HIGH so the chip stays deselected and in deep sleep.
    // CLK/MOSI/MISO LOW — defined idle levels, no floating buffers on the MCU side.
    const uint8_t qspiLow[] = { mosiPin, misoPin, sckPin };
    for (uint8_t pin : qspiLow) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    const uint8_t qspiHigh[] = { csPin, wpPin, holdPin };
    for (uint8_t pin : qspiHigh) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }
    #else
    writeSerial("External flash power-down not implemented for ESP32");
    return false;
    #endif
    return false;
}
