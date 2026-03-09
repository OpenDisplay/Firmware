#include "main.h"
#include "logo.h"

void setup() {
    #ifndef DISABLE_USB_SERIAL
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
    ble_init();
    writeSerial("BLE advertising started - waiting for connections...");
    initDisplay();
    writeSerial("Display initialized");
    #ifdef TARGET_ESP32
    initWiFi();
    #endif
    updatemsdata();
    initButtons();
    writeSerial("=== Setup completed successfully ===");
}

void loop() {
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
        return;
    }
    if (commandQueueTail != commandQueueHead) {
        writeSerial("ESP32: Processing queued command (" + String(commandQueue[commandQueueTail].len) + " bytes)");
        imageDataWritten(NULL, NULL, commandQueue[commandQueueTail].data, commandQueue[commandQueueTail].len);
        commandQueue[commandQueueTail].pending = false;
        commandQueueTail = (commandQueueTail + 1) % COMMAND_QUEUE_SIZE;
        writeSerial("Command processed");
    }
    if (responseQueueTail != responseQueueHead && pTxCharacteristic && pServer && pServer->getConnectedCount() > 0) {
        writeSerial("ESP32: Sending queued response (" + String(responseQueue[responseQueueTail].len) + " bytes)");
        pTxCharacteristic->setValue(responseQueue[responseQueueTail].data, responseQueue[responseQueueTail].len);
        pTxCharacteristic->notify();
        responseQueue[responseQueueTail].pending = false;
        responseQueueTail = (responseQueueTail + 1) % RESPONSE_QUEUE_SIZE;
        writeSerial("Response sent successfully");
    }
    if (directWriteActive && directWriteStartTime > 0) {
        uint32_t directWriteDuration = millis() - directWriteStartTime;
        if (directWriteDuration > 120000) {  // 120 second timeout
            writeSerial("ERROR: Direct write timeout (" + String(directWriteDuration) + " ms) - cleaning up stuck state");
            cleanupDirectWriteState(true);
        }
    }
    #ifdef TARGET_ESP32
    handleWiFiServer();
    if (wifiServerConnected && wifiClient.connected() && !wifiImageRequestPending) {
        uint32_t now = millis();
        bool timeToSend = false;
        if (wifiNextImageRequestTime == 0) {
            timeToSend = true;  // Send immediately
        } else if (now >= wifiNextImageRequestTime) {
            timeToSend = true;  // Time has come
        } else if ((wifiNextImageRequestTime - now) > 0x7FFFFFFF) {
            timeToSend = true;  // millis() overflow detected (wifiNextImageRequestTime is in the past)
        }
        if (timeToSend) {
            writeSerial("Sending scheduled Image Request (poll_interval=" + String(wifiPollInterval) + "s)");
            sendImageRequest();
        }
    }
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
            // Reinitialize mDNS on reconnection
            String deviceName = "OD" + getChipIdHex();
            if (MDNS.begin(deviceName.c_str())) {
                writeSerial("mDNS responder restarted: " + deviceName + ".local");
            } else {
                writeSerial("ERROR: Failed to restart mDNS responder");
            }
            // Attempt to reconnect to server via mDNS
            discoverAndConnectWiFiServer();
        }
    }
    #endif
    bool bleActive = (commandQueueTail != commandQueueHead) || 
                     (responseQueueTail != responseQueueHead) ||
                     (pServer && pServer->getConnectedCount() > 0);
    if (bleActive) {
      // Check for button events in fast loop
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
    writeSerial("Loop end: " + String(millis() / 100));
    #endif
}

void processButtonEvents() {
    if (buttonEventPending) {
        buttonEventPending = false; 
        uint32_t currentTime = millis();
        uint8_t changedButtonIndex = lastChangedButtonIndex;
        lastChangedButtonIndex = 0xFF;  
        writeSerial("Button event pending: " + String(changedButtonIndex));
        if (changedButtonIndex < MAX_BUTTONS && buttonStates[changedButtonIndex].initialized) {
            ButtonState* btn = &buttonStates[changedButtonIndex];
            if (btn->current_state == 1) {
                bool resetCount = false;
                if (btn->last_press_time == 0 || currentTime - btn->last_press_time > 5000) {
                    resetCount = true;  // First press or more than 5 seconds since last press
                }
                if (btn->last_press_time > 0) {
                    for (uint8_t j = 0; j < buttonStateCount; j++) {
                        if (j != changedButtonIndex && buttonStates[j].initialized && 
                            buttonStates[j].last_press_time > 0 &&  // Has been pressed before
                            buttonStates[j].last_press_time > btn->last_press_time &&  // Pressed more recently than this button
                            (currentTime - buttonStates[j].last_press_time) < 5000) {  // Within 5 seconds
                            resetCount = true;  // Another button was pressed more recently
                            break;
                        }
                    }
                }
                if (resetCount) {
                    for (uint8_t j = 0; j < buttonStateCount; j++) {
                        if (buttonStates[j].initialized) {
                            buttonStates[j].press_count = 0;
                        }
                    }
                    btn->press_count = 1;
                }
                btn->last_press_time = currentTime;
            }
        }
        if (changedButtonIndex < MAX_BUTTONS && buttonStates[changedButtonIndex].initialized) {
            ButtonState* btn = &buttonStates[changedButtonIndex];
            bool pinState = digitalRead(btn->pin);
            bool logicalPressed = btn->inverted ? !pinState : pinState;
            writeSerial("Pin state: " + String(pinState) + ", Logical pressed: " + String(logicalPressed) + ",inverted: " + String(btn->inverted));
            uint8_t logicalState = logicalPressed ? 1 : 0;
            btn->current_state = logicalState;
            writeSerial("Button: " + String(btn->button_id) + ", Press count: " + String(btn->press_count) + ", Current state: " + String(btn->current_state));
            uint8_t buttonData = (btn->button_id & 0x07) |                    // Bits 0-2: button_id
                                 ((btn->press_count & 0x0F) << 3) |           // Bits 3-6: press_count (4 bits, 0-15)
                                 ((btn->current_state & 0x01) << 7);          // Bit 7: current_state (logical, with inversion)
            if (btn->byte_index < 11) {
                dynamicreturndata[btn->byte_index] = buttonData;
            }
        }
        updatemsdata();
    }
}

void flashLed(uint8_t color, uint8_t brightness) {
    if (activeLedInstance == 0xFF) {
        for (uint8_t i = 0; i < globalConfig.led_count; i++) {
            if (globalConfig.leds[i].led_type == 1) {  // RGB LED type
                activeLedInstance = i;
                break;
            }
        }
        if (activeLedInstance == 0xFF) {
            return;  // No RGB LED configured
        }
    }
    struct LedConfig* led = &globalConfig.leds[activeLedInstance];
    uint8_t ledRedPin = led->led_1_r;
    uint8_t ledGreenPin = led->led_2_g;
    uint8_t ledBluePin = led->led_3_b;
    bool invertRed = (led->led_flags & 0x01) != 0;
    bool invertGreen = (led->led_flags & 0x02) != 0;
    bool invertBlue = (led->led_flags & 0x04) != 0;
    uint8_t colorred = (color >> 5) & 0b00000111;
    uint8_t colorgreen = (color >> 2) & 0b00000111;
    uint8_t colorblue = color & 0b00000011;
    for (uint16_t i = 0; i < brightness; i++) {
        digitalWrite(ledRedPin, invertRed ? !(colorred >= 7) : (colorred >= 7));
        digitalWrite(ledGreenPin, invertGreen ? !(colorgreen >= 7) : (colorgreen >= 7));
        digitalWrite(ledBluePin, invertBlue ? !(colorblue >= 3) : (colorblue >= 3));
        delayMicroseconds(100);
        digitalWrite(ledRedPin, invertRed ? !(colorred >= 1) : (colorred >= 1));
        digitalWrite(ledGreenPin, invertGreen ? !(colorgreen >= 1) : (colorgreen >= 1));
        delayMicroseconds(100);
        digitalWrite(ledRedPin, invertRed ? !(colorred >= 6) : (colorred >= 6));
        digitalWrite(ledGreenPin, invertGreen ? !(colorgreen >= 6) : (colorgreen >= 6));
        digitalWrite(ledBluePin, invertBlue ? !(colorblue >= 1) : (colorblue >= 1));
        delayMicroseconds(100);
        digitalWrite(ledRedPin, invertRed ? !(colorred >= 2) : (colorred >= 2));
        digitalWrite(ledGreenPin, invertGreen ? !(colorgreen >= 2) : (colorgreen >= 2));
        delayMicroseconds(100);
        digitalWrite(ledRedPin, invertRed ? !(colorred >= 5) : (colorred >= 5));
        digitalWrite(ledGreenPin, invertGreen ? !(colorgreen >= 5) : (colorgreen >= 5));
        delayMicroseconds(100);
        digitalWrite(ledRedPin, invertRed ? !(colorred >= 3) : (colorred >= 3));
        digitalWrite(ledGreenPin, invertGreen ? !(colorgreen >= 3) : (colorgreen >= 3));
        digitalWrite(ledBluePin, invertBlue ? !(colorblue >= 2) : (colorblue >= 2));
        delayMicroseconds(100);
        digitalWrite(ledRedPin, invertRed ? !(colorred >= 4) : (colorred >= 4));
        digitalWrite(ledGreenPin, invertGreen ? !(colorgreen >= 4) : (colorgreen >= 4));
        delayMicroseconds(100);
        digitalWrite(ledRedPin, invertRed ? HIGH : LOW);
        digitalWrite(ledGreenPin, invertGreen ? HIGH : LOW);
        digitalWrite(ledBluePin, invertBlue ? HIGH : LOW);
    }
}

void ledFlashLogic() {
    writeSerial("=== ledFlashLogic START ===");
    writeSerial("ledFlashActive: " + String(ledFlashActive ? "true" : "false"));
    writeSerial("activeLedInstance: 0x" + String(activeLedInstance, HEX));
    writeSerial("led_count: " + String(globalConfig.led_count));
    if (!ledFlashActive) {
        writeSerial("LED flash not active, returning");
        return;
    }
    if (activeLedInstance == 0xFF) {
        writeSerial("Searching for RGB LED...");
        for (uint8_t i = 0; i < globalConfig.led_count; i++) {
            writeSerial("LED[" + String(i) + "]: type=" + String(globalConfig.leds[i].led_type));
            if (globalConfig.leds[i].led_type == 1) {  // RGB LED type
                activeLedInstance = i;
                writeSerial("Found RGB LED at instance " + String(i));
                break;
            }
        }
        if (activeLedInstance == 0xFF) {
            writeSerial("ERROR: No RGB LED configured");
            return;
        }
    }
    struct LedConfig* led = &globalConfig.leds[activeLedInstance];
    uint8_t* ledcfg = led->reserved;
    uint8_t brightness = ((ledcfg[0] >> 4) & 0x0F) + 1;  // Bits 4-7: brightness (1-16)
    uint8_t mode = ledcfg[0] & 0x0F;  // Bits 0-3: mode
    writeSerial("Brightness: " + String(brightness) + " (from bits 4-7: " + String((ledcfg[0] >> 4) & 0x0F) + ")");
    writeSerial("Mode: " + String(mode) + " (from bits 0-3)");
    if (mode == 1) {
        writeSerial("Mode 1: Flash pattern enabled");
        const uint8_t interloopdelayfactor = 100;
        const uint8_t loopdelayfactor = 100;
        uint8_t c1 = ledcfg[1];
        uint8_t c2 = ledcfg[4];
        uint8_t c3 = ledcfg[7];
        uint8_t loop1delay = (ledcfg[2] >> 4) & 0x0F;
        uint8_t loop2delay = (ledcfg[5] >> 4) & 0x0F;
        uint8_t loop3delay = (ledcfg[8] >> 4) & 0x0F;
        uint8_t loopcnt1 = ledcfg[2] & 0x0F;
        uint8_t loopcnt2 = ledcfg[5] & 0x0F;
        uint8_t loopcnt3 = ledcfg[8] & 0x0F;
        uint8_t ildelay1 = ledcfg[3];
        uint8_t ildelay2 = ledcfg[6];
        uint8_t ildelay3 = ledcfg[9];
        uint8_t grouprepeats = ledcfg[10] + 1;
        writeSerial("Loop 1: color=0x" + String(c1, HEX) + " count=" + String(loopcnt1) + 
                    " delay=" + String(loop1delay) + " inter=" + String(ildelay1));
        writeSerial("Loop 2: color=0x" + String(c2, HEX) + " count=" + String(loopcnt2) + 
                    " delay=" + String(loop2delay) + " inter=" + String(ildelay2));
        writeSerial("Loop 3: color=0x" + String(c3, HEX) + " count=" + String(loopcnt3) + 
                    " delay=" + String(loop3delay) + " inter=" + String(ildelay3));
        writeSerial("Group repeats: " + String(grouprepeats) + " (255 = infinite)");
        while (ledFlashActive) {
            if (ledFlashPosition >= grouprepeats && grouprepeats != 255) {
                writeSerial("Group repeats reached (" + String(grouprepeats) + "), stopping");
                brightness = 0;
                ledcfg[0] = 0x00;  // Disable mode
                ledFlashPosition = 0;
                break;
            }
            writeSerial("Group repeat " + String(ledFlashPosition + 1) + "/" + String(grouprepeats == 255 ? "inf" : String(grouprepeats)));
            for (int i = 0; i < loopcnt1; i++) {
                flashLed(c1, brightness);
                delay(loop1delay * loopdelayfactor);
            }
            delay(ildelay1 * interloopdelayfactor);
            for (int i = 0; i < loopcnt2; i++) {
                flashLed(c2, brightness);
                delay(loop2delay * loopdelayfactor);
            }
            delay(ildelay2 * interloopdelayfactor);
            for (int i = 0; i < loopcnt3; i++) {
                flashLed(c3, brightness);
                delay(loop3delay * loopdelayfactor);
            }
            delay(ildelay3 * interloopdelayfactor);
            
            ledFlashPosition++;
        }
        writeSerial("Flash pattern completed");
    } else {
        writeSerial("Mode 0 or disabled, nothing to do");
    }
    writeSerial("=== ledFlashLogic END ===");
}

void idleDelay(uint32_t delayMs) {
    const uint32_t CHECK_INTERVAL_MS = 100;
    uint32_t remainingDelay = delayMs;
    while (remainingDelay > 0) {
        processButtonEvents();
        uint32_t chunkDelay = (remainingDelay > CHECK_INTERVAL_MS) ? CHECK_INTERVAL_MS : remainingDelay;
        delay(chunkDelay);
        remainingDelay -= chunkDelay;
    }
}

void initio(){
    if(globalConfig.led_count > 0){
        for (uint8_t i = 0; i < globalConfig.led_count; i++) {
            struct LedConfig* led = &globalConfig.leds[i];
            bool invertRed = (led->led_flags & 0x01) != 0;
            bool invertGreen = (led->led_flags & 0x02) != 0;
            bool invertBlue = (led->led_flags & 0x04) != 0;
            bool invertLed4 = (led->led_flags & 0x08) != 0;
                if (led->led_1_r != 0xFF) {
                    pinMode(led->led_1_r, OUTPUT);
                    digitalWrite(led->led_1_r, invertRed ? HIGH : LOW);
                }
                if (led->led_2_g != 0xFF) {
                    pinMode(led->led_2_g, OUTPUT);
                    digitalWrite(led->led_2_g, invertGreen ? HIGH : LOW);
                }
                if (led->led_3_b != 0xFF) {
                    pinMode(led->led_3_b, OUTPUT);
                    digitalWrite(led->led_3_b, invertBlue ? HIGH : LOW);
                }
                if (led->led_4 != 0xFF) {
                    pinMode(led->led_4, OUTPUT);
                    digitalWrite(led->led_4, invertLed4 ? HIGH : LOW);
                }
        }
        for (uint8_t i = 0; i < globalConfig.led_count; i++) {
            if (globalConfig.leds[i].led_type == 0) {
                activeLedInstance = i;
                flashLed(0xE0, 15);
                flashLed(0x1C, 15);
                flashLed(0x03, 15);
                flashLed(0xFF, 15);
            }
        }
    }
    if(globalConfig.system_config.pwr_pin != 0xFF){
    pinMode(globalConfig.system_config.pwr_pin, OUTPUT);
    digitalWrite(globalConfig.system_config.pwr_pin, LOW);
    }
    else{
        writeSerial("Power pin not set");
    }
    initDataBuses();
    initSensors();
}

#ifdef TARGET_NRF
uint8_t pinToButtonIndex[64] = {0xFF};  // Map pin number to button index (max 64 pins)
#endif

#ifdef TARGET_ESP32
void IRAM_ATTR handleButtonISR(uint8_t buttonIndex) {
#else
void handleButtonISR(uint8_t buttonIndex) {
#endif
    if (buttonIndex >= MAX_BUTTONS || !buttonStates[buttonIndex].initialized) {
        return;
    }
    ButtonState* btn = &buttonStates[buttonIndex];
    bool pinState = digitalRead(btn->pin);
    bool pressed = btn->inverted ? !pinState : pinState;
    uint8_t newState = pressed ? 1 : 0;
    if (newState != btn->current_state) {
        btn->current_state = newState;
        lastChangedButtonIndex = buttonIndex;
        if (pressed) {
            if (btn->press_count < 15) {
                btn->press_count++;
            }
        }
        buttonEventPending = true;
    }
}

#ifdef TARGET_ESP32
void IRAM_ATTR buttonISR(void* arg) {
    uint8_t buttonIndex = (uint8_t)(uintptr_t)arg;
    handleButtonISR(buttonIndex);
}
#elif defined(TARGET_NRF)
void buttonISRGeneric() {
    for (uint8_t i = 0; i < buttonStateCount; i++) {
        if (buttonStates[i].initialized) {
            ButtonState* btn = &buttonStates[i];
            bool pinState = digitalRead(btn->pin);
            bool pressed = btn->inverted ? !pinState : pinState;
            uint8_t newState = pressed ? 1 : 0;
            if (newState != btn->current_state) {
                handleButtonISR(i);
                break;
            }
        }
    }
}
#endif

void initButtons() {
    writeSerial("=== Initializing Buttons ===");
    buttonStateCount = 0;
    for (uint8_t i = 0; i < MAX_BUTTONS; i++) {
        buttonStates[i].initialized = false;
        buttonStates[i].button_id = 0;
        buttonStates[i].press_count = 0;
        buttonStates[i].last_press_time = 0;
        buttonStates[i].current_state = 0;
        buttonStates[i].byte_index = 0xFF;
        buttonStates[i].pin = 0xFF;
        buttonStates[i].instance_index = 0xFF;
    }
    if (globalConfig.binary_input_count == 0) {
        writeSerial("No binary inputs configured");
        return;
    }
    for (uint8_t instanceIdx = 0; instanceIdx < globalConfig.binary_input_count; instanceIdx++) {
        struct BinaryInputs* input = &globalConfig.binary_inputs[instanceIdx];
        if (input->input_type != 1) {
            continue;
        }
        if (input->button_data_byte_index > 10) {
            writeSerial("WARNING: BinaryInputs instance " + String(instanceIdx) + " has invalid byte_index (" + String(input->button_data_byte_index) + "), skipping");
            continue;
        }
        uint8_t* instancePins[8] = {
            &input->reserved_pin_1,
            &input->reserved_pin_2,
            &input->reserved_pin_3,
            &input->reserved_pin_4,
            &input->reserved_pin_5,
            &input->reserved_pin_6,
            &input->reserved_pin_7,
            &input->reserved_pin_8
        };
        for (uint8_t pinIdx = 0; pinIdx < 8; pinIdx++) {
            uint8_t pin = *instancePins[pinIdx];
            if (pin == 0xFF) {
                continue;
            }
            if (buttonStateCount >= MAX_BUTTONS) {
                writeSerial("WARNING: Maximum button count (" + String(MAX_BUTTONS) + ") reached, skipping remaining pins");
                break;
            }
            ButtonState* btn = &buttonStates[buttonStateCount];
            btn->button_id = (input->instance_number * 8) + pinIdx;
            if (btn->button_id > 7) {
                btn->button_id = btn->button_id % 8;  // Wrap to 0-7 for encoding
            }
            btn->byte_index = input->button_data_byte_index;
            btn->pin = pin;
            btn->instance_index = instanceIdx;
            btn->press_count = 0;
            btn->last_press_time = 0;
            btn->pin_offset = pinIdx;  // Cache pin offset for ISR
            btn->inverted = (input->invert & (1 << pinIdx)) != 0;  // Cache invert flag for ISR
            pinMode(pin, INPUT);
            bool hasPullup = (input->pullups & (1 << pinIdx)) != 0;
            #ifdef TARGET_ESP32
            bool hasPulldown = (input->pulldowns & (1 << pinIdx)) != 0;
            if (hasPullup) {
                pinMode(pin, INPUT_PULLUP);
            } else if (hasPulldown) {
                pinMode(pin, INPUT_PULLDOWN);
            }
            #elif defined(TARGET_NRF)
            if (hasPullup) {
                pinMode(pin, INPUT_PULLUP);
            }
            #endif
            delay(10);  // Small delay to let pin settle
            bool initialPinState = digitalRead(pin);
            bool initialPressed = btn->inverted ? !initialPinState : initialPinState;
            btn->current_state = initialPressed ? 1 : 0;
            #ifdef TARGET_ESP32
            attachInterruptArg(pin, buttonISR, (void*)(uintptr_t)buttonStateCount, CHANGE);
            #elif defined(TARGET_NRF)
            attachInterrupt(pin, buttonISRGeneric, CHANGE);
            writeSerial("nRF: Attached interrupt to pin " + String(pin) + " (button index " + String(buttonStateCount) + ")");
            #endif
            btn->initialized = true;
            buttonStateCount++;
            writeSerial("Button initialized: instance=" + String(instanceIdx) + ", pin_idx=" + String(pinIdx) + ", pin=" + String(pin) + ", byte_index=" + String(btn->byte_index) + ", button_id=" + String(btn->button_id));
        }
    }
    
    writeSerial("Total buttons initialized: " + String(buttonStateCount));
}

uint8_t getFirmwareMajor(){
    String version = String(BUILD_VERSION);
    int dotIndex = version.indexOf('.');
    if (dotIndex > 0) {
        return version.substring(0, dotIndex).toInt();
    }
    return 0;
}

uint8_t getFirmwareMinor(){
    String version = String(BUILD_VERSION);
    int dotIndex = version.indexOf('.');
    if (dotIndex > 0 && dotIndex < (int)(version.length() - 1)) {
        return version.substring(dotIndex + 1).toInt();
    }
    return 0;
}

void initDataBuses(){
    writeSerial("=== Initializing Data Buses ===");
    if(globalConfig.data_bus_count == 0){
        writeSerial("No data buses configured");
        return;
    }
    for(uint8_t i = 0; i < globalConfig.data_bus_count; i++){
        struct DataBus* bus = &globalConfig.data_buses[i];
        if(bus->bus_type == 0x01){ // I2C bus
            writeSerial("Initializing I2C bus " + String(i) + " (instance " + String(bus->instance_number) + ")");
            if(bus->pin_1 == 0xFF || bus->pin_2 == 0xFF){
                writeSerial("ERROR: Invalid I2C pins for bus " + String(i) + " (SCL=" + String(bus->pin_1) + ", SDA=" + String(bus->pin_2) + ")");
                continue;
            }
            uint32_t busSpeed = (bus->bus_speed_hz > 0) ? bus->bus_speed_hz : 100000;
            #ifdef TARGET_ESP32
            pinMode(bus->pin_1, INPUT);
            pinMode(bus->pin_2, INPUT);
            if(bus->pullups & 0x01){
                pinMode(bus->pin_1, INPUT_PULLUP);
            }
            if(bus->pullups & 0x02){
                pinMode(bus->pin_2, INPUT_PULLUP);
            }
            if(bus->pulldowns & 0x01){
                pinMode(bus->pin_1, INPUT_PULLDOWN);
            }
            if(bus->pulldowns & 0x02){
                pinMode(bus->pin_2, INPUT_PULLDOWN);
            }
            #endif
            #ifdef TARGET_NRF
            pinMode(bus->pin_1, INPUT);
            pinMode(bus->pin_2, INPUT);
            if(bus->pullups & 0x01){
                pinMode(bus->pin_1, INPUT_PULLUP);
            }
            if(bus->pullups & 0x02){
                pinMode(bus->pin_2, INPUT_PULLUP);
            }
            #endif
            if(i == 0){
                #ifdef TARGET_ESP32
                Wire.begin(bus->pin_2, bus->pin_1); // SDA, SCL
                Wire.setClock(busSpeed);
                #endif
                #ifdef TARGET_NRF
                Wire.begin(); // Uses default I2C pins
                Wire.setClock(busSpeed);
                writeSerial("NOTE: nRF52840 using default I2C pins (config pins: SCL=" + String(bus->pin_1) + ", SDA=" + String(bus->pin_2) + ")");
                #endif
                writeSerial("I2C bus " + String(i) + " initialized: SCL=pin" + String(bus->pin_1) + ", SDA=pin" + String(bus->pin_2) + ", Speed=" + String(busSpeed) + "Hz");
            } else {
                writeSerial("WARNING: I2C bus " + String(i) + " configured but not initialized (only first bus supported)");
                writeSerial("  SCL=pin" + String(bus->pin_1) + ", SDA=pin" + String(bus->pin_2) + ", Speed=" + String(busSpeed) + "Hz");
            }
        }
        else if(bus->bus_type == 0x02){
            writeSerial("SPI bus " + String(i) + " detected (not yet implemented)");
            writeSerial("  Instance: " + String(bus->instance_number));
        }
        else{
            writeSerial("WARNING: Unknown bus type 0x" + String(bus->bus_type, HEX) + " for bus " + String(i));
        }
    }
    writeSerial("=== Data Bus Initialization Complete ===");
}

void scanI2CDevices(){
    writeSerial("=== Scanning I2C Bus for Devices ===");
    uint8_t deviceCount = 0;
    uint8_t foundDevices[128];
    for(uint8_t address = 0x08; address < 0x78; address++){
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        if(error == 0){
            foundDevices[deviceCount] = address;
            deviceCount++;
            writeSerial("I2C device found at address 0x" + String(address, HEX) + " (" + String(address) + ")");
        }
        else if(error == 4){
            writeSerial("ERROR: Unknown error at address 0x" + String(address, HEX));
        }
    }
    if(deviceCount == 0){
        writeSerial("No I2C devices found on bus");
    } else {
        writeSerial("Found " + String(deviceCount) + " I2C device(s)");
        writeSerial("Device addresses: ");
        String addrList = "";
        for(uint8_t i = 0; i < deviceCount; i++){
            if(i > 0) addrList += ", ";
            addrList += "0x" + String(foundDevices[i], HEX);
        }
        writeSerial(addrList);
    }
    writeSerial("=== I2C Scan Complete ===");
}

void initSensors(){
    writeSerial("=== Initializing Sensors ===");
    if(globalConfig.sensor_count == 0){
        writeSerial("No sensors configured");
        return;
    }
    for(uint8_t i = 0; i < globalConfig.sensor_count; i++){
        struct SensorData* sensor = &globalConfig.sensors[i];
        writeSerial("Initializing sensor " + String(i) + " (instance " + String(sensor->instance_number) + ")");
        writeSerial("  Type: 0x" + String(sensor->sensor_type, HEX));
        writeSerial("  Bus ID: " + String(sensor->bus_id));
        if(sensor->sensor_type == 0x0003){ // AXP2101 PMIC
            writeSerial("  Detected AXP2101 PMIC sensor");
        }
        else if(sensor->sensor_type == 0x0001){ // Temperature sensor
            writeSerial("  Temperature sensor (initialization not implemented)");
        }
        else if(sensor->sensor_type == 0x0002){ // Humidity sensor
            writeSerial("  Humidity sensor (initialization not implemented)");
        }
        else{
            writeSerial("  Unknown sensor type 0x" + String(sensor->sensor_type, HEX));
        }
    }
    writeSerial("=== Sensor Initialization Complete ===");
}

void initAXP2101(uint8_t busId){
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);
    delay(100);
    digitalWrite(21, HIGH);
    writeSerial("=== Initializing AXP2101 PMIC ===");
    if(busId >= globalConfig.data_bus_count){
        writeSerial("ERROR: Invalid bus ID " + String(busId) + " (only " + String(globalConfig.data_bus_count) + " buses configured)");
        return;
    }
    struct DataBus* bus = &globalConfig.data_buses[busId];
    if(bus->bus_type != 0x01){
        writeSerial("ERROR: Bus " + String(busId) + " is not an I2C bus");
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX) + " (error: " + String(error) + ")");
        return;
    }
    writeSerial("AXP2101 detected at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX));
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t status = Wire.read();
            writeSerial("Power status: 0x" + String(status, HEX));
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_VOL0_CTRL);
    Wire.write(0x12); // 18 decimal = 0x12, gives 3.3V (1500 + 18*100 = 3300)
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DCDC1 voltage set to 3.3V");
    } else {
        writeSerial("ERROR: Failed to set DCDC1 voltage");
    }
    delay(10); // Small delay after setting voltage
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    uint8_t dcEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            dcEnable = Wire.read();
        }
    }
    dcEnable |= 0x01;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    Wire.write(dcEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DCDC1 enabled (3.3V)");
    } else {
        writeSerial("ERROR: Failed to enable DCDC1");
    }
    delay(10);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    uint8_t aldoEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldoEnable = Wire.read();
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL2_CTRL);
    error = Wire.endTransmission();
    uint8_t aldo3VolReg = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldo3VolReg = Wire.read();
        }
    }
    aldo3VolReg = (aldo3VolReg & 0xE0) | 0x1C;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL2_CTRL);
    Wire.write(aldo3VolReg);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO3 voltage set to 3.3V");
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL3_CTRL);
    error = Wire.endTransmission();
    uint8_t aldo4VolReg = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldo4VolReg = Wire.read();
        }
    }
    aldo4VolReg = (aldo4VolReg & 0xE0) | 0x1C; // Preserve upper bits, set voltage
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL3_CTRL);
    Wire.write(aldo4VolReg);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO4 voltage set to 3.3V");
    }
    aldoEnable |= 0x0C; // Set bits 2 and 3 for ALDO3 and ALDO4
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    Wire.write(aldoEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO3 and ALDO4 enabled (3.3V)");
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t wakeupCtl = Wire.read();
            writeSerial("Wakeup control: 0x" + String(wakeupCtl, HEX));
            if(wakeupCtl & 0x01){
                writeSerial("Wakeup already enabled");
            } else {
                Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
                Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
                Wire.write(wakeupCtl | 0x01); // Set bit 0
                error = Wire.endTransmission();
                if(error == 0){
                    writeSerial("Wakeup enabled");
                }
            }
        }
    }
    writeSerial("=== AXP2101 PMIC Initialization Complete ===");
}

void readAXP2101Data(){
    writeSerial("=== Reading AXP2101 PMIC Data ===");
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX));
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_CHANNEL_CTRL);
    Wire.write(0xFF); // Enable all ADC channels
    error = Wire.endTransmission();
    delay(10); // Wait for ADC to stabilize
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t status1 = Wire.read();
            uint8_t status2 = Wire.read();
            writeSerial("Power Status 1: 0x" + String(status1, HEX));
            writeSerial("Power Status 2: 0x" + String(status2, HEX));
            bool batteryPresent = (status1 & 0x20) != 0;
            bool charging = (status1 & 0x04) != 0;
            bool vbusPresent = (status1 & 0x08) != 0;
            writeSerial("Battery Present: " + String(batteryPresent ? "Yes" : "No"));
            writeSerial("Charging: " + String(charging ? "Yes" : "No"));
            writeSerial("VBUS Present: " + String(vbusPresent ? "Yes" : "No"));
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_PWRON_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t pwronStatus = Wire.read();
            writeSerial("Power On Status: 0x" + String(pwronStatus, HEX));
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_BAT_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t batVolH = Wire.read();
            uint8_t batVolL = Wire.read();
            // Battery voltage = (H << 4 | L) * 0.5mV
            uint16_t batVolRaw = ((uint16_t)batVolH << 4) | (batVolL & 0x0F);
            float batVoltage = batVolRaw * 0.5; // in mV
            writeSerial("Battery Voltage: " + String(batVoltage, 1) + " mV (" + String(batVoltage / 1000.0, 2) + " V)");
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_VBUS_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t vbusVolH = Wire.read();
            uint8_t vbusVolL = Wire.read();
            // VBUS voltage = (H << 4 | L) * 1.7mV
            uint16_t vbusVolRaw = ((uint16_t)vbusVolH << 4) | (vbusVolL & 0x0F);
            float vbusVoltage = vbusVolRaw * 1.7; // in mV
            writeSerial("VBUS Voltage: " + String(vbusVoltage, 1) + " mV (" + String(vbusVoltage / 1000.0, 2) + " V)");
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_SYS_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t sysVolH = Wire.read();
            uint8_t sysVolL = Wire.read();
            // System voltage = (H << 4 | L) * 1.4mV
            uint16_t sysVolRaw = ((uint16_t)sysVolH << 4) | (sysVolL & 0x0F);
            float sysVoltage = sysVolRaw * 1.4; // in mV
            writeSerial("System Voltage: " + String(sysVoltage, 1) + " mV (" + String(sysVoltage / 1000.0, 2) + " V)");
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_BAT_PERCENT_DATA);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t batPercent = Wire.read();
            if(batPercent <= 100){
                writeSerial("Battery Percentage: " + String(batPercent) + "%");
            } else {
                writeSerial("Battery Percentage: Not available (fuel gauge may be disabled)");
            }
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t dcEnable = Wire.read();
            writeSerial("DC Enable Status: 0x" + String(dcEnable, HEX));
            writeSerial("  DCDC1: " + String((dcEnable & 0x01) ? "ON" : "OFF"));
            writeSerial("  DCDC2: " + String((dcEnable & 0x02) ? "ON" : "OFF"));
            writeSerial("  DCDC3: " + String((dcEnable & 0x04) ? "ON" : "OFF"));
            writeSerial("  DCDC4: " + String((dcEnable & 0x08) ? "ON" : "OFF"));
            writeSerial("  DCDC5: " + String((dcEnable & 0x10) ? "ON" : "OFF"));
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t aldoEnable = Wire.read();
            writeSerial("ALDO Enable Status: 0x" + String(aldoEnable, HEX));
            writeSerial("  ALDO1: " + String((aldoEnable & 0x01) ? "ON" : "OFF"));
            writeSerial("  ALDO2: " + String((aldoEnable & 0x02) ? "ON" : "OFF"));
            writeSerial("  ALDO3: " + String((aldoEnable & 0x04) ? "ON" : "OFF"));
            writeSerial("  ALDO4: " + String((aldoEnable & 0x08) ? "ON" : "OFF"));
        }
    }
    
    writeSerial("=== AXP2101 Data Read Complete ===");
}

void powerDownAXP2101(){
    writeSerial("=== Powering Down AXP2101 PMIC Rails ===");
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX) + " (error: " + String(error) + ")");
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_IRQ_ENABLE1);
    Wire.write(0x00); // Disable all IRQs in register 1
    error = Wire.endTransmission();
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_ENABLE2);
        Wire.write(0x00); // Disable all IRQs in register 2
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_ENABLE3);
        Wire.write(0x00); // Disable all IRQs in register 3
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_ENABLE4);
        Wire.write(0x00); // Disable all IRQs in register 4
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS1);
        Wire.write(0xFF); // Clear all IRQ status bits
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS2);
        Wire.write(0xFF);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS3);
        Wire.write(0xFF);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS4);
        Wire.write(0xFF);
        error = Wire.endTransmission();
        if(error == 0){
            writeSerial("All IRQs disabled and status cleared");
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    uint8_t dcEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            dcEnable = Wire.read();
        }
    }
    dcEnable &= 0x01; // Keep only DC1 (bit 0), clear bits 1-4 for DC2-5
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    Wire.write(dcEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DC2-5 disabled (DC1 kept enabled)");
    } else {
        writeSerial("ERROR: Failed to disable DC2-5");
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL1);
    Wire.write(0x00); // Disable all LDOs in register 1
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("BLDO1-2, CPUSLDO, DLDO1-2 disabled");
    } else {
        writeSerial("ERROR: Failed to disable BLDO/CPUSLDO/DLDO rails");
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    uint8_t aldoEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldoEnable = Wire.read();
        }
    }
    aldoEnable &= ~0x0F; // Clear bits 0-3 to disable ALDO1, ALDO2, ALDO3, and ALDO4
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    Wire.write(aldoEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO1-4 disabled");
    } else {
        writeSerial("ERROR: Failed to disable ALDO rails");
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
    error = Wire.endTransmission();
    uint8_t wakeupCtrl = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            wakeupCtrl = Wire.read();
        }
    }
    if(!(wakeupCtrl & 0x04)) {
        wakeupCtrl |= 0x04; // Set bit 2: Wake-up power setting same as before sleep
    }
    if(wakeupCtrl & 0x08) {
        wakeupCtrl &= ~0x08; // Clear bit 3: PWROK doesn't need to be pulled low on wake-up
    }
    if(!(wakeupCtrl & 0x10)) {
        wakeupCtrl |= 0x10; // Set bit 4: IRQ pin can wake up
    }
    wakeupCtrl |= 0x80; // Set bit 7 to enable sleep mode
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
    Wire.write(wakeupCtrl);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("AXP2101 wake-up configured and sleep mode enabled");
    } else {
        writeSerial("ERROR: Failed to configure AXP2101 sleep mode");
    }
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_ADC_CHANNEL_CTRL);
        Wire.write(0x00); // Disable all ADC channels
        error = Wire.endTransmission();
        if(error == 0){
            writeSerial("All ADC channels disabled");
        } else {
            writeSerial("ERROR: Failed to disable ADC channels");
        }
    writeSerial("=== AXP2101 PMIC Rails Powered Down ===");
}

void updatemsdata(){
    float batteryVoltage = readBatteryVoltage();
    float chipTemperature = readChipTemperature();
    writeSerial("Battery voltage: " + String(batteryVoltage) + "V");
    writeSerial("Chip temperature: " + String(chipTemperature) + "C");
    uint16_t batteryVoltageMv = (uint16_t)(batteryVoltage * 1000);
    // Convert to 10mV steps and clamp to 9 bits (0-511, max 5.11V)
    uint16_t batteryVoltage10mv = batteryVoltageMv / 10;
    if (batteryVoltage10mv > 511) {
        batteryVoltage10mv = 511;  // Clamp to maximum 9-bit value
    }
    // Encode temperature in byte 13 with 0.5°C accuracy
    // Range: -40°C to +87.5°C, encoding: (temperature + 40) * 2
    // This gives: -40°C = 0, 0°C = 80, +87.5°C = 255
    int16_t tempEncoded = (int16_t)((chipTemperature + 40.0f) * 2.0f);
    if (tempEncoded < 0) {
        tempEncoded = 0;  // Clamp to minimum (-40°C)
    } else if (tempEncoded > 255) {
        tempEncoded = 255;  // Clamp to maximum (+87.5°C)
    }
    uint8_t temperatureByte = (uint8_t)tempEncoded;
    // Encode battery voltage lower 8 bits (byte 14): Battery voltage in 10mV steps, lower 8 bits
    uint8_t batteryVoltageLowByte = (uint8_t)(batteryVoltage10mv & 0xFF);
    // Encode status byte (byte 15): Battery voltage MSB (B) + Reboot flag (I) + Connection requested (C) + RFU (1 bit) + mloopcounter (4 bits)
    // Format: B I C RFU R R R R
    uint8_t statusByte = ((batteryVoltage10mv >> 8) & 0x01) |           // Bit 0: Battery voltage MSB
                         ((rebootFlag & 0x01) << 1) |                     // Bit 1: Reboot flag
                         ((connectionRequested & 0x01) << 2) |            // Bit 2: Connection requested (reserved for future features)
                         ((mloopcounter & 0x0F) << 4);                    // Bits 4-7: mloopcounter (4 bits)
                                                                          // Bit 3: RFU (reserved, set to 0)
    // Update global msd_payload array (public, can be read via handleReadMSD())
    uint16_t msd_cid = 0x2446;
    memset(msd_payload, 0, sizeof(msd_payload));
    memcpy(msd_payload, (uint8_t*)&msd_cid, sizeof(msd_cid));
    memcpy(&msd_payload[2], dynamicreturndata, sizeof(dynamicreturndata)); 
    msd_payload[13] = temperatureByte;      // Temperature with 0.5°C accuracy (-40°C to +87.5°C)
    msd_payload[14] = batteryVoltageLowByte; // Battery voltage lower 8 bits (10mV steps)
    msd_payload[15] = statusByte;            // Battery voltage MSB + Reboot flag + Connection requested + RFU + mloopcounter
#ifdef TARGET_NRF
Bluefruit.Advertising.clearData();
Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
Bluefruit.Advertising.addName();
Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, msd_payload, sizeof(msd_payload));
Bluefruit.Advertising.setInterval(32, 400);
Bluefruit.Advertising.setFastTimeout(1);
Bluefruit.Advertising.stop();
Bluefruit.Advertising.start(0);
#endif
#ifdef TARGET_ESP32
if (advertisementData != nullptr) {
        String manufacturerDataStr;
        manufacturerDataStr.reserve(16);
        for (int i = 0; i < 16; i++) {
            manufacturerDataStr += (char)msd_payload[i];
        }
        advertisementData->setManufacturerData(manufacturerDataStr);
        BLEAdvertising *pAdvertising = nullptr;
        if (pServer != nullptr) {
            pAdvertising = pServer->getAdvertising();
        }
        if (pAdvertising == nullptr) {
            pAdvertising = BLEDevice::getAdvertising();
        }
        if (pAdvertising != nullptr) {
            pAdvertising->stop();
            BLEUUID serviceUUID;
            if (pService != nullptr) {
                serviceUUID = pService->getUUID();
            }
            BLEAdvertisementData freshAdvertisementData;
            static String savedDeviceName = "";
            if (savedDeviceName.length() == 0) {
                savedDeviceName = "OD" + getChipIdHex();
            }
            freshAdvertisementData.setName(savedDeviceName);
            freshAdvertisementData.setManufacturerData(manufacturerDataStr);
            *advertisementData = freshAdvertisementData;
            pAdvertising->setAdvertisementData(freshAdvertisementData);
            if (pService != nullptr) {
                pAdvertising->addServiceUUID(serviceUUID);
            }
            pAdvertising->setScanResponse(false);
            pAdvertising->setMinPreferred(0x06);
            pAdvertising->setMinPreferred(0x12);
            pAdvertising->start();
        } else {
            writeSerial("ERROR: Failed to get advertising object for update");
        }
} else {
    writeSerial("WARNING: updatemsdata called without advertisementData for ESP32");
}
#endif
mloopcounter++;
mloopcounter &= 0x0F;  // Wrap mloopcounter to 4 bits (0-15)
writeSerial("MSD data updated: " + String(mloopcounter));
}

void full_config_init(){
    writeSerial("Initializing config storage...");
    if (initConfigStorage()) {
        writeSerial("Config storage initialized successfully");
    } else {
        writeSerial("Config storage initialization failed");
    }
    writeSerial("Loading global configuration...");
    if (loadGlobalConfig()) {
        writeSerial("Global configuration loaded successfully");
        printConfigSummary();
        // Initialize encryption session
        clearEncryptionSession();
        encryptionInitialized = true;
        // Check reset pin after config is loaded
        checkResetPin();
        #ifdef TARGET_NRF
        if (globalConfig.loaded && (globalConfig.system_config.device_flags & DEVICE_FLAG_XIAOINIT)) {
            writeSerial("Device flag DEVICE_FLAG_XIAOINIT is set, calling xiaoinit()...");
            xiaoinit();
            writeSerial("xiaoinit() completed");
        }
        #endif
        if (globalConfig.loaded && (globalConfig.system_config.device_flags & DEVICE_FLAG_WS_PP_INIT)) {
            writeSerial("Device flag DEVICE_FLAG_WS_PP_INIT is set, calling ws_pp_init()...");
            ws_pp_init();
            writeSerial("ws_pp_init() completed");
        }
    } else {
       writeSerial("Global configuration load failed or no config found");
    }
}

void ble_init(){
    #ifdef TARGET_NRF
    Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.autoConnLed(false);
    Bluefruit.setTxPower(globalConfig.power_option.tx_power);
    Bluefruit.begin(1, 0);
    // Only initialize DFU service if encryption is NOT enabled
    // If encryption is enabled, DFU entry is gated behind CMD_ENTER_DFU (0x0051)
    if (!isEncryptionEnabled()) {
        bledfu.begin();
        writeSerial("BLE DFU initialized successfully (encryption disabled)");
    } else {
        writeSerial("BLE DFU service NOT initialized (encryption enabled - use CMD_ENTER_DFU)");
    }
    writeSerial("BLE initialized successfully");
    writeSerial("Setting up BLE service 0x2446...");
    imageService.begin();
    writeSerial("BLE service started");
    imageCharacteristic.setWriteCallback(imageDataWritten);
    writeSerial("BLE write callback set");
    imageCharacteristic.begin();
    writeSerial("BLE characteristic started");
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
    writeSerial("BLE callbacks registered");
    String deviceName = "OD" + getChipIdHex();
    Bluefruit.setName(deviceName.c_str());
    writeSerial("Device name set to: " + deviceName);
    writeSerial("Configuring power management...");
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    writeSerial("Power management configured");
    writeSerial("Configuring BLE advertising...");
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    updatemsdata();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 200);
    Bluefruit.Advertising.setFastTimeout(10);
    writeSerial("Starting BLE advertising...");
    Bluefruit.Advertising.start(0);
    #endif
    #ifdef TARGET_ESP32
    ble_init_esp32(true); // Update manufacturer data for full setup
    #endif
}

#ifdef TARGET_ESP32
void ble_init_esp32(bool update_manufacturer_data) {
    writeSerial("=== Initializing ESP32 BLE ===");
    String deviceName = "OD" + getChipIdHex();
    writeSerial("Device name will be: " + deviceName);
    BLEDevice::init(deviceName.c_str());
    writeSerial("Setting BLE MTU to 512...");
    BLEDevice::setMTU(512);
    pServer = BLEDevice::createServer();
    if (pServer == nullptr) {
        writeSerial("ERROR: Failed to create BLE server");
        return;
    }
    pServer->setCallbacks(&staticServerCallbacks);
    writeSerial("Server callbacks configured");
    BLEUUID serviceUUID("00002446-0000-1000-8000-00805F9B34FB");
    pService = pServer->createService(serviceUUID);
    if (pService == nullptr) {
        writeSerial("ERROR: Failed to create BLE service");
        return;
    }
    writeSerial("BLE service 0x2446 created successfully");
    BLEUUID charUUID("00002446-0000-1000-8000-00805F9B34FB");
    pTxCharacteristic = pService->createCharacteristic(
        charUUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR
    );
    if (pTxCharacteristic == nullptr) {
        writeSerial("ERROR: Failed to create BLE characteristic");
        return;
    }
    writeSerial("Characteristic created with properties: READ, NOTIFY, WRITE, WRITE_NR");
    pTxCharacteristic->setCallbacks(&staticCharCallbacks);
    pRxCharacteristic = pTxCharacteristic;
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    if (pAdvertising == nullptr) {
        writeSerial("ERROR: Failed to get advertising object");
        return;
    }
    pAdvertising->addServiceUUID(serviceUUID);
    writeSerial("Service UUID added to advertising");
    advertisementData->setName(deviceName);
    writeSerial("Device name added to advertising");
    if (update_manufacturer_data) {
        updatemsdata();
    }
    pAdvertising->setAdvertisementData(*advertisementData);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0006);
    pAdvertising->setMinPreferred(0x0012);
    writeSerial("Advertising intervals set");
    pServer->getAdvertising()->setMinPreferred(0x06);
    pServer->getAdvertising()->setMinPreferred(0x12);
    pServer->getAdvertising()->start();
    writeSerial("=== BLE advertising started successfully ===");
    writeSerial("Device ready: " + deviceName);
    writeSerial("Waiting for BLE connections...");
}

void initWiFi() {
    #ifdef TARGET_ESP32
    writeSerial("=== Initializing WiFi ===");
    
    if (!(globalConfig.system_config.communication_modes & COMM_MODE_WIFI)) {
        writeSerial("WiFi not enabled in communication_modes, skipping");
        wifiInitialized = false;
        return;
    }
    if (!wifiConfigured || wifiSsid[0] == '\0' || strlen(wifiSsid) == 0) {
        writeSerial("WiFi configuration not available or SSID empty, skipping");
        wifiInitialized = false;
        return;
    }
    writeSerial("SSID: \"" + String(wifiSsid) + "\"");
    String deviceName = "OD" + getChipIdHex();
    WiFi.setAutoReconnect(true);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    wifiSsid[32] = '\0';
    wifiPassword[32] = '\0';
    writeSerial("Encryption type: 0x" + String(wifiEncryptionType, HEX));
    wifiConnected = false;
    wifiInitialized = true;
    WiFi.begin(wifiSsid, wifiPassword);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    writeSerial("Waiting for WiFi connection...");
    const int maxRetries = 3;
    const unsigned long timeoutPerRetry = 10000;
    bool connected = false;
    for (int retry = 0; retry < maxRetries && !connected; retry++) {
        unsigned long startAttempt = millis();
        bool abortCurrentRetry = false;
        while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt < timeoutPerRetry)) {
            delay(500);
            wl_status_t status = WiFi.status();
            writeSerial("WiFi status: " + String(status));
            if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
                 writeSerial("Connection failed immediately (Status: " + String(status) + ")");
                 abortCurrentRetry = true;
                 break;
            }
        }
        if (WiFi.status() == WL_CONNECTED) {
            connected = true;
            break;
        } else {
            if (!abortCurrentRetry) {
                writeSerial("Connection attempt " + String(retry + 1) + " timed out");
            }
            if (retry < maxRetries - 1) {
                delay(2000);
            }
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        writeSerial("=== WiFi Connected Successfully ===");
        writeSerial("SSID: " + String(wifiSsid));
        writeSerial("IP Address: " + WiFi.localIP().toString());
        writeSerial("RSSI: " + String(WiFi.RSSI()) + " dBm");
        writeSerial("=== Initializing mDNS ===");
        if (MDNS.begin(deviceName.c_str())) {
            writeSerial("mDNS responder started: " + deviceName + ".local");
        } else {
            writeSerial("ERROR: Failed to start mDNS responder");
        }
        
        discoverAndConnectWiFiServer();
        
        // If we woke from deep sleep, reset next image request time to send immediately
        #ifdef TARGET_ESP32
        extern bool woke_from_deep_sleep;
        if (woke_from_deep_sleep) {
            wifiNextImageRequestTime = 0;  // Send Image Request immediately after wake
            wifiPollInterval = 60;  // Reset to default
            writeSerial("Deep sleep wake detected - will send Image Request immediately when connected");
        }
        #endif
    } else {
        wifiConnected = false;
        writeSerial("=== WiFi Connection Failed ===");
        writeSerial("Final Status: " + String(WiFi.status()));
    }
    #else
    writeSerial("WiFi not supported on this platform");
    wifiInitialized = false;
    #endif
}

#ifdef TARGET_ESP32
void discoverAndConnectWiFiServer() {
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        writeSerial("WiFi not connected, cannot discover server");
        return;
    }
    if (wifiClient.connected()) {
        writeSerial("Already connected to server");
        return;
    }
    writeSerial("=== Discovering OpenDisplay Server via mDNS ===");
    int n = MDNS.queryService("opendisplay", "tcp");
    
    if (n == 0) {
        writeSerial("No OpenDisplay server found via mDNS");
        wifiServerLastConnectAttempt = millis();
        return;
    }
    writeSerial("Found " + String(n) + " OpenDisplay server(s)");
    String serverName;
    int serverPort = 0;
    bool hostnameValid = false;
    for (int retry = 0; retry < 5 && !hostnameValid; retry++) {
        delay(200 * (retry + 1)); // Increasing delay: 200ms, 400ms, 600ms, 800ms, 1000ms
        String tempHostname = MDNS.hostname(0);
        serverPort = MDNS.port(0);
        if (tempHostname.length() > 0) {
            const char* hostnameCStr = tempHostname.c_str();
            if (hostnameCStr != nullptr && hostnameCStr[0] != '\0') {
                serverName = String(hostnameCStr);
                hostnameValid = true;
            }
        }
        
        if (!hostnameValid && retry < 4) {
            writeSerial("Hostname not available yet, retrying... (" + String(retry + 1) + "/5)");
        }
    }
    if (serverPort == 0) {
        serverPort = 2446;
        writeSerial("Port not found in mDNS, using default: " + String(serverPort));
    }
    IPAddress serverIP;
    bool gotIPFromTxt = false;
    if (MDNS.hasTxt(0, "ip")) {
        String ipFromTxt = MDNS.txt(0, "ip");
        if (ipFromTxt.length() > 0) {
            if (serverIP.fromString(ipFromTxt)) {
                gotIPFromTxt = true;
                writeSerial("Got IP from mDNS TXT record: " + serverIP.toString());
            }
        }
    }
    if (!gotIPFromTxt && hostnameValid && serverName.length() > 0) {
        writeSerial("Server discovered:");
        writeSerial("  Name: ");
        writeSerial(serverName);
        writeSerial("  Port: " + String(serverPort));
        const char* serverNameCStr = serverName.c_str();
        if (serverNameCStr != nullptr && WiFi.hostByName(serverNameCStr, serverIP)) {
            gotIPFromTxt = true;
            writeSerial("  IP from hostname resolution: " + serverIP.toString());
        } else {
            String hostnameWithLocal = serverName + ".local";
            const char* hostnameLocalCStr = hostnameWithLocal.c_str();
            if (hostnameLocalCStr != nullptr && WiFi.hostByName(hostnameLocalCStr, serverIP)) {
                gotIPFromTxt = true;
                writeSerial("  IP from hostname resolution (.local): " + serverIP.toString());
            }
        }
    }
    if (!gotIPFromTxt) {
        writeSerial("ERROR: Could not get IP address from mDNS (TXT, hostname, or direct IP)");
        wifiServerLastConnectAttempt = millis();
        return;
    }
    writeSerial("Server discovered:");
    writeSerial("  Port: " + String(serverPort));
    writeSerial("  IP: " + serverIP.toString());
    writeSerial("=== Connecting to TCP Server ===");
    writeSerial("Server: " + serverIP.toString() + ":" + String(serverPort));
    wifiClient.setTimeout(10000);  // 10 second timeout
    bool connected = wifiClient.connect(serverIP, serverPort);
    if (connected) {
        wifiServerConnected = true;
        wifiServerLastConnectAttempt = millis();
        writeSerial("=== TCP Server Connected Successfully ===");
        writeSerial("Remote IP: " + wifiClient.remoteIP().toString());
        writeSerial("Remote Port: " + String(wifiClient.remotePort()));
        sendConnectionNotification(0x01);  // 0x01 = connected
        delay(100);
        wifiNextImageRequestTime = 0;
        wifiPollInterval = 60;
        sendImageRequest();
    } else {
        wifiServerConnected = false;
        wifiServerLastConnectAttempt = millis();
        writeSerial("=== TCP Server Connection Failed ===");
        writeSerial("Error: " + String(wifiClient.getWriteError()));
        writeSerial("Will retry in " + String(WIFI_SERVER_RECONNECT_DELAY / 1000) + " seconds");
    }
}

void sendConnectionNotification(uint8_t status) {
    if (!wifiClient.connected()) {
        writeSerial("Cannot send connection notification - not connected");
        return;
    }
    writeSerial("=== Sending Connection Notification ===");
    String deviceName = "OD" + getChipIdHex();
    uint32_t timestamp = millis() / 1000;  // Seconds since boot
    uint8_t packet[1024];
    uint32_t pos = 0;
    uint32_t lengthPos = pos;
    pos += 2;
    packet[pos++] = 0x01;
    packet[pos++] = 0x00;  // Packet number
    packet[pos++] = 0x27;  // Packet ID: wifi_connection_notification
    memset(&packet[pos], 0, 32);
    uint8_t nameLen = deviceName.length();
    if (nameLen > 31) nameLen = 31;
    memcpy(&packet[pos], deviceName.c_str(), nameLen);
    pos += 32;
    packet[pos++] = getFirmwareMajor();
    packet[pos++] = getFirmwareMinor();
    packet[pos++] = status;
    packet[pos++] = timestamp & 0xFF;
    packet[pos++] = (timestamp >> 8) & 0xFF;
    packet[pos++] = (timestamp >> 16) & 0xFF;
    packet[pos++] = (timestamp >> 24) & 0xFF;
    memset(&packet[pos], 0, 25);
    pos += 25;
    uint32_t dataLen = pos - 2;  // Length without the 2-byte length field
    uint16_t crc = calculateCRC16CCITT(&packet[2], dataLen);
    packet[pos++] = crc & 0xFF;
    packet[pos++] = (crc >> 8) & 0xFF;
    uint16_t totalLength = pos;
    packet[lengthPos] = totalLength & 0xFF;
    packet[lengthPos + 1] = (totalLength >> 8) & 0xFF;
    size_t bytesWritten = wifiClient.write(packet, pos);
    // Note: flush() is deprecated, TCP sockets send data immediately
    if (bytesWritten == pos) {
        writeSerial("Connection notification sent successfully (" + String(bytesWritten) + " bytes)");
        writeSerial("Status: " + String(status == 0x01 ? "Connected" : "Disconnected"));
    } else {
        writeSerial("ERROR: Failed to send complete connection notification (expected " + 
                   String(pos) + ", wrote " + String(bytesWritten) + ")");
    }
}

void sendDisplayAnnouncement() {
    if (!wifiClient.connected()) {
        writeSerial("Cannot send Display Announcement - not connected");
        return;
    }
    writeSerial("=== Sending Display Announcement (0x01) ===");
    if (globalConfig.display_count == 0) {
        writeSerial("ERROR: No display configured, cannot send announcement");
        return;
    }
    uint8_t packet[1024];
    uint32_t pos = 0;
    uint32_t lengthPos = pos;
    pos += 2;
    packet[pos++] = 0x01;
    packet[pos++] = 0x00;  // Packet number
    packet[pos++] = 0x01;  // Packet ID: Display Announcement
    packet[pos++] = 0x01;  // packet_type (always 0x01)
    uint16_t width = globalConfig.displays[0].pixel_width;
    packet[pos++] = width & 0xFF;
    packet[pos++] = (width >> 8) & 0xFF;
    uint16_t height = globalConfig.displays[0].pixel_height;
    packet[pos++] = height & 0xFF;
    packet[pos++] = (height >> 8) & 0xFF;
    packet[pos++] = globalConfig.displays[0].color_scheme;
    uint16_t firmwareId = 0x0001;  // Default, can be configured
    packet[pos++] = firmwareId & 0xFF;
    packet[pos++] = (firmwareId >> 8) & 0xFF;
    uint16_t firmwareVersion = (getFirmwareMajor() << 8) | getFirmwareMinor();
    packet[pos++] = firmwareVersion & 0xFF;
    packet[pos++] = (firmwareVersion >> 8) & 0xFF;
    uint16_t manufacturerId = globalConfig.manufacturer_data.manufacturer_id;
    packet[pos++] = manufacturerId & 0xFF;
    packet[pos++] = (manufacturerId >> 8) & 0xFF;
    uint16_t modelId = 0x0001;  // Default, can be configured
    packet[pos++] = modelId & 0xFF;
    packet[pos++] = (modelId >> 8) & 0xFF;
    uint16_t maxCompressedSize = 0;  // TODO: Set based on compression support
    if (globalConfig.displays[0].transmission_modes & TRANSMISSION_MODE_ZIP) {
        maxCompressedSize = MAX_IMAGE_SIZE;  // Use max image size as limit
    }
    packet[pos++] = maxCompressedSize & 0xFF;
    packet[pos++] = (maxCompressedSize >> 8) & 0xFF;
    uint8_t rotation = globalConfig.displays[0].rotation;
    if (rotation > 3) rotation = 0;  // Clamp to valid range
    packet[pos++] = rotation;
    uint32_t dataLen = pos - 2;  // Length without the 2-byte length field
    uint16_t crc = calculateCRC16CCITT(&packet[2], dataLen);
    packet[pos++] = crc & 0xFF;
    packet[pos++] = (crc >> 8) & 0xFF;
    uint16_t totalLength = pos;
    packet[lengthPos] = totalLength & 0xFF;
    packet[lengthPos + 1] = (totalLength >> 8) & 0xFF;
    size_t bytesWritten = wifiClient.write(packet, pos);
    if (bytesWritten == pos) {
        writeSerial("Display Announcement sent successfully (" + String(bytesWritten) + " bytes)");
    } else {
        writeSerial("ERROR: Failed to send complete Display Announcement (expected " + 
                   String(pos) + ", wrote " + String(bytesWritten) + ")");
    }
}

void sendImageRequest() {
    if (!wifiClient.connected()) {
        writeSerial("Cannot send Image Request - not connected");
        return;
    }
    wifiImageRequestPending = true;
    wifiNextImageRequestTime = millis() + (wifiPollInterval * 1000);
    writeSerial("=== Sending Image Request (0x02) ===");
    uint8_t packet[1024];
    uint32_t pos = 0;
    uint32_t lengthPos = pos;
    pos += 2;
    packet[pos++] = 0x01;
    packet[pos++] = 0x00;  // Packet number
    packet[pos++] = 0x02;  // Packet ID: Image Request
    packet[pos++] = 0x02;  // packet_type (always 0x02)
    float batteryVoltage = readBatteryVoltage();
    uint8_t batteryPercent = 0xFF;  // Default to AC powered
    if (batteryVoltage > 0) {
        if (batteryVoltage >= 4.2) {
            batteryPercent = 100;
        } else if (batteryVoltage >= 3.0) {
            batteryPercent = (uint8_t)(((batteryVoltage - 3.0) / 1.2) * 100);
        } else {
            batteryPercent = 0;
        }
    }
    packet[pos++] = batteryPercent;
    int8_t rssi = (int8_t)WiFi.RSSI();
    packet[pos++] = (uint8_t)rssi;
    uint32_t dataLen = pos - 2;  // Length without the 2-byte length field
    uint16_t crc = calculateCRC16CCITT(&packet[2], dataLen);
    packet[pos++] = crc & 0xFF;
    packet[pos++] = (crc >> 8) & 0xFF;
    uint16_t totalLength = pos;
    packet[lengthPos] = totalLength & 0xFF;
    packet[lengthPos + 1] = (totalLength >> 8) & 0xFF;
    size_t bytesWritten = wifiClient.write(packet, pos);
    // Note: flush() is deprecated, TCP sockets send data immediately
    if (bytesWritten == pos) {
        writeSerial("Image Request sent successfully (" + String(bytesWritten) + " bytes)");
        writeSerial("Battery: " + String(batteryPercent == 0xFF ? "AC" : String(batteryPercent) + "%") + ", RSSI: " + String(rssi) + " dBm");
    } else {
        writeSerial("ERROR: Failed to send complete Image Request (expected " + 
                   String(pos) + ", wrote " + String(bytesWritten) + ")");
    }
}

void disconnectWiFiServer() {
    if (wifiClient.connected()) {
        writeSerial("=== Disconnecting from TCP Server ===");
        sendConnectionNotification(0x00);  // 0x00 = disconnected
        delay(100);  // Give time for notification to be sent
        wifiClient.stop();
        wifiServerConnected = false;
        writeSerial("TCP connection closed");
    }
}

void handleWiFiServer() {
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        if (wifiServerConnected) {
            writeSerial("WiFi disconnected, closing TCP connection");
            disconnectWiFiServer();
        }
        return;
    }
    if (!wifiServerConnected) {
        uint32_t now = millis();
        if (now - wifiServerLastConnectAttempt >= WIFI_SERVER_RECONNECT_DELAY) {
            discoverAndConnectWiFiServer();
        }
        return;
    }
    if (!wifiClient.connected()) {
        if (wifiServerConnected) {
            writeSerial("TCP connection lost");
            wifiServerConnected = false;
            wifiServerLastConnectAttempt = millis();
        }
        return;
    }
    int available = wifiClient.available();
    if (available > 0) {
        int bytesToRead = available;
        if (tcpReceiveBufferPos + bytesToRead > sizeof(tcpReceiveBuffer)) {
            bytesToRead = sizeof(tcpReceiveBuffer) - tcpReceiveBufferPos;
            writeSerial("WARNING: Receive buffer full, truncating data");
        }
        int bytesRead = wifiClient.read(&tcpReceiveBuffer[tcpReceiveBufferPos], bytesToRead);
        if (bytesRead > 0) {
            tcpReceiveBufferPos += bytesRead;
            uint32_t parsePos = 0;
            while (parsePos + 5 <= tcpReceiveBufferPos) {
                uint16_t packetLength = tcpReceiveBuffer[parsePos] | (tcpReceiveBuffer[parsePos + 1] << 8);
                if (packetLength < 5 || packetLength > sizeof(tcpReceiveBuffer)) {
                    writeSerial("ERROR: Invalid packet length: " + String(packetLength));
                    parsePos++;
                    continue;
                }
                if (parsePos + packetLength > tcpReceiveBufferPos) {
                    break;
                }
                writeSerial("Received TCP packet: " + String(packetLength) + " bytes");
                uint32_t packetOffset = parsePos + 2;  // Skip length field
                if (packetLength < 5) {
                    writeSerial("ERROR: Packet too short");
                    parsePos++;
                    continue;
                }
                uint8_t version = tcpReceiveBuffer[packetOffset++];
                if (version != 0x01) {
                    writeSerial("ERROR: Unsupported protocol version: " + String(version));
                    parsePos += packetLength;
                    continue;
                }
                uint32_t dataEnd = parsePos + packetLength - 2;  // Exclude CRC (last 2 bytes)
                uint32_t currentOffset = packetOffset;
                while (currentOffset + 2 <= dataEnd) {
                    uint8_t packetNumber = tcpReceiveBuffer[currentOffset++];
                    uint8_t packetId = tcpReceiveBuffer[currentOffset++];
                    uint16_t payloadLen = dataEnd - currentOffset;
                    uint8_t* payload = &tcpReceiveBuffer[currentOffset];
                    if (packetId == 0x81) {
                        if (payloadLen >= 4) {
                            uint32_t pollInterval = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
                            writeSerial("Server response: No new image, poll again in " + String(pollInterval) + " seconds");
                            wifiPollInterval = pollInterval;
                            wifiNextImageRequestTime = millis() + (pollInterval * 1000);
                            wifiImageRequestPending = false;  // Response received, can send next request
                            writeSerial("Next Image Request scheduled in " + String(pollInterval) + " seconds");
                        } else {
                            writeSerial("ERROR: Packet 0x81 payload too short: " + String(payloadLen));
                        }
                    } else if (packetId == 0x82) {
                        if (payloadLen >= 7) {
                            uint16_t imageLength = payload[0] | (payload[1] << 8);
                            uint32_t pollInterval = payload[2] | (payload[3] << 8) | (payload[4] << 16) | (payload[5] << 24);
                            uint8_t refreshType = payload[6];
                            
                            writeSerial("Server response: New image (" + String(imageLength) + " bytes), poll again in " + String(pollInterval) + " seconds, refresh_type=" + String(refreshType));
                            if (payloadLen >= 7 + imageLength) {
                                uint8_t* imageData = payload + 7;
                                uint8_t emptyData[4] = {0, 0, 0, 0};  // Empty data for uncompressed mode
                                handleDirectWriteStart(emptyData, 0);
                                uint16_t remaining = imageLength;
                                uint16_t offset = 0;
                                while (remaining > 0) {
                                    uint16_t chunkSize = (remaining > 512) ? 512 : remaining;
                                    handleDirectWriteData(imageData + offset, chunkSize);
                                    offset += chunkSize;
                                    remaining -= chunkSize;
                                }
                                uint8_t refreshData = refreshType;
                                handleDirectWriteEnd(&refreshData, 1);
                                wifiPollInterval = pollInterval;
                                wifiNextImageRequestTime = millis() + (pollInterval * 1000);
                                wifiImageRequestPending = false;  // Response received, can send next request
                                writeSerial("Next Image Request scheduled in " + String(pollInterval) + " seconds");
                            } else {
                                writeSerial("ERROR: Incomplete image data (have " + String(payloadLen - 7) + ", need " + String(imageLength) + ")");
                            }
                        } else {
                            writeSerial("ERROR: Packet 0x82 payload too short: " + String(payloadLen));
                        }
                    } else if (packetId == 0x83) {
                        writeSerial("Server requests configuration, sending Display Announcement");
                        sendDisplayAnnouncement();
                    } else {
                        if (payloadLen >= 3) {
                            imageDataWritten(NULL, NULL, payload + 1, payloadLen - 1);
                        } else {
                            writeSerial("ERROR: Unknown packet ID 0x" + String(packetId, HEX) + ", payload too short: " + String(payloadLen));
                        }
                    }
                    break;
                }
                parsePos += packetLength;
            }
            if (parsePos > 0) {
                uint32_t remaining = tcpReceiveBufferPos - parsePos;
                if (remaining > 0) {
                    memmove(tcpReceiveBuffer, &tcpReceiveBuffer[parsePos], remaining);
                }
                tcpReceiveBufferPos = remaining;
            }
        }
    }
}
#endif

void minimalSetup() {
    writeSerial("=== Minimal Setup (Deep Sleep Wake) ===");
    full_config_init();
    initio();
    ble_init_esp32(true); // Update manufacturer data
    writeSerial("=== BLE advertising started (minimal mode) ===");
    writeSerial("Advertising for 10 seconds, waiting for connection...");
    advertising_timeout_active = true;
    advertising_start_time = millis();
}

void fullSetupAfterConnection() {
    writeSerial("=== Full Setup After Connection ===");
    memset(&bbep, 0, sizeof(BBEPDISP));
    int panelType = mapEpd(globalConfig.displays[0].panel_ic_type);
    writeSerial("Panel type: " + String(panelType));
    bbepSetPanelType(&bbep, panelType);
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
        if(globalConfig.sensors[i].sensor_type == 0x0003){
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
    if(onoff){
        pinMode(globalConfig.displays[0].reset_pin, OUTPUT);
        pinMode(globalConfig.displays[0].cs_pin, OUTPUT);
        pinMode(globalConfig.displays[0].dc_pin, OUTPUT);
        pinMode(globalConfig.displays[0].clk_pin, OUTPUT);
        pinMode(globalConfig.displays[0].data_pin, OUTPUT);
        delay(200);
    }
    else{
        SPI.end();
        Wire.end();
        pinMode(globalConfig.displays[0].reset_pin, INPUT);
        pinMode(globalConfig.displays[0].cs_pin, INPUT);
        pinMode(globalConfig.displays[0].dc_pin, INPUT);
        pinMode(globalConfig.displays[0].clk_pin, INPUT);
        pinMode(globalConfig.displays[0].data_pin, INPUT);
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
    #ifndef DISABLE_USB_SERIAL
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

int mapEpd(int id){
    switch(id) {
        case 0x0000: return EP_PANEL_UNDEFINED; // ep_panel_undefined
        case 0x0001: return EP42_400x300; // ep42_400x300
        case 0x0002: return EP42B_400x300; // ep42b_400x300
        case 0x0003: return EP213_122x250; // ep213_122x250
        case 0x0004: return EP213B_122x250; // ep213b_122x250
        case 0x0005: return EP293_128x296; // ep293_128x296
        case 0x0006: return EP294_128x296; // ep294_128x296
        case 0x0007: return EP295_128x296; // ep295_128x296
        case 0x0008: return EP295_128x296_4GRAY; // ep295_128x296_4gray
        case 0x0009: return EP266_152x296; // ep266_152x296
        case 0x000A: return EP102_80x128; // ep102_80x128
        case 0x000B: return EP27B_176x264; // ep27b_176x264
        case 0x000C: return EP29R_128x296; // ep29r_128x296
        case 0x000D: return EP122_192x176; // ep122_192x176
        case 0x000E: return EP154R_152x152; // ep154r_152x152
        case 0x000F: return EP42R_400x300; // ep42r_400x300
        case 0x0010: return EP42R2_400x300; // ep42r2_400x300
        case 0x0011: return EP37_240x416; // ep37_240x416
        case 0x0012: return EP37B_240x416; // ep37b_240x416
        case 0x0013: return EP213_104x212; // ep213_104x212
        case 0x0014: return EP75_800x480; // ep75_800x480 (older version)
        case 0x0015: return EP75_800x480_4GRAY; // ep75_800x480_4gray (older version)
        case 0x0016: return EP75_800x480_4GRAY_V2; // ep75_800x480_4gray_v2 (renamed from ep75_800x480_4gray_old)
        case 0x0017: return EP29_128x296; // ep29_128x296
        case 0x0018: return EP29_128x296_4GRAY; // ep29_128x296_4gray
        case 0x0019: return EP213R_122x250; // ep213r_122x250
        case 0x001A: return EP154_200x200; // ep154_200x200
        case 0x001B: return EP154B_200x200; // ep154b_200x200
        case 0x001C: return EP266YR_184x360; // ep266yr_184x360
        case 0x001D: return EP29YR_128x296; // ep29yr_128x296
        case 0x001E: return EP29YR_168x384; // ep29yr_168x384
        case 0x001F: return EP583_648x480; // ep583_648x480
        case 0x0020: return EP296_128x296; // ep296_128x296
        case 0x0021: return EP26R_152x296; // ep26r_152x296
        case 0x0022: return EP73_800x480; // ep73_800x480
        case 0x0023: return EP73_SPECTRA_800x480; // ep73_spectra_800x480
        case 0x0024: return EP74R_640x384; // ep74r_640x384
        case 0x0025: return EP583R_600x448; // ep583r_600x448
        case 0x0026: return EP75R_800x480; // ep75r_800x480
        case 0x0027: return EP426_800x480; // ep426_800x480
        case 0x0028: return EP426_800x480_4GRAY; // ep426_800x480_4gray
        case 0x0029: return EP29R2_128x296; // ep29r2_128x296
        case 0x002A: return EP41_640x400; // ep41_640x400
        case 0x002B: return EP81_SPECTRA_1024x576; // ep81_spectra_1024x576
        case 0x002C: return EP7_960x640; // ep7_960x640
        case 0x002D: return EP213R2_122x250; // ep213r2_122x250
        case 0x002E: return EP29Z_128x296; // ep29z_128x296
        case 0x002F: return EP29Z_128x296_4GRAY; // ep29z_128x296_4gray
        case 0x0030: return EP213Z_122x250; // ep213z_122x250
        case 0x0031: return EP213Z_122x250_4GRAY; // ep213z_122x250_4gray
        case 0x0032: return EP154Z_152x152; // ep154z_152x152
        case 0x0033: return EP579_792x272; // ep579_792x272
        case 0x0034: return EP213YR_122x250; // ep213yr_122x250
        case 0x0035: return EP37YR_240x416; // ep37yr_240x416
        case 0x0036: return EP35YR_184x384; // ep35yr_184x384
        case 0x0037: return EP397YR_800x480; // ep397yr_800x480
        case 0x0038: return EP154YR_200x200; // ep154yr_200x200
        case 0x0039: return EP266YR2_184x360; // ep266yr2_184x360
        case 0x003A: return EP42YR_400x300; // ep42yr_400x300
        case 0x003B: return EP75_800x480_GEN2; // ep75_800x480_gen2
        case 0x003C: return EP75_800x480_4GRAY_GEN2; // ep75_800x480_4gray_gen2
        case 0x003D: return EP215YR_160x296; // ep215yr_160x296
        case 0x003E: return EP1085_1360x480; // ep1085_1360x480
        case 0x003F: return EP31_240x320; // ep31_240x320
        case 0x0040: return EP75YR_800x480;
        default: return EP_PANEL_UNDEFINED; // Unknown panel type
    }
}

void initDisplay(){
    writeSerial("=== Initializing Display ===");
    if(globalConfig.display_count > 0){
    pwrmgm(true);
    memset(&bbep, 0, sizeof(BBEPDISP));
    int panelType = mapEpd(globalConfig.displays[0].panel_ic_type);
    bbepSetPanelType(&bbep, panelType);
    bbepSetRotation(&bbep, globalConfig.displays[0].rotation * 90);
    bbepInitIO(&bbep, globalConfig.displays[0].dc_pin, globalConfig.displays[0].reset_pin, globalConfig.displays[0].busy_pin, globalConfig.displays[0].cs_pin, globalConfig.displays[0].data_pin, globalConfig.displays[0].clk_pin, 8000000);
    writeSerial(String("Height: ") + String(globalConfig.displays[0].pixel_height));
    writeSerial(String("Width: ") + String(globalConfig.displays[0].pixel_width));
    bbepWakeUp(&bbep);
    bbepSendCMDSequence(&bbep, bbep.pInitFull);
    if (! (globalConfig.displays[0].transmission_modes & TRANSMISSION_MODE_CLEAR_ON_BOOT)){
    drawBootScreen();
    bbepRefresh(&bbep, REFRESH_FULL);
    waitforrefresh(60);
    bbepSleep(&bbep, 1);  // Put display to sleep before power down
    delay(200);  // Brief delay after sleep command
    }
    pwrmgm(false);
    }
    else{
        writeSerial("No display found");
    }
}

bool waitforrefresh(int timeout){
    for (size_t i = 0; i < (size_t)(timeout * 10); i++){
        delay(100);
        if(i % 5 == 0)writeSerial(".",false);
        if(!bbepIsBusy(&bbep)){ 
            if(i == 0){
                writeSerial("ERROR: Epaper not busy after refresh command - refresh may not have started");
                return false;
            }
            writeSerial(".");
            writeSerial("Refresh took ",false);
            writeSerial((String)((float)i / 10),false);
            writeSerial(" seconds");
            delay(200);
            return true;
        }
    }
    writeSerial("Refresh timed out");
    return false;
}

void connect_callback(uint16_t conn_handle) {
    writeSerial("=== BLE CLIENT CONNECTED ===");
    rebootFlag = 0;  // Clear reboot flag after BLE connection established
    updatemsdata();  // Update advertising data with cleared flag
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;
    writeSerial("=== BLE CLIENT DISCONNECTED ===");
    writeSerial("Disconnect reason: " + String(reason));
    cleanupDirectWriteState(true);
}

String getChipIdHex() {
    #ifdef TARGET_NRF
    uint32_t id1 = NRF_FICR->DEVICEID[0];
    uint32_t id2 = NRF_FICR->DEVICEID[1]; 
    uint32_t last3Bytes = id2 & 0xFFFFFF;
    String hexId = String(last3Bytes, HEX);
    hexId.toUpperCase();
    while (hexId.length() < 6) {
        hexId = "0" + hexId;
    }
    writeSerial("Chip ID: " + String(id1, HEX) + String(id2, HEX));
    writeSerial("Using last 3 bytes: " + hexId);
    return hexId;
    #endif
    #ifdef TARGET_ESP32
    uint64_t macAddress = ESP.getEfuseMac();
    uint32_t chipId = (uint32_t)(macAddress >> 24) & 0xFFFFFF;
    String hexId = String(chipId, HEX);
    hexId.toUpperCase();
    while (hexId.length() < 6) {
        hexId = "0" + hexId;
    }
    writeSerial("Chip ID: " + String(chipId, HEX));
    writeSerial("Using chip ID: " + hexId);
    return hexId;
    #endif
}

// Securely erase config by overwriting with zeros
void secureEraseConfig() {
    writeSerial("=== SECURE ERASE CONFIG ===");
    static uint8_t zeroBuffer[512];
    memset(zeroBuffer, 0, sizeof(zeroBuffer));
    
    #ifdef TARGET_NRF
    if (InternalFS.exists(CONFIG_FILE_PATH)) {
        // Open file and overwrite with zeros
        File file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_WRITE);
        if (file) {
            // Get file size
            size_t fileSize = file.size();
            file.seek(0);
            // Overwrite entire file with zeros
            size_t written = 0;
            while (written < fileSize) {
                size_t toWrite = (fileSize - written < sizeof(zeroBuffer)) ? (fileSize - written) : sizeof(zeroBuffer);
                file.write(zeroBuffer, toWrite);
                written += toWrite;
            }
            file.close();
            writeSerial("Config file securely erased (" + String(written) + " bytes)");
        }
        InternalFS.remove(CONFIG_FILE_PATH);
    }
    #elif defined(TARGET_ESP32)
    if (LittleFS.exists(CONFIG_FILE_PATH)) {
        // Open file and overwrite with zeros
        File file = LittleFS.open(CONFIG_FILE_PATH, FILE_WRITE);
        if (file) {
            // Get file size
            size_t fileSize = file.size();
            file.seek(0);
            // Overwrite entire file with zeros
            size_t written = 0;
            while (written < fileSize) {
                size_t toWrite = (fileSize - written < sizeof(zeroBuffer)) ? (fileSize - written) : sizeof(zeroBuffer);
                file.write(zeroBuffer, toWrite);
                written += toWrite;
            }
            file.close();
            writeSerial("Config file securely erased (" + String(written) + " bytes)");
        }
        LittleFS.remove(CONFIG_FILE_PATH);
    }
    #endif
    writeSerial("Config securely erased");
}

// Check reset pin on boot
void checkResetPin() {
    // Check if reset pin is enabled
    if (!(securityConfig.flags & SECURITY_FLAG_RESET_PIN_ENABLED)) {
        return; // Reset pin disabled
    }
    
    uint8_t pin = securityConfig.reset_pin;
    bool polarity = (securityConfig.flags & SECURITY_FLAG_RESET_PIN_POLARITY) != 0;
    bool pullup = (securityConfig.flags & SECURITY_FLAG_RESET_PIN_PULLUP) != 0;
    bool pulldown = (securityConfig.flags & SECURITY_FLAG_RESET_PIN_PULLDOWN) != 0;
    
    writeSerial("Checking reset pin " + String(pin) + " (polarity: " + String(polarity ? "HIGH" : "LOW") + 
                ", pullup: " + String(pullup) + ", pulldown: " + String(pulldown) + ")");
    
    #ifdef TARGET_ESP32
    pinMode(pin, INPUT);
    if (pullup) {
        pinMode(pin, INPUT_PULLUP);
    } else if (pulldown) {
        pinMode(pin, INPUT_PULLDOWN);
    }
    #elif defined(TARGET_NRF)
    pinMode(pin, INPUT);
    if (pullup) {
        pinMode(pin, INPUT_PULLUP);
    }
    // nRF doesn't have built-in pulldown, would need external resistor
    #endif
    
    // Wait 100ms and check pin state
    delay(100);
    bool pinState = digitalRead(pin);
    
    // Check if pin state matches trigger polarity
    if (pinState == polarity) {
        writeSerial("Reset pin triggered! Securely erasing config and rebooting...");
        secureEraseConfig();
        delay(100);
        reboot();
    } else {
        writeSerial("Reset pin not triggered (state: " + String(pinState ? "HIGH" : "LOW") + ")");
    }
}

void reboot(){
    writeSerial("=== REBOOT COMMAND (0x000F) ===");
    delay(100);
    #ifdef TARGET_NRF
    NVIC_SystemReset();
    #endif
    #ifdef TARGET_ESP32
    esp_restart();
    #endif
}

#ifdef TARGET_NRF
extern "C" void bootloader_util_app_start(uint32_t start_addr);
#endif

// Enter DFU/OTA bootloader mode
void enterDFUMode() {
    writeSerial("=== ENTER DFU MODE COMMAND (0x0051) ===");
    
    #ifdef TARGET_NRF
    writeSerial("Preparing to enter DFU bootloader mode...");
    
    // The Adafruit nRF52 bootloader requires a direct warm jump, not a cold reset.
    // This replicates the exact sequence from BLEDfu.cpp in the Adafruit framework.
    
    // 1. Stop advertising on disconnect so it doesn't restart
    Bluefruit.Advertising.restartOnDisconnect(false);
    
    // 2. Disconnect BLE if connected
    if (Bluefruit.connected()) {
        writeSerial("Disconnecting BLE...");
        Bluefruit.disconnect(Bluefruit.connHandle());
        delay(100);
    }
    
    // 3. Set GPREGRET to DFU OTA magic via SoftDevice API
    sd_power_gpregret_clr(0, 0xFF);
    sd_power_gpregret_set(0, 0xB1); // DFU_OTA_MAGIC
    
    // 4. Disable the SoftDevice
    sd_softdevice_disable();
    
    // 5. Disable all interrupts
    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICPR[0] = 0xFFFFFFFF;
    #if defined(__NRF_NVIC_ISER_COUNT) && __NRF_NVIC_ISER_COUNT == 2
    NVIC->ICER[1] = 0xFFFFFFFF;
    NVIC->ICPR[1] = 0xFFFFFFFF;
    #endif
    
    // 6. Set vector table to bootloader and jump directly to it
    sd_softdevice_vector_table_base_set(NRF_UICR->NRFFW[0]);
    __set_CONTROL(0); // Switch to MSP (required for FreeRTOS)
    bootloader_util_app_start(NRF_UICR->NRFFW[0]);
    
    while(1) {} // Never reached
    #endif
    
    #ifdef TARGET_ESP32
    // For ESP32, we simply reboot - OTA is typically handled via WiFi/HTTP
    // If a bootloader is configured to check for OTA mode, it can be set via NVS
    // For now, we just reboot normally
    writeSerial("ESP32: Rebooting (OTA typically handled via WiFi)");
    delay(100); // Allow logs to flush
    esp_restart();
    #endif
}

// Send unencrypted response (for error messages that need to be readable by old tooling)
void sendResponseUnencrypted(uint8_t* response, uint8_t len) {
    // Skip encryption entirely - just send the response as-is
    writeSerial("Sending unencrypted response (error/status):");
    writeSerial("  Length: " + String(len) + " bytes");
    writeSerial("  Command: 0x" + String(response[0], HEX) + String(response[1], HEX));
    String hexDump = "  Full command: ";
    for (int i = 0; i < len && i < 32; i++) { // Limit hex dump to 32 bytes
        if (i > 0) hexDump += " ";
        if (response[i] < 16) hexDump += "0";
        hexDump += String(response[i], HEX);
    }
    if (len > 32) hexDump += " ...";
    writeSerial(hexDump);
    #ifdef TARGET_ESP32
    if (wifiServerConnected && wifiClient.connected()) {
        uint8_t tcpPacket[1024];
        uint32_t pos = 0;
        uint32_t lengthPos = pos;
        pos += 2;
        tcpPacket[pos++] = 0x01;
        tcpPacket[pos++] = 0x00;
        uint8_t responsePacketId = (len > 1) ? response[1] : 0x00;
        tcpPacket[pos++] = responsePacketId;
        memcpy(&tcpPacket[pos], response, len);
        pos += len;
        uint32_t dataLen = pos - 2;
        uint16_t crc = calculateCRC16CCITT(&tcpPacket[2], dataLen);
        tcpPacket[pos++] = crc & 0xFF;
        tcpPacket[pos++] = (crc >> 8) & 0xFF;
        uint16_t totalLength = pos;
        tcpPacket[lengthPos] = totalLength & 0xFF;
        tcpPacket[lengthPos + 1] = (totalLength >> 8) & 0xFF;
        size_t bytesWritten = wifiClient.write(tcpPacket, pos);
        if (bytesWritten == pos) {
            writeSerial("TCP response sent (" + String(bytesWritten) + " bytes)");
        } else {
            writeSerial("ERROR: TCP response incomplete (expected " + String(pos) + ", wrote " + String(bytesWritten) + ")");
        }
    }
    if (len <= MAX_RESPONSE_SIZE) {
        uint8_t nextHead = (responseQueueHead + 1) % RESPONSE_QUEUE_SIZE;
        if (nextHead != responseQueueTail) {
            memcpy(responseQueue[responseQueueHead].data, response, len);
            responseQueue[responseQueueHead].len = len;
            responseQueue[responseQueueHead].pending = true;
            responseQueueHead = nextHead;
            writeSerial("ESP32: Response queued (queue size: " + String((responseQueueHead - responseQueueTail + RESPONSE_QUEUE_SIZE) % RESPONSE_QUEUE_SIZE) + ")");
        } else {
            writeSerial("ERROR: Response queue full, dropping response");
        }
    } else {
        writeSerial("ERROR: Response too large for queue (" + String(len) + " > " + String(MAX_RESPONSE_SIZE) + ")");
    }
    #endif
    #ifdef TARGET_NRF
    if (Bluefruit.connected() && imageCharacteristic.notifyEnabled()) {
        // Log response details for debugging
        String hexDump = "NRF: Sending unencrypted response: ";
        for (int i = 0; i < len && i < 32; i++) {
            if (i > 0) hexDump += " ";
            if (response[i] < 16) hexDump += "0";
            hexDump += String(response[i], HEX);
        }
        if (len > 32) hexDump += "...";
        writeSerial(hexDump);
        writeSerial("NRF: BLE notification sent (" + String(len) + " bytes)");
        imageCharacteristic.notify(response, len);
        // Increase delay to ensure notification is sent before next operation
        delay(50); // Increased delay to let BLE stack process
    } else {
        writeSerial("ERROR: Cannot send BLE response - not connected or notifications not enabled");
        writeSerial("  Connected: " + String(Bluefruit.connected() ? "yes" : "no"));
        writeSerial("  Notify enabled: " + String(imageCharacteristic.notifyEnabled() ? "yes" : "no"));
    }
    #endif
}

void sendResponse(uint8_t* response, uint8_t len){
    // Encrypted response buffer must be at function scope so it stays valid
    // until the response is actually sent (queued for BLE notification)
    static uint8_t encrypted_response[600];
    
    // Check if response should be encrypted
    if (isAuthenticated() && len >= 2) {
        // Don't encrypt authentication (0x0050), firmware version (0x0043), and error responses (0xFE, 0xFF)
        uint16_t command = (response[0] << 8) | response[1];
        uint8_t status = (len >= 3) ? response[2] : 0x00;
        
        // Skip encryption for:
        // - Authentication command (0x0050)
        // - Firmware version command (0x0043)
        // - Error responses (status 0xFE = auth required, 0xFF = generic error)
        if (command != 0x0050 && command != 0x0043 && status != 0xFE && status != 0xFF) {
            uint8_t nonce[16];
            uint8_t auth_tag[12];
            uint16_t encrypted_len = 0;
            
            if (encryptResponse(response, len, encrypted_response, &encrypted_len, nonce, auth_tag)) {
                writeSerial("Sending encrypted response:");
                writeSerial("  Original length: " + String(len) + " bytes");
                writeSerial("  Encrypted length: " + String(encrypted_len) + " bytes");
                // Send encrypted response instead
                response = encrypted_response;
                len = encrypted_len;
            } else {
                writeSerial("WARNING: Failed to encrypt response, sending unencrypted error response");
                // Send generic error response instead of unencrypted data
                uint8_t errorResponse[] = {0xFF, (uint8_t)(command & 0xFF), 0x00};
                response = errorResponse;
                len = sizeof(errorResponse);
            }
        } else {
            writeSerial("Sending unencrypted response (authentication/firmware version/error)");
        }
    }
    
    writeSerial("Sending response:");
    writeSerial("  Length: " + String(len) + " bytes");
    writeSerial("  Command: 0x" + String(response[0], HEX) + String(response[1], HEX));
    String hexDump = "  Full command: ";
    for (int i = 0; i < len && i < 32; i++) { // Limit hex dump to 32 bytes
        if (i > 0) hexDump += " ";
        if (response[i] < 16) hexDump += "0";
        hexDump += String(response[i], HEX);
    }
    if (len > 32) hexDump += " ...";
    writeSerial(hexDump);
    #ifdef TARGET_ESP32
    if (wifiServerConnected && wifiClient.connected()) {
        uint8_t tcpPacket[1024];
        uint32_t pos = 0;
        uint32_t lengthPos = pos;
        pos += 2;
        tcpPacket[pos++] = 0x01;
        tcpPacket[pos++] = 0x00;
        uint8_t responsePacketId = (len > 1) ? response[1] : 0x00;
        tcpPacket[pos++] = responsePacketId;
        memcpy(&tcpPacket[pos], response, len);
        pos += len;
        uint32_t dataLen = pos - 2;
        uint16_t crc = calculateCRC16CCITT(&tcpPacket[2], dataLen);
        tcpPacket[pos++] = crc & 0xFF;
        tcpPacket[pos++] = (crc >> 8) & 0xFF;
        uint16_t totalLength = pos;
        tcpPacket[lengthPos] = totalLength & 0xFF;
        tcpPacket[lengthPos + 1] = (totalLength >> 8) & 0xFF;
        size_t bytesWritten = wifiClient.write(tcpPacket, pos);
        // Note: flush() is deprecated, TCP sockets send data immediately
        if (bytesWritten == pos) {
            writeSerial("TCP response sent (" + String(bytesWritten) + " bytes)");
        } else {
            writeSerial("ERROR: TCP response incomplete (expected " + String(pos) + ", wrote " + String(bytesWritten) + ")");
        }
    }
    if (len <= MAX_RESPONSE_SIZE) {
        uint8_t nextHead = (responseQueueHead + 1) % RESPONSE_QUEUE_SIZE;
        if (nextHead != responseQueueTail) {
            memcpy(responseQueue[responseQueueHead].data, response, len);
            responseQueue[responseQueueHead].len = len;
            responseQueue[responseQueueHead].pending = true;
            responseQueueHead = nextHead;
            writeSerial("ESP32: Response queued (queue size: " + String((responseQueueHead - responseQueueTail + RESPONSE_QUEUE_SIZE) % RESPONSE_QUEUE_SIZE) + ")");
        } else {
            writeSerial("ERROR: Response queue full, dropping response");
        }
    } else {
        writeSerial("ERROR: Response too large for queue (" + String(len) + " > " + String(MAX_RESPONSE_SIZE) + ")");
    }
    #endif
    #ifdef TARGET_NRF
    if (Bluefruit.connected() && imageCharacteristic.notifyEnabled()) {
        // Log response details for debugging
        String hexDump = "NRF: Sending response: ";
        for (int i = 0; i < len && i < 32; i++) {
            if (i > 0) hexDump += " ";
            if (response[i] < 16) hexDump += "0";
            hexDump += String(response[i], HEX);
        }
        if (len > 32) hexDump += "...";
        writeSerial(hexDump);
        writeSerial("NRF: BLE notification sent (" + String(len) + " bytes)");
        imageCharacteristic.notify(response, len);
        // Increase delay to ensure notification is sent before next operation
        delay(50); // Increased delay to let BLE stack process
    } else {
        writeSerial("ERROR: Cannot send BLE response - not connected or notifications not enabled");
        writeSerial("  Connected: " + String(Bluefruit.connected() ? "yes" : "no"));
        writeSerial("  Notify enabled: " + String(imageCharacteristic.notifyEnabled() ? "yes" : "no"));
    }
    #endif
}

bool initConfigStorage(){
    #ifdef TARGET_NRF
    if (!InternalFS.begin()) {
        writeSerial("ERROR: Failed to mount internal file system");
        return false;
    }
    return true;
    #endif
    #ifdef TARGET_ESP32
    if (!LittleFS.begin(true)) { // true = format on failure
        writeSerial("ERROR: Failed to mount LittleFS");
        return false;
    }
    return true;
    #endif
    return false; // Should never reach here
}

void formatConfigStorage(){
    #ifdef TARGET_NRF
    InternalFS.format();
    #endif
    #ifdef TARGET_ESP32
    LittleFS.format();
    #endif
}

bool saveConfig(uint8_t* configData, uint32_t len){
    if (len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Config data too large (" + String(len) + " bytes)");
        return false;
    }
    static config_storage_t config;
    config.magic = 0xDEADBEEF;
    config.version = 1;
    config.data_len = len;
    config.crc = calculateConfigCRC(configData, len);
    memcpy(config.data, configData, len);
    size_t headerSize = sizeof(config_storage_t) - MAX_CONFIG_SIZE; // Size without data array
    size_t totalSize = headerSize + len; // Header + actual data length
    #ifdef TARGET_NRF
    if (InternalFS.exists(CONFIG_FILE_PATH)) {
        InternalFS.remove(CONFIG_FILE_PATH);
    }
    File file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_WRITE);
    #elif defined(TARGET_ESP32)
    if (LittleFS.exists(CONFIG_FILE_PATH)) {
        LittleFS.remove(CONFIG_FILE_PATH);
    }
    File file = LittleFS.open(CONFIG_FILE_PATH, FILE_WRITE);
    #endif
    if (!file) {
        writeSerial("ERROR: Failed to open config file for writing");
        #ifdef TARGET_NRF
        file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_WRITE);
        #elif defined(TARGET_ESP32)
        file = LittleFS.open(CONFIG_FILE_PATH, FILE_WRITE);
        #endif
        if (!file) {
            writeSerial("ERROR: Failed to open config file for writing with CREATE|WRITE");
        return false;
        }
    }
    size_t bytesWritten = file.write((uint8_t*)&config, totalSize);
    file.close();
    if (bytesWritten != totalSize) {
        writeSerial("ERROR: Failed to write complete config data (expected " + String(totalSize) + ", wrote " + String(bytesWritten) + ")");
        return false;
    }
    return true;
}

bool loadConfig(uint8_t* configData, uint32_t* len){
    #ifdef TARGET_NRF
    File file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_READ);
    #elif defined(TARGET_ESP32)
    File file = LittleFS.open(CONFIG_FILE_PATH, FILE_READ);
    #endif
    if (!file) {
        return false;
    }
    static config_storage_t config;
    static size_t bytesRead;
    static size_t headerSize = sizeof(config_storage_t) - MAX_CONFIG_SIZE; // Size without data array
    bytesRead = file.read((uint8_t*)&config, headerSize);
    if (bytesRead != headerSize) {
        writeSerial("ERROR: Failed to read config header (expected " + String(headerSize) + ", got " + String(bytesRead) + ")");
        file.close();
        return false;
    }
    if (config.magic != 0xDEADBEEF) {
        writeSerial("ERROR: Invalid config magic number");
        file.close();
        return false;
    }
    if (config.data_len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Config data too large");
        file.close();
        return false;
    }
    bytesRead = file.read(config.data, config.data_len);
    file.flush();
    file.close();
    if (bytesRead != config.data_len) {
        writeSerial("ERROR: Failed to read complete config data (expected " + String(config.data_len) + ", read " + String(bytesRead) + ")");
        return false;
    }
    uint32_t calculatedCRC = calculateConfigCRC(config.data, config.data_len);
    if (config.crc != calculatedCRC) {
        writeSerial("ERROR: Config CRC mismatch");
        return false;
    }
    if (config.data_len > *len) {
        writeSerial("ERROR: Config data larger than buffer");
        return false;
    }
    for (uint32_t i = 0; i < config.data_len && i < *len; i++) {
        configData[i] = config.data[i];
    }
    *len = config.data_len;
    return true;
}

uint32_t calculateConfigCRC(uint8_t* data, uint32_t len){
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return ~crc;
}

uint16_t calculateCRC16CCITT(uint8_t* data, uint32_t len){
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
            crc &= 0xFFFF;
        }
    }
    return crc;
}

void handleReadConfig(){
    uint8_t configData[MAX_CONFIG_SIZE];
    uint32_t configLen = MAX_CONFIG_SIZE;
    if (loadConfig(configData, &configLen)) {
        writeSerial("Sending config data in chunks...");
        uint32_t remaining = configLen;
        uint32_t offset = 0;
        uint16_t chunkNumber = 0;
        const uint16_t maxChunks = 10;
        while (remaining > 0 && chunkNumber < maxChunks) {
            uint16_t responseLen = 0;
            configReadResponseBuffer[responseLen++] = 0x00; // Response type
            configReadResponseBuffer[responseLen++] = 0x40; // Command echo
            configReadResponseBuffer[responseLen++] = chunkNumber & 0xFF;
            configReadResponseBuffer[responseLen++] = (chunkNumber >> 8) & 0xFF;
            if (chunkNumber == 0) {
                configReadResponseBuffer[responseLen++] = configLen & 0xFF;
                configReadResponseBuffer[responseLen++] = (configLen >> 8) & 0xFF;
            }
            uint16_t maxDataSize = MAX_RESPONSE_DATA_SIZE - responseLen;
            uint16_t chunkSize = (remaining < maxDataSize) ? remaining : maxDataSize;
            if (chunkSize == 0) {
                writeSerial("ERROR: Chunk size is 0, breaking");
                break;
            }
            memcpy(configReadResponseBuffer + responseLen, configData + offset, chunkSize);
            responseLen += chunkSize;
            if (responseLen > 100) {
                writeSerial("ERROR: Response too large (" + String(responseLen) + " bytes), skipping");
                break;
            }
            if (responseLen == 0) {
                writeSerial("ERROR: Empty response, skipping");
                break;
            }
            sendResponse(configReadResponseBuffer, responseLen);
            offset += chunkSize;
            remaining -= chunkSize;
            chunkNumber++;
            writeSerial("Sent chunk " + String(chunkNumber) + " (" + String(chunkSize) + " bytes)");
            delay(50);
        }
        if (chunkNumber >= maxChunks) {
            writeSerial("WARNING: Hit chunk limit, transmission may be incomplete");
        }
        writeSerial("Config read response sent (" + String(configLen) + " bytes) in " + String(chunkNumber) + " chunks");
    } else {
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_READ, 0x00, 0x00}; // Error, command, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        writeSerial("Config read failed - sent error response");
    }
    writeSerial("About to return from handleReadConfig");
    delay(100);
    writeSerial("handleReadConfig function completed successfully");
}

void handleReadMSD() {
    writeSerial("=== READ MSD COMMAND (0x0044) ===");
    uint8_t response[2 + 16];  // Response type + command echo + 16 bytes of MSD data
    uint16_t responseLen = 0;
    response[responseLen++] = 0x00;  // Success
    response[responseLen++] = RESP_MSD_READ;  // Command echo
    memcpy(&response[responseLen], msd_payload, sizeof(msd_payload));
    responseLen += sizeof(msd_payload);
    sendResponse(response, responseLen);
    writeSerial("MSD read response sent (" + String(responseLen) + " bytes)");
    String hexDump = "MSD payload: ";
    for (int i = 0; i < 16; i++) {
        if (i > 0) hexDump += " ";
        if (msd_payload[i] < 16) hexDump += "0";
        hexDump += String(msd_payload[i], HEX);
    }
    writeSerial(hexDump);
}

void handleFirmwareVersion(){
    writeSerial("Building Firmware Version response...");
    uint8_t major = getFirmwareMajor();
    uint8_t minor = getFirmwareMinor();
    const char* shaCStr = SHA_STRING;
    String shaStr = String(shaCStr);
    if (shaStr.length() >= 2 && shaStr.charAt(0) == '"' && shaStr.charAt(shaStr.length() - 1) == '"') {
        shaStr = shaStr.substring(1, shaStr.length() - 1);
    }
    writeSerial("Firmware version: " + String(major) + "." + String(minor));
    writeSerial("SHA: " + shaStr);
    uint8_t shaLen = shaStr.length();
    if (shaLen > 40) shaLen = 40;
    uint8_t response[2 + 1 + 1 + 1 + 40];
    uint16_t offset = 0;
    response[offset++] = 0x00;
    response[offset++] = 0x43;
    response[offset++] = major;
    response[offset++] = minor;
    response[offset++] = shaLen;
    for (uint8_t i = 0; i < shaLen && i < 40; i++) {
        response[offset++] = shaStr.charAt(i);
    }
    sendResponse(response, offset);
    writeSerial("Firmware version response sent");
}

void handleWriteConfig(uint8_t* data, uint16_t len){
    if (len == 0) {
        writeSerial("ERROR: No config data received");
        return;
    }
    
    // Check if encryption is enabled and rewrite is not allowed
    if (isEncryptionEnabled() && !isAuthenticated()) {
        // Check if rewrite_allowed flag is set
        bool rewriteAllowed = (securityConfig.flags & SECURITY_FLAG_REWRITE_ALLOWED) != 0;
        if (!rewriteAllowed) {
            writeSerial("ERROR: Config write requires authentication (encryption enabled)");
            uint8_t response[] = {0x00, (uint8_t)(0x0041 & 0xFF), 0xFE}; // Auth required
            sendResponseUnencrypted(response, sizeof(response));
            return;
        } else {
            // Rewrite allowed - securely erase old config before writing new one
            writeSerial("Rewrite allowed: Securely erasing old config before write...");
            secureEraseConfig();
        }
    }
    if (len > CONFIG_CHUNK_SIZE) {
        writeSerial("Starting chunked write (received: " + String(len) + " bytes)");
        chunkedWriteState.active = true;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.expectedChunks = 0;
        chunkedWriteState.receivedChunks = 0;
        if (len >= CONFIG_CHUNK_SIZE_WITH_PREFIX) {
            chunkedWriteState.totalSize = data[0] | (data[1] << 8);
            chunkedWriteState.expectedChunks = (chunkedWriteState.totalSize + CONFIG_CHUNK_SIZE - 1) / CONFIG_CHUNK_SIZE;
            uint16_t chunkDataSize = ((len - 2) < CONFIG_CHUNK_SIZE) ? (len - 2) : CONFIG_CHUNK_SIZE;
            memcpy(chunkedWriteState.buffer, data + 2, chunkDataSize);
            chunkedWriteState.receivedSize = chunkDataSize;
            chunkedWriteState.receivedChunks = 1;
            writeSerial("First chunk received: " + String(chunkDataSize) + " bytes, total size: " + String(chunkedWriteState.totalSize) + " bytes, expected chunks: " + String(chunkedWriteState.expectedChunks));
        } else {
            chunkedWriteState.totalSize = len;
            chunkedWriteState.expectedChunks = 1;
            uint16_t chunkSize = (len < CONFIG_CHUNK_SIZE) ? len : CONFIG_CHUNK_SIZE;
            memcpy(chunkedWriteState.buffer, data, chunkSize);
            chunkedWriteState.receivedSize = chunkSize;
            chunkedWriteState.receivedChunks = 1;
            writeSerial("Large single transmission: " + String(chunkSize) + " bytes");
        }
        uint8_t ackResponse[] = {0x00, RESP_CONFIG_WRITE, 0x00, 0x00}; // Success, command, chunk received
        sendResponse(ackResponse, sizeof(ackResponse));
        return;
    }
    if (saveConfig(data, len)) {
        uint8_t successResponse[] = {0x00, RESP_CONFIG_WRITE, 0x00, 0x00}; // Success, command, no data
        sendResponse(successResponse, sizeof(successResponse));
        writeSerial("Config write successful");
    } else {
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_WRITE, 0x00, 0x00}; // Error, command, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        writeSerial("Config write failed");
    }
}

void handleWriteConfigChunk(uint8_t* data, uint16_t len){
    if (!chunkedWriteState.active) {
        writeSerial("ERROR: No chunked write in progress");
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_CHUNK, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    
    // Check if encryption is enabled and rewrite is not allowed (only check on first chunk)
    if (chunkedWriteState.receivedChunks == 1 && isEncryptionEnabled() && !isAuthenticated()) {
        // Check if rewrite_allowed flag is set
        bool rewriteAllowed = (securityConfig.flags & SECURITY_FLAG_REWRITE_ALLOWED) != 0;
        if (!rewriteAllowed) {
            writeSerial("ERROR: Config write requires authentication (encryption enabled)");
            chunkedWriteState.active = false;
            uint8_t response[] = {0x00, (uint8_t)(0x0042 & 0xFF), 0xFE}; // Auth required
            sendResponseUnencrypted(response, sizeof(response));
            return;
        } else {
            // Rewrite allowed - securely erase old config before writing new one (only once on first chunk)
            writeSerial("Rewrite allowed: Securely erasing old config before write...");
            secureEraseConfig();
        }
    }
    if (len == 0) {
        writeSerial("ERROR: No chunk data received");
        return;
    }
    if (len > CONFIG_CHUNK_SIZE) {
        writeSerial("ERROR: Chunk too large (" + String(len) + " bytes)");
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_CHUNK, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    if (chunkedWriteState.receivedSize + len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Chunk would exceed max config size");
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_CHUNK, 0x00, 0x00}; // Error, command, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    if (chunkedWriteState.receivedChunks >= MAX_CONFIG_CHUNKS) {
        writeSerial("ERROR: Too many chunks received");
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, RESP_CONFIG_CHUNK, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    memcpy(chunkedWriteState.buffer + chunkedWriteState.receivedSize, data, len);
    chunkedWriteState.receivedSize += len;
    chunkedWriteState.receivedChunks++;
    writeSerial("Chunk " + String(chunkedWriteState.receivedChunks) + "/" + String(chunkedWriteState.expectedChunks) + " received (" + String(len) + " bytes)");
    if (chunkedWriteState.receivedChunks >= chunkedWriteState.expectedChunks) {
        writeSerial("All chunks received, saving config (" + String(chunkedWriteState.receivedSize) + " bytes)");
        if (saveConfig(chunkedWriteState.buffer, chunkedWriteState.receivedSize)) {
            uint8_t successResponse[] = {0x00, RESP_CONFIG_CHUNK, 0x00, 0x00}; // Success, command, no data
            sendResponse(successResponse, sizeof(successResponse));
            writeSerial("Chunked config write successful");
    } else {
            uint8_t errorResponse[] = {0xFF, RESP_CONFIG_CHUNK, 0x00, 0x00}; // Error, command, no data
            sendResponse(errorResponse, sizeof(errorResponse));
            writeSerial("Chunked config write failed");
        }
        chunkedWriteState.active = false;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.receivedChunks = 0;
    } else {
        uint8_t ackResponse[] = {0x00, RESP_CONFIG_CHUNK, 0x00, 0x00}; // Success, command, chunk received
        sendResponse(ackResponse, sizeof(ackResponse));
    }
}

bool loadGlobalConfig(){
    memset(&globalConfig, 0, sizeof(globalConfig));
    // Initialize security config defaults
    memset(&securityConfig, 0, sizeof(securityConfig));
    // Reset pin defaults to disabled (flag not set)
    wifiConfigured = false;
    wifiSsid[0] = '\0';
    wifiPassword[0] = '\0';
    wifiEncryptionType = 0;
    static uint8_t configData[MAX_CONFIG_SIZE];
    static uint32_t configLen = MAX_CONFIG_SIZE;
    if (!loadConfig(configData, &configLen)) {
        globalConfig.loaded = false;
        return false;
    }
    if (configLen < 3) {
        writeSerial("ERROR: Config too short");
        globalConfig.loaded = false;
        return false;
    }
    uint32_t offset = 0;
    offset += 2;
    globalConfig.version = configData[offset++];
    globalConfig.minor_version = 0; // Not stored in current format
    while (offset < configLen - 2) { // -2 for CRC
        if (offset + 2 > configLen - 2) break;
        offset++;
        uint8_t packetId = configData[offset++];
        switch (packetId) {
            case 0x01: // system_config
                if (offset + sizeof(struct SystemConfig) <= configLen - 2) {
                    memcpy(&globalConfig.system_config, &configData[offset], sizeof(struct SystemConfig));
                    offset += sizeof(struct SystemConfig);
                } else {
                    writeSerial("ERROR: Not enough data for system_config");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x02: // manufacturer_data
                if (offset + sizeof(struct ManufacturerData) <= configLen - 2) {
                    memcpy(&globalConfig.manufacturer_data, &configData[offset], sizeof(struct ManufacturerData));
                    offset += sizeof(struct ManufacturerData);
                } else {
                    writeSerial("ERROR: Not enough data for manufacturer_data");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x04: // power_option
                if (offset + sizeof(struct PowerOption) <= configLen - 2) {
                    memcpy(&globalConfig.power_option, &configData[offset], sizeof(struct PowerOption));
                    offset += sizeof(struct PowerOption);
                } else {
                    writeSerial("ERROR: Not enough data for power_option");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x20: // display
                if (globalConfig.display_count < 4 && offset + sizeof(struct DisplayConfig) <= configLen - 2) {
                    memcpy(&globalConfig.displays[globalConfig.display_count], &configData[offset], sizeof(struct DisplayConfig));
                    offset += sizeof(struct DisplayConfig);
                    globalConfig.display_count++;
                } else if (globalConfig.display_count >= 4) {
                    writeSerial("WARNING: Maximum display count reached, skipping");
                    offset += sizeof(struct DisplayConfig);
                } else {
                    writeSerial("ERROR: Not enough data for display");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x21: // led
                if (globalConfig.led_count < 4 && offset + sizeof(struct LedConfig) <= configLen - 2) {
                    memcpy(&globalConfig.leds[globalConfig.led_count], &configData[offset], sizeof(struct LedConfig));
                    offset += sizeof(struct LedConfig);
                    globalConfig.led_count++;
                    // Reset active LED instance to re-detect RGB LEDs after config change
                    activeLedInstance = 0xFF;
                } else if (globalConfig.led_count >= 4) {
                    writeSerial("WARNING: Maximum LED count reached, skipping");
                    offset += sizeof(struct LedConfig);
                } else {
                    writeSerial("ERROR: Not enough data for LED");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x23: // sensor_data
                if (globalConfig.sensor_count < 4 && offset + sizeof(struct SensorData) <= configLen - 2) {
                    memcpy(&globalConfig.sensors[globalConfig.sensor_count], &configData[offset], sizeof(struct SensorData));
                    offset += sizeof(struct SensorData);
                    globalConfig.sensor_count++;
                } else if (globalConfig.sensor_count >= 4) {
                    writeSerial("WARNING: Maximum sensor count reached, skipping");
                    offset += sizeof(struct SensorData);
                } else {
                    writeSerial("ERROR: Not enough data for sensor");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x24: // data_bus
                if (globalConfig.data_bus_count < 4 && offset + sizeof(struct DataBus) <= configLen - 2) {
                    memcpy(&globalConfig.data_buses[globalConfig.data_bus_count], &configData[offset], sizeof(struct DataBus));
                    offset += sizeof(struct DataBus);
                    globalConfig.data_bus_count++;
                } else if (globalConfig.data_bus_count >= 4) {
                    writeSerial("WARNING: Maximum data_bus count reached, skipping");
                    offset += sizeof(struct DataBus);
                } else {
                    writeSerial("ERROR: Not enough data for data_bus");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x25: // binary_inputs
                if (globalConfig.binary_input_count < 4 && offset + sizeof(struct BinaryInputs) <= configLen - 2) {
                    memcpy(&globalConfig.binary_inputs[globalConfig.binary_input_count], &configData[offset], sizeof(struct BinaryInputs));
                    offset += sizeof(struct BinaryInputs);
                    globalConfig.binary_input_count++;
                } else if (globalConfig.binary_input_count >= 4) {
                    writeSerial("WARNING: Maximum binary_input count reached, skipping");
                    offset += sizeof(struct BinaryInputs);
                } else {
                    writeSerial("ERROR: Not enough data for binary_input");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x26: // wifi_config
                {
                    const uint16_t WIFI_CONFIG_SIZE = 162;
                    if (offset + WIFI_CONFIG_SIZE <= configLen) {
                        memcpy(wifiSsid, &configData[offset], 32);
                        wifiSsid[32] = '\0';  // Ensure null termination
                        uint8_t ssidLen = 0;
                        while (ssidLen < 32 && wifiSsid[ssidLen] != '\0') ssidLen++;
                        offset += 32;
                        memcpy(wifiPassword, &configData[offset], 32);
                        wifiPassword[32] = '\0';  // Ensure null termination
                        uint8_t passwordLen = 0;
                        while (passwordLen < 32 && wifiPassword[passwordLen] != '\0') passwordLen++;
                        offset += 32;
                        wifiEncryptionType = configData[offset++];
                        #ifdef TARGET_ESP32
                        // Parse server configuration from reserved bytes
                        // First, read as string (like SSID)
                        memcpy(wifiServerUrl, &configData[offset], 64);
                        wifiServerUrl[64] = '\0';  // Ensure null termination
                        
                        // Check if it's stored as a string (has null terminator in first few bytes)
                        // or as a 4-byte IP address (numeric format from config tool)
                        bool isStringFormat = false;
                        for (int i = 0; i < 64; i++) {
                            if (wifiServerUrl[i] == '\0') {
                                isStringFormat = true;
                                break;
                            }
                            // If we find a non-printable character (except null), it's likely binary
                            if (i > 0 && wifiServerUrl[i] < 32 && wifiServerUrl[i] != '\0') {
                                break;
                            }
                        }
                        
                        // If first 4 bytes look like an IP address in numeric format (little-endian)
                        // and there's no null terminator in first 5 bytes, convert to IP string
                        // Check if bytes 0-3 are non-zero and byte 4 is null (indicating 4-byte format)
                        if (!isStringFormat && wifiServerUrl[4] == '\0' && 
                            (wifiServerUrl[0] != 0 || wifiServerUrl[1] != 0 || 
                             wifiServerUrl[2] != 0 || wifiServerUrl[3] != 0)) {
                            // The config tool stores IP as 32-bit integer in little-endian format
                            // Bytes are: [byte0][byte1][byte2][byte3] = IP address
                            // Convert 4-byte IP (stored as little-endian) to string format
                            uint8_t ip[4];
                            ip[0] = configData[offset];
                            ip[1] = configData[offset + 1];
                            ip[2] = configData[offset + 2];
                            ip[3] = configData[offset + 3];
                            snprintf(wifiServerUrl, 65, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
                            writeSerial("Converted numeric IP to string: \"" + String(wifiServerUrl) + "\"");
                        } else if (!isStringFormat && wifiServerUrl[0] != '\0') {
                            // Try to interpret as 32-bit integer (little-endian) and convert to IP
                            uint32_t ipNum = (uint32_t)configData[offset] | 
                                            ((uint32_t)configData[offset + 1] << 8) |
                                            ((uint32_t)configData[offset + 2] << 16) |
                                            ((uint32_t)configData[offset + 3] << 24);
                            // Convert to IP string (interpret as big-endian IP address)
                            uint8_t ip[4];
                            ip[0] = (ipNum >> 24) & 0xFF;
                            ip[1] = (ipNum >> 16) & 0xFF;
                            ip[2] = (ipNum >> 8) & 0xFF;
                            ip[3] = ipNum & 0xFF;
                            snprintf(wifiServerUrl, 65, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
                            writeSerial("Converted 32-bit integer to IP string: \"" + String(wifiServerUrl) + "\"");
                        }
                        
                        offset += 64;
                        // Read port (2 bytes, network byte order)
                        wifiServerPort = (configData[offset] << 8) | configData[offset + 1];
                        offset += 2;
                        
                        // Check if server is configured (URL not empty and not "0.0.0.0")
                        wifiServerConfigured = (wifiServerUrl[0] != '\0' && 
                                               strcmp(wifiServerUrl, "0.0.0.0") != 0);
                        if (wifiServerConfigured) {
                            writeSerial("Server configured: YES");
                            writeSerial("Server URL: \"" + String(wifiServerUrl) + "\"");
                            writeSerial("Server Port: " + String(wifiServerPort));
                        } else {
                            writeSerial("Server configured: NO");
                            if (wifiServerUrl[0] == '\0') {
                                writeSerial("Reason: URL is empty");
                            } else if (strcmp(wifiServerUrl, "0.0.0.0") == 0) {
                                writeSerial("Reason: URL is \"0.0.0.0\"");
                            }
                        }
                        offset += 29;  // Skip remaining reserved bytes
                        #else
                        offset += 95;  // Skip all reserved bytes on non-ESP32
                        #endif
                        wifiConfigured = true;
                        writeSerial("=== WiFi Configuration Loaded ===");
                        writeSerial("SSID: \"" + String(wifiSsid) + "\"");
                        if (passwordLen > 0) {
                            writeSerial("Password: \"" + String(wifiPassword) + "\"");
                        } else {
                            writeSerial("Password: (empty)");
                        }
                        String encTypeStr = "Unknown";
                        switch(wifiEncryptionType) {
                            case 0x00: encTypeStr = "None (Open)"; break;
                            case 0x01: encTypeStr = "WEP"; break;
                            case 0x02: encTypeStr = "WPA"; break;
                            case 0x03: encTypeStr = "WPA2"; break;
                            case 0x04: encTypeStr = "WPA3"; break;
                        }
                        writeSerial("Encryption Type: 0x" + String(wifiEncryptionType, HEX) + " (" + encTypeStr + ")");
                        writeSerial("SSID length: " + String(ssidLen) + " bytes");
                        writeSerial("Password length: " + String(passwordLen) + " bytes");
                        writeSerial("WiFi configured: true");
                    } else {
                        writeSerial("ERROR: Not enough data for wifi_config");
                        globalConfig.loaded = false;
                        return false;
                    }
                }
                break;
            case 0x27: // security_config
                {
                    if (offset + sizeof(struct SecurityConfig) <= configLen) {
                        memcpy(&securityConfig, &configData[offset], sizeof(struct SecurityConfig));
                        offset += sizeof(struct SecurityConfig);
                        // Check if key is all zeros (encryption disabled)
                        bool keyIsZero = true;
                        for (int i = 0; i < 16; i++) {
                            if (securityConfig.encryption_key[i] != 0) {
                                keyIsZero = false;
                                break;
                            }
                        }
                        if (keyIsZero) {
                            securityConfig.encryption_enabled = 0;
                            writeSerial("Security config: Encryption disabled (key is all zeros)");
                        } else if (securityConfig.encryption_enabled) {
                            writeSerial("Security config: Encryption enabled");
                            writeSerial("Session timeout: " + String(securityConfig.session_timeout_seconds) + " seconds");
                        } else {
                            writeSerial("Security config: Encryption disabled (flag set to 0)");
                        }
                        // Log security flags
                        if (securityConfig.flags & SECURITY_FLAG_REWRITE_ALLOWED) {
                            writeSerial("Security config: Rewrite allowed (unauthorized config writes permitted)");
                        }
                        if (securityConfig.flags & SECURITY_FLAG_SHOW_KEY_ON_SCREEN) {
                            writeSerial("Security config: Show key on screen enabled (future feature)");
                        }
                        if (securityConfig.flags & SECURITY_FLAG_RESET_PIN_ENABLED) {
                            writeSerial("Security config: Reset pin " + String(securityConfig.reset_pin) + 
                                       " enabled (polarity: " + String((securityConfig.flags & SECURITY_FLAG_RESET_PIN_POLARITY) ? "HIGH" : "LOW") + 
                                       ", pullup: " + String((securityConfig.flags & SECURITY_FLAG_RESET_PIN_PULLUP) ? "yes" : "no") + 
                                       ", pulldown: " + String((securityConfig.flags & SECURITY_FLAG_RESET_PIN_PULLDOWN) ? "yes" : "no") + ")");
                        } else {
                            writeSerial("Security config: Reset pin disabled");
                        }
                    } else {
                        writeSerial("ERROR: Not enough data for security_config");
                        globalConfig.loaded = false;
                        return false;
                    }
                }
                break;
            default:
                writeSerial("WARNING: Unknown packet ID 0x" + String(packetId, HEX) + ", skipping");
                offset = configLen - 2; // Skip to CRC
                break;
        }
    }
    if (offset < configLen - 2) {
        uint16_t crcGiven = configData[configLen - 2] | (configData[configLen - 1] << 8);
        uint32_t crcCalculated32 = calculateConfigCRC(configData, configLen - 2);
        uint16_t crcCalculated = (uint16_t)(crcCalculated32 & 0xFFFF);  // Use lower 16 bits for backwards compatibility
        if (crcGiven != crcCalculated) {
            writeSerial("WARNING: Config CRC mismatch (given: 0x" + String(crcGiven, HEX) + 
                       ", calculated: 0x" + String(crcCalculated, HEX) + ")");
        }
    }
    globalConfig.loaded = true;
    return true;
}

void printConfigSummary(){
    if (!globalConfig.loaded) {
        writeSerial("Config not loaded");
        return;
    }
    writeSerial("=== Configuration Summary ===");
    writeSerial("Version: " + String(globalConfig.version) + "." + String(globalConfig.minor_version));
    writeSerial("Loaded: " + String(globalConfig.loaded ? "Yes" : "No"));
    writeSerial("");
    writeSerial("--- System Configuration ---");
    writeSerial("IC Type: 0x" + String(globalConfig.system_config.ic_type, HEX));
    writeSerial("Communication Modes: 0x" + String(globalConfig.system_config.communication_modes, HEX));
    writeSerial("  BLE: " + String((globalConfig.system_config.communication_modes & COMM_MODE_BLE) ? "enabled" : "disabled"));
    writeSerial("  OEPL: " + String((globalConfig.system_config.communication_modes & COMM_MODE_OEPL) ? "enabled" : "disabled"));
    writeSerial("  WiFi: " + String((globalConfig.system_config.communication_modes & COMM_MODE_WIFI) ? "enabled" : "disabled"));
    #ifdef TARGET_ESP32
    if (globalConfig.system_config.communication_modes & COMM_MODE_WIFI) {
        if (wifiConfigured) {
            writeSerial("  WiFi SSID: \"" + String(wifiSsid) + "\"");
            if (wifiInitialized) {
                if (wifiConnected) {
                    writeSerial("  WiFi Status: Connected (IP: " + WiFi.localIP().toString() + ")");
                } else {
                    writeSerial("  WiFi Status: Disconnected");
                }
            } else {
                writeSerial("  WiFi Status: Not initialized");
            }
        } else {
            writeSerial("  WiFi Status: Configured but not loaded");
        }
    }
    #endif
    writeSerial("Device Flags: 0x" + String(globalConfig.system_config.device_flags, HEX));
    writeSerial("  PWR_PIN flag: " + String((globalConfig.system_config.device_flags & DEVICE_FLAG_PWR_PIN) ? "enabled" : "disabled"));
    #ifdef TARGET_NRF
    writeSerial("  XIAOINIT flag: " + String((globalConfig.system_config.device_flags & DEVICE_FLAG_XIAOINIT) ? "enabled" : "disabled"));
    #endif
    writeSerial("  WS_PP_INIT flag: " + String((globalConfig.system_config.device_flags & DEVICE_FLAG_WS_PP_INIT) ? "enabled" : "disabled"));
    writeSerial("Power Pin: " + String(globalConfig.system_config.pwr_pin));
    writeSerial("");
    writeSerial("--- Manufacturer Data ---");
    writeSerial("Manufacturer ID: 0x" + String(globalConfig.manufacturer_data.manufacturer_id, HEX));
    writeSerial("Board Type: " + String(globalConfig.manufacturer_data.board_type));
    writeSerial("Board Revision: " + String(globalConfig.manufacturer_data.board_revision));
    writeSerial("");
    writeSerial("--- Power Configuration ---");
    writeSerial("Power Mode: " + String(globalConfig.power_option.power_mode));
    writeSerial("Battery Capacity: " + String(globalConfig.power_option.battery_capacity_mah[0]) + 
               " " + String(globalConfig.power_option.battery_capacity_mah[1]) + 
               " " + String(globalConfig.power_option.battery_capacity_mah[2]) + " mAh");
    writeSerial("Awake Timeout: " + String(globalConfig.power_option.sleep_timeout_ms) + " ms");
    writeSerial("Deep Sleep Time: " + String(globalConfig.power_option.deep_sleep_time_seconds) + " seconds");
    writeSerial("TX Power: " + String(globalConfig.power_option.tx_power));
    writeSerial("Sleep Flags: 0x" + String(globalConfig.power_option.sleep_flags, HEX));
    writeSerial("Battery Sense Pin: " + String(globalConfig.power_option.battery_sense_pin));
    writeSerial("Battery Sense Enable Pin: " + String(globalConfig.power_option.battery_sense_enable_pin));
    writeSerial("Battery Sense Flags: 0x" + String(globalConfig.power_option.battery_sense_flags, HEX));
    writeSerial("Capacity Estimator: " + String(globalConfig.power_option.capacity_estimator));
    writeSerial("Voltage Scaling Factor: " + String(globalConfig.power_option.voltage_scaling_factor));
    writeSerial("Deep Sleep Current: " + String(globalConfig.power_option.deep_sleep_current_ua) + " uA");
    writeSerial("");
    writeSerial("--- Display Configurations (" + String(globalConfig.display_count) + ") ---");
    for (int i = 0; i < globalConfig.display_count; i++) {
        writeSerial("Display " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.displays[i].instance_number));
        writeSerial("  Technology: 0x" + String(globalConfig.displays[i].display_technology, HEX));
        writeSerial("  Panel IC Type: 0x" + String(globalConfig.displays[i].panel_ic_type, HEX));
        writeSerial("  Resolution: " + String(globalConfig.displays[i].pixel_width) + "x" + String(globalConfig.displays[i].pixel_height));
        writeSerial("  Size: " + String(globalConfig.displays[i].active_width_mm) + "x" + String(globalConfig.displays[i].active_height_mm) + " mm");
        writeSerial("  Tag Type: 0x" + String(globalConfig.displays[i].tag_type, HEX));
        writeSerial("  Rotation: " + String(globalConfig.displays[i].rotation * 90) + " degrees");
        writeSerial("  Reset Pin: " + String(globalConfig.displays[i].reset_pin));
        writeSerial("  Busy Pin: " + String(globalConfig.displays[i].busy_pin));
        writeSerial("  DC Pin: " + String(globalConfig.displays[i].dc_pin));
        writeSerial("  CS Pin: " + String(globalConfig.displays[i].cs_pin));
        writeSerial("  Data Pin: " + String(globalConfig.displays[i].data_pin));
        writeSerial("  Partial Update: " + String(globalConfig.displays[i].partial_update_support ? "Yes" : "No"));
        writeSerial("  Color Scheme: 0x" + String(globalConfig.displays[i].color_scheme, HEX));
        writeSerial("  Transmission Modes: 0x" + String(globalConfig.displays[i].transmission_modes, HEX));
        writeSerial("    RAW: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_RAW) ? "enabled" : "disabled"));
        writeSerial("    ZIP: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_ZIP) ? "enabled" : "disabled"));
        writeSerial("    G5: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_G5) ? "enabled" : "disabled"));
        writeSerial("    DIRECT_WRITE: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_DIRECT_WRITE) ? "enabled" : "disabled"));
        writeSerial("    CLEAR_ON_BOOT: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_CLEAR_ON_BOOT) ? "enabled" : "disabled"));
        writeSerial("");
    }
    writeSerial("--- LED Configurations (" + String(globalConfig.led_count) + ") ---");
    for (int i = 0; i < globalConfig.led_count; i++) {
        writeSerial("LED " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.leds[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.leds[i].led_type, HEX));
        writeSerial("  Pins: R=" + String(globalConfig.leds[i].led_1_r) + 
                   " G=" + String(globalConfig.leds[i].led_2_g) + 
                   " B=" + String(globalConfig.leds[i].led_3_b) + 
                   " 4=" + String(globalConfig.leds[i].led_4));
        writeSerial("  Flags: 0x" + String(globalConfig.leds[i].led_flags, HEX));
        writeSerial("");
    }
    writeSerial("--- Sensor Configurations (" + String(globalConfig.sensor_count) + ") ---");
    for (int i = 0; i < globalConfig.sensor_count; i++) {
        writeSerial("Sensor " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.sensors[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.sensors[i].sensor_type, HEX));
        writeSerial("  Bus ID: " + String(globalConfig.sensors[i].bus_id));
        writeSerial("");
    }
    writeSerial("--- Data Bus Configurations (" + String(globalConfig.data_bus_count) + ") ---");
    for (int i = 0; i < globalConfig.data_bus_count; i++) {
        writeSerial("Data Bus " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.data_buses[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.data_buses[i].bus_type, HEX));
        writeSerial("  Pins: 1=" + String(globalConfig.data_buses[i].pin_1) + 
                   " 2=" + String(globalConfig.data_buses[i].pin_2) + 
                   " 3=" + String(globalConfig.data_buses[i].pin_3) + 
                   " 4=" + String(globalConfig.data_buses[i].pin_4) + 
                   " 5=" + String(globalConfig.data_buses[i].pin_5) + 
                   " 6=" + String(globalConfig.data_buses[i].pin_6) + 
                   " 7=" + String(globalConfig.data_buses[i].pin_7));
        writeSerial("  Speed: " + String(globalConfig.data_buses[i].bus_speed_hz) + " Hz");
        writeSerial("  Flags: 0x" + String(globalConfig.data_buses[i].bus_flags, HEX));
        writeSerial("  Pullups: 0x" + String(globalConfig.data_buses[i].pullups, HEX));
        writeSerial("  Pulldowns: 0x" + String(globalConfig.data_buses[i].pulldowns, HEX));
        writeSerial("");
    }
    writeSerial("--- Binary Input Configurations (" + String(globalConfig.binary_input_count) + ") ---");
    for (int i = 0; i < globalConfig.binary_input_count; i++) {
        writeSerial("Binary Input " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.binary_inputs[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.binary_inputs[i].input_type, HEX));
        writeSerial("  Display As: 0x" + String(globalConfig.binary_inputs[i].display_as, HEX));
        writeSerial("  Pins: 1=" + String(globalConfig.binary_inputs[i].reserved_pin_1) + 
                   " 2=" + String(globalConfig.binary_inputs[i].reserved_pin_2) + 
                   " 3=" + String(globalConfig.binary_inputs[i].reserved_pin_3) + 
                   " 4=" + String(globalConfig.binary_inputs[i].reserved_pin_4) + 
                   " 5=" + String(globalConfig.binary_inputs[i].reserved_pin_5) + 
                   " 6=" + String(globalConfig.binary_inputs[i].reserved_pin_6) + 
                   " 7=" + String(globalConfig.binary_inputs[i].reserved_pin_7) + 
                   " 8=" + String(globalConfig.binary_inputs[i].reserved_pin_8));
        writeSerial("  Input Flags: 0x" + String(globalConfig.binary_inputs[i].input_flags, HEX));
        writeSerial("  Invert: 0x" + String(globalConfig.binary_inputs[i].invert, HEX));
        writeSerial("  Pullups: 0x" + String(globalConfig.binary_inputs[i].pullups, HEX));
        writeSerial("  Pulldowns: 0x" + String(globalConfig.binary_inputs[i].pulldowns, HEX));
        writeSerial("");
    }
    writeSerial("=============================");
}

float readBatteryVoltage() {
    if (globalConfig.power_option.battery_sense_pin == 0xFF) {
        return -1.0;
    }
    uint8_t sensePin = globalConfig.power_option.battery_sense_pin;
    uint8_t enablePin = globalConfig.power_option.battery_sense_enable_pin;
    uint16_t scalingFactor = globalConfig.power_option.voltage_scaling_factor;
    pinMode(sensePin, INPUT);
    if (enablePin != 0xFF) {
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, HIGH);
        delay(10);
    }
    const int numSamples = 10;
    uint32_t adcSum = 0;
    for (int i = 0; i < numSamples; i++) {
        adcSum += analogRead(sensePin);
        delay(2);
    }
    uint32_t adcAverage = adcSum / numSamples;
    if (enablePin != 0xFF) {
        digitalWrite(enablePin, LOW);
    }
    float voltage = -1.0;
    if (scalingFactor > 0) {
        voltage = (adcAverage * scalingFactor) / (100000.0);
    }
     return voltage;
}

float readChipTemperature() {
    #ifdef TARGET_ESP32
    float temp = temperatureRead();
    return temp; 
    #elif defined(TARGET_NRF)
    int32_t tempRaw = 0;
    uint32_t err_code = sd_temp_get(&tempRaw);
    if (err_code == 0) {
        float tempC = tempRaw * 0.25;
        return tempC;
    }
    return -999.0; // Fallback if SoftDevice API fails
    #else
    return -999.0;
    #endif
}

void handleDirectWriteStart(uint8_t* data, uint16_t len) {
    writeSerial("=== DIRECT WRITE START ===");
    if (directWriteActive) {
        writeSerial("WARNING: Previous direct write session was active - cleaning up before starting new session");
        cleanupDirectWriteState(false);
    }
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    directWriteBitplanes = (colorScheme == 1 || colorScheme == 2); // BWR/BWY use bitplanes
    directWritePlane2 = false; // Start with plane 1
    directWriteCompressed = (len >= 4);
    if (directWriteCompressed) {
        memcpy(&directWriteDecompressedTotal, data, 4);
        writeSerial("Compressed direct write mode");
        writeSerial("Expected decompressed size: " + String(directWriteDecompressedTotal) + " bytes");
        directWriteWidth = globalConfig.displays[0].pixel_width;
        directWriteHeight = globalConfig.displays[0].pixel_height;
        if (directWriteBitplanes) {
            // Bitplanes: each plane is 1BPP, total data is 2x plane size
            directWriteTotalBytes = (directWriteWidth * directWriteHeight + 7) / 8; // Bytes per plane
            writeSerial("Bitplane mode: " + String(directWriteTotalBytes) + " bytes per plane, " + String(directWriteTotalBytes * 2) + " total");
        } else {
            int bitsPerPixel = getBitsPerPixel();
            if (bitsPerPixel == 4) {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 1) / 2; // 2 pixels per byte
            } else if (bitsPerPixel == 2) {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 3) / 4; // 4 pixels per byte
            } else {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 7) / 8; // 8 pixels per byte
            }
        }
        directWriteCompressedBuffer = compressedDataBuffer;
        directWriteCompressedSize = 0;  // Unknown until we receive all data
        directWriteCompressedReceived = 0;
        if (len > 4) {
            uint32_t compressedDataLen = len - 4;
            if (compressedDataLen <= MAX_IMAGE_SIZE) {
                memcpy(directWriteCompressedBuffer, data + 4, compressedDataLen);
                directWriteCompressedReceived = compressedDataLen;
                writeSerial("Initial compressed data: " + String(compressedDataLen) + " bytes");
            } else {
                writeSerial("ERROR: Initial compressed data too large for static buffer (" + String(compressedDataLen) + " > " + String(MAX_IMAGE_SIZE) + ")");
                writeSerial("Rejecting compressed upload - client should use uncompressed mode");
                cleanupDirectWriteState(false);
                uint8_t errorResponse[] = {0xFF, RESP_DIRECT_WRITE_ERROR};  // Error response: use uncompressed upload
                sendResponse(errorResponse, sizeof(errorResponse));
                return;
            }
        }
    } else {
        directWriteWidth = globalConfig.displays[0].pixel_width;
        directWriteHeight = globalConfig.displays[0].pixel_height;
        if (directWriteBitplanes) {
            directWriteTotalBytes = (directWriteWidth * directWriteHeight + 7) / 8; // Bytes per plane
            writeSerial("Bitplane mode: " + String(directWriteTotalBytes) + " bytes per plane, " + String(directWriteTotalBytes * 2) + " total");
        } else {
            int bitsPerPixel = getBitsPerPixel();
            if (bitsPerPixel == 4) {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 1) / 2; // 2 pixels per byte
            } else if (bitsPerPixel == 2) {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 3) / 4; // 4 pixels per byte
            } else {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 7) / 8; // 8 pixels per byte
            }
        }
    }
    writeSerial("Display dimensions: " + String(directWriteWidth) + "x" + String(directWriteHeight));
    writeSerial("Expected total bytes: " + String(directWriteTotalBytes) + (directWriteBitplanes ? " per plane" : ""));
    directWriteActive = true;
    directWriteBytesWritten = 0;
    directWriteStartTime = millis();
    if (displayPowerState) {
        writeSerial("WARNING: Display already powered on - powering off first to ensure clean state");
        pwrmgm(false);
        delay(100);  // Brief delay to ensure power down completes
    }
    pwrmgm(true);
    writeSerial("Power management enabled");
    bbepInitIO(&bbep, globalConfig.displays[0].dc_pin, globalConfig.displays[0].reset_pin, globalConfig.displays[0].busy_pin, globalConfig.displays[0].cs_pin, globalConfig.displays[0].data_pin, globalConfig.displays[0].clk_pin, 8000000);
    writeSerial("Display IO initialized");
    bbepWakeUp(&bbep);
    writeSerial("Display woken up");
    bbepSendCMDSequence(&bbep, bbep.pInitFull);// important for some displays
    writeSerial("Display init sequence sent");
    bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
    writeSerial("Display address window set");
    bbepStartWrite(&bbep, directWriteBitplanes ? PLANE_0 : getplane());
    writeSerial("Display write started");
    uint8_t ackResponse[] = {0x00, RESP_DIRECT_WRITE_START_ACK};
    sendResponse(ackResponse, sizeof(ackResponse));
    writeSerial("Direct write mode started, ready for data");
}

int getplane() {
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    if (colorScheme == 0) {
        return PLANE_0; // B/W uses PLANE_0
    } else if (colorScheme == 1 || colorScheme == 2) {
        return PLANE_0; // BWR/BWY use PLANE_0 (but use bitplanes for direct write)
    } else if (colorScheme == 5) {
        return PLANE_1; // 4 grayscale uses PLANE_1 with 2BPP (like BWRY)
    }
    else {
        return PLANE_1; // BWRY and 6-color use PLANE_1
    }
}

int getBitsPerPixel() {
    if (globalConfig.displays[0].color_scheme == 4) {
        return 4; // 4 bits per pixel (2 pixels per byte)
    }
    if (globalConfig.displays[0].color_scheme == 3) {
        return 2; // 2 bits per pixel (4 pixels per byte) for 3-4 color displays
    }
    if (globalConfig.displays[0].color_scheme == 5) {
        return 2; // 2 bits per pixel (4 pixels per byte) for 4 grayscale
    }
    return 1; // 1 bit per pixel (8 pixels per byte)
}

static void renderChar_4BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, int fontScale) {
    for (int col = 0; col < charWidth; col += fontScale) {
        uint8_t fontByte;
        int fontCol = col / fontScale;
        if (fontCol == 0 || fontCol > 7) {
            fontByte = 0x00;
        } else {
            fontByte = fontData[fontCol - 1];
        }
        uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
        uint8_t pixelNibble = (pixelBit == 1) ? 0x0 : 0xF;  // 1=black=0x0, 0=white=0xF
        for (int s = 0; s < fontScale; s++) {
            int pixelX = startX + charIdx * charWidth + col + s;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            int bytePos = pixelX / 2;
            if (bytePos >= pitch) break;
            if ((pixelX % 2) == 0) {
                rowBuffer[bytePos] = (rowBuffer[bytePos] & 0x0F) | (pixelNibble << 4);
            } else {
                rowBuffer[bytePos] = (rowBuffer[bytePos] & 0xF0) | pixelNibble;
            }
        }
    }
}

static void renderChar_2BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, uint8_t colorScheme, int fontScale) {
    uint8_t whiteCode = (colorScheme == 5) ? 0x03 : 0x01; // 11 for grayscale, 01 for 3-4 color
    int pixelsPerByte = 4;
    for (int col = 0; col < charWidth; col += pixelsPerByte) {
        uint8_t pixelByte = 0;
        for (int p = 0; p < pixelsPerByte; p++) {
            int pixelX = startX + charIdx * charWidth + col + p;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            uint8_t fontByte;
            int fontCol = (col + p) / fontScale;
            if (fontCol == 0 || fontCol > 7) {
                fontByte = 0x00;
            } else {
                fontByte = fontData[fontCol - 1];
            }
            uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
            uint8_t pixelValue = (pixelBit == 1) ? 0x00 : whiteCode;
            pixelByte |= (pixelValue << (6 - p * 2));
        }
        int bytePos = (startX + charIdx * charWidth + col) / 4;
        if (bytePos < pitch) {
            rowBuffer[bytePos] = pixelByte;
        }
    }
}

static void renderChar_1BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, int fontScale) {
    for (int col = 0; col < charWidth; col += fontScale) {
        uint8_t fontByte;
        int fontCol = col / fontScale;
        if (fontCol == 0 || fontCol > 7) {
            fontByte = 0x00;
        } else {
            fontByte = fontData[fontCol - 1];
        }
        uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
        for (int s = 0; s < fontScale; s++) {
            int pixelX = startX + charIdx * charWidth + col + s;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            
            int bytePos = pixelX / 8;
            int bitPos = 7 - (pixelX % 8);
            if (bytePos < pitch) {
                if (pixelBit == 1) {
                    rowBuffer[bytePos] &= ~(1 << bitPos);
                }
            }
        }
    }
}

void writeTextAndFill(const char* text) {
    if (text == nullptr || globalConfig.displays[0].pixel_width == 0) return;
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    bool useBitplanes = (colorScheme == 1 || colorScheme == 2); // BWR/BWY use bitplanes (grayscale uses 2BPP, not bitplanes)
    int bitsPerPixel = getBitsPerPixel();
    int pitch;
    uint8_t whiteValue;
    if (bitsPerPixel == 4) {
        pitch = globalConfig.displays[0].pixel_width / 2;
        whiteValue = 0xFF;
    } else if (bitsPerPixel == 2) {
        pitch = (globalConfig.displays[0].pixel_width + 3) / 4;
        if (colorScheme == 5) {
            whiteValue = 0xFF; // All pixels = 11 (white) for grayscale
        } else {
            whiteValue = 0x55; // All pixels = 01 (white) for 3-4 color displays
        }
    } else {
        pitch = (globalConfig.displays[0].pixel_width + 7) / 8;
        whiteValue = 0xFF;
    }
    int fontScale = (globalConfig.displays[0].pixel_width < FONT_SMALL_THRESHOLD) ? 1 : 2;
    int charWidth = FONT_BASE_WIDTH * fontScale;
    int charHeight = FONT_BASE_HEIGHT * fontScale;
    uint8_t* whiteRow = staticWhiteRow;
    memset(whiteRow, whiteValue, pitch);
    uint8_t* rowBuffer = staticRowBuffer;
    bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
    bbepStartWrite(&bbep, getplane());
    int maxChars = globalConfig.displays[0].pixel_width / charWidth;
    int lineCount = 1; // At least one line
    const char* countPtr = text;
    while (*countPtr != '\0') {
        if (*countPtr == '\n' || *countPtr == '\r') {
            lineCount++;
            if (*countPtr == '\r' && *(countPtr + 1) == '\n') {
                countPtr++; // Skip \r\n
            }
        }
        countPtr++;
    }
    int totalTextHeight = lineCount * charHeight;
    int remainingHeight = globalConfig.displays[0].pixel_height - totalTextHeight;
    int spacing = 0;
    if (lineCount > 0) {
        spacing = remainingHeight / (lineCount + 1);
    }
    const char* lineStart = text;
    const char* current = text;
    int currentY = 0;
    int lineIndex = 0;
    for (int s = 0; s < spacing && currentY < globalConfig.displays[0].pixel_height; s++) {
        bbepWriteData(&bbep, whiteRow, pitch);
        currentY++;
    }
    while (*current != '\0') {
        if (*current == '\n' || *current == '\r') {
            int lineLen = current - lineStart;
            if (lineLen > 0) {
                char* line = staticLineBuffer;
                int copyLen = (lineLen < 255) ? lineLen : 255;
                memcpy(line, lineStart, copyLen);
                line[copyLen] = '\0';
                int textLen = strlen(line);
                if (textLen > maxChars) textLen = maxChars;
                int textWidthPixels = textLen * charWidth;
                int startX = (globalConfig.displays[0].pixel_width - textWidthPixels) / 2;
                if (startX < 0) startX = 0;
                if (lineIndex > 0) {
                    for (int s = 0; s < spacing && currentY < globalConfig.displays[0].pixel_height; s++) {
                        bbepWriteData(&bbep, whiteRow, pitch);
                        currentY++;
                    }
                }
                for (int row = 0; row < charHeight && currentY < globalConfig.displays[0].pixel_height; row++) {
                    memset(rowBuffer, whiteValue, pitch);
                    int fontRow = row / fontScale;
                    for (int charIdx = 0; charIdx < textLen; charIdx++) {
                        uint8_t c = (uint8_t)line[charIdx];
                        if (c < 32 || c > 127) c = 32;
                        uint8_t fontData[7];
                        int fontOffset = (c - 32) * 7;
                        memcpy_P(fontData, &writelineFont[fontOffset], 7);
                        if (bitsPerPixel == 4) {
                            renderChar_4BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, fontScale);
                        } else if (bitsPerPixel == 2) {
                            renderChar_2BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, colorScheme, fontScale);
                        } else {
                            renderChar_1BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, fontScale);
                        }
                    }
                    bbepWriteData(&bbep, rowBuffer, pitch);
                    currentY++;
                }
                lineIndex++;
            } else {
                if (lineIndex > 0) {
                    for (int s = 0; s < spacing && currentY < globalConfig.displays[0].pixel_height; s++) {
                        bbepWriteData(&bbep, whiteRow, pitch);
                        currentY++;
                    }
                }
                for (int row = 0; row < charHeight && currentY < globalConfig.displays[0].pixel_height; row++) {
                    bbepWriteData(&bbep, whiteRow, pitch);
                    currentY++;
                }
                lineIndex++;
            }
            if (*current == '\r' && *(current + 1) == '\n') {
                current++;
            }
            current++;
            lineStart = current;
        } else {
            current++;
        }
    }
    if (current > lineStart) {
        int lineLen = current - lineStart;
        char* line = staticLineBuffer;
        int copyLen = (lineLen < 255) ? lineLen : 255;
        memcpy(line, lineStart, copyLen);
        line[copyLen] = '\0';
        if (lineIndex > 0) {
            for (int s = 0; s < spacing && currentY < globalConfig.displays[0].pixel_height; s++) {
                bbepWriteData(&bbep, whiteRow, pitch);
                currentY++;
            }
        }
        int textLen = strlen(line);
        if (textLen > maxChars) textLen = maxChars;
        int textWidthPixels = textLen * charWidth;
        int startX = (globalConfig.displays[0].pixel_width - textWidthPixels) / 2;
        if (startX < 0) startX = 0;
        
        for (int row = 0; row < charHeight && currentY < globalConfig.displays[0].pixel_height; row++) {
            memset(rowBuffer, whiteValue, pitch);
            int fontRow = row / fontScale;
            for (int charIdx = 0; charIdx < textLen; charIdx++) {
                uint8_t c = (uint8_t)line[charIdx];
                if (c < 32 || c > 127) c = 32;
                uint8_t fontData[7];
                int fontOffset = (c - 32) * 7;
                memcpy_P(fontData, &writelineFont[fontOffset], 7);
                if (bitsPerPixel == 4) {
                    renderChar_4BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, fontScale);
                } else if (bitsPerPixel == 2) {
                    renderChar_2BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, colorScheme, fontScale);
                } else {
                    renderChar_1BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, fontScale);
                }
            }
            
            bbepWriteData(&bbep, rowBuffer, pitch);
            currentY++;
        }
    }
    for (int s = 0; s < spacing && currentY < globalConfig.displays[0].pixel_height; s++) {
        bbepWriteData(&bbep, whiteRow, pitch);
        currentY++;
    }
    while (currentY < globalConfig.displays[0].pixel_height) {
        bbepWriteData(&bbep, whiteRow, pitch);
        currentY++;
    }
    if (currentY != globalConfig.displays[0].pixel_height) {
        writeSerial("WARNING: writeTextAndFill wrote " + String(currentY) + " rows, expected " + String(globalConfig.displays[0].pixel_height));
    } else {
        writeSerial("writeTextAndFill: Wrote " + String(currentY) + " rows (" + String(pitch) + " bytes/row, " + String(pitch * currentY) + " total bytes)");
    }
    if (useBitplanes) {
        memset(whiteRow, 0x00, pitch); // Reuse whiteRow, set to zeros = no red/yellow
        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
        bbepStartWrite(&bbep, PLANE_1);
        for (int row = 0; row < globalConfig.displays[0].pixel_height; row++) {
            bbepWriteData(&bbep, whiteRow, pitch);
        }
        writeSerial("writeTextAndFill: Wrote plane 2 (R/Y) with zeros for bitplane display");
    }
    if (colorScheme == 5) {
        int otherPlane = (getplane() == PLANE_0) ? PLANE_1 : PLANE_0;
        int otherPlanePitch = (globalConfig.displays[0].pixel_width + 7) / 8; // 1BPP pitch for the other plane
        memset(whiteRow, 0x00, otherPlanePitch); // Reuse whiteRow, set to zeros
        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
        bbepStartWrite(&bbep, otherPlane);
        for (int row = 0; row < globalConfig.displays[0].pixel_height; row++) {
            bbepWriteData(&bbep, whiteRow, otherPlanePitch);
        }
        writeSerial("writeTextAndFill: Cleared other plane (PLANE_" + String(otherPlane == PLANE_0 ? "0" : "1") + ") with zeros for grayscale display");
    }
}

// One entry per panel color: fill byte for PLANE_0 and PLANE_1 (BWR/BWY only)
struct BootColor { uint8_t fill0; uint8_t fill1; };

static int getBootColors(uint8_t colorScheme, BootColor out[]) {
    switch (colorScheme) {
        case 0:                                                       // BW (1bpp)
            out[0] = {0x00, 0x00}; out[1] = {0xFF, 0x00}; return 2;
        case 1:                                                       // BWR (1bpp dual-plane)
            out[0] = {0x00, 0x00}; out[1] = {0xFF, 0x00}; out[2] = {0xFF, 0xFF}; return 3;
        case 2:                                                       // BWY (1bpp dual-plane)
            out[0] = {0x00, 0x00}; out[1] = {0xFF, 0x00}; out[2] = {0xFF, 0xFF}; return 3;
        case 3:                                                       // 3-4 color (2bpp)
            out[0] = {0x00, 0x00}; out[1] = {0x55, 0x00};
            out[2] = {0xAA, 0x00}; out[3] = {0xFF, 0x00}; return 4;
        case 4:                                                       // 4bpp gray (16 levels → 4 shown)
            out[0] = {0x00, 0x00}; out[1] = {0x55, 0x00};
            out[2] = {0xAA, 0x00}; out[3] = {0xFF, 0x00}; return 4;
        default:                                                      // 4-gray 2bpp
            out[0] = {0x00, 0x00}; out[1] = {0x55, 0x00};
            out[2] = {0xAA, 0x00}; out[3] = {0xFF, 0x00}; return 4;
    }
}

static void setBlackPixel(uint8_t* buf, int pitch, int x, int bitsPerPixel) {
    if (bitsPerPixel == 4) {
        int bp = x / 2;
        if (bp >= pitch) return;
        if (x % 2 == 0) buf[bp] &= 0x0F; else buf[bp] &= 0xF0;
    } else if (bitsPerPixel == 2) {
        int bp = x / 4;
        if (bp >= pitch) return;
        buf[bp] &= ~(0x03 << (6 - (x % 4) * 2));
    } else {
        int bp = x / 8;
        if (bp >= pitch) return;
        buf[bp] &= ~(0x80 >> (x % 8));
    }
}

static void bootRenderTextRow(uint8_t* rowBuf, int pitch, int fontRow,
                              const char* str, int startX,
                              int charW, int fontScale,
                              int bitsPerPixel, uint8_t colorScheme) {
    int len = (int)strlen(str);
    for (int ci = 0; ci < len; ci++) {
        uint8_t c = (uint8_t)str[ci];
        if (c < 32 || c > 127) c = 32;
        uint8_t fontData[7];
        memcpy_P(fontData, &writelineFont[(c - 32) * 7], 7);
        if (bitsPerPixel == 4)
            renderChar_4BPP(rowBuf, fontData, fontRow, ci, startX, charW, pitch, fontScale);
        else if (bitsPerPixel == 2)
            renderChar_2BPP(rowBuf, fontData, fontRow, ci, startX, charW, pitch, colorScheme, fontScale);
        else
            renderChar_1BPP(rowBuf, fontData, fontRow, ci, startX, charW, pitch, fontScale);
    }
}

void drawBootScreen() {
    int W = globalConfig.displays[0].pixel_width;
    int H = globalConfig.displays[0].pixel_height;
    if (W == 0 || H == 0) return;

    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    int bitsPerPixel = getBitsPerPixel();
    int pitch;
    uint8_t whiteValue;
    if (bitsPerPixel == 4) {
        pitch = W / 2;
        whiteValue = 0xFF;
    } else if (bitsPerPixel == 2) {
        pitch = (W + 3) / 4;
        whiteValue = (colorScheme == 5) ? 0xFF : 0x55;
    } else {
        pitch = (W + 7) / 8;
        whiteValue = 0xFF;
    }
    int fontScale = (W < FONT_SMALL_THRESHOLD) ? 1 : 2;
    int charW     = FONT_BASE_WIDTH  * fontScale;
    int charH     = FONT_BASE_HEIGHT * fontScale;
    int gap       = charH / 2;
    int lineGap   = charH / 4;

    // Single centered info string: "ODxxxxxx - FW dev"
    String chipId = getChipIdHex();
    char deviceFwStr[32];
    if (getFirmwareMajor() == 0 && getFirmwareMinor() == 0)
        snprintf(deviceFwStr, sizeof(deviceFwStr), "OD%s - FW dev", chipId.c_str());
    else
        snprintf(deviceFwStr, sizeof(deviceFwStr), "OD%s - FW %d.%d", chipId.c_str(), getFirmwareMajor(), getFirmwareMinor());

    // Color squares — arranged in a grid (max 4 per row)
    BootColor squareColors[16];
    int numSquares = getBootColors(colorScheme, squareColors);
    int pixPerByte = (bitsPerPixel == 4) ? 2 : (bitsPerPixel == 2) ? 4 : 8;
    int numCols;
    if      (numSquares <= 3) numCols = numSquares;
    else if (numSquares == 4) numCols = 2;
    else                      numCols = min(4, (numSquares + 1) / 2);
    int numRows    = (numSquares + numCols - 1) / numCols;
    int squareSize = min(H / 12, W / (numCols * 5));
    squareSize     = (squareSize / pixPerByte) * pixPerByte;
    if (squareSize < pixPerByte * 2) squareSize = pixPerByte * 2;
    int sqSpacing  = (squareSize / 4 / pixPerByte) * pixPerByte;
    if (sqSpacing < pixPerByte) sqSpacing = pixPerByte;
    int sqGroupW   = numCols * squareSize + (numCols - 1) * sqSpacing;
    int sqGroupH   = numRows * squareSize + (numRows - 1) * sqSpacing;
    uint8_t lMask  = (bitsPerPixel == 4) ? 0x0F : (bitsPerPixel == 2) ? 0x3F : 0x7F;
    uint8_t rMask  = (bitsPerPixel == 4) ? 0xF0 : (bitsPerPixel == 2) ? 0xFC : 0xFE;

    // Bottom info strip: rule + gap + deviceFw + lineGap + credit + gap
    int infoAreaH = 1 + gap + charH + lineGap + charH + gap;
    int logoAreaH = H - infoAreaH;
    if (logoAreaH < 0) logoAreaH = 0;

    // Logo centered in logoAreaH.
    // Small displays zoom in (130% of the smaller dimension) so the icon
    // content fills the screen; the rendering loop clips naturally at the edges.
    // Large displays use 95% for a small margin.
    int logoBase = min(logoAreaH, W);
    int logoPct  = (W < FONT_SMALL_THRESHOLD) ? 130 : 95;
    int logoSize = (logoBase * logoPct / 100 / pixPerByte) * pixPerByte;
    int logoX    = (W - logoSize) / 2;   // may be negative on small displays → clips
    int logoY    = (logoAreaH - logoSize) / 2;  // may be negative → clips top/bottom

    // Color square grid: bottom-left corner inside the logo's white interior.
    // The source icon has visible content between ~8% and ~68% of its height,
    // so anchor the BOTTOM of the square group at 65% of logoSize from logoY.
    int sqLeftInset  = ((logoX + logoSize * 14 / 100) / pixPerByte) * pixPerByte;
    int sqOriginX_logo = sqLeftInset;
    int sqBottomY    = logoY + logoSize * 65 / 100;
    int sqStripY     = sqBottomY - sqGroupH;
    if (sqStripY < logoY) sqStripY = logoY;

    auto cx = [&](const char* s) -> int {
        int x = (W - (int)strlen(s) * charW) / 2;
        return (x < 2) ? 2 : x;
    };

    uint8_t* whiteRow = staticWhiteRow;
    uint8_t* rowBuf   = staticRowBuffer;
    memset(whiteRow, whiteValue, pitch);

    bbepSetAddrWindow(&bbep, 0, 0, W, H);
    bbepStartWrite(&bbep, getplane());

    int currentY = 0;

    // Logo area — decode G5 logo, stream row-by-row, overlay squares at sqStripY
    #ifdef BOOT_HAS_LOGO
    {
        int srcW     = BOOT_LOGO_G5_WIDTH;
        int srcH     = BOOT_LOGO_G5_HEIGHT;
        int srcPitch = (srcW + 7) / 8;

        uint8_t* srcBuf = (uint8_t*)malloc(srcH * srcPitch);
        if (srcBuf) {
            G5DECODER g5dec;
            g5dec.init(srcW, srcH, (uint8_t*)boot_logo_g5 + 8, sizeof(boot_logo_g5) - 8);
            for (int y = 0; y < srcH; y++)
                g5dec.decodeLine(srcBuf + y * srcPitch);

            for (; currentY < logoAreaH && currentY < H; currentY++) {
                memset(rowBuf, whiteValue, pitch);

                // Logo pixels (logoX/logoY may be negative when zoomed in)
                if (currentY >= logoY && currentY < logoY + logoSize) {
                    int srcRow = (currentY - logoY) * srcH / logoSize;
                    const uint8_t* srcLine = srcBuf + srcRow * srcPitch;
                    int dxStart = (logoX < 0) ? -logoX : 0;
                    for (int dx = dxStart; dx < logoSize; dx++) {
                        int dispX  = logoX + dx;
                        if (dispX >= W) break;
                        int srcCol = dx * srcW / logoSize;
                        if (!((srcLine[srcCol / 8] >> (7 - srcCol % 8)) & 1))
                            setBlackPixel(rowBuf, pitch, dispX, bitsPerPixel);
                    }
                }

                // Color square grid overlaid on top of logo
                int sqRow = currentY - sqStripY;
                if (sqRow >= 0 && sqRow < sqGroupH) {
                    int gridRow    = sqRow / (squareSize + sqSpacing);
                    int pixInRow   = sqRow % (squareSize + sqSpacing);
                    if (gridRow < numRows && pixInRow < squareSize) {
                        bool borderRow  = (pixInRow == 0 || pixInRow == squareSize - 1);
                        // Bottom-up: gridRow 0 = top display row = last color row
                        int colorRow   = numRows - 1 - gridRow;
                        int sqStart    = colorRow * numCols;
                        int sqEnd      = min(sqStart + numCols, numSquares);
                        for (int sq = sqStart; sq < sqEnd; sq++) {
                            int col       = sq - sqStart;
                            int sx        = sqOriginX_logo + col * (squareSize + sqSpacing);
                            int byteStart = sx / pixPerByte;
                            int byteCount = squareSize / pixPerByte;
                            if (borderRow) {
                                memset(rowBuf + byteStart, 0x00, byteCount);
                            } else {
                                memset(rowBuf + byteStart, squareColors[sq].fill0, byteCount);
                                rowBuf[byteStart]                 &= lMask;
                                rowBuf[byteStart + byteCount - 1] &= rMask;
                            }
                        }
                    }
                }

                bbepWriteData(&bbep, rowBuf, pitch);
            }
            free(srcBuf);
        } else {
            for (; currentY < logoAreaH && currentY < H; currentY++)
                bbepWriteData(&bbep, whiteRow, pitch);
        }
    }
    #else
    for (; currentY < logoAreaH && currentY < H; currentY++)
        bbepWriteData(&bbep, whiteRow, pitch);
    #endif

    // Rule
    if (currentY < H) {
        memset(rowBuf, 0x00, pitch);
        bbepWriteData(&bbep, rowBuf, pitch);
        currentY++;
    }

    // Gap
    for (int i = 0; i < gap && currentY < H; i++, currentY++)
        bbepWriteData(&bbep, whiteRow, pitch);

    // Device + FW string centered
    for (int row = 0; row < charH && currentY < H; row++, currentY++) {
        memset(rowBuf, whiteValue, pitch);
        bootRenderTextRow(rowBuf, pitch, row / fontScale, deviceFwStr, cx(deviceFwStr), charW, fontScale, bitsPerPixel, colorScheme);
        bbepWriteData(&bbep, rowBuf, pitch);
    }

    // Credit line
    for (int i = 0; i < lineGap && currentY < H; i++, currentY++)
        bbepWriteData(&bbep, whiteRow, pitch);
    for (int row = 0; row < charH && currentY < H; row++, currentY++) {
        memset(rowBuf, whiteValue, pitch);
        bootRenderTextRow(rowBuf, pitch, row / fontScale, "by Jonas Niesner", cx("by Jonas Niesner"), charW, fontScale, bitsPerPixel, colorScheme);
        bbepWriteData(&bbep, rowBuf, pitch);
    }

    // Gap
    for (int i = 0; i < gap && currentY < H; i++, currentY++)
        bbepWriteData(&bbep, whiteRow, pitch);

    // Fill any remaining rows
    while (currentY < H) {
        bbepWriteData(&bbep, whiteRow, pitch);
        currentY++;
    }

    // BWR/BWY: PLANE_1 — all zeros except color squares at sqStripY
    if (colorScheme == 1 || colorScheme == 2) {
        memset(whiteRow, 0x00, pitch);
        bbepSetAddrWindow(&bbep, 0, 0, W, H);
        bbepStartWrite(&bbep, PLANE_1);
        for (int y = 0; y < H; y++) {
            int sqRow = y - sqStripY;
            if (sqRow < 0 || sqRow >= sqGroupH) {
                bbepWriteData(&bbep, whiteRow, pitch);
                continue;
            }
            int gridRow  = sqRow / (squareSize + sqSpacing);
            int pixInRow = sqRow % (squareSize + sqSpacing);
            if (gridRow >= numRows || pixInRow >= squareSize) {
                bbepWriteData(&bbep, whiteRow, pitch);
                continue;
            }
            memset(rowBuf, 0x00, pitch);
            bool borderRow = (pixInRow == 0 || pixInRow == squareSize - 1);
            int colorRow   = numRows - 1 - gridRow;
            int sqStart    = colorRow * numCols;
            int sqEnd      = min(sqStart + numCols, numSquares);
            for (int sq = sqStart; sq < sqEnd; sq++) {
                if (squareColors[sq].fill1 == 0x00) continue;
                int col       = sq - sqStart;
                int sx        = sqOriginX_logo + col * (squareSize + sqSpacing);
                int byteStart = sx / pixPerByte;
                int byteCount = squareSize / pixPerByte;
                if (!borderRow) {
                    memset(rowBuf + byteStart, squareColors[sq].fill1, byteCount);
                    rowBuf[byteStart]                 &= lMask;
                    rowBuf[byteStart + byteCount - 1] &= rMask;
                }
            }
            bbepWriteData(&bbep, rowBuf, pitch);
        }
    }
}

void handleDirectWriteData(uint8_t* data, uint16_t len) {
    if (!directWriteActive) {
        writeSerial("ERROR: Direct write data received but mode not active");
        return;
    }
    if (len == 0) {
        writeSerial("WARNING: Empty data packet received");
        return;
    }
    if (directWriteCompressed) {
        handleDirectWriteCompressedData(data, len);
    } else {
        if (directWriteBitplanes) {
            uint16_t dataOffset = 0;
            uint16_t remainingLen = len;
            while (remainingLen > 0) {
                if (!directWritePlane2) {
                    uint32_t remainingInPlane = directWriteTotalBytes - directWriteBytesWritten;
                    uint16_t bytesToWrite = (remainingLen > remainingInPlane) ? remainingInPlane : remainingLen;
                    if (bytesToWrite > 0) {
                        bbepWriteData(&bbep, data + dataOffset, bytesToWrite);
                        directWriteBytesWritten += bytesToWrite;
                        writeSerial("Direct write plane 1: " + String(bytesToWrite) + " bytes written (total: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes) + ")");
                        dataOffset += bytesToWrite;
                        remainingLen -= bytesToWrite;
                    }
                    if (remainingLen > 0 && directWriteBytesWritten >= directWriteTotalBytes) {
                        writeSerial("WARNING: Received " + String(remainingLen) + " extra bytes after plane 1 complete - ignoring");
                        remainingLen = 0;
                    }
                    if (directWriteBytesWritten >= directWriteTotalBytes && !directWritePlane2) {
                        writeSerial("Plane 1 complete, switching to plane 2");
                        directWritePlane2 = true;
                        directWriteBytesWritten = 0;
                        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
                        bbepStartWrite(&bbep, PLANE_1);
                    }
                } else {
                    uint32_t remainingInPlane = directWriteTotalBytes - directWriteBytesWritten;
                    uint16_t bytesToWrite = (remainingLen > remainingInPlane) ? remainingInPlane : remainingLen;
                    if (bytesToWrite > 0) {
                        bbepWriteData(&bbep, data + dataOffset, bytesToWrite);
                        directWriteBytesWritten += bytesToWrite;
                        writeSerial("Direct write plane 2: " + String(bytesToWrite) + " bytes written (total: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes) + ")");
                        dataOffset += bytesToWrite;
                        remainingLen -= bytesToWrite;
                    }
                    if (remainingLen > 0 && directWriteBytesWritten >= directWriteTotalBytes) {
                        writeSerial("WARNING: Received " + String(remainingLen) + " extra bytes after plane 2 complete - ignoring");
                        remainingLen = 0;
                    }
                    if (directWriteBytesWritten >= directWriteTotalBytes) {
                        writeSerial("Plane 2 complete");
                        break;
                    }
                }
            }
            if (directWritePlane2 && directWriteBytesWritten >= directWriteTotalBytes) {
                writeSerial("All planes written, ending direct write mode");
                handleDirectWriteEnd();
            } else {
                uint8_t ackResponse[] = {0x00, RESP_DIRECT_WRITE_DATA_ACK};
                sendResponse(ackResponse, sizeof(ackResponse));
            }
        } else {
            uint32_t remainingBytes = (directWriteBytesWritten < directWriteTotalBytes) ? (directWriteTotalBytes - directWriteBytesWritten) : 0;
            uint16_t bytesToWrite = (len > remainingBytes) ? remainingBytes : len;
            if (bytesToWrite > 0) {
                bbepWriteData(&bbep, data, bytesToWrite);
                directWriteBytesWritten += bytesToWrite;
                writeSerial("Direct write: " + String(bytesToWrite) + " bytes written (total: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes) + ")");
            }
            if (len > remainingBytes) {
                writeSerial("WARNING: Received " + String(len) + " bytes but only " + String(remainingBytes) + " bytes expected - ignoring excess data");
            }
            if (directWriteBytesWritten >= directWriteTotalBytes) {
                writeSerial("All data written, ending direct write mode");
                handleDirectWriteEnd();
            } else {
                uint8_t ackResponse[] = {0x00, RESP_DIRECT_WRITE_DATA_ACK};
                sendResponse(ackResponse, sizeof(ackResponse));
            }
        }
    }
}

void handleDirectWriteCompressedData(uint8_t* data, uint16_t len) {
    uint32_t newTotalSize = directWriteCompressedReceived + len;
    if (newTotalSize > MAX_IMAGE_SIZE) {
        writeSerial("ERROR: Compressed data exceeds static buffer size (" + String(newTotalSize) + " > " + String(MAX_IMAGE_SIZE) + ")");
        writeSerial("Rejecting compressed upload - client should use uncompressed mode");
        cleanupDirectWriteState(true);
        uint8_t errorResponse[] = {0xFF, 0xFF};  // Error response: use uncompressed upload
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    memcpy(directWriteCompressedBuffer + directWriteCompressedReceived, data, len);
    directWriteCompressedReceived += len;
    writeSerial("Accumulated compressed data: " + String(directWriteCompressedReceived) + " bytes");
    uint8_t ackResponse[] = {0x00, RESP_DIRECT_WRITE_DATA_ACK};
    sendResponse(ackResponse, sizeof(ackResponse));
}

void decompressDirectWriteData() {
    if (directWriteCompressedReceived == 0) {
        writeSerial("ERROR: No compressed data to decompress");
        return;
    }
    writeSerial("Starting decompression of " + String(directWriteCompressedReceived) + " bytes compressed data");
    struct uzlib_uncomp d;
    memset(&d, 0, sizeof(d));
    d.source = directWriteCompressedBuffer;
    d.source_limit = directWriteCompressedBuffer + directWriteCompressedReceived;
    d.source_read_cb = NULL;
    uzlib_init();
    int hdr = uzlib_zlib_parse_header(&d);
    if (hdr < 0) {
        writeSerial("ERROR: Invalid zlib header: " + String(hdr));
        return;
    }
    uint16_t window = 0x100 << hdr;
    if (window > MAX_DICT_SIZE) window = MAX_DICT_SIZE;
    uzlib_uncompress_init(&d, dictionaryBuffer, window);
    if (directWriteBitplanes) {
        writeSerial("Bitplane mode: streaming decompression");
        bool decompressingPlane2 = false;
        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
        bbepStartWrite(&bbep, PLANE_0);
        directWriteBytesWritten = 0;
        int res;
        do {
            d.dest_start = decompressionChunk;
            d.dest = decompressionChunk;
            d.dest_limit = decompressionChunk + DECOMP_CHUNK_SIZE;
            res = uzlib_uncompress(&d);
            size_t bytesOut = d.dest - d.dest_start;
            if (bytesOut > 0) {
                uint32_t remainingInPlane1 = directWriteTotalBytes - directWriteBytesWritten;
                if (bytesOut > remainingInPlane1) {
                    bbepWriteData(&bbep, decompressionChunk, remainingInPlane1);
                    directWriteBytesWritten = directWriteTotalBytes;
                    writeSerial("Plane 1 complete: " + String(directWriteTotalBytes) + " bytes");
                    decompressingPlane2 = true;
                    bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
                    bbepStartWrite(&bbep, PLANE_1);
                    directWriteBytesWritten = 0;
                    uint32_t plane2Bytes = bytesOut - remainingInPlane1;
                    bbepWriteData(&bbep, decompressionChunk + remainingInPlane1, plane2Bytes);
                    directWriteBytesWritten = plane2Bytes;
                    writeSerial("Plane 2 started: " + String(plane2Bytes) + " bytes");
                } else {
                    bbepWriteData(&bbep, decompressionChunk, bytesOut);
                    directWriteBytesWritten += bytesOut;
                    if (directWriteBytesWritten >= directWriteTotalBytes) {
                        writeSerial("Plane 1 complete: " + String(directWriteTotalBytes) + " bytes");
                        decompressingPlane2 = true;
                        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
                        bbepStartWrite(&bbep, PLANE_1);
                        directWriteBytesWritten = 0;
                        writeSerial("Switched to plane 2");
                    }
                }
            }
            if (res == TINF_DATA_ERROR) {
                writeSerial("ERROR: Decompression data error");
                break;
            }
        } while (res == TINF_OK && !decompressingPlane2);
        if (decompressingPlane2 && res == TINF_OK) {
            do {
                d.dest_start = decompressionChunk;
                d.dest = decompressionChunk;
                d.dest_limit = decompressionChunk + DECOMP_CHUNK_SIZE;
                res = uzlib_uncompress(&d);
                size_t bytesOut = d.dest - d.dest_start;
                if (bytesOut > 0) {
                    uint32_t remainingInPlane2 = directWriteTotalBytes - directWriteBytesWritten;
                    uint32_t bytesToWrite = (bytesOut > remainingInPlane2) ? remainingInPlane2 : bytesOut;
                    bbepWriteData(&bbep, decompressionChunk, bytesToWrite);
                    directWriteBytesWritten += bytesToWrite;
                    writeSerial("Plane 2: " + String(bytesToWrite) + " bytes, total: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes));
                }
                if (res == TINF_DATA_ERROR) {
                    writeSerial("ERROR: Decompression data error");
                    break;
                }
            } while (res == TINF_OK && directWriteBytesWritten < directWriteTotalBytes);
        }
        if (res == TINF_DONE || (decompressingPlane2 && directWriteBytesWritten >= directWriteTotalBytes)) {
            writeSerial("Bitplane decompression complete: plane 1 + plane 2 = " + String(directWriteTotalBytes * 2) + " bytes");
        } else {
            writeSerial("ERROR: Decompression failed with code: " + String(res));
        }
    } else {
        int res;
        do {
            d.dest_start = decompressionChunk;
            d.dest = decompressionChunk;
            d.dest_limit = decompressionChunk + DECOMP_CHUNK_SIZE;
            res = uzlib_uncompress(&d);
            size_t bytesOut = d.dest - d.dest_start;
            if (bytesOut > 0) {
                bbepWriteData(&bbep, decompressionChunk, bytesOut);
                directWriteBytesWritten += bytesOut;
                writeSerial("Decompressed: " + String(bytesOut) + " bytes, written: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes));
            }
            if (res == TINF_DATA_ERROR) {
                writeSerial("ERROR: Decompression data error");
                break;
            }
        } while (res == TINF_OK && directWriteBytesWritten < directWriteTotalBytes);
        
        if (res == TINF_DONE || directWriteBytesWritten >= directWriteTotalBytes) {
            writeSerial("Decompression complete: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes) + " bytes written");
        } else {
            writeSerial("ERROR: Decompression failed with code: " + String(res));
        }
    }
}

void cleanupDirectWriteState(bool refreshDisplay) {
    directWriteActive = false;
    directWriteCompressed = false;
    directWriteBitplanes = false;
    directWritePlane2 = false;
    directWriteBytesWritten = 0;
    directWriteCompressedReceived = 0;
    directWriteCompressedSize = 0;
    directWriteDecompressedTotal = 0;
    directWriteCompressedBuffer = nullptr;
    directWriteWidth = 0;
    directWriteHeight = 0;
    directWriteTotalBytes = 0;
    directWriteRefreshMode = 0;
    directWriteStartTime = 0;
    if (refreshDisplay && displayPowerState) {
        bbepSleep(&bbep, 1);  // Put display to sleep before power down
        delay(200);  // Brief delay after sleep command
    }
    displayPowerState = false;
    pwrmgm(false);
    writeSerial("Direct write state cleaned up");
}

void handleDirectWriteEnd(uint8_t* data, uint16_t len) {
    if (!directWriteActive) {
        writeSerial("WARNING: Direct write end called but mode not active");
        return;
    }
    writeSerial("=== DIRECT WRITE END ===");
    if (directWriteCompressed && directWriteCompressedReceived > 0) {
        writeSerial("Decompressing accumulated compressed data...");
        decompressDirectWriteData();
    }
    if (directWriteBitplanes) {
        writeSerial("Total bytes written: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes) + " per plane (" + String(directWritePlane2 ? "plane 2" : "plane 1") + ")");
    } else {
        writeSerial("Total bytes written: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes));
    }
    if (directWriteCompressed) {
        writeSerial("Compressed bytes received: " + String(directWriteCompressedReceived));
    }
    int refreshMode = REFRESH_FULL;
    if (data != nullptr && len >= 1) {
        uint8_t refreshFlag = data[0];
        if (refreshFlag == 1) {
            refreshMode = REFRESH_FAST;
            writeSerial("Using fast/partial refresh mode (requested)");
        } else if (refreshFlag == 0) {
            refreshMode = REFRESH_FULL;
            writeSerial("Full refresh explicitly requested");
        } else {
            refreshMode = REFRESH_FULL;
            writeSerial("Unknown refresh flag value (" + String(refreshFlag) + "), using full refresh");
        }
    } else {
        writeSerial("No refresh mode specified, using full refresh (backward compatible)");
    }
    uint8_t ackResponse[] = {0x00, RESP_DIRECT_WRITE_END_ACK};
    sendResponse(ackResponse, sizeof(ackResponse));
    delay(100);
    bbepRefresh(&bbep, refreshMode);
    bool refreshSuccess = waitforrefresh(60);
    bbepSleep(&bbep, 1);  // Put display to sleep before power down
    delay(200);  // Brief delay after sleep command
    cleanupDirectWriteState(false);
    if (refreshSuccess) {
        uint8_t refreshResponse[] = {0x00, RESP_DIRECT_WRITE_REFRESH_SUCCESS};
        sendResponse(refreshResponse, sizeof(refreshResponse));
        writeSerial("Direct write completed and display refreshed successfully");
    } else {
        uint8_t timeoutResponse[] = {0x00, RESP_DIRECT_WRITE_REFRESH_TIMEOUT};
        sendResponse(timeoutResponse, sizeof(timeoutResponse));
        writeSerial("Direct write completed but display refresh timed out");
    }
}

// ============================================================================
// Encryption and Authentication Functions
// ============================================================================

bool isEncryptionEnabled() {
    return (securityConfig.encryption_enabled == 1) && 
           (securityConfig.encryption_key[0] != 0 || 
            memcmp(securityConfig.encryption_key, securityConfig.encryption_key + 1, 15) != 0);
}

bool isAuthenticated() {
    return encryptionSession.authenticated && 
           (encryptionSession.session_start_time > 0) &&
           checkEncryptionSessionTimeout();
}

void clearEncryptionSession() {
    // Zeroize session key
    memset(encryptionSession.session_key, 0, 16);
    memset(encryptionSession.client_nonce, 0, 16);
    memset(encryptionSession.server_nonce, 0, 16);
    memset(encryptionSession.pending_server_nonce, 0, 16);
    encryptionSession.authenticated = false;
    encryptionSession.nonce_counter = 0;
    encryptionSession.last_seen_counter = 0;
    encryptionSession.integrity_failures = 0;
    encryptionSession.session_start_time = 0;
    encryptionSession.last_activity = 0;
    encryptionSession.auth_attempts = 0;
    encryptionSession.server_nonce_time = 0;
    memset(encryptionSession.replay_window, 0, sizeof(encryptionSession.replay_window));
    writeSerial("Encryption session cleared");
}

bool checkEncryptionSessionTimeout() {
    if (!encryptionSession.authenticated) return false;
    if (securityConfig.session_timeout_seconds == 0) return true; // No timeout
    
    uint32_t currentTime = millis() / 1000; // Convert to seconds
    uint32_t sessionAge = currentTime - (encryptionSession.session_start_time / 1000);
    
    if (sessionAge >= securityConfig.session_timeout_seconds) {
        writeSerial("Encryption session timeout (" + String(sessionAge) + "s >= " + 
                   String(securityConfig.session_timeout_seconds) + "s)");
        clearEncryptionSession();
        return false;
    }
    return true;
}

void updateEncryptionSessionActivity() {
    if (encryptionSession.authenticated) {
        encryptionSession.last_activity = millis();
    }
}

// Simple constant-time comparison
bool constantTimeCompare(const uint8_t* a, const uint8_t* b, size_t len) {
    uint8_t result = 0;
    for (size_t i = 0; i < len; i++) {
        result |= a[i] ^ b[i];
    }
    return result == 0;
}

// Platform-specific crypto includes
#ifdef TARGET_ESP32
#include "mbedtls/aes.h"
#include "mbedtls/ccm.h"
#include "mbedtls/cmac.h"
#include "esp_random.h"

// AES-CMAC implementation using mbedtls
bool aes_cmac(const uint8_t* key, const uint8_t* message, size_t message_len, uint8_t* mac) {
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t* cipher_info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
    
    if (cipher_info == NULL) {
        writeSerial("ERROR: Failed to get cipher info for AES-128-ECB");
        return false;
    }
    
    mbedtls_cipher_init(&ctx);
    if (mbedtls_cipher_setup(&ctx, cipher_info) != 0) {
        writeSerial("ERROR: Failed to setup cipher");
        mbedtls_cipher_free(&ctx);
        return false;
    }
    
    if (mbedtls_cipher_cmac_starts(&ctx, key, 128) != 0) {
        writeSerial("ERROR: Failed to start CMAC");
        mbedtls_cipher_free(&ctx);
        return false;
    }
    
    if (mbedtls_cipher_cmac_update(&ctx, message, message_len) != 0) {
        writeSerial("ERROR: Failed to update CMAC");
        mbedtls_cipher_free(&ctx);
        return false;
    }
    
    if (mbedtls_cipher_cmac_finish(&ctx, mac) != 0) {
        writeSerial("ERROR: Failed to finish CMAC");
        mbedtls_cipher_free(&ctx);
        return false;
    }
    
    mbedtls_cipher_free(&ctx);
    return true;
}

// AES-ECB encryption
bool aes_ecb_encrypt(const uint8_t* key, const uint8_t* input, uint8_t* output) {
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    
    if (mbedtls_aes_setkey_enc(&aes, key, 128) != 0) {
        writeSerial("ERROR: Failed to set AES key");
        mbedtls_aes_free(&aes);
        return false;
    }
    
    if (mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, input, output) != 0) {
        writeSerial("ERROR: Failed to encrypt with AES-ECB");
        mbedtls_aes_free(&aes);
        return false;
    }
    
    mbedtls_aes_free(&aes);
    return true;
}

// AES-CCM encryption
bool aes_ccm_encrypt(const uint8_t* key, const uint8_t* nonce, size_t nonce_len,
                     const uint8_t* ad, size_t ad_len,
                     const uint8_t* plaintext, size_t plaintext_len,
                     uint8_t* ciphertext, uint8_t* tag, size_t tag_len) {
    mbedtls_ccm_context ccm;
    mbedtls_ccm_init(&ccm);
    
    if (mbedtls_ccm_setkey(&ccm, MBEDTLS_CIPHER_ID_AES, key, 128) != 0) {
        writeSerial("ERROR: Failed to set CCM key");
        mbedtls_ccm_free(&ccm);
        return false;
    }
    
    int ret = mbedtls_ccm_encrypt_and_tag(&ccm, plaintext_len, nonce, nonce_len,
                                          ad, ad_len, plaintext, ciphertext, tag, tag_len);
    mbedtls_ccm_free(&ccm);
    
    if (ret != 0) {
        writeSerial("ERROR: CCM encrypt failed: " + String((int)ret));
        return false;
    }
    
    return true;
}

// AES-CCM decryption
bool aes_ccm_decrypt(const uint8_t* key, const uint8_t* nonce, size_t nonce_len,
                     const uint8_t* ad, size_t ad_len,
                     const uint8_t* ciphertext, size_t ciphertext_len,
                     uint8_t* plaintext, const uint8_t* tag, size_t tag_len) {
    mbedtls_ccm_context ccm;
    mbedtls_ccm_init(&ccm);
    
    if (mbedtls_ccm_setkey(&ccm, MBEDTLS_CIPHER_ID_AES, key, 128) != 0) {
        writeSerial("ERROR: Failed to set CCM key");
        mbedtls_ccm_free(&ccm);
        return false;
    }
    
    // Validate CCM parameters before calling
    // CCM nonce length must be between 7 and 13 bytes
    if (nonce_len < 7 || nonce_len > 13) {
        writeSerial("ERROR: Invalid CCM nonce length (must be 7-13 bytes)");
        mbedtls_ccm_free(&ccm);
        return false;
    }
    
    // CCM tag length must be 4, 6, 8, 10, 12, 14, or 16 bytes
    if (tag_len != 4 && tag_len != 6 && tag_len != 8 && tag_len != 10 && tag_len != 12 && tag_len != 14 && tag_len != 16) {
        writeSerial("ERROR: Invalid CCM tag length (must be 4, 6, 8, 10, 12, 14, or 16 bytes)");
        mbedtls_ccm_free(&ccm);
        return false;
    }
    
    // CCM requires at least 1 byte of plaintext/ciphertext
    if (ciphertext_len == 0) {
        writeSerial("ERROR: CCM ciphertext length is 0 (must be at least 1 byte)");
        mbedtls_ccm_free(&ccm);
        return false;
    }
    
    int ret = mbedtls_ccm_auth_decrypt(&ccm, ciphertext_len, nonce, nonce_len,
                                       ad, ad_len, ciphertext, plaintext, tag, tag_len);
    mbedtls_ccm_free(&ccm);
    
    if (ret != 0) {
        char err_buf[128];
        snprintf(err_buf, sizeof(err_buf), "ERROR: CCM decrypt failed: %d (ciphertext_len=%zu, nonce_len=%zu, tag_len=%zu)", 
                 ret, ciphertext_len, nonce_len, tag_len);
        writeSerial(err_buf);
        // Error -15 is MBEDTLS_ERR_CCM_BAD_INPUT
        if (ret == -15) {
            writeSerial("ERROR: MBEDTLS_ERR_CCM_BAD_INPUT - invalid input parameters");
        }
        return false;
    }
    
    return true;
}

// RNG
void secure_random(uint8_t* output, size_t len) {
    esp_fill_random(output, len);
}

#else // TARGET_NRF
// For nRF52840, use CryptoCell CC310 hardware accelerator via Adafruit_nRFCrypto
// This uses the ARM CryptoCell CC310 built into the nRF52840 chip.
// Lowest memory footprint: all crypto runs on hardware, no software AES needed.
#include <string.h>
#include <Adafruit_nRFCrypto.h>
#include "nrf_cc310/include/crys_aesccm.h"
#include "nrf_cc310/include/ssi_aes.h"
#include "nrf_cc310/include/ssi_aes_defs.h"

static bool cc310_initialized = false;

static bool init_cc310() {
    if (cc310_initialized) return true;
    if (!nRFCrypto.begin()) {
        writeSerial("ERROR: Failed to initialize CryptoCell CC310");
        return false;
    }
    cc310_initialized = true;
    writeSerial("CryptoCell CC310 initialized successfully");
    return true;
}

// AES-CMAC using CC310 hardware (SaSi_Aes API with CMAC mode)
bool aes_cmac(const uint8_t* key, const uint8_t* message, size_t message_len, uint8_t* mac) {
    if (!init_cc310()) return false;

    SaSiAesUserContext_t ctx;
    SaSiAesUserKeyData_t keyData;
    keyData.pKey = (uint8_t*)key;
    keyData.keySize = 16;

    SaSiError_t err = SaSi_AesInit(&ctx, SASI_AES_ENCRYPT, SASI_AES_MODE_CMAC, SASI_AES_PADDING_NONE);
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesInit (CMAC) failed: 0x" + String((unsigned long)err, HEX));
        return false;
    }

    err = SaSi_AesSetKey(&ctx, SASI_AES_USER_KEY, &keyData, sizeof(keyData));
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesSetKey (CMAC) failed: 0x" + String((unsigned long)err, HEX));
        SaSi_AesFree(&ctx);
        return false;
    }

    // For CMAC, SaSi_AesFinish must process the last block (applies K1/K2 XOR).
    // SaSi_AesBlock handles all intermediate blocks EXCEPT the last one.
    size_t block_len = 0;    // bytes to process via SaSi_AesBlock
    size_t finish_offset = 0;
    size_t finish_len = message_len;

    if (message_len > 16) {
        if (message_len % 16 == 0) {
            // Exact multiple of block size: keep last full block for Finish (K1 XOR)
            block_len = message_len - 16;
        } else {
            // Partial last block: keep it for Finish (K2 XOR)
            block_len = (message_len / 16) * 16;
        }
        finish_offset = block_len;
        finish_len = message_len - block_len;
    }

    if (block_len > 0) {
        err = SaSi_AesBlock(&ctx, (uint8_t*)message, block_len, NULL);
        if (err != SASI_OK) {
            writeSerial("ERROR: SaSi_AesBlock (CMAC) failed: 0x" + String((unsigned long)err, HEX));
            SaSi_AesFree(&ctx);
            return false;
        }
    }

    size_t mac_size = 16;
    err = SaSi_AesFinish(&ctx, finish_len, (uint8_t*)message + finish_offset, finish_len, mac, &mac_size);
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesFinish (CMAC) failed: 0x" + String((unsigned long)err, HEX));
        SaSi_AesFree(&ctx);
        return false;
    }

    SaSi_AesFree(&ctx);
    return true;
}

// AES-ECB single-block encryption using CC310 hardware
bool aes_ecb_encrypt(const uint8_t* key, const uint8_t* input, uint8_t* output) {
    if (!init_cc310()) return false;

    SaSiAesUserContext_t ctx;
    SaSiAesUserKeyData_t keyData;
    keyData.pKey = (uint8_t*)key;
    keyData.keySize = 16;

    SaSiError_t err = SaSi_AesInit(&ctx, SASI_AES_ENCRYPT, SASI_AES_MODE_ECB, SASI_AES_PADDING_NONE);
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesInit (ECB) failed: 0x" + String((unsigned long)err, HEX));
        return false;
    }

    err = SaSi_AesSetKey(&ctx, SASI_AES_USER_KEY, &keyData, sizeof(keyData));
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesSetKey (ECB) failed: 0x" + String((unsigned long)err, HEX));
        SaSi_AesFree(&ctx);
        return false;
    }

    // For ECB, single block: use Finish directly (dataSize=16, one block)
    size_t out_size = 16;
    err = SaSi_AesFinish(&ctx, 16, (uint8_t*)input, 16, output, &out_size);
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesFinish (ECB) failed: 0x" + String((unsigned long)err, HEX));
        SaSi_AesFree(&ctx);
        return false;
    }

    SaSi_AesFree(&ctx);
    return true;
}

// AES-CCM encryption using CC310 hardware (integrated CRYS_AESCCM function)
bool aes_ccm_encrypt(const uint8_t* key, const uint8_t* nonce, size_t nonce_len,
                     const uint8_t* ad, size_t ad_len,
                     const uint8_t* plaintext, size_t plaintext_len,
                     uint8_t* ciphertext, uint8_t* tag, size_t tag_len) {
    if (!init_cc310()) return false;

    // Validate nonce length (CCM requires 7-13)
    if (nonce_len < 7 || nonce_len > 13) {
        writeSerial("ERROR: Invalid CCM nonce length: " + String((int)nonce_len));
        return false;
    }
    // Validate tag length (CCM requires 4, 6, 8, 10, 12, 14, or 16)
    if (tag_len < 4 || tag_len > 16 || (tag_len % 2 != 0)) {
        writeSerial("ERROR: Invalid CCM tag length: " + String((int)tag_len));
        return false;
    }

    // Prepare key buffer (CC310 expects 32-byte key buffer)
    CRYS_AESCCM_Key_t ccmKey;
    memset(ccmKey, 0, sizeof(ccmKey));
    memcpy(ccmKey, key, 16);

    CRYS_AESCCM_Mac_Res_t macRes;
    memset(macRes, 0, sizeof(macRes));

    CRYSError_t err = CRYS_AESCCM(
        SASI_AES_ENCRYPT,
        ccmKey,
        CRYS_AES_Key128BitSize,
        (uint8_t*)nonce, (uint8_t)nonce_len,
        (uint8_t*)ad, (uint32_t)ad_len,
        (uint8_t*)plaintext, (uint32_t)plaintext_len,
        ciphertext,
        (uint8_t)tag_len,
        macRes
    );

    if (err != CRYS_OK) {
        writeSerial("ERROR: CRYS_AESCCM (encrypt) failed: 0x" + String((unsigned long)err, HEX));
        return false;
    }

    // Copy the tag from macRes
    memcpy(tag, macRes, tag_len);
    return true;
}

// AES-CCM decryption using CC310 hardware (integrated CRYS_AESCCM function)
bool aes_ccm_decrypt(const uint8_t* key, const uint8_t* nonce, size_t nonce_len,
                     const uint8_t* ad, size_t ad_len,
                     const uint8_t* ciphertext, size_t ciphertext_len,
                     uint8_t* plaintext, const uint8_t* tag, size_t tag_len) {
    if (!init_cc310()) return false;

    if (nonce_len < 7 || nonce_len > 13) {
        writeSerial("ERROR: Invalid CCM nonce length: " + String((int)nonce_len));
        return false;
    }
    if (tag_len < 4 || tag_len > 16 || (tag_len % 2 != 0)) {
        writeSerial("ERROR: Invalid CCM tag length: " + String((int)tag_len));
        return false;
    }

    CRYS_AESCCM_Key_t ccmKey;
    memset(ccmKey, 0, sizeof(ccmKey));
    memcpy(ccmKey, key, 16);

    // CC310 decrypt expects the tag in macRes; it verifies it internally
    CRYS_AESCCM_Mac_Res_t macRes;
    memset(macRes, 0, sizeof(macRes));
    memcpy(macRes, tag, tag_len);

    CRYSError_t err = CRYS_AESCCM(
        SASI_AES_DECRYPT,
        ccmKey,
        CRYS_AES_Key128BitSize,
        (uint8_t*)nonce, (uint8_t)nonce_len,
        (uint8_t*)ad, (uint32_t)ad_len,
        (uint8_t*)ciphertext, (uint32_t)ciphertext_len,
        plaintext,
        (uint8_t)tag_len,
        macRes
    );

    if (err != CRYS_OK) {
        writeSerial("ERROR: CRYS_AESCCM (decrypt) failed: 0x" + String((unsigned long)err, HEX));
        return false;
    }

    return true;
}

// Secure RNG using CC310 hardware
void secure_random(uint8_t* output, size_t len) {
    if (!init_cc310()) {
        writeSerial("WARNING: CC310 not initialized, using non-secure random");
        for (size_t i = 0; i < len; i++) {
            output[i] = random(256);
        }
        return;
    }

    if (!nRFCrypto.Random.generate(output, (uint16_t)len)) {
        writeSerial("ERROR: CC310 RNG failed, using non-secure fallback");
        for (size_t i = 0; i < len; i++) {
            output[i] = random(256);
        }
    }
}

#endif // TARGET_NRF

// Derive session key using AES-KDF (SP 800-108 Counter Mode)
bool deriveSessionKey(const uint8_t* master_key, const uint8_t* client_nonce, 
                     const uint8_t* server_nonce, uint8_t* session_key) {
    // Build context: "OpenDisplay session" || device_id || client_nonce || server_nonce
    // For device_id, we'll use a simple identifier (can be improved)
    uint8_t device_id[4] = {0x00, 0x00, 0x00, 0x01}; // Placeholder device ID
    
    // Build input for CMAC
    uint8_t cmac_input[64];
    size_t offset = 0;
    const char* label = "OpenDisplay session";
    memcpy(cmac_input + offset, label, strlen(label));
    offset += strlen(label);
    cmac_input[offset++] = 0x00; // Separator
    memcpy(cmac_input + offset, device_id, 4);
    offset += 4;
    memcpy(cmac_input + offset, client_nonce, 16);
    offset += 16;
    memcpy(cmac_input + offset, server_nonce, 16);
    offset += 16;
    // Key length in bits (128 = 0x0080, big-endian)
    cmac_input[offset++] = 0x00; // High byte
    cmac_input[offset++] = 0x80; // Low byte
    
    // Compute intermediate via CMAC
    uint8_t intermediate[16];
    if (!aes_cmac(master_key, cmac_input, offset, intermediate)) {
        return false;
    }
    
    // Build final input block: counter (8 bytes, value=1, big-endian) || intermediate (8 bytes)
    uint8_t final_input[16];
    uint64_t counter_be = 1; // Counter = 1, big-endian
    // Convert to big-endian
    for (int i = 0; i < 8; i++) {
        final_input[i] = (counter_be >> (56 - i * 8)) & 0xFF;
    }
    memcpy(final_input + 8, intermediate, 8);
    
    // Derive session key via AES-ECB
    if (!aes_ecb_encrypt(master_key, final_input, session_key)) {
        return false;
    }
    
    return true;
}

// Derive session ID from nonces
void deriveSessionId(const uint8_t* session_key, const uint8_t* client_nonce,
                     const uint8_t* server_nonce, uint8_t* session_id) {
    uint8_t input[32];
    memcpy(input, client_nonce, 16);
    memcpy(input + 16, server_nonce, 16);
    
    uint8_t cmac_output[16];
    if (aes_cmac(session_key, input, 32, cmac_output)) {
        // Copy 8 bytes (session_id is 8 bytes)
        for (int i = 0; i < 8; i++) {
            session_id[i] = cmac_output[i];
        }
    } else {
        memset(session_id, 0, 8);
    }
}

// Verify nonce replay protection (sliding window)
bool verifyNonceReplay(uint8_t* nonce) {
    if (!encryptionSession.authenticated) return false;
    
    // Extract session_id and counter from nonce (16 bytes: 8 bytes session_id + 8 bytes counter)
    uint8_t nonce_session_id[8];
    uint64_t nonce_counter = 0;
    memcpy(nonce_session_id, nonce, 8);
    
    // Extract counter (big-endian)
    for (int i = 0; i < 8; i++) {
        nonce_counter = (nonce_counter << 8) | nonce[8 + i];
    }
    
    // Verify session_id matches
    if (!constantTimeCompare(nonce_session_id, encryptionSession.session_id, 8)) {
        char buf[256];
        snprintf(buf, sizeof(buf), "ERROR: Nonce session_id mismatch\n  Nonce ID: %02X%02X%02X%02X%02X%02X%02X%02X\n  Expected: %02X%02X%02X%02X%02X%02X%02X%02X",
                 nonce_session_id[0], nonce_session_id[1], nonce_session_id[2], nonce_session_id[3],
                 nonce_session_id[4], nonce_session_id[5], nonce_session_id[6], nonce_session_id[7],
                 encryptionSession.session_id[0], encryptionSession.session_id[1], encryptionSession.session_id[2], encryptionSession.session_id[3],
                 encryptionSession.session_id[4], encryptionSession.session_id[5], encryptionSession.session_id[6], encryptionSession.session_id[7]);
        writeSerial(buf);
        return false;
    }
    
    // Check if counter is within replay window (±32 from last_seen_counter)
    int64_t counter_diff = (int64_t)nonce_counter - (int64_t)encryptionSession.last_seen_counter;
    
    if (counter_diff < -32 || counter_diff > 32) {
        char buf[128];
        snprintf(buf, sizeof(buf), "ERROR: Nonce counter outside replay window (counter=%llu, last_seen=%llu, diff=%lld)", 
                 (unsigned long long)nonce_counter, (unsigned long long)encryptionSession.last_seen_counter, (long long)counter_diff);
        writeSerial(buf);
        return false;
    }
    
    // Check if counter was already seen (simple check - can be improved with bitmap)
    // For now, we'll just check if it's greater than last_seen
    if (nonce_counter <= encryptionSession.last_seen_counter && counter_diff != 0) {
        // Check if it's in the seen window
        bool already_seen = false;
        for (int i = 0; i < 64; i++) {
            if (encryptionSession.replay_window[i] == nonce_counter) {
                already_seen = true;
                break;
            }
        }
        if (already_seen) {
            writeSerial("ERROR: Nonce counter already seen (replay detected)");
            return false;
        }
    }
    
    // Update replay window
    if (nonce_counter > encryptionSession.last_seen_counter) {
        encryptionSession.last_seen_counter = nonce_counter;
    }
    
    // Add to replay window (simple circular buffer)
    static uint8_t replay_window_index = 0;
    encryptionSession.replay_window[replay_window_index] = nonce_counter;
    replay_window_index = (replay_window_index + 1) % 64;
    
    return true;
}

// Get current nonce for encryption
void getCurrentNonce(uint8_t* nonce) {
    if (!encryptionSession.authenticated) {
        memset(nonce, 0, 16);
        return;
    }
    
    // Build nonce: session_id (8 bytes) || counter (8 bytes, big-endian)
    memcpy(nonce, encryptionSession.session_id, 8);
    
    // Convert counter to big-endian
    uint64_t counter = encryptionSession.nonce_counter;
    for (int i = 0; i < 8; i++) {
        nonce[8 + i] = (counter >> (56 - i * 8)) & 0xFF;
    }
}

// Increment nonce counter
void incrementNonceCounter() {
    if (encryptionSession.authenticated) {
        encryptionSession.nonce_counter++;
    }
}

// Handle authentication command (0x0050)
bool handleAuthenticate(uint8_t* data, uint16_t len) {
    if (!isEncryptionEnabled()) {
        uint8_t response[] = {0x00, 0x50, 0x03}; // Status: Encryption not configured
        sendResponse(response, sizeof(response));
        return false;
    }
    
    // Check rate limiting
    uint32_t currentTime = millis();
    if (encryptionSession.last_auth_time > 0) {
        uint32_t timeSinceLastAuth = (currentTime - encryptionSession.last_auth_time) / 1000;
        if (timeSinceLastAuth < 60) { // 1 minute window
            if (encryptionSession.auth_attempts >= 10) {
                uint8_t response[] = {0x00, 0x50, 0x04}; // Status: Rate limit exceeded
                sendResponse(response, sizeof(response));
                return false;
            }
        } else {
            encryptionSession.auth_attempts = 0; // Reset counter
        }
    }
    
    encryptionSession.auth_attempts++;
    encryptionSession.last_auth_time = currentTime;
    
    // Step 1: Client requests authentication (len == 1, data[0] == 0x00)
    if (len == 1 && data[0] == 0x00) {
        // Check if already authenticated
        if (encryptionSession.authenticated && checkEncryptionSessionTimeout()) {
            // Client is requesting new authentication - clear old session and start fresh
            writeSerial("New authentication requested, clearing existing session");
            clearEncryptionSession();
            // Fall through to generate new challenge
        }
        
        // Generate server nonce for new challenge
        secure_random(encryptionSession.pending_server_nonce, 16);
        encryptionSession.server_nonce_time = currentTime;
        
        // Send challenge: status (0x00) || server_nonce (16 bytes)
        uint8_t response[2 + 1 + 16]; // Header (2) + status (1) + server_nonce (16) = 19 bytes
        response[0] = 0x00;
        response[1] = 0x50;
        response[2] = 0x00; // Status: Challenge sent
        memcpy(response + 3, encryptionSession.pending_server_nonce, 16);
        sendResponse(response, sizeof(response));
        
        writeSerial("Authentication challenge sent");
        return false; // Not authenticated yet
    }
    
    // Step 2: Client responds with client_nonce (16 bytes) || challenge_response (16 bytes)
    if (len == 32) {
        uint8_t client_nonce[16];
        uint8_t challenge_response[16];
        memcpy(client_nonce, data, 16);
        memcpy(challenge_response, data + 16, 16);
        
        // Check if server nonce is still valid (30 second timeout)
        if (currentTime - encryptionSession.server_nonce_time > 30000) {
            writeSerial("ERROR: Server nonce expired");
            uint8_t response[] = {0x00, 0x50, 0xFF}; // Status: Error
            sendResponse(response, sizeof(response));
            return false;
        }
        
        // Compute expected challenge response
        uint8_t device_id[4] = {0x00, 0x00, 0x00, 0x01}; // Placeholder device ID
        uint8_t challenge_input[36]; // server_nonce (16) || client_nonce (16) || device_id (4)
        memcpy(challenge_input, encryptionSession.pending_server_nonce, 16);
        memcpy(challenge_input + 16, client_nonce, 16);
        memcpy(challenge_input + 32, device_id, 4);
        
        // Debug: log challenge input
        char challenge_hex[80];
        snprintf(challenge_hex, sizeof(challenge_hex), "Challenge input (36 bytes): %02X%02X%02X%02X...%02X%02X%02X%02X",
                 challenge_input[0], challenge_input[1], challenge_input[2], challenge_input[3],
                 challenge_input[32], challenge_input[33], challenge_input[34], challenge_input[35]);
        writeSerial(challenge_hex);
        
        uint8_t expected_response[16];
        writeSerial("Computing expected CMAC...");
        if (!aes_cmac(securityConfig.encryption_key, challenge_input, 36, expected_response)) {
            writeSerial("ERROR: Failed to compute expected CMAC");
            uint8_t response[] = {0x00, 0x50, 0xFF}; // Status: Error
            sendResponse(response, sizeof(response));
            return false;
        }
        
        // Debug: log expected response
        char expected_hex[50];
        snprintf(expected_hex, sizeof(expected_hex), "Expected response: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                 expected_response[0], expected_response[1], expected_response[2], expected_response[3],
                 expected_response[4], expected_response[5], expected_response[6], expected_response[7],
                 expected_response[8], expected_response[9], expected_response[10], expected_response[11],
                 expected_response[12], expected_response[13], expected_response[14], expected_response[15]);
        writeSerial(expected_hex);
        
        // Debug: log received challenge response
        char received_hex[50];
        snprintf(received_hex, sizeof(received_hex), "Received response: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                 challenge_response[0], challenge_response[1], challenge_response[2], challenge_response[3],
                 challenge_response[4], challenge_response[5], challenge_response[6], challenge_response[7],
                 challenge_response[8], challenge_response[9], challenge_response[10], challenge_response[11],
                 challenge_response[12], challenge_response[13], challenge_response[14], challenge_response[15]);
        writeSerial(received_hex);
        
        // Constant-time comparison
        if (!constantTimeCompare(challenge_response, expected_response, 16)) {
            writeSerial("ERROR: Authentication failed (wrong key)");
            uint8_t response[] = {0x00, 0x50, 0x01}; // Status: Authentication failed
            sendResponse(response, sizeof(response));
            // Zeroize pending server nonce
            memset(encryptionSession.pending_server_nonce, 0, 16);
            return false;
        }
        
        // Authentication successful - establish session
        memcpy(encryptionSession.client_nonce, client_nonce, 16);
        memcpy(encryptionSession.server_nonce, encryptionSession.pending_server_nonce, 16);
        
        // Derive session key
        if (!deriveSessionKey(securityConfig.encryption_key, client_nonce, 
                            encryptionSession.pending_server_nonce, encryptionSession.session_key)) {
            writeSerial("ERROR: Failed to derive session key");
            uint8_t response[] = {0x00, 0x50, 0xFF}; // Status: Error
            sendResponse(response, sizeof(response));
            return false;
        }
        
        // Derive session ID (use server_nonce which was copied from pending_server_nonce)
        deriveSessionId(encryptionSession.session_key, client_nonce, 
                       encryptionSession.server_nonce, encryptionSession.session_id);
        
        // Debug: verify session ID was set correctly
        char session_id_hex[17];
        for (int i = 0; i < 8; i++) {
            sprintf(&session_id_hex[i*2], "%02X", encryptionSession.session_id[i]);
        }
        session_id_hex[16] = '\0';
        writeSerial("Session ID after authentication: " + String(session_id_hex));
        
        // Verify session ID is not all zeros
        bool session_id_valid = false;
        for (int i = 0; i < 8; i++) {
            if (encryptionSession.session_id[i] != 0) {
                session_id_valid = true;
                break;
            }
        }
        if (!session_id_valid) {
            writeSerial("ERROR: Session ID is invalid (all zeros)!");
            uint8_t response[] = {0x00, 0x50, 0xFF}; // Status: Error
            sendResponse(response, sizeof(response));
            return false;
        }
        
        // Initialize session
        encryptionSession.authenticated = true;
        encryptionSession.nonce_counter = 0;
        encryptionSession.last_seen_counter = 0;
        encryptionSession.integrity_failures = 0;
        encryptionSession.session_start_time = currentTime;
        encryptionSession.last_activity = currentTime;
        memset(encryptionSession.replay_window, 0, sizeof(encryptionSession.replay_window));
        
        // Zeroize pending server nonce
        memset(encryptionSession.pending_server_nonce, 0, 16);
        encryptionSession.server_nonce_time = 0;
        
        // Send server response
        // Note: Use server_nonce (already copied from pending_server_nonce) for consistency
        uint8_t server_response[16];
        uint8_t server_input[36];
        memcpy(server_input, encryptionSession.server_nonce, 16); // Use server_nonce (same as pending_server_nonce at this point)
        memcpy(server_input + 16, client_nonce, 16);
        memcpy(server_input + 32, device_id, 4);
        if (!aes_cmac(encryptionSession.session_key, server_input, 36, server_response)) {
            writeSerial("ERROR: Failed to compute server response");
            clearEncryptionSession();
            uint8_t response[] = {0x00, 0x50, 0xFF}; // Status: Error
            sendResponse(response, sizeof(response));
            return false;
        }
        
        uint8_t response[2 + 1 + 16]; // Header (2) + status (1) + server_response (16) = 19 bytes
        response[0] = 0x00;
        response[1] = 0x50;
        response[2] = 0x00; // Status: Success
        memcpy(response + 3, server_response, 16);
        
        // Debug: log the response being sent
        char response_hex[64];
        snprintf(response_hex, sizeof(response_hex), "Sending auth success response: %02X %02X %02X %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                 response[0], response[1], response[2],
                 response[3], response[4], response[5], response[6], response[7],
                 response[8], response[9], response[10], response[11], response[12],
                 response[13], response[14], response[15], response[16], response[17], response[18]);
        writeSerial(response_hex);
        
        sendResponse(response, sizeof(response));
        
        writeSerial("Authentication successful, session established");
        return true;
    }
    
    // Invalid request format
    writeSerial("ERROR: Invalid authentication request format (len=" + String(len) + ")");
    uint8_t response[] = {0x00, 0x50, 0xFF}; // Status: Error
    sendResponse(response, sizeof(response));
    return false;
}

// Decrypt command
bool decryptCommand(uint8_t* ciphertext, uint16_t ciphertext_len, uint8_t* plaintext, 
                   uint16_t* plaintext_len, uint8_t* nonce_full, uint8_t* auth_tag, uint16_t command_header) {
    if (!isAuthenticated()) {
        return false;
    }
    
    // Verify nonce replay protection (uses full 16-byte nonce)
    if (!verifyNonceReplay(nonce_full)) {
        encryptionSession.integrity_failures++;
        if (encryptionSession.integrity_failures >= 3) {
            writeSerial("Too many integrity failures, clearing session");
            clearEncryptionSession();
        }
        return false;
    }
    
    // Extract encrypted payload length
    uint16_t encrypted_len = ciphertext_len;
    if (encrypted_len > 512) { // Sanity check
        writeSerial("ERROR: Encrypted payload too large");
        return false;
    }
    
    // Debug: log encrypted payload length
    char len_buf[64];
    snprintf(len_buf, sizeof(len_buf), "DecryptCommand: encrypted_len=%u, command=0x%04X", encrypted_len, command_header);
    writeSerial(len_buf);
    
    // Extract 13-byte nonce for CCM (last 13 bytes of 16-byte nonce)
    // Nonce format: [session_id: 8 bytes][counter: 8 bytes]
    // CCM uses: [session_id last 5 bytes][counter: 8 bytes] = 13 bytes
    uint8_t nonce[13];
    memcpy(nonce, nonce_full + 3, 13); // Skip first 3 bytes, use last 13
    
    // Debug: log nonce and parameters
    char nonce_buf[64];
    snprintf(nonce_buf, sizeof(nonce_buf), "CCM nonce (13 bytes): %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
             nonce[0], nonce[1], nonce[2], nonce[3], nonce[4], nonce[5], nonce[6], nonce[7],
             nonce[8], nonce[9], nonce[10], nonce[11], nonce[12]);
    writeSerial(nonce_buf);
    char param_buf[128];
    snprintf(param_buf, sizeof(param_buf), "CCM decrypt params: nonce_len=13, ad_len=2, ciphertext_len=%u, tag_len=12",
             encrypted_len);
    writeSerial(param_buf);
    
    // Decrypt using AES-CCM
    // Additional data: command header (2 bytes, big-endian)
    uint8_t ad[2];
    ad[0] = (command_header >> 8) & 0xFF;
    ad[1] = command_header & 0xFF;
    
    // Handle zero-byte ciphertext: CCM requires at least 1 byte
    // Solution: Client always includes a 1-byte length field, so encrypted_len should never be 0
    if (encrypted_len == 0) {
        writeSerial("ERROR: Encrypted payload is 0 bytes (should include length byte)");
        return false;
    }
    
    // Decrypt: first byte is payload length, rest is actual payload
    // Use static buffer to avoid stack overflow (ESP32 has limited stack)
    static uint8_t decrypted_with_length[512];
    bool success = aes_ccm_decrypt(encryptionSession.session_key, nonce, 13, // CCM uses 13-byte nonce
                                   ad, 2, ciphertext, encrypted_len,
                                   decrypted_with_length, auth_tag, 12);
    
    if (success) {
        // Extract payload length (first byte)
        uint8_t payload_length = decrypted_with_length[0];
        
        // Validate length
        if (payload_length > encrypted_len - 1) {
            writeSerial("ERROR: Invalid payload length in decrypted data");
            return false;
        }
        
        // Copy actual payload (skip length byte)
        if (payload_length > 0) {
            memcpy(plaintext, decrypted_with_length + 1, payload_length);
        }
        *plaintext_len = payload_length;
        encryptionSession.integrity_failures = 0; // Reset on success
        updateEncryptionSessionActivity();
        return true;
    } else {
        encryptionSession.integrity_failures++;
        if (encryptionSession.integrity_failures >= 3) {
            writeSerial("Too many integrity failures, clearing session");
            clearEncryptionSession();
        }
        return false;
    }
}

// Encrypt response
bool encryptResponse(uint8_t* plaintext, uint16_t plaintext_len, uint8_t* ciphertext,
                    uint16_t* ciphertext_len, uint8_t* nonce, uint8_t* auth_tag) {
    if (!isAuthenticated()) {
        return false;
    }
    
    // Get current nonce (16 bytes: session_id + counter)
    getCurrentNonce(nonce);
    incrementNonceCounter();
    
    // Extract 13-byte nonce for CCM (last 13 bytes of 16-byte nonce)
    // Nonce format: [session_id: 8 bytes][counter: 8 bytes]
    // CCM uses: [session_id last 5 bytes][counter: 8 bytes] = 13 bytes
    uint8_t nonce_ccm[13];
    memcpy(nonce_ccm, nonce + 3, 13); // Skip first 3 bytes, use last 13
    
    // Encrypt using AES-CCM
    // Additional data: response header (2 bytes)
    uint8_t ad[2] = {plaintext[0], plaintext[1]}; // Command header
    
    // Debug: log encryption parameters
    // Use static buffer to avoid stack overflow (ESP32 has limited stack)
    static char enc_buf[256];
    snprintf(enc_buf, sizeof(enc_buf), "EncryptResponse: plaintext_len=%u, payload_len=%u, nonce_16bytes=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
             plaintext_len, plaintext_len - 2,
             nonce[0], nonce[1], nonce[2], nonce[3], nonce[4], nonce[5], nonce[6], nonce[7],
             nonce[8], nonce[9], nonce[10], nonce[11], nonce[12], nonce[13], nonce[14], nonce[15]);
    writeSerial(enc_buf);
    
    // Handle zero-byte payload: CCM requires at least 1 byte
    // Solution: Always include a 1-byte length field as part of the encrypted payload
    // Use static buffer to avoid stack overflow (ESP32 has limited stack)
    static uint8_t payload_with_length[513]; // Max payload + 1 byte length
    uint16_t payload_len = plaintext_len - 2;
    payload_with_length[0] = payload_len & 0xFF; // Length byte (1 byte, supports up to 255)
    if (payload_len > 0) {
        memcpy(payload_with_length + 1, plaintext + 2, payload_len);
    }
    uint16_t total_payload_len = 1 + payload_len; // Always at least 1 byte (length)
    
    // Encrypt: output goes to ciphertext + 2 + 16 (after header and nonce)
    bool success = aes_ccm_encrypt(encryptionSession.session_key, nonce_ccm, 13, // CCM uses 13-byte nonce
                                   ad, 2, payload_with_length, total_payload_len,
                                   ciphertext + 2 + 16, auth_tag, 12);
    
    if (success) {
        // Build encrypted response: header (2) || nonce (16) || encrypted_data || tag (12)
        // IMPORTANT: Copy nonce FIRST before any other operations to avoid buffer issues
        uint8_t nonce_copy[16];
        memcpy(nonce_copy, nonce, 16); // Make a copy to prevent overwriting
        
        ciphertext[0] = plaintext[0];
        ciphertext[1] = plaintext[1];
        memcpy(ciphertext + 2, nonce_copy, 16); // Insert nonce after header
        // Encrypted data (including length byte) is already at ciphertext + 2 + 16
        memcpy(ciphertext + 2 + 16 + total_payload_len, auth_tag, 12);
        *ciphertext_len = 2 + 16 + total_payload_len + 12;
        
        // Debug: verify nonce was copied correctly
        char nonce_debug[64];
        snprintf(nonce_debug, sizeof(nonce_debug), "Response nonce (before): %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                 nonce[0], nonce[1], nonce[2], nonce[3], nonce[4], nonce[5], nonce[6], nonce[7],
                 nonce[8], nonce[9], nonce[10], nonce[11], nonce[12], nonce[13], nonce[14], nonce[15]);
        writeSerial(nonce_debug);
        char nonce_after[64];
        snprintf(nonce_after, sizeof(nonce_after), "Response nonce (after copy): %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                 ciphertext[2], ciphertext[3], ciphertext[4], ciphertext[5], ciphertext[6], ciphertext[7],
                 ciphertext[8], ciphertext[9], ciphertext[10], ciphertext[11], ciphertext[12], ciphertext[13],
                 ciphertext[14], ciphertext[15], ciphertext[16], ciphertext[17]);
        writeSerial(nonce_after);
        updateEncryptionSessionActivity();
        return true;
    }
    
    return false;
}

void imageDataWritten(BLEConnHandle conn_hdl, BLECharPtr chr, uint8_t* data, uint16_t len) {
    if (len < 2) {
        writeSerial("ERROR: Command too short (" + String(len) + " bytes)");
        return;
    }
    
    uint16_t command = (data[0] << 8) | data[1];  // Fixed byte order
    writeSerial("Processing command: 0x" + String(command, HEX));
    
    // Handle authentication command (always unencrypted)
    if (command == 0x0050) {
        writeSerial("=== AUTHENTICATE COMMAND (0x0050) ===");
        handleAuthenticate(data + 2, len - 2);
        return;
    }
    
    // Handle firmware version command (always unencrypted)
    if (command == 0x0043) {
        writeSerial("=== FIRMWARE VERSION COMMAND (0x0043) ===");
        handleFirmwareVersion();
        return;
    }
    
    // Check if encryption is required for other commands
    if (isEncryptionEnabled()) {
        if (!isAuthenticated()) {
            writeSerial("ERROR: Command requires authentication (encryption enabled)");
            // Send unencrypted error response so existing tooling can understand it
            uint8_t response[] = {0x00, (uint8_t)(command & 0xFF), 0xFE}; // Auth required
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }
        
        // Check if command is encrypted or unencrypted
        // Encrypted format: [command: 2] [nonce: 16] [encrypted_data] [tag: 12] (minimum 30 bytes)
        // Unencrypted format: [command: 2] [payload...] (typically 2-20 bytes)
        if (len < 2 + 16 + 12) {
            // Command is too short to be encrypted - treat as unencrypted command
            writeSerial("ERROR: Unencrypted command received when encryption is enabled");
            // Send unencrypted error response so existing tooling can understand it
            uint8_t response[] = {0x00, (uint8_t)(command & 0xFF), 0xFE}; // Auth required (unencrypted)
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }
        
        uint8_t nonce_full[16];
        uint8_t nonce[13]; // CCM uses 13-byte nonce (extract last 13 bytes from 16-byte nonce)
        uint8_t auth_tag[12];
        // Use static buffer to avoid stack overflow (ESP32 has limited stack)
        static uint8_t plaintext[512];
        uint16_t plaintext_len = 0;
        
        // Extract full 16-byte nonce and tag
        memcpy(nonce_full, data + 2, 16);
        memcpy(auth_tag, data + len - 12, 12);
        
        // Calculate encrypted data length
        uint16_t encrypted_data_len = len - 2 - 16 - 12; // Total - header - nonce - tag
        
        // Debug: log received data structure
        // Use static buffers to avoid stack overflow (ESP32 has limited stack)
        static char data_buf[256];
        snprintf(data_buf, sizeof(data_buf), "Encrypted command: len=%u, command=0x%04X, encrypted_data_len=%u", 
                 len, command, encrypted_data_len);
        writeSerial(data_buf);
        static char nonce_buf[64];
        snprintf(nonce_buf, sizeof(nonce_buf), "Full nonce: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                 nonce_full[0], nonce_full[1], nonce_full[2], nonce_full[3],
                 nonce_full[4], nonce_full[5], nonce_full[6], nonce_full[7],
                 nonce_full[8], nonce_full[9], nonce_full[10], nonce_full[11],
                 nonce_full[12], nonce_full[13], nonce_full[14], nonce_full[15]);
        writeSerial(nonce_buf);
        
        // Extract last 13 bytes for CCM (nonce format: [session_id: 8 bytes][counter: 8 bytes])
        // CCM uses last 13 bytes: [session_id last 5 bytes][counter: 8 bytes]
        memcpy(nonce, nonce_full + 3, 13); // Skip first 3 bytes, use last 13
        
        // Decrypt (pass nonce_full for replay protection, decryptCommand will extract 13-byte nonce for CCM)
        if (!decryptCommand(data + 2 + 16, encrypted_data_len, plaintext, &plaintext_len, nonce_full, auth_tag, command)) {
            writeSerial("ERROR: Decryption failed");
            // Send unencrypted error response so existing tooling can understand it
            uint8_t response[] = {0x00, (uint8_t)(command & 0xFF), 0xFF}; // Generic error (unencrypted)
            sendResponseUnencrypted(response, sizeof(response));
            return;
        }
        
        // Reconstruct command with decrypted data
        // Use static buffer to avoid stack overflow (ESP32 has limited stack)
        static uint8_t decrypted_data[512];
        decrypted_data[0] = data[0];
        decrypted_data[1] = data[1];
        memcpy(decrypted_data + 2, plaintext, plaintext_len);
        len = 2 + plaintext_len;
        data = decrypted_data;
    }
    
    // Process decrypted/unencrypted command
    switch (command) {
        case 0x0040: // Read Config command
            writeSerial("=== READ CONFIG COMMAND (0x0040) ===");
            writeSerial("Command received at time: " + String(millis()));
            handleReadConfig();
            writeSerial("Returned from handleReadConfig");
            break;
        case 0x0041: // Write Config command
            writeSerial("=== WRITE CONFIG COMMAND (0x0041) ===");
            handleWriteConfig(data + 2, len - 2);
            break;
        case 0x0042: // Write Config Chunk command
            writeSerial("=== WRITE CONFIG CHUNK COMMAND (0x0042) ===");
            handleWriteConfigChunk(data + 2, len - 2);
            break;
        case 0x000F: // Reboot
            writeSerial("=== Reboot COMMAND (0x000F) ===");
            delay(100);
            reboot();
            break;
        case 0x0043: // Firmware Version command
            writeSerial("=== FIRMWARE VERSION COMMAND (0x0043) ===");
            handleFirmwareVersion();
            break;
        case 0x0044: // Read MSD command
            writeSerial("=== READ MSD COMMAND (0x0044) ===");
            handleReadMSD();
            break;
        case 0x0070: // Direct Write Start command
            writeSerial("=== DIRECT WRITE START COMMAND (0x0070) ===");
            handleDirectWriteStart(data + 2, len - 2);
            break;
        case 0x0071: // Direct Write Data command
            handleDirectWriteData(data + 2, len - 2);
            break;
        case 0x0072: // Direct Write End command
            writeSerial("=== DIRECT WRITE END COMMAND (0x0072) ===");
            handleDirectWriteEnd(data + 2, len - 2);  // Pass data after command bytes (2 bytes for command)
            break;
        case 0x0073: // LED Activate command
            writeSerial("=== LED ACTIVATE COMMAND (0x0073) ===");
            handleLedActivate(data + 2, len - 2);
            break;
        case 0x0051: // Enter DFU Mode command
            writeSerial("=== ENTER DFU MODE COMMAND (0x0051) ===");
            // This command is already authenticated (passed through encryption gate)
            enterDFUMode();
            break;
        default:
            writeSerial("ERROR: Unknown command: 0x" + String(command, HEX));
            writeSerial("Expected: 0x0011 (read config), 0x0064 (image info), 0x0065 (block data), or 0x0003 (finalize)");
            break;
    }
    writeSerial("Command processing completed successfully");
}

void handleLedActivate(uint8_t* data, uint16_t len) {
    writeSerial("=== handleLedActivate START ===");
    writeSerial("Command length: " + String(len) + " bytes");
    // LED activation command format:
    // data[0]: LED instance number (0-based index)
    // data[1-12]: LED flash config (12 bytes, same format as reserved[0-11] in LedConfig)
    // If len == 1, only instance is provided (use existing config for that instance)
    // If len == 13, both instance and config are provided
    if (len < 1) {
        writeSerial("ERROR: LED activate command too short (len=" + String(len) + ", need at least 1 byte for instance)");
        uint8_t errorResponse[] = {0xFF, 0x73, 0x01, 0x00};  // Error, command, error code, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    uint8_t ledInstance = data[0];
    writeSerial("LED instance: " + String(ledInstance));
    String dataHex = "Data bytes: ";
    for (uint16_t i = 0; i < len && i < 20; i++) {
        if (i > 0) dataHex += " ";
        if (data[i] < 0x10) dataHex += "0";
        dataHex += String(data[i], HEX);
    }
    if (len > 20) dataHex += " ...";
    writeSerial(dataHex);
    writeSerial("Current activeLedInstance: 0x" + String(activeLedInstance, HEX));
    writeSerial("Current led_count: " + String(globalConfig.led_count));
    if (ledInstance >= globalConfig.led_count) {
        writeSerial("ERROR: LED instance " + String(ledInstance) + " out of range (led_count=" + String(globalConfig.led_count) + ")");
        uint8_t errorResponse[] = {0xFF, 0x73, 0x02, 0x00};  // Error, command, error code, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    struct LedConfig* led = &globalConfig.leds[ledInstance];
    activeLedInstance = ledInstance;
    writeSerial("Using LED instance " + String(ledInstance));
    writeSerial("LED config: type=" + String(led->led_type) + 
                " R=" + String(led->led_1_r) + 
                " G=" + String(led->led_2_g) + 
                " B=" + String(led->led_3_b));
    uint8_t* ledcfg = led->reserved;
    String currentCfg = "Current LED config: ";
    for (uint8_t i = 0; i < 12; i++) {
        if (i > 0) currentCfg += " ";
        if (ledcfg[i] < 0x10) currentCfg += "0";
        currentCfg += String(ledcfg[i], HEX);
    }
    writeSerial(currentCfg);
    if (len >= 13) {
        writeSerial("Updating LED flash config from command data...");
        memcpy(ledcfg, data + 1, 12);
        String newCfg = "New LED config: ";
        for (uint8_t i = 0; i < 12; i++) {
            if (i > 0) newCfg += " ";
            if (ledcfg[i] < 0x10) newCfg += "0";
            newCfg += String(ledcfg[i], HEX);
        }
        writeSerial(newCfg);
        uint8_t modeByte = ledcfg[0];
        uint8_t mode = modeByte & 0x0F;
        uint8_t brightnessRaw = (modeByte >> 4) & 0x0F;
        uint8_t brightness = brightnessRaw + 1;
        writeSerial("Mode byte 0x" + String(modeByte, HEX) + " decoded: mode=" + String(mode) + 
                    " brightness=" + String(brightness) + " (raw=" + String(brightnessRaw) + ")");
        if (mode == 0) {
            writeSerial("WARNING: Mode is 0 (disabled)! Set bit 0 to 1 for mode 1.");
            writeSerial("Example: 0x80 -> 0x81 (mode=1, brightness=8)");
        }
        
        writeSerial("LED flash config updated");
    } else {
        writeSerial("No config provided (len=" + String(len) + "), using existing config");
    }
    writeSerial("Activating LED flash (runs indefinitely until group repeats complete)");
    ledFlashActive = true;
    ledFlashPosition = 0;
    writeSerial("Calling ledFlashLogic()...");
    ledFlashLogic();
    writeSerial("ledFlashLogic returned");
    ledFlashActive = false;
    uint8_t successResponse[] = {0x00, 0x73, 0x00, 0x00};  // Success, command, no error, no data
    sendResponse(successResponse, sizeof(successResponse));
    writeSerial("LED flash completed, response sent");
    writeSerial("=== handleLedActivate END ===");
}