#include "device_control.h"
#include "structs.h"
#include <string.h>

#ifdef TARGET_NRF
#include <bluefruit.h>
extern "C" {
#include "nrf_soc.h"
}
extern "C" void bootloader_util_app_start(uint32_t start_addr);
#endif
#ifdef TARGET_ESP32
#include <esp_system.h>
#include "driver/gpio.h"
#include "esp32-hal-gpio.h"
#endif

extern uint8_t rebootFlag;
extern struct GlobalConfig globalConfig;
extern uint8_t activeLedInstance;
extern bool ledFlashActive;
extern uint8_t ledFlashPosition;
extern uint8_t dynamicreturndata[11];
extern uint8_t buttonStateCount;
extern volatile bool buttonEventPending;
extern volatile uint8_t lastChangedButtonIndex;
extern "C" uint8_t pinToButtonIndex[64];
void updatemsdata();
void cleanupDirectWriteState(bool refreshDisplay);
void sendResponse(uint8_t* response, uint8_t len);
void writeSerial(String message, bool newLine = true);

struct ButtonState {
    uint8_t button_id;
    uint8_t press_count;
    volatile uint32_t last_press_time;
    volatile uint8_t current_state;
    uint8_t byte_index;
    uint8_t pin;
    uint8_t instance_index;
    bool initialized;
    uint8_t pin_offset;
    bool inverted;
};
#define MAX_BUTTONS 32
extern ButtonState buttonStates[MAX_BUTTONS];

void connect_callback(uint16_t conn_handle) {
    (void)conn_handle;
    writeSerial("=== BLE CLIENT CONNECTED ===", true);
    rebootFlag = 0;
    updatemsdata();
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;
    writeSerial("=== BLE CLIENT DISCONNECTED ===", true);
    writeSerial("Disconnect reason: " + String(reason), true);
    cleanupDirectWriteState(true);
}

void reboot(){
    writeSerial("=== REBOOT COMMAND (0x000F) ===", true);
    delay(100);
#ifdef TARGET_NRF
    NVIC_SystemReset();
#endif
#ifdef TARGET_ESP32
    esp_restart();
#endif
}

void handleLedActivate(uint8_t* data, uint16_t len) {
    writeSerial("=== handleLedActivate START ===", true);
    writeSerial("Command length: " + String(len) + " bytes", true);
    if (len < 1) {
        writeSerial("ERROR: LED activate command too short (len=" + String(len) + ", need at least 1 byte for instance)", true);
        uint8_t errorResponse[] = {0xFF, 0x73, 0x01, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    uint8_t ledInstance = data[0];
    writeSerial("LED instance: " + String(ledInstance), true);
    if (ledInstance >= globalConfig.led_count) {
        writeSerial("ERROR: LED instance " + String(ledInstance) + " out of range (led_count=" + String(globalConfig.led_count) + ")", true);
        uint8_t errorResponse[] = {0xFF, 0x73, 0x02, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    struct LedConfig* led = &globalConfig.leds[ledInstance];
    activeLedInstance = ledInstance;
    uint8_t* ledcfg = led->reserved;
    if (len >= 13) {
        memcpy(ledcfg, data + 1, 12);
    }
    ledFlashActive = true;
    ledFlashPosition = 0;
    ledFlashLogic();
    ledFlashActive = false;
    uint8_t successResponse[] = {0x00, 0x73, 0x00, 0x00};
    sendResponse(successResponse, sizeof(successResponse));
    writeSerial("=== handleLedActivate END ===", true);
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
                    resetCount = true;
                }
                if (btn->last_press_time > 0) {
                    for (uint8_t j = 0; j < buttonStateCount; j++) {
                        if (j != changedButtonIndex && buttonStates[j].initialized &&
                            buttonStates[j].last_press_time > 0 &&
                            buttonStates[j].last_press_time > btn->last_press_time &&
                            (currentTime - buttonStates[j].last_press_time) < 5000) {
                            resetCount = true;
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
            uint8_t buttonData = (btn->button_id & 0x07) |
                                 ((btn->press_count & 0x0F) << 3) |
                                 ((btn->current_state & 0x01) << 7);
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
            if (globalConfig.leds[i].led_type == 1) {
                activeLedInstance = i;
                break;
            }
        }
        if (activeLedInstance == 0xFF) return;
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
    if (!ledFlashActive) return;
    if (activeLedInstance == 0xFF) {
        for (uint8_t i = 0; i < globalConfig.led_count; i++) {
            if (globalConfig.leds[i].led_type == 1) {
                activeLedInstance = i;
                break;
            }
        }
        if (activeLedInstance == 0xFF) return;
    }
    struct LedConfig* led = &globalConfig.leds[activeLedInstance];
    uint8_t* ledcfg = led->reserved;
    uint8_t brightness = ((ledcfg[0] >> 4) & 0x0F) + 1;
    uint8_t mode = ledcfg[0] & 0x0F;
    if (mode == 1) {
        const uint8_t interloopdelayfactor = 100;
        const uint8_t loopdelayfactor = 100;
        uint8_t c1 = ledcfg[1], c2 = ledcfg[4], c3 = ledcfg[7];
        uint8_t loop1delay = (ledcfg[2] >> 4) & 0x0F, loop2delay = (ledcfg[5] >> 4) & 0x0F, loop3delay = (ledcfg[8] >> 4) & 0x0F;
        uint8_t loopcnt1 = ledcfg[2] & 0x0F, loopcnt2 = ledcfg[5] & 0x0F, loopcnt3 = ledcfg[8] & 0x0F;
        uint8_t ildelay1 = ledcfg[3], ildelay2 = ledcfg[6], ildelay3 = ledcfg[9];
        uint8_t grouprepeats = ledcfg[10] + 1;
        while (ledFlashActive) {
            if (ledFlashPosition >= grouprepeats && grouprepeats != 255) {
                brightness = 0;
                ledcfg[0] = 0x00;
                ledFlashPosition = 0;
                break;
            }
            for (int i = 0; i < loopcnt1; i++) { flashLed(c1, brightness); delay(loop1delay * loopdelayfactor); }
            delay(ildelay1 * interloopdelayfactor);
            for (int i = 0; i < loopcnt2; i++) { flashLed(c2, brightness); delay(loop2delay * loopdelayfactor); }
            delay(ildelay2 * interloopdelayfactor);
            for (int i = 0; i < loopcnt3; i++) { flashLed(c3, brightness); delay(loop3delay * loopdelayfactor); }
            delay(ildelay3 * interloopdelayfactor);
            ledFlashPosition++;
        }
    }
    writeSerial("=== ledFlashLogic END ===");
}

#ifdef TARGET_ESP32
void IRAM_ATTR handleButtonISR(uint8_t buttonIndex) {
#else
void handleButtonISR(uint8_t buttonIndex) {
#endif
    if (buttonIndex >= MAX_BUTTONS || !buttonStates[buttonIndex].initialized) return;
    ButtonState* btn = &buttonStates[buttonIndex];
    bool pinState = digitalRead(btn->pin);
    bool pressed = btn->inverted ? !pinState : pinState;
    uint8_t newState = pressed ? 1 : 0;
    if (newState != btn->current_state) {
        btn->current_state = newState;
        lastChangedButtonIndex = buttonIndex;
        if (pressed && btn->press_count < 15) btn->press_count++;
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
    if (globalConfig.binary_input_count == 0) return;
    for (uint8_t instanceIdx = 0; instanceIdx < globalConfig.binary_input_count; instanceIdx++) {
        struct BinaryInputs* input = &globalConfig.binary_inputs[instanceIdx];
        if (input->input_type != 1) continue;
        if (input->button_data_byte_index > 10) continue;
        uint8_t* instancePins[8] = {
            &input->reserved_pin_1,&input->reserved_pin_2,&input->reserved_pin_3,&input->reserved_pin_4,
            &input->reserved_pin_5,&input->reserved_pin_6,&input->reserved_pin_7,&input->reserved_pin_8
        };
        for (uint8_t pinIdx = 0; pinIdx < 8; pinIdx++) {
            uint8_t pin = *instancePins[pinIdx];
            if (pin == 0xFF) continue;
            if (buttonStateCount >= MAX_BUTTONS) break;
            ButtonState* btn = &buttonStates[buttonStateCount];
            btn->button_id = (input->instance_number * 8) + pinIdx;
            if (btn->button_id > 7) btn->button_id = btn->button_id % 8;
            btn->byte_index = input->button_data_byte_index;
            btn->pin = pin;
            btn->instance_index = instanceIdx;
            btn->press_count = 0;
            btn->last_press_time = 0;
            btn->pin_offset = pinIdx;
            btn->inverted = (input->invert & (1 << pinIdx)) != 0;
            pinMode(pin, INPUT);
            bool hasPullup = (input->pullups & (1 << pinIdx)) != 0;
#ifdef TARGET_ESP32
            bool hasPulldown = (input->pulldowns & (1 << pinIdx)) != 0;
            if (hasPullup) pinMode(pin, INPUT_PULLUP);
            else if (hasPulldown) pinMode(pin, INPUT_PULLDOWN);
#elif defined(TARGET_NRF)
            if (hasPullup) pinMode(pin, INPUT_PULLUP);
#endif
            delay(10);
            bool initialPinState = digitalRead(pin);
            bool initialPressed = btn->inverted ? !initialPinState : initialPinState;
            btn->current_state = initialPressed ? 1 : 0;
#ifdef TARGET_ESP32
            attachInterruptArg(pin, buttonISR, (void*)(uintptr_t)buttonStateCount, CHANGE);
#elif defined(TARGET_NRF)
            attachInterrupt(pin, buttonISRGeneric, CHANGE);
#endif
            btn->initialized = true;
            buttonStateCount++;
        }
    }
    if (buttonStateCount > 0) {
#ifdef TARGET_ESP32
        for (uint8_t i = 0; i < buttonStateCount; i++) {
            if (buttonStates[i].initialized) {
                gpio_intr_disable((gpio_num_t)digitalPinToGPIONumber(buttonStates[i].pin));
            }
        }
#elif defined(TARGET_NRF)
        for (uint8_t i = 0; i < buttonStateCount; i++) {
            if (buttonStates[i].initialized) {
                detachInterrupt(buttonStates[i].pin);
            }
        }
#endif
        delay(50);
        buttonEventPending = false;
        lastChangedButtonIndex = 0xFF;
        for (uint8_t i = 0; i < buttonStateCount; i++) {
            if (!buttonStates[i].initialized) continue;
            ButtonState* btn = &buttonStates[i];
            bool pinState = digitalRead(btn->pin);
            bool initialPressed = btn->inverted ? !pinState : pinState;
            btn->current_state = initialPressed ? 1 : 0;
            btn->press_count = 0;
            btn->last_press_time = 0;
        }
#ifdef TARGET_ESP32
        for (uint8_t i = 0; i < buttonStateCount; i++) {
            if (buttonStates[i].initialized) {
                gpio_intr_enable((gpio_num_t)digitalPinToGPIONumber(buttonStates[i].pin));
            }
        }
#elif defined(TARGET_NRF)
        for (uint8_t i = 0; i < buttonStateCount; i++) {
            if (!buttonStates[i].initialized) continue;
            uint8_t pin = buttonStates[i].pin;
            attachInterrupt(pin, buttonISRGeneric, CHANGE);
        }
#endif
    }
}

void enterDFUMode() {
    writeSerial("=== ENTER DFU MODE COMMAND (0x0051) ===", true);

#ifdef TARGET_NRF
    writeSerial("Preparing to enter DFU bootloader mode...", true);

    Bluefruit.Advertising.restartOnDisconnect(false);

    if (Bluefruit.connected()) {
        writeSerial("Disconnecting BLE...", true);
        Bluefruit.disconnect(Bluefruit.connHandle());
        delay(100);
    }

    sd_power_gpregret_clr(0, 0xFF);
    sd_power_gpregret_set(0, 0xB1);

    sd_softdevice_disable();

    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICPR[0] = 0xFFFFFFFF;
#if defined(__NRF_NVIC_ISER_COUNT) && __NRF_NVIC_ISER_COUNT == 2
    NVIC->ICER[1] = 0xFFFFFFFF;
    NVIC->ICPR[1] = 0xFFFFFFFF;
#endif

    sd_softdevice_vector_table_base_set(NRF_UICR->NRFFW[0]);
    __set_CONTROL(0);
    bootloader_util_app_start(NRF_UICR->NRFFW[0]);

    while (1) {}
#endif

#ifdef TARGET_ESP32
    writeSerial("ESP32: Rebooting (OTA typically handled via WiFi)", true);
    delay(100);
    esp_restart();
#endif
}
