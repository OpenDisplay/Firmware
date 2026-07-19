#include "device_control.h"
#include "structs.h"
#include "touch_input.h"
#include "power_latch.h"
#include "buzzer_control.h"
#include <string.h>

#ifdef TARGET_ESP32
void enterDeepSleep(bool force = false, uint16_t overrideSleepSeconds = 0);
#endif

#ifdef TARGET_NRF
#include <bluefruit.h>
#include "ble_init.h"
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
void updatemsdata();
void cleanupDirectWriteState(bool refreshDisplay);
void cleanupPartialWriteOnDisconnect(void);
void resetPipeWriteState(void);
void sendResponse(uint8_t* response, uint16_t len);
void writeSerial(String message, bool newLine = true);

extern ButtonState buttonStates[MAX_BUTTONS];

#ifdef TARGET_ESP32
static bool s_pwrOffReleased[MAX_BUTTONS];
static bool s_pwrOffPressing[MAX_BUTTONS];
static bool s_pwrOffDone[MAX_BUTTONS];
static uint32_t s_pwrOffStartMs[MAX_BUTTONS];

static void pollConfiguredPowerOffButtons() {
    if (!powerLatchDffConfigured() && !powerLatchMosfetConfigured()) {
        return;
    }
    for (uint8_t i = 0; i < buttonStateCount; i++) {
        ButtonState* btn = &buttonStates[i];
        if (!btn->initialized || !btn->power_off) {
            continue;
        }
        bool pinState = digitalRead(btn->pin);
        bool down = btn->inverted ? !pinState : pinState;
        if (!down) {
            s_pwrOffReleased[i] = true;
            s_pwrOffPressing[i] = false;
            s_pwrOffDone[i] = false;
            continue;
        }
        if (!s_pwrOffReleased[i]) {
            continue;
        }
        if (!s_pwrOffPressing[i]) {
            s_pwrOffPressing[i] = true;
            s_pwrOffStartMs[i] = millis();
            continue;
        }
        if (!s_pwrOffDone[i] && millis() - s_pwrOffStartMs[i] >= btn->power_off_hold_ms) {
            s_pwrOffDone[i] = true;
            passiveBuzzerPowerOffAlert();
            powerLatchTriggerOff();
        }
    }
}
#endif

void connect_callback(uint16_t conn_handle) {
    (void)conn_handle;
    writeSerial("=== BLE CLIENT CONNECTED ===", true);
    rebootFlag = 0;
    updatemsdata();
#ifdef TARGET_NRF
    ble_nrf_log_link_params(conn_handle, "at connect");  // baseline (pre-negotiation)
    ble_nrf_request_fast_link(conn_handle);              // request 2M PHY + 251-octet DLE
    ble_nrf_arm_link_diag(conn_handle);                  // re-log once negotiation settles
#endif
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;
    writeSerial("=== BLE CLIENT DISCONNECTED ===", true);
    writeSerial("Disconnect reason: " + String(reason), true);
    // Panel power on disconnect follows the ACTIVE-only-teardown invariant, so no
    // logic change is needed here: a WARM (post-successful-refresh) panel SURVIVES
    // disconnect and keeps its keep-alive window — a reconnect within the window
    // pays only a warm re-acquire. Only a disconnect mid-transfer (still PWR_ACTIVE) tears the
    // panel down, via the cleanup calls below (which no-op on power when WARM).
    cleanupDirectWriteState(true);
    cleanupPartialWriteOnDisconnect();   // 0x76 / pipe-partial session bookkeeping + panel power
    resetPipeWriteState();   // clear any pipe transfer + reorder queue on disconnect
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

#define LED_DELAY_FACTOR_MS 100u
#define LED_MIN_STEP_DELAY_MS 1u

typedef enum {
    LED_PHASE_IDLE = 0,
    LED_PHASE_GROUP,
    LED_PHASE_LOOP1,
    LED_PHASE_LOOP1_DELAY,
    LED_PHASE_INTER1_DELAY,
    LED_PHASE_LOOP2,
    LED_PHASE_LOOP2_DELAY,
    LED_PHASE_INTER2_DELAY,
    LED_PHASE_LOOP3,
    LED_PHASE_LOOP3_DELAY,
    LED_PHASE_INTER3_DELAY,
} led_phase_t;

static struct {
    bool active;
    uint8_t instance;
    struct LedConfig* led;
    uint8_t brightness;
    uint8_t c1;
    uint8_t c2;
    uint8_t c3;
    uint8_t loop1delay;
    uint8_t loop2delay;
    uint8_t loop3delay;
    uint8_t loopcnt1;
    uint8_t loopcnt2;
    uint8_t loopcnt3;
    uint8_t ildelay1;
    uint8_t ildelay2;
    uint8_t ildelay3;
    uint8_t grouprepeats;
    uint8_t group_pos;
    uint8_t i1;
    uint8_t i2;
    uint8_t i3;
    led_phase_t phase;
    bool waiting_delay;
    uint32_t delay_until_ms;
} s_led;

static void led_all_off(struct LedConfig* led) {
    if (led == NULL) {
        return;
    }
    bool invertRed = (led->led_flags & 0x01) != 0;
    bool invertGreen = (led->led_flags & 0x02) != 0;
    bool invertBlue = (led->led_flags & 0x04) != 0;
    if (led->led_1_r != 0xFF) {
        digitalWrite(led->led_1_r, invertRed ? HIGH : LOW);
    }
    if (led->led_2_g != 0xFF) {
        digitalWrite(led->led_2_g, invertGreen ? HIGH : LOW);
    }
    if (led->led_3_b != 0xFF) {
        digitalWrite(led->led_3_b, invertBlue ? HIGH : LOW);
    }
}

static void led_stop_internal(bool clear_mode) {
    struct LedConfig* led = s_led.led;
    s_led.waiting_delay = false;
    if (led != NULL) {
        led_all_off(led);
        if (clear_mode) {
            led->reserved[0] = 0x00;
        }
    }
    memset(&s_led, 0, sizeof(s_led));
    ledFlashActive = false;
    activeLedInstance = 0xFF;
    ledFlashPosition = 0;
}

static void led_schedule_delay_ms(uint16_t ms) {
    if (ms == 0) {
        s_led.waiting_delay = false;
        return;
    }
    s_led.waiting_delay = true;
    s_led.delay_until_ms = millis() + ms;
}

static void led_load_config(struct LedConfig* led) {
    uint8_t* ledcfg = led->reserved;
    s_led.led = led;
    s_led.brightness = (uint8_t)(((ledcfg[0] >> 4) & 0x0F) + 1);
    s_led.c1 = ledcfg[1];
    s_led.c2 = ledcfg[4];
    s_led.c3 = ledcfg[7];
    s_led.loop1delay = (uint8_t)((ledcfg[2] >> 4) & 0x0F);
    s_led.loop2delay = (uint8_t)((ledcfg[5] >> 4) & 0x0F);
    s_led.loop3delay = (uint8_t)((ledcfg[8] >> 4) & 0x0F);
    s_led.loopcnt1 = (uint8_t)(ledcfg[2] & 0x0F);
    s_led.loopcnt2 = (uint8_t)(ledcfg[5] & 0x0F);
    s_led.loopcnt3 = (uint8_t)(ledcfg[8] & 0x0F);
    s_led.ildelay1 = ledcfg[3];
    s_led.ildelay2 = ledcfg[6];
    s_led.ildelay3 = ledcfg[9];
    s_led.grouprepeats = (uint8_t)(ledcfg[10] + 1);
    s_led.group_pos = 0;
    s_led.i1 = 0;
    s_led.i2 = 0;
    s_led.i3 = 0;
    s_led.phase = LED_PHASE_GROUP;
    s_led.waiting_delay = false;
}

static void led_run_finish(void) {
    led_stop_internal(false);
}

static void led_run_step(void) {
    struct LedConfig* led;
    uint8_t mode;

    if (!s_led.active || s_led.led == NULL) {
        return;
    }
    led = s_led.led;
    activeLedInstance = s_led.instance;
    mode = (uint8_t)(led->reserved[0] & 0x0F);
    if (mode != 1) {
        led_run_finish();
        return;
    }

    for (;;) {
        if (!s_led.active) {
            return;
        }

        switch (s_led.phase) {
            case LED_PHASE_GROUP:
                if (s_led.group_pos >= s_led.grouprepeats && s_led.grouprepeats != 255) {
                    led->reserved[0] = 0x00;
                    led_run_finish();
                    return;
                }
                s_led.i1 = 0;
                s_led.i2 = 0;
                s_led.i3 = 0;
                s_led.phase = LED_PHASE_LOOP1;
                break;

            case LED_PHASE_LOOP1:
                if (s_led.i1 >= s_led.loopcnt1) {
                    if (s_led.ildelay1 > 0) {
                        s_led.phase = LED_PHASE_INTER1_DELAY;
                        led_schedule_delay_ms((uint16_t)(s_led.ildelay1 * LED_DELAY_FACTOR_MS));
                        return;
                    }
                    s_led.phase = LED_PHASE_LOOP2;
                    break;
                }
                flashLed(s_led.c1, s_led.brightness);
                s_led.i1++;
                if (s_led.loop1delay > 0) {
                    s_led.phase = LED_PHASE_LOOP1_DELAY;
                    led_schedule_delay_ms((uint16_t)(s_led.loop1delay * LED_DELAY_FACTOR_MS));
                } else {
                    led_schedule_delay_ms(LED_MIN_STEP_DELAY_MS);
                }
                return;

            case LED_PHASE_LOOP1_DELAY:
                s_led.phase = LED_PHASE_LOOP1;
                break;

            case LED_PHASE_INTER1_DELAY:
                s_led.phase = LED_PHASE_LOOP2;
                break;

            case LED_PHASE_LOOP2:
                if (s_led.i2 >= s_led.loopcnt2) {
                    if (s_led.ildelay2 > 0) {
                        s_led.phase = LED_PHASE_INTER2_DELAY;
                        led_schedule_delay_ms((uint16_t)(s_led.ildelay2 * LED_DELAY_FACTOR_MS));
                        return;
                    }
                    s_led.phase = LED_PHASE_LOOP3;
                    break;
                }
                flashLed(s_led.c2, s_led.brightness);
                s_led.i2++;
                if (s_led.loop2delay > 0) {
                    s_led.phase = LED_PHASE_LOOP2_DELAY;
                    led_schedule_delay_ms((uint16_t)(s_led.loop2delay * LED_DELAY_FACTOR_MS));
                } else {
                    led_schedule_delay_ms(LED_MIN_STEP_DELAY_MS);
                }
                return;

            case LED_PHASE_LOOP2_DELAY:
                s_led.phase = LED_PHASE_LOOP2;
                break;

            case LED_PHASE_INTER2_DELAY:
                s_led.phase = LED_PHASE_LOOP3;
                break;

            case LED_PHASE_LOOP3:
                if (s_led.i3 >= s_led.loopcnt3) {
                    if (s_led.ildelay3 > 0) {
                        s_led.phase = LED_PHASE_INTER3_DELAY;
                        led_schedule_delay_ms((uint16_t)(s_led.ildelay3 * LED_DELAY_FACTOR_MS));
                        return;
                    }
                    s_led.group_pos++;
                    s_led.phase = LED_PHASE_GROUP;
                    led_schedule_delay_ms(LED_MIN_STEP_DELAY_MS);
                    return;
                }
                flashLed(s_led.c3, s_led.brightness);
                s_led.i3++;
                if (s_led.loop3delay > 0) {
                    s_led.phase = LED_PHASE_LOOP3_DELAY;
                    led_schedule_delay_ms((uint16_t)(s_led.loop3delay * LED_DELAY_FACTOR_MS));
                } else {
                    led_schedule_delay_ms(LED_MIN_STEP_DELAY_MS);
                }
                return;

            case LED_PHASE_LOOP3_DELAY:
                s_led.phase = LED_PHASE_LOOP3;
                break;

            case LED_PHASE_INTER3_DELAY:
                s_led.group_pos++;
                s_led.phase = LED_PHASE_GROUP;
                break;

            default:
                led_run_finish();
                return;
        }
    }
}

void processLedFlash() {
    if (!s_led.active) {
        return;
    }
    if (s_led.waiting_delay) {
        if ((int32_t)(millis() - s_led.delay_until_ms) < 0) {
            return;
        }
        s_led.waiting_delay = false;
    }
    led_run_step();
}

void handleLedActivate(uint8_t* data, uint16_t len) {
    if (len < 1) {
        uint8_t errorResponse[] = {RESP_NACK, RESP_LED_ACTIVATE_ACK, 0x01, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    uint8_t ledInstance = data[0];
    if (ledInstance >= globalConfig.led_count) {
        uint8_t errorResponse[] = {RESP_NACK, RESP_LED_ACTIVATE_ACK, 0x02, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    struct LedConfig* led = &globalConfig.leds[ledInstance];
    if (len >= 13) {
        memcpy(led->reserved, data + 1, 12);
    }
    uint8_t mode = (uint8_t)(led->reserved[0] & 0x0F);
    if (mode != 1) {
        led_stop_internal(true);
        uint8_t successResponse[] = {RESP_ACK, RESP_LED_ACTIVATE_ACK, 0x00, 0x00};
        sendResponse(successResponse, sizeof(successResponse));
        return;
    }

    led_stop_internal(false);
    s_led.active = true;
    s_led.instance = ledInstance;
    activeLedInstance = ledInstance;
    ledFlashActive = true;
    ledFlashPosition = 0;
    led_load_config(led);
    led_run_step();

    uint8_t successResponse[] = {RESP_ACK, RESP_LED_ACTIVATE_ACK, 0x00, 0x00};
    sendResponse(successResponse, sizeof(successResponse));
}

void handleLedStop(uint8_t* data, uint16_t len) {
    if (s_led.active && len >= 1 && data[0] != s_led.instance) {
        uint8_t errorResponse[] = {RESP_NACK, RESP_LED_STOP_ACK, 0x02, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    led_stop_internal(true);
    uint8_t successResponse[] = {RESP_ACK, RESP_LED_STOP_ACK, 0x00, 0x00};
    sendResponse(successResponse, sizeof(successResponse));
}

void processButtonEvents() {
    powerButtonPoll();
#ifdef TARGET_ESP32
    pollConfiguredPowerOffButtons();
#endif
    if (buttonEventPending) {
        noInterrupts();
        buttonEventPending = false;
        uint8_t changedButtonIndex = lastChangedButtonIndex;
        lastChangedButtonIndex = 0xFF;
        interrupts();
        writeSerial("Button event pending: " + String(changedButtonIndex));
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
#ifdef TARGET_NRF
        ble_nrf_boost_advertising();
#endif
    }
}

static inline void ledFlashWrite(uint8_t pin, bool level) {
    if (pin != 0xFF) {
        digitalWrite(pin, level ? HIGH : LOW);
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
        ledFlashWrite(ledRedPin,invertRed ? !(colorred >= 7) : (colorred >= 7));
        ledFlashWrite(ledGreenPin,invertGreen ? !(colorgreen >= 7) : (colorgreen >= 7));
        ledFlashWrite(ledBluePin,invertBlue ? !(colorblue >= 3) : (colorblue >= 3));
        delayMicroseconds(100);
        ledFlashWrite(ledRedPin,invertRed ? !(colorred >= 1) : (colorred >= 1));
        ledFlashWrite(ledGreenPin,invertGreen ? !(colorgreen >= 1) : (colorgreen >= 1));
        delayMicroseconds(100);
        ledFlashWrite(ledRedPin,invertRed ? !(colorred >= 6) : (colorred >= 6));
        ledFlashWrite(ledGreenPin,invertGreen ? !(colorgreen >= 6) : (colorgreen >= 6));
        ledFlashWrite(ledBluePin,invertBlue ? !(colorblue >= 1) : (colorblue >= 1));
        delayMicroseconds(100);
        ledFlashWrite(ledRedPin,invertRed ? !(colorred >= 2) : (colorred >= 2));
        ledFlashWrite(ledGreenPin,invertGreen ? !(colorgreen >= 2) : (colorgreen >= 2));
        delayMicroseconds(100);
        ledFlashWrite(ledRedPin,invertRed ? !(colorred >= 5) : (colorred >= 5));
        ledFlashWrite(ledGreenPin,invertGreen ? !(colorgreen >= 5) : (colorgreen >= 5));
        delayMicroseconds(100);
        ledFlashWrite(ledRedPin,invertRed ? !(colorred >= 3) : (colorred >= 3));
        ledFlashWrite(ledGreenPin,invertGreen ? !(colorgreen >= 3) : (colorgreen >= 3));
        ledFlashWrite(ledBluePin,invertBlue ? !(colorblue >= 2) : (colorblue >= 2));
        delayMicroseconds(100);
        ledFlashWrite(ledRedPin,invertRed ? !(colorred >= 4) : (colorred >= 4));
        ledFlashWrite(ledGreenPin,invertGreen ? !(colorgreen >= 4) : (colorgreen >= 4));
        delayMicroseconds(100);
        ledFlashWrite(ledRedPin,invertRed ? HIGH : LOW);
        ledFlashWrite(ledGreenPin,invertGreen ? HIGH : LOW);
        ledFlashWrite(ledBluePin,invertBlue ? HIGH : LOW);
    }
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
        if (pressed) btn->press_count = (btn->press_count + 1) & 0x0F;
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
        buttonStates[i].current_state = 0;
        buttonStates[i].byte_index = 0xFF;
        buttonStates[i].pin = 0xFF;
        buttonStates[i].instance_index = 0xFF;
        buttonStates[i].power_off = false;
        buttonStates[i].power_off_hold_ms = 0;
    }
    if (globalConfig.binary_input_count == 0) return;
    for (uint8_t instanceIdx = 0; instanceIdx < globalConfig.binary_input_count; instanceIdx++) {
        struct BinaryInputs* input = &globalConfig.binary_inputs[instanceIdx];
        if (input->input_type != 1) continue;
        if (input->button_data_byte_index > 10) continue;
        uint16_t instanceHoldMs = (input->power_off_hold_sec == 0) ? 3000u : (uint16_t)input->power_off_hold_sec * 1000u;
        uint8_t* instancePins[8] = {
            &input->reserved_pin_1,&input->reserved_pin_2,&input->reserved_pin_3,&input->reserved_pin_4,
            &input->reserved_pin_5,&input->reserved_pin_6,&input->reserved_pin_7,&input->reserved_pin_8
        };
        for (uint8_t pinIdx = 0; pinIdx < 8; pinIdx++) {
            if (input->input_flags != 0 && (input->input_flags & (1 << pinIdx)) == 0) {
                continue;
            }
            uint8_t pin = *instancePins[pinIdx];
            if (pin == 0xFF) continue;
            if (touch_input_gpio_is_touch_int(pin)) {
                writeSerial("Button: skip pin " + String(pin) + " (reserved for GT911 INT)", true);
                continue;
            }
            if (buttonStateCount >= MAX_BUTTONS) break;
            ButtonState* btn = &buttonStates[buttonStateCount];
            btn->button_id = (input->instance_number * 8) + pinIdx;
            if (btn->button_id > 7) btn->button_id = btn->button_id % 8;
            btn->byte_index = input->button_data_byte_index;
            btn->pin = pin;
            btn->instance_index = instanceIdx;
            btn->press_count = 0;
            btn->pin_offset = pinIdx;
            btn->inverted = (input->invert & (1 << pinIdx)) != 0;
            btn->power_off = (input->power_off_flags & (1 << pinIdx)) != 0;
            btn->power_off_hold_ms = instanceHoldMs;
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

void handleDeepSleepCommand(const uint8_t* payload, uint16_t payloadLen) {
    writeSerial("=== DEEP SLEEP COMMAND (0x0052) ===", true);
#ifdef TARGET_ESP32
    // Optional 2-byte big-endian seconds payload overrides the configured
    // deep-sleep duration for exactly one cycle. 0x0000 = explicit no-override.
    uint16_t overrideSeconds = 0;
    if (payloadLen >= 2) {
        overrideSeconds = ((uint16_t)payload[0] << 8) | payload[1];
        // Bytes beyond 2 ignored for forward compatibility.
    } else if (payloadLen == 1) {
        writeSerial("WARNING: malformed 0x0052 payload length 1 - ignoring", true);
    }
    if (powerLatchDffConfigured()) {
        if (overrideSeconds != 0) {
            writeSerial("0x0052 duration payload ignored (D-FF hard power off has no timer)", true);
        }
        uint8_t ok[] = {RESP_ACK, RESP_DEEP_SLEEP, 0x00, 0x00};
        sendResponse(ok, sizeof(ok));
        delay(100);
        powerLatchPowerOff();
        return;
    }
    if (globalConfig.power_option.power_mode != 1) {
        writeSerial("Device not battery powered - 0x0052 rejected", true);
        uint8_t errorResponse[] = {RESP_NACK, RESP_DEEP_SLEEP, 0x02, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    if (globalConfig.power_option.deep_sleep_time_seconds == 0) {
        writeSerial("Deep sleep disabled in config - 0x0052 rejected", true);
        uint8_t errorResponse[] = {RESP_NACK, RESP_DEEP_SLEEP, 0x01, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    // Explicit host request: sleep even though the requesting client is connected.
    enterDeepSleep(true, overrideSeconds);
#else
    (void)payload;
    (void)payloadLen;
    writeSerial("Deep sleep command not supported on this target", true);
#endif
}
