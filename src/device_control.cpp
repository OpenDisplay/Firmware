#include "device_control.h"
#include "structs.h"
#include "touch_input.h"
#include "power_latch.h"
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

// BinaryInputs.input_type wire values (see structs.h for the full contract).
// 2 is reserved for switches (host-side feature); the ADC ladder uses 3.
#define BINARY_INPUT_TYPE_DIGITAL    1
#define BINARY_INPUT_TYPE_ADC_LADDER 3

// --- ADC resistor-ladder buttons (e.g. XTEINK X4) -------------------------
// Several buttons share one ADC pin via a resistor ladder, distinguished by
// voltage. They have no edge interrupt, so they are polled. Reported through
// the same MSD button byte as digital buttons for a uniform host contract.
#ifdef TARGET_ESP32
#define MAX_ADC_LADDERS     4
#define MAX_LADDER_BUTTONS  5    // reserved[] holds at most N+1 = 6 LE uint16 thresholds
#define ADC_LADDER_POLL_MS  5
#define ADC_LADDER_DEBOUNCE 3    // consecutive equal samples required to accept a change

// Thresholds for N+1 buttons must fit in BinaryInputs.reserved[]; fail the build if not.
static_assert(2 + 2 * (MAX_LADDER_BUTTONS + 1) <= sizeof(BinaryInputs::reserved),
              "ADC ladder thresholds would overflow BinaryInputs.reserved[]");

struct AdcLadder {
    uint8_t  pin;
    uint8_t  num_buttons;
    uint8_t  id_base;
    uint8_t  byte_index;
    uint16_t thresholds[MAX_LADDER_BUTTONS + 1];  // descending; [0] = idle ceiling
    int8_t   current_button;     // -1 = none pressed
    int8_t   candidate_button;   // debounce: last raw classification
    uint8_t  candidate_count;    // consecutive samples equal to candidate
    uint8_t  press_count;        // 0-15, increments per press (5 s reset window)
    uint8_t  last_button_id;     // id of most recent press (for clean release reporting)
    uint32_t last_press_time;
};
static AdcLadder adcLadders[MAX_ADC_LADDERS];
static uint8_t   adcLadderCount = 0;

// Returns button index 0..num_buttons-1, or -1 when nothing is pressed.
static int classifyAdcLadder(int adc, const AdcLadder* l) {
    if (adc > (int)l->thresholds[0]) return -1;            // above idle ceiling
    for (uint8_t i = 0; i < l->num_buttons; i++) {
        if (adc > (int)l->thresholds[i + 1]) return (int)i; // thr[i+1] < adc <= thr[i]
    }
    return (int)l->num_buttons - 1;                        // catch-all bottom bucket
}

static void registerAdcLadder(const struct BinaryInputs* input) {
    if (adcLadderCount >= MAX_ADC_LADDERS) return;
    uint8_t n = input->reserved[0];                        // num_buttons
    if (n == 0 || n > MAX_LADDER_BUTTONS) {
        writeSerial("ADC ladder: count " + String(n) + " out of range 1.." +
                    String(MAX_LADDER_BUTTONS) + " on pin " + String(input->reserved_pin_1) + ", skipping", true);
        return;
    }
    if (input->button_data_byte_index > 10) {              // index into the 11-byte MSD block
        writeSerial("ADC ladder: byte_index " + String(input->button_data_byte_index) +
                    " out of range 0..10 on pin " + String(input->reserved_pin_1) + ", skipping", true);
        return;
    }
    if ((int)input->reserved[1] + n > 8) {                 // 3-bit id field: id_base..id_base+n-1 must be <= 7
        writeSerial("ADC ladder: id_base " + String(input->reserved[1]) + " + count " + String(n) +
                    " exceeds 3-bit id space on pin " + String(input->reserved_pin_1) + ", skipping", true);
        return;
    }
    AdcLadder* l = &adcLadders[adcLadderCount];
    l->pin = input->reserved_pin_1;                        // ADC GPIO
    l->num_buttons = n;
    l->id_base = input->reserved[1];
    l->byte_index = input->button_data_byte_index;
    for (uint8_t k = 0; k <= n; k++) {
        l->thresholds[k] = (uint16_t)input->reserved[2 + 2 * k] |
                           ((uint16_t)input->reserved[3 + 2 * k] << 8);
    }
    for (uint8_t k = 0; k < n; k++) {                      // contract: thresholds strictly descending
        if (l->thresholds[k] <= l->thresholds[k + 1]) {    // reject malformed config from any host
            writeSerial("ADC ladder: thresholds not strictly descending on pin " +
                        String(input->reserved_pin_1) + ", skipping", true);
            return;
        }
    }
    l->current_button = -1;
    l->candidate_button = -1;
    l->candidate_count = 0;
    l->press_count = 0;
    l->last_button_id = (uint8_t)(l->id_base & 0x07);
    l->last_press_time = 0;
    pinMode(l->pin, INPUT);
    analogSetPinAttenuation(l->pin, ADC_11db);
    adcLadderCount++;
    writeSerial("ADC ladder: pin " + String(l->pin) + " n " + String(n) +
                " idBase " + String(l->id_base) + " byteIdx " + String(l->byte_index), true);
}

static void pollAdcButtons() {
    if (adcLadderCount == 0) return;
    static uint32_t lastPoll = 0;
    uint32_t now = millis();
    if (now - lastPoll < ADC_LADDER_POLL_MS) return;
    lastPoll = now;
    for (uint8_t i = 0; i < adcLadderCount; i++) {
        AdcLadder* l = &adcLadders[i];
        int adc = analogRead(l->pin);
        int btn = classifyAdcLadder(adc, l);
        if (btn == l->candidate_button) {
            if (l->candidate_count < 255) l->candidate_count++;
        } else {
            l->candidate_button = (int8_t)btn;
            l->candidate_count = 1;
        }
        if (l->candidate_count < ADC_LADDER_DEBOUNCE) continue;  // not yet stable
        if (btn == l->current_button) continue;                 // no change
        uint8_t state;
        if (btn >= 0) {
            if (l->last_press_time == 0 || now - l->last_press_time > 5000) l->press_count = 0;
            l->press_count = (uint8_t)((l->press_count + 1) & 0x0F);
            l->last_press_time = now;
            l->last_button_id = (uint8_t)((l->id_base + btn) & 0x07);
            state = 1;
        } else {
            state = 0;  // released; last_button_id identifies which button
        }
        l->current_button = (int8_t)btn;
        uint8_t data = (uint8_t)((l->last_button_id & 0x07) |
                                 ((l->press_count & 0x0F) << 3) |
                                 ((state & 0x01) << 7));
        if (l->byte_index < 11) dynamicreturndata[l->byte_index] = data;
        updatemsdata();
        writeSerial("ADC btn pin " + String(l->pin) + " adc=" + String(adc) + " idx=" + String(btn) +
                    " id=" + String(l->last_button_id) + " cnt=" + String(l->press_count) +
                    " state=" + String(state), true);
    }
}
#else
static void pollAdcButtons() {}
#endif

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

#define LED_DELAY_FACTOR_MS 100u

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
                    return;
                }
                break;

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
                    return;
                }
                break;

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
                    break;
                }
                flashLed(s_led.c3, s_led.brightness);
                s_led.i3++;
                if (s_led.loop3delay > 0) {
                    s_led.phase = LED_PHASE_LOOP3_DELAY;
                    led_schedule_delay_ms((uint16_t)(s_led.loop3delay * LED_DELAY_FACTOR_MS));
                    return;
                }
                break;

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
        uint8_t errorResponse[] = {0xFF, 0x73, 0x01, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    uint8_t ledInstance = data[0];
    if (ledInstance >= globalConfig.led_count) {
        uint8_t errorResponse[] = {0xFF, 0x73, 0x02, 0x00};
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
        uint8_t successResponse[] = {0x00, 0x73, 0x00, 0x00};
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

    uint8_t successResponse[] = {0x00, 0x73, 0x00, 0x00};
    sendResponse(successResponse, sizeof(successResponse));
}

void handleLedStop(uint8_t* data, uint16_t len) {
    if (s_led.active && len >= 1 && data[0] != s_led.instance) {
        uint8_t errorResponse[] = {0xFF, 0x75, 0x02, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    led_stop_internal(true);
    uint8_t successResponse[] = {0x00, 0x75, 0x00, 0x00};
    sendResponse(successResponse, sizeof(successResponse));
}

void processButtonEvents() {
    powerButtonPoll();
    pollAdcButtons();
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
#ifdef TARGET_ESP32
    adcLadderCount = 0;
#endif
    if (globalConfig.binary_input_count == 0) return;
    for (uint8_t instanceIdx = 0; instanceIdx < globalConfig.binary_input_count; instanceIdx++) {
        struct BinaryInputs* input = &globalConfig.binary_inputs[instanceIdx];
#ifdef TARGET_ESP32
        if (input->input_type == BINARY_INPUT_TYPE_ADC_LADDER) {
            registerAdcLadder(input);
            continue;
        }
#endif
        if (input->input_type != BINARY_INPUT_TYPE_DIGITAL) continue;
        if (input->button_data_byte_index > 10) continue;
        uint8_t* instancePins[8] = {
            &input->reserved_pin_1,&input->reserved_pin_2,&input->reserved_pin_3,&input->reserved_pin_4,
            &input->reserved_pin_5,&input->reserved_pin_6,&input->reserved_pin_7,&input->reserved_pin_8
        };
        for (uint8_t pinIdx = 0; pinIdx < 8; pinIdx++) {
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
