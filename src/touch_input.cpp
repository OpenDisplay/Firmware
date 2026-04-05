#include "touch_input.h"
#include "display_service.h"
#include "structs.h"
#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#if defined(TARGET_ESP32)
#define TOUCH_ISR_ATTR IRAM_ATTR
#else
#define TOUCH_ISR_ATTR
#endif

extern struct GlobalConfig globalConfig;
extern uint8_t dynamicreturndata[11];
void updatemsdata(void);
void writeSerial(String message, bool newLine = true);

static_assert(sizeof(TouchController) == 32, "TouchController must be 32 bytes for packet 0x28");

#define GT911_REG_PID 0x8140u
#define GT911_REG_STATUS 0x814Eu
/* First contact: track id @0x814F, X @0x8150 — same as GT911-main readTouchPoints (COORD_ADDR+1) */
#define GT911_REG_POINT1 0x814Fu
#define GT911_POST_RESET_SETTLE_MS 200
#define GT911_PRE_RESET_DELAY_MS 300
/* GT911 0x814E: bit7 = buffer has coordinates ready (see GT911-main readTouches / Goodix docs) */
#define GT911_STATUS_BUFFER_READY 0x80u
#define GT911_MAX_CONTACTS 5u
/* MSD byte0 low nibble: GT911 contact count 1–5 while touching; 6 = released (last x,y in bytes 1–4; high nibble = last track id). 0 = never touched (5-byte block cleared). */

#ifndef TOUCH_DEBUG
#define TOUCH_DEBUG 1
#endif

struct TouchRuntime {
    uint8_t addr7;
    uint8_t ok;
    uint8_t reg_high_first; // 0: 16-bit reg addr low byte first (common); 1: high byte first (some GT911 / docs)
    uint8_t int_irq_attached; // 1: GPIO FALLING ISR active; 0: no INT or attachInterrupt failed
    uint16_t last_x;
    uint16_t last_y;
    uint8_t last_count;
    uint8_t last_id;
    uint32_t last_poll_ms;
    uint32_t last_i2c_warn_ms;
    uint8_t touch_latched;
};

static TouchRuntime s_touch_rt[4];

static volatile uint8_t s_touch_irq_mask = 0;

static void TOUCH_ISR_ATTR touch_isr_0(void) {
    s_touch_irq_mask |= 1u << 0;
}
static void TOUCH_ISR_ATTR touch_isr_1(void) {
    s_touch_irq_mask |= 1u << 1;
}
static void TOUCH_ISR_ATTR touch_isr_2(void) {
    s_touch_irq_mask |= 1u << 2;
}
static void TOUCH_ISR_ATTR touch_isr_3(void) {
    s_touch_irq_mask |= 1u << 3;
}

static void (*const s_touch_isrs[4])(void) = {touch_isr_0, touch_isr_1, touch_isr_2, touch_isr_3};

static void gt911_int_wake_before_irq(const TouchController* t) {
    if (t->int_pin == 0xFF) {
        return;
    }
    pinMode(t->int_pin, OUTPUT);
    digitalWrite(t->int_pin, HIGH);
    delay(10);
    pinMode(t->int_pin, INPUT_PULLUP);
}

static void attach_touch_int(uint8_t idx, uint8_t pin) {
    if (idx >= 4 || pin == 0xFF) {
        return;
    }
    int irq_num = digitalPinToInterrupt(pin);
    if (irq_num < 0) {
        writeSerial("Touch[" + String(idx) + "]: digitalPinToInterrupt failed for GPIO " + String(pin) + " — using poll only", true);
        return;
    }
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(irq_num, s_touch_isrs[idx], FALLING);
    s_touch_rt[idx].int_irq_attached = 1;
}

static const uint32_t TOUCH_I2C_WARN_INTERVAL_MS = 3000;

static bool gt911_write_reg(uint8_t addr7, uint16_t reg, const uint8_t* buf, uint8_t len, bool reg_high_first) {
    Wire.beginTransmission(addr7);
    if (reg_high_first) {
        Wire.write((uint8_t)(reg >> 8));
        Wire.write((uint8_t)(reg & 0xFFu));
    } else {
        Wire.write((uint8_t)(reg & 0xFFu));
        Wire.write((uint8_t)(reg >> 8));
    }
    for (uint8_t i = 0; i < len; i++) {
        Wire.write(buf[i]);
    }
    return Wire.endTransmission() == 0;
}

static bool gt911_read_reg(uint8_t addr7, uint16_t reg, uint8_t* buf, uint8_t len, bool reg_high_first) {
    Wire.beginTransmission(addr7);
    if (reg_high_first) {
        Wire.write((uint8_t)(reg >> 8));
        Wire.write((uint8_t)(reg & 0xFFu));
    } else {
        Wire.write((uint8_t)(reg & 0xFFu));
        Wire.write((uint8_t)(reg >> 8));
    }
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    size_t n = Wire.requestFrom((int)addr7, (int)len);
    if (n != (size_t)len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)Wire.read();
    }
    return true;
}

static bool gt911_product_id_match(const uint8_t* id) {
    return id[0] == '9' && id[1] == '1' && id[2] == '1';
}

static bool gt911_probe_product(uint8_t addr7, uint8_t* reg_high_first) {
    uint8_t id[4];
    if (gt911_read_reg(addr7, GT911_REG_PID, id, 4, false) && gt911_product_id_match(id)) {
        *reg_high_first = 0;
#if TOUCH_DEBUG
        writeSerial("GT911: PID OK @0x" + String(addr7, HEX) + " LE", true);
#endif
        return true;
    }
    if (gt911_read_reg(addr7, GT911_REG_PID, id, 4, true) && gt911_product_id_match(id)) {
        *reg_high_first = 1;
#if TOUCH_DEBUG
        writeSerial("GT911: PID OK @0x" + String(addr7, HEX) + " BE", true);
#endif
        return true;
    }
    writeSerial("GT911: PID probe failed at 0x" + String(addr7, HEX), true);
    return false;
}

/** Matches GT911-main `reset()` timing/order; 0x14 => INT high before RST release, 0x5D => INT low.
 *  Library uses `pinMode(RST, INPUT)` to release; without an external RST pull-up we drive RST HIGH instead. */
static void gt911_hw_reset(const TouchController* t, bool int_low_for_addr_5d) {
    if (t->rst_pin == 0xFF) {
        return;
    }
    if (t->int_pin == 0xFF) {
        pinMode(t->rst_pin, OUTPUT);
        digitalWrite(t->rst_pin, LOW);
        delay(10);
        digitalWrite(t->rst_pin, HIGH);
        delay(60);
        pinMode(t->rst_pin, OUTPUT);
        digitalWrite(t->rst_pin, HIGH);
        return;
    }
    delay(1);
    pinMode(t->int_pin, OUTPUT);
    pinMode(t->rst_pin, OUTPUT);
    digitalWrite(t->int_pin, LOW);
    digitalWrite(t->rst_pin, LOW);
    delay(11);
    digitalWrite(t->int_pin, int_low_for_addr_5d ? LOW : HIGH);
    delayMicroseconds(110);
    pinMode(t->rst_pin, OUTPUT);
    digitalWrite(t->rst_pin, HIGH);
    delay(6);
    digitalWrite(t->int_pin, LOW);
    delay(51);
    pinMode(t->rst_pin, OUTPUT);
    digitalWrite(t->rst_pin, HIGH);
    pinMode(t->int_pin, INPUT_PULLUP);
}

static bool touch_bus_ok(const TouchController* t) {
    if (globalConfig.data_bus_count == 0) {
        return true;
    }
    uint8_t bid = t->bus_id;
    if (bid == 0xFF) {
        bid = 0;
    }
    if (bid >= globalConfig.data_bus_count) {
        return false;
    }
    return globalConfig.data_buses[bid].bus_type == 0x01 && bid == 0;
}

static uint8_t gt911_resolve_and_init(const TouchController* t, TouchRuntime* rt) {
    const uint8_t a5d = 0x5D;
    const uint8_t a14 = 0x14;
    uint8_t want = t->i2c_addr_7bit;

    if (want != 0 && want != 0xFF) {
        if (t->rst_pin != 0xFF) {
            delay(GT911_PRE_RESET_DELAY_MS);
            gt911_hw_reset(t, want == a5d);
            delay(GT911_POST_RESET_SETTLE_MS);
        } else {
            delay(10);
        }
        if (gt911_probe_product(want, &rt->reg_high_first)) {
            return want;
        }
        writeSerial("GT911: probe failed at configured addr 0x" + String(want, HEX), true);
        return 0;
    }

    if (t->rst_pin != 0xFF) {
        delay(GT911_PRE_RESET_DELAY_MS);
        gt911_hw_reset(t, true);
        delay(GT911_POST_RESET_SETTLE_MS);
        if (gt911_probe_product(a5d, &rt->reg_high_first)) {
            return a5d;
        }
        delay(GT911_PRE_RESET_DELAY_MS);
        gt911_hw_reset(t, false);
        delay(GT911_POST_RESET_SETTLE_MS);
        if (gt911_probe_product(a14, &rt->reg_high_first)) {
            return a14;
        }
    } else {
        if (gt911_probe_product(a5d, &rt->reg_high_first)) {
            return a5d;
        }
        if (gt911_probe_product(a14, &rt->reg_high_first)) {
            return a14;
        }
    }
    writeSerial("GT911: not found on I2C (check wiring, pull-ups, RST/INT)", true);
    return 0;
}

static void gt911_clear_status(uint8_t addr7, bool reg_high_first) {
    uint8_t z = 0;
    gt911_write_reg(addr7, GT911_REG_STATUS, &z, 1, reg_high_first);
}

static void apply_touch_map(const TouchController* t, uint16_t* x, uint16_t* y) {
    if (t->flags & TOUCH_FLAG_SWAP_XY) {
        uint16_t tmp = *x;
        *x = *y;
        *y = tmp;
    }
    uint16_t w = 0;
    uint16_t h = 0;
    if (t->display_instance < globalConfig.display_count) {
        w = globalConfig.displays[t->display_instance].pixel_width;
        h = globalConfig.displays[t->display_instance].pixel_height;
    }
    if (t->flags & TOUCH_FLAG_INVERT_X && w > 0) {
        *x = (w > *x) ? (w - 1u - *x) : 0;
    }
    if (t->flags & TOUCH_FLAG_INVERT_Y && h > 0) {
        *y = (h > *y) ? (h - 1u - *y) : 0;
    }
    if (w > 0 && *x >= w) {
        *x = w - 1u;
    }
    if (h > 0 && *y >= h) {
        *y = h - 1u;
    }
}

void initTouchInput(void) {
    memset(s_touch_rt, 0, sizeof(s_touch_rt));
    s_touch_irq_mask = 0;
    if (globalConfig.touch_controller_count == 0) {
        return;
    }
    uint8_t enabled = 0;
    for (uint8_t j = 0; j < globalConfig.touch_controller_count; j++) {
        if (globalConfig.touch_controllers[j].touch_ic_type != TOUCH_IC_NONE) {
            enabled++;
        }
    }
    if (enabled == 0) {
        return;
    }
#if TOUCH_DEBUG
    writeSerial("Touch: init " + String(enabled) + " enabled / " + String(globalConfig.touch_controller_count) + " packet(s)", true);
#endif
    initOrRestoreWireForOpenDisplay();
    for (uint8_t i = 0; i < globalConfig.touch_controller_count; i++) {
        TouchController* tc = &globalConfig.touch_controllers[i];
        TouchRuntime* rt = &s_touch_rt[i];
        if (tc->touch_ic_type == TOUCH_IC_NONE) {
            continue;
        }
        if (tc->touch_ic_type != TOUCH_IC_GT911) {
            if (tc->touch_ic_type != TOUCH_IC_NONE) {
                writeSerial("Touch[" + String(i) + "]: skipped (only GT911=1 implemented, got " + String(tc->touch_ic_type) + ")", true);
            }
            continue;
        }
        if (!touch_bus_ok(tc)) {
            writeSerial("Touch[" + String(i) + "]: bus rejected — need I2C data_bus instance 0 (or bus_id 0xFF mapping to 0); data_bus_count=" +
                        String(globalConfig.data_bus_count), true);
            continue;
        }
        if (tc->touch_data_start_byte > 6u) {
            writeSerial("Touch[" + String(i) + "]: touch_data_start_byte must be 0–6 (5-byte window)", true);
            continue;
        }
        uint8_t addr = gt911_resolve_and_init(tc, rt);
        if (addr == 0) {
            writeSerial("Touch[" + String(i) + "]: init failed", true);
            continue;
        }
        rt->addr7 = addr;
        rt->ok = 1;
        gt911_clear_status(addr, rt->reg_high_first != 0);
        {
            const bool rh = rt->reg_high_first != 0;
            uint16_t xres = 0;
            uint16_t yres = 0;
            uint8_t inf[12];
            if (gt911_read_reg(addr, GT911_REG_PID, inf, sizeof(inf), rh)) {
                xres = (uint16_t)inf[6] | ((uint16_t)inf[7] << 8);
                yres = (uint16_t)inf[8] | ((uint16_t)inf[9] << 8);
            }
            writeSerial("Touch[" + String(i) + "]: GT911 @0x" + String(addr, HEX) + " " + String(rh ? "BE" : "LE") +
                        " " + String(xres) + "x" + String(yres) + (tc->int_pin != 0xFF ? " INT+poll" : " poll"), true);
        }
        if (tc->int_pin != 0xFF) {
            gt911_int_wake_before_irq(tc);
#if TOUCH_DEBUG
            writeSerial("Touch[" + String(i) + "]: INT GPIO " + String(tc->int_pin) + " FALLING", true);
#endif
            attach_touch_int(i, tc->int_pin);
            if (!rt->int_irq_attached) {
                writeSerial("Touch[" + String(i) + "]: INT attach failed — polling only", true);
            }
        }
        rt->last_poll_ms = millis();
    }
}

bool touch_input_gpio_is_touch_int(uint8_t pin) {
    if (pin == 0xFF) {
        return false;
    }
    for (uint8_t i = 0; i < globalConfig.touch_controller_count; i++) {
        const TouchController* tc = &globalConfig.touch_controllers[i];
        if (tc->touch_ic_type == TOUCH_IC_GT911 && tc->int_pin == pin) {
            return true;
        }
    }
    return false;
}

void processTouchInput(void) {
    uint32_t now = millis();
    for (uint8_t i = 0; i < globalConfig.touch_controller_count; i++) {
        TouchController* tc = &globalConfig.touch_controllers[i];
        TouchRuntime* rt = &s_touch_rt[i];
        if (tc->touch_ic_type != TOUCH_IC_GT911 || !rt->ok) {
            continue;
        }

        uint8_t interval = tc->poll_interval_ms ? tc->poll_interval_ms : 25;
        const bool irq_mode = (rt->int_irq_attached != 0);
        bool from_irq = false;
        bool line_low = false;
        bool want_read = false;
        bool timed_poll = false;
        if (irq_mode) {
            bool edge = (s_touch_irq_mask & (1u << i)) != 0;
            line_low = (digitalRead(tc->int_pin) == LOW);
            timed_poll = (uint32_t)(now - rt->last_poll_ms) >= interval;
            want_read = edge || timed_poll || line_low;
            if (!want_read) {
                continue;
            }
            if (edge) {
                from_irq = true;
                s_touch_irq_mask &= (uint8_t)~(1u << i);
            }
        } else {
            timed_poll = (uint32_t)(now - rt->last_poll_ms) >= interval;
            want_read = timed_poll;
            if (!want_read) {
                continue;
            }
        }

        const bool rh = rt->reg_high_first != 0;
        uint8_t st = 0;
        if (!gt911_read_reg(rt->addr7, GT911_REG_STATUS, &st, 1, rh)) {
            if ((uint32_t)(now - rt->last_i2c_warn_ms) >= TOUCH_I2C_WARN_INTERVAL_MS) {
                rt->last_i2c_warn_ms = now;
                writeSerial("Touch[" + String(i) + "]: I2C read fail (status reg 0x814E, addr 0x" + String(rt->addr7, HEX) + ")", true);
            }
            continue;
        }

        if ((st & GT911_STATUS_BUFFER_READY) == 0) {
            if (!from_irq && !line_low) {
                rt->last_poll_ms = now;
            }
            continue;
        }
        uint8_t n = st & 0x0Fu;
        if (n > GT911_MAX_CONTACTS) {
            if (!from_irq && !line_low) {
                rt->last_poll_ms = now;
            }
            continue;
        }
        rt->last_poll_ms = now;

        uint16_t x = 0;
        uint16_t y = 0;
        uint8_t tid = 0;
        if (n > 0) {
            uint8_t p[8];
            if (!gt911_read_reg(rt->addr7, GT911_REG_POINT1, p, 8, rh)) {
                if ((uint32_t)(now - rt->last_i2c_warn_ms) >= TOUCH_I2C_WARN_INTERVAL_MS) {
                    rt->last_i2c_warn_ms = now;
                    writeSerial("Touch[" + String(i) + "]: I2C read fail (point1 @0x814F, addr 0x" + String(rt->addr7, HEX) + ")", true);
                }
                continue;
            }
            tid = p[0];
            x = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
            y = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
            rt->touch_latched = 1;
        } else {
            x = rt->last_x;
            y = rt->last_y;
            tid = rt->last_id;
        }
        gt911_clear_status(rt->addr7, rh);

        if (n > 0) {
            apply_touch_map(tc, &x, &y);
        }

        bool changed = (n != rt->last_count) || (n > 0 && ((x != rt->last_x) || (y != rt->last_y) || (tid != rt->last_id)));
        rt->last_count = n;
        rt->last_x = x;
        rt->last_y = y;
        rt->last_id = tid;

        uint8_t s = tc->touch_data_start_byte;
        if (s + 5 > 11) {
            continue;
        }
        if (n == 0 && !rt->touch_latched) {
            memset(&dynamicreturndata[s], 0, 5);
        } else if (n == 0) {
            dynamicreturndata[s] = (uint8_t)(6u | ((tid & 0x0Fu) << 4));
            dynamicreturndata[s + 1] = (uint8_t)(x & 0xFFu);
            dynamicreturndata[s + 2] = (uint8_t)(x >> 8);
            dynamicreturndata[s + 3] = (uint8_t)(y & 0xFFu);
            dynamicreturndata[s + 4] = (uint8_t)(y >> 8);
        } else {
            dynamicreturndata[s] = (uint8_t)((n & 0x0Fu) | ((tid & 0x0Fu) << 4));
            dynamicreturndata[s + 1] = (uint8_t)(x & 0xFFu);
            dynamicreturndata[s + 2] = (uint8_t)(x >> 8);
            dynamicreturndata[s + 3] = (uint8_t)(y & 0xFFu);
            dynamicreturndata[s + 4] = (uint8_t)(y >> 8);
        }
        if (changed) {
#if TOUCH_DEBUG
            if (n == 0) {
                writeSerial("Touch[" + String(i) + "]: release (MSD low nibble 6, last xy kept)", true);
            } else {
                const char* src = irq_mode ? (from_irq ? "irq" : (line_low ? "line" : "poll")) : "poll";
                writeSerial("Touch[" + String(i) + "]: n=" + String(n) + " id=" + String(tid) + " (" + String(x) + "," + String(y) + ") st=0x" +
                                String(st, HEX) + " " + String(src), true);
            }
#endif
            updatemsdata();
        }
    }
}
