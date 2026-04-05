#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)

// 4bpp framebuffer (initGrayMode(16)): matches TFT_eSprite / Seeed EPaper.pushImage 4bpp —
// per byte, left pixel (even x) = high nibble, right = low. Nibble value = TFT_GRAY_0..15
// (0 black .. 15 white). Row stride = (width+1)/2 bytes.

#include "display_seeed_gfx.h"
#include "display_service.h"
#include "structs.h"
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <TFT_eSPI.h>
#include "OpenDisplay/opendisplay_runtime_pins.h"

extern struct GlobalConfig globalConfig;

#ifndef TRANSMISSION_MODE_DIRECT_WRITE
#define TRANSMISSION_MODE_DIRECT_WRITE (1 << 3)
#endif

static int8_t seeed_gfx_aux_pin(uint8_t p, int8_t default_gpio) {
    if (p == 0 || p == 0xFF) {
        return default_gpio;
    }
    return (int8_t)p;
}

extern "C" {

int8_t opnd_seeed_runtime_sclk = 7;
int8_t opnd_seeed_runtime_miso = 8;
int8_t opnd_seeed_runtime_mosi = 9;
int8_t opnd_seeed_runtime_cs = 10;
int8_t opnd_seeed_runtime_rst = 12;
int8_t opnd_seeed_runtime_busy = 13;
int8_t opnd_seeed_runtime_tft_enable = 11;
int8_t opnd_seeed_runtime_ite_enable = 21;

volatile bool opnd_seeed_tcon_busy_timed_out = false;

void opnd_seeed_tcon_busy_timeout_reset(void) {
    opnd_seeed_tcon_busy_timed_out = false;
}

bool opnd_seeed_tcon_busy_timeout_occurred(void) {
    return opnd_seeed_tcon_busy_timed_out;
}

void opendisplay_seeed_gfx_load_pins_from_display(const struct DisplayConfig* d, const struct SystemConfig* sys, uint16_t panel_ic_type) {
    if (!d) return;

    switch (panel_ic_type) {
        case PANEL_IC_SEEED_ED103TC2_1872X1404:
        case PANEL_IC_SEEED_ED103TC2_1872X1404_4GRAY:
            if (d->clk_pin != 0xFF) opnd_seeed_runtime_sclk = (int8_t)d->clk_pin;
            if (d->data_pin != 0xFF) opnd_seeed_runtime_mosi = (int8_t)d->data_pin;
            if (d->dc_pin != 0xFF) opnd_seeed_runtime_miso = (int8_t)d->dc_pin;
            else opnd_seeed_runtime_miso = 8;
            if (d->cs_pin != 0xFF) opnd_seeed_runtime_cs = (int8_t)d->cs_pin;
            if (d->reset_pin != 0xFF) opnd_seeed_runtime_rst = (int8_t)d->reset_pin;
            if (d->busy_pin != 0xFF) opnd_seeed_runtime_busy = (int8_t)d->busy_pin;
            if (sys) {
                opnd_seeed_runtime_tft_enable = seeed_gfx_aux_pin(sys->pwr_pin_2, 11);
                opnd_seeed_runtime_ite_enable = seeed_gfx_aux_pin(sys->pwr_pin_3, 21);
            }
            break;
        default:
            break;
    }
}

} // extern "C"

static EPaper g_seeed_epaper;
static uint32_t seeed_direct_offset;

static bool seeed_gfx_panel_is_4gray(void) {
    if (globalConfig.display_count < 1) return false;
    return globalConfig.displays[0].panel_ic_type == PANEL_IC_SEEED_ED103TC2_1872X1404_4GRAY;
}

static size_t fb_byte_size(void) {
    uint32_t w = globalConfig.displays[0].pixel_width;
    uint32_t h = globalConfig.displays[0].pixel_height;
    if (seeed_gfx_panel_is_4gray()) {
        return (size_t)((w * h + 1) / 2);
    }
    return (size_t)((w * h + 7) / 8);
}

void seeed_gfx_prepare_hardware(void) {
    if (globalConfig.display_count < 1) {
        return;
    }
    const struct DisplayConfig& d = globalConfig.displays[0];
    opendisplay_seeed_gfx_load_pins_from_display(&d, &globalConfig.system_config, d.panel_ic_type);
}

void seeed_gfx_epaper_begin(void) {
    seeed_gfx_prepare_hardware();
    opnd_seeed_tcon_busy_timeout_reset();
    initOrRestoreWireForOpenDisplay();
    if (globalConfig.display_count >= 1) {
        if (seeed_gfx_panel_is_4gray()) {
            g_seeed_epaper.initGrayMode(16);
        } else {
            g_seeed_epaper.deinitGrayMode();
        }
    }
    g_seeed_epaper.begin(0);
}

void seeed_gfx_full_update(void) {
    g_seeed_epaper.update();
}

bool seeed_gfx_wait_refresh(int timeout_sec) {
    (void)timeout_sec;
    delay(300);
    return true;
}

void seeed_gfx_sleep_after_refresh(void) {
    g_seeed_epaper.sleep();
}

void seeed_gfx_boot_write_row(uint16_t y, const uint8_t* row, unsigned pitch) {
    void* p = g_seeed_epaper.getPointer();
    if (!p || !row) return;
    unsigned w = globalConfig.displays[0].pixel_width;
    unsigned row_pitch = seeed_gfx_panel_is_4gray() ? (unsigned)((w + 1) / 2) : (unsigned)((w + 7) / 8);
    if (pitch < row_pitch) return;
    memcpy((uint8_t*)p + (size_t)y * row_pitch, row, row_pitch);
}

void seeed_gfx_boot_skip_planes(void) {
}

void seeed_gfx_direct_write_reset(void) {
    seeed_gfx_prepare_hardware();
    g_seeed_epaper.wake();
    seeed_direct_offset = 0;
    void* p = g_seeed_epaper.getPointer();
    if (p) {
        memset(p, 0xFF, fb_byte_size());
    }
}

void seeed_gfx_direct_write_chunk(const uint8_t* data, uint32_t len) {
    if (!data || len == 0) return;
    uint8_t* base = (uint8_t*)g_seeed_epaper.getPointer();
    if (!base) return;
    size_t maxb = fb_byte_size();
    size_t room = (seeed_direct_offset < maxb) ? (maxb - seeed_direct_offset) : 0;
    size_t n = (len > room) ? room : (size_t)len;
    if (n) {
        memcpy(base + seeed_direct_offset, data, n);
        seeed_direct_offset += n;
    }
}

void seeed_gfx_direct_refresh(int refresh_mode) {
    g_seeed_epaper.update();
    if (refresh_mode != 1) {
        g_seeed_epaper.update();
    }
}

void seeed_gfx_direct_sleep(void) {
    g_seeed_epaper.sleep();
}

#endif
