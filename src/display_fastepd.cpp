#if defined(TARGET_ESP32) && defined(OPENDISPLAY_FASTEPD)

#include "display_fastepd.h"
#include "display_service.h"
#include "structs.h"
#include <Arduino.h>
#include <SPI.h>
#include <string.h>
#include <FastEPD.h>

extern struct GlobalConfig globalConfig;

// FastEPD compiles these IT8951 helpers into FastEPD.cpp (via FastEPD.inl).
extern void it8951WaitForReady(FASTEPDSTATE* pState);
extern void it8951WaitForLUTReady(FASTEPDSTATE* pState);
extern void it8951SetImgBufBaseAddr(FASTEPDSTATE* pState);
extern void it8951LoadImgAreaStart(FASTEPDSTATE* pState, uint16_t endian, uint16_t pix_fmt, uint16_t rotate,
                                   uint16_t x, uint16_t y, uint16_t w, uint16_t h);
extern void it8951WriteCmdCode(FASTEPDSTATE* pState, uint16_t cmd);
extern void it8951DisplayArea1Bit(FASTEPDSTATE* pState, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                  uint16_t mode, uint8_t bg_gray, uint8_t fg_gray);
extern void it8951DisplayArea(FASTEPDSTATE* pState, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t mode);
extern uint16_t it8951ReadReg(FASTEPDSTATE* pState, uint16_t addr);
extern void it8951WriteReg(FASTEPDSTATE* pState, uint16_t addr, uint16_t val);
// FastEPD's cached PCAL6416A write
extern void bbepPCALDigitalWrite(uint8_t pin, uint8_t value);
static void fastepd_inkplate_wakeup_off(void);

class OdFastEPD : public FASTEPD {
public:
    FASTEPDSTATE* state() { return &_state; }
};

// OpenDisplay panel_ic_type -> FastEPD native parallel panel id (-1 if not a
// parallel panel). FastEPD owns the board's bus/PMIC/IO-expanders internally.
static int fastepd_parallel_panel(uint16_t panel_ic_type) {
    switch (panel_ic_type) {
        case OD_PANEL_IC_INKPLATE5V2_1280X720:   return BB_PANEL_INKPLATE5V2;
        case OD_PANEL_IC_INKPLATE10_1200X825:    return BB_PANEL_INKPLATE10;
        case OD_PANEL_IC_M5PAPERS3_960X540:      return BB_PANEL_M5PAPERS3;
        default:                                 return -1;
    }
}

static bool fastepd_is_parallel(void) {
    if (globalConfig.display_count < 1) return false;
    return fastepd_parallel_panel(globalConfig.displays[0].panel_ic_type) >= 0;
}

static int8_t fastepd_aux_pin(uint8_t p, int8_t default_gpio) {
    if (p == 0 || p == 0xFF) {
        return default_gpio;
    }
    return (int8_t)p;
}

static int8_t s_mosi = 9;
static int8_t s_miso = 8;
static int8_t s_sclk = 7;
static int8_t s_cs = 10;
static int8_t s_rst = 12;
static int8_t s_busy = 13;
static int8_t s_en = 11;
static int8_t s_ite_en = 21;

static bool s_init_failed = false;

extern "C" {

void opendisplay_fastepd_load_pins_from_display(const struct DisplayConfig* d, const struct SystemConfig* sys, uint16_t panel_ic_type) {
    if (!d) return;

    switch (panel_ic_type) {
        case OD_PANEL_IC_ED103TC2_1872X1404:
        case OD_PANEL_IC_ED103TC2_1872X1404_4GRAY:
            if (d->clk_pin != 0xFF) s_sclk = (int8_t)d->clk_pin;
            if (d->data_pin != 0xFF) s_mosi = (int8_t)d->data_pin;
            if (d->dc_pin != 0xFF) s_miso = (int8_t)d->dc_pin;
            else s_miso = 8;
            if (d->cs_pin != 0xFF) s_cs = (int8_t)d->cs_pin;
            if (d->reset_pin != 0xFF) s_rst = (int8_t)d->reset_pin;
            if (d->busy_pin != 0xFF) s_busy = (int8_t)d->busy_pin;
            if (sys) {
                s_en = fastepd_aux_pin(sys->pwr_pin_2, 11);
                s_ite_en = fastepd_aux_pin(sys->pwr_pin_3, 21);
            }
            break;
        case OD_PANEL_IC_M5PAPER_960X540:
            // M5Paper: IT8951 reset is not wired to the ESP32; GPIO23 gates the
            // IT8951 supply, so reset_pin doubles as RST/EN/ITE_EN (same as
            // FastEPD's own BB_PANEL_M5PAPER definition). pwr_pin_2/pwr_pin_3
            // are the board's power latch here, never panel enables.
            s_sclk = (d->clk_pin != 0xFF) ? (int8_t)d->clk_pin : 14;
            s_mosi = (d->data_pin != 0xFF) ? (int8_t)d->data_pin : 12;
            s_miso = (d->dc_pin != 0xFF) ? (int8_t)d->dc_pin : 13;
            s_cs = (d->cs_pin != 0xFF) ? (int8_t)d->cs_pin : 15;
            s_busy = (d->busy_pin != 0xFF) ? (int8_t)d->busy_pin : 27;
            s_rst = (d->reset_pin != 0xFF) ? (int8_t)d->reset_pin : 23;
            s_en = s_rst;
            s_ite_en = s_rst;
            break;
        default:
            break;
    }
}

bool fastepd_init_failed(void) {
    return s_init_failed;
}

} // extern "C"

static OdFastEPD g_epd;
static uint32_t s_direct_offset;
static bool s_hw_initialized = false;

static uint16_t s_partial_x;
static uint16_t s_partial_y;
static uint16_t s_partial_w;
static uint16_t s_partial_h;
static uint32_t s_partial_plane_size;
static uint32_t s_partial_bytes_written;
static uint32_t s_partial_expected;

// True when the framebuffer is 4bpp (16-level gray): ED103TC2 gray or GRAY16.
static bool fastepd_panel_is_4gray(void) {
    if (globalConfig.display_count < 1) return false;
    const struct DisplayConfig& d = globalConfig.displays[0];
    if (d.panel_ic_type == OD_PANEL_IC_ED103TC2_1872X1404_4GRAY) return true;
    return d.color_scheme == OD_COLOR_SCHEME_GRAY16;
}

// The effective pixel mode follows the runtime config (color_scheme), which
// can change between transfers via CONFIG_WRITE without a reboot. FastEPD's
// mode is sticky per hardware init, so warm transfer paths must re-assert it
// or incoming data is interpreted in the previous config's depth.
static void fastepd_apply_mode(void) {
    if (!s_hw_initialized || !g_epd.currentBuffer()) return;
    int want = fastepd_panel_is_4gray() ? BB_MODE_4BPP : BB_MODE_1BPP;
    if (g_epd.getMode() != want) {
        g_epd.setMode(want);
        g_epd.setPreviousMode((uint8_t)g_epd.getMode());
    }
}

static size_t fb_byte_size(void) {
    uint32_t w = globalConfig.displays[0].pixel_width;
    uint32_t h = globalConfig.displays[0].pixel_height;
    if (fastepd_panel_is_4gray()) {
        return (size_t)((w + 1) / 2) * h;
    }
    return (size_t)((w + 7) / 8) * h;
}

static unsigned row_pitch_bytes(void) {
    unsigned w = globalConfig.displays[0].pixel_width;
    return fastepd_panel_is_4gray() ? (unsigned)((w + 1) / 2) : (unsigned)((w + 7) / 8);
}

static void blit_1bpp_rect(uint8_t* dst, const uint8_t* src, uint32_t src_len,
                           uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                           uint32_t* io_offset) {
    if (!dst || !src || !io_offset) return;
    const unsigned panel_pitch = (globalConfig.displays[0].pixel_width + 7u) / 8u;
    const unsigned rect_pitch = (w + 7u) / 8u;
    uint32_t off = *io_offset;
    uint32_t end = off + src_len;
    uint32_t src_i = 0;
    while (off < end && src_i < src_len) {
        uint32_t row = off / rect_pitch;
        uint32_t col = off % rect_pitch;
        if (row >= h) break;
        uint32_t chunk = rect_pitch - col;
        if (chunk > end - off) chunk = end - off;
        memcpy(dst + (size_t)(y + row) * panel_pitch + (x / 8u) + col, src + src_i, chunk);
        off += chunk;
        src_i += chunk;
    }
    *io_offset = off;
}

// The IT8951 1bpp mode takes the leftmost pixel from bit 0; FastEPD packs it
// in bit 7. Mirrored panels get away with reversing the byte order of each
// row (that, plus LSB-first, mirrors the whole row). Unmirrored panels need
// each byte bit-reversed (verified on an M5Paper).
static void it8951_write_1bpp_row_lsb_first(const uint8_t* src, unsigned n) {
    uint8_t chunk[64];
    while (n) {
        const unsigned k = n < sizeof(chunk) ? n : (unsigned)sizeof(chunk);
        for (unsigned i = 0; i < k; i++) {
            uint8_t b = src[i];
            b = (uint8_t)((b >> 4) | (b << 4));
            b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
            chunk[i] = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
        }
        SPI.writeBytes(chunk, k);
        src += k;
        n -= k;
    }
}

/** Full-screen 1bpp DU (mode 1) — non-flashing. Matches FastEPD full-frame
 *  packing (MIRROR_X → B_ENDIAN + byte-reverse) with IT8951 DU waveform. */
static bool it8951_fullscreen_du(void) {
    FASTEPDSTATE* st = g_epd.state();
    uint8_t* cur = g_epd.currentBuffer();
    if (!st || !cur) return false;

    const uint16_t w = (uint16_t)st->native_width;
    const uint16_t h = (uint16_t)st->native_height;
    const bool mirror_x = (st->iFlags & BB_PANEL_FLAG_MIRROR_X) != 0;
    const unsigned pitch = (unsigned)((w + 7) / 8);

    g_epd.einkPower(1);
    it8951WaitForLUTReady(st);
    it8951SetImgBufBaseAddr(st);

    // B_ENDIAN either way: L_ENDIAN swaps the bytes of each 16-bit word (see
    // it8951_full_write_unmirrored()).
    it8951LoadImgAreaStart(st, IT8951_LDIMG_B_ENDIAN, IT8951_8BPP, IT8951_ROTATE_0, 0, 0, pitch, h);

    uint8_t* rowbuf = mirror_x ? (uint8_t*)malloc(pitch) : nullptr;
    if (mirror_x && !rowbuf) return false;

    gpio_set_level((gpio_num_t)st->u8CS, LOW);
    SPI.beginTransaction(SPISettings(st->spi_frequency, MSBFIRST, SPI_MODE0));
    it8951WaitForReady(st);
    SPI.transfer16(0x0000);
    it8951WaitForReady(st);

    for (uint16_t row = 0; row < h; row++) {
        const uint8_t* src = cur + (size_t)row * pitch;
        if (mirror_x) {
            for (unsigned i = 0; i < pitch; i++) {
                rowbuf[pitch - 1 - i] = src[i];
            }
            SPI.writeBytes(rowbuf, pitch);
        } else {
            it8951_write_1bpp_row_lsb_first(src, pitch);
        }
        if ((row & 7) == 0) yield();
    }

    SPI.endTransaction();
    gpio_set_level((gpio_num_t)st->u8CS, HIGH);
    free(rowbuf);

    it8951WriteCmdCode(st, IT8951_TCON_LD_IMG_END);
    it8951DisplayArea1Bit(st, 0, 0, w, h, IT8951_MODE_1, 0x00, 0xff);
    it8951WaitForReady(st);
    st->prev_mode = st->mode;
    return true;
}

void fastepd_prepare_hardware(void) {
    if (globalConfig.display_count < 1) {
        return;
    }
    const struct DisplayConfig& d = globalConfig.displays[0];
    opendisplay_fastepd_load_pins_from_display(&d, &globalConfig.system_config, d.panel_ic_type);
}

void fastepd_epaper_begin(void) {
    fastepd_prepare_hardware();
    s_init_failed = false;
    initOrRestoreWireForOpenDisplay();

    // Native parallel panels (Inkplate): initPanel() sets up the bus/PMIC and
    // allocates the buffers. Guard on currentBuffer() — initPanel() re-allocs
    // without freeing, so a warm re-init would leak; deInit() keeps the bus up.
    int parallel = fastepd_parallel_panel(globalConfig.displays[0].panel_ic_type);
    if (parallel >= 0) {
        if (!g_epd.currentBuffer()) {
            int prc = g_epd.initPanel(parallel);
            if (prc != BBEP_SUCCESS || !g_epd.currentBuffer()) {
                s_init_failed = true;
                s_hw_initialized = false;
                return;
            }
            g_epd.setMode(fastepd_panel_is_4gray() ? BB_MODE_4BPP : BB_MODE_1BPP);
            g_epd.setPreviousMode((uint8_t)g_epd.getMode());
        } else {
            g_epd.einkPower(1);
            s_hw_initialized = true;  // apply_mode requires it; set before the call
            fastepd_apply_mode();
        }
        s_hw_initialized = true;
        return;
    }

    int rc = g_epd.initIT8951((uint8_t)s_mosi, (uint8_t)s_miso, (uint8_t)s_sclk,
                              (uint8_t)s_cs, (uint8_t)s_busy, (uint8_t)s_rst,
                              (uint8_t)s_en, (uint8_t)s_ite_en);
    if (rc != BBEP_SUCCESS) {
        s_init_failed = true;
        s_hw_initialized = false;
        return;
    }

    if (!g_epd.currentBuffer()) {
        // M5Paper: ED047TC1 960x540, unmirrored. FastEPD 2.2.0 has no
        // BBEP_DISPLAY_ED047TC1, so size it explicitly. The IT8951 runs its own
        // waveforms; the gray matrix that setPanelSize(id) would add is unused.
        if (globalConfig.displays[0].panel_ic_type == OD_PANEL_IC_M5PAPER_960X540) {
            rc = g_epd.setPanelSize(960, 540, BB_PANEL_FLAG_NONE, -1600);
        } else {
            rc = g_epd.setPanelSize(BBEP_DISPLAY_ED103TC2);
        }
        if (rc != BBEP_SUCCESS || !g_epd.currentBuffer()) {
            s_init_failed = true;
            s_hw_initialized = false;
            return;
        }
    }

    if (fastepd_panel_is_4gray()) {
        g_epd.setMode(BB_MODE_4BPP);
    } else {
        g_epd.setMode(BB_MODE_1BPP);
    }
    g_epd.setPreviousMode((uint8_t)g_epd.getMode());
    s_hw_initialized = true;
}

// FastEPD 2.2.0 loads an unmirrored IT8951 frame as L_ENDIAN, which swaps the
// two bytes of every 16-bit word: 4bpp pixel pairs land 2 px off, 1bpp bytes
// 8 px off (verified on an M5Paper). Same frame and GC16 waveform as
// it8951WriteFramebuffer4Bit/1Bit, loaded B_ENDIAN as M5EPD does; 1bpp rows
// are also bit-reversed (it8951_write_1bpp_row_lsb_first()). Mirrored panels
// (ED103TC2) already get B_ENDIAN from FastEPD.
static void it8951_full_write_unmirrored(void) {
    FASTEPDSTATE* st = g_epd.state();
    const uint8_t* s = g_epd.currentBuffer();
    if (!st || !s) return;
    const bool gray = (g_epd.getMode() == BB_MODE_4BPP);
    const uint16_t w = (uint16_t)st->native_width;
    const uint16_t h = (uint16_t)st->native_height;
    const unsigned pitch = gray ? (unsigned)((w + 1) / 2) : (unsigned)((w + 7) / 8);

    g_epd.einkPower(1);
    it8951WaitForLUTReady(st);
    it8951SetImgBufBaseAddr(st);
    if (gray) {
        // Leave 1bpp mode (it8951DisplayArea1Bit() sets it).
        it8951WriteReg(st, IT8951_REG_UP1SR + 2, it8951ReadReg(st, IT8951_REG_UP1SR + 2) & ~(1 << 2));
        it8951LoadImgAreaStart(st, IT8951_LDIMG_B_ENDIAN, IT8951_4BPP, IT8951_ROTATE_0, 0, 0, w, h);
    } else {
        it8951LoadImgAreaStart(st, IT8951_LDIMG_B_ENDIAN, IT8951_8BPP, IT8951_ROTATE_0, 0, 0, pitch, h);
    }

    gpio_set_level((gpio_num_t)st->u8CS, LOW);
    SPI.beginTransaction(SPISettings(st->spi_frequency, MSBFIRST, SPI_MODE0));
    it8951WaitForReady(st);
    SPI.transfer16(0x0000);
    it8951WaitForReady(st);
    for (uint16_t row = 0; row < h; row++) {
        if (gray) SPI.writeBytes(s, pitch);
        else it8951_write_1bpp_row_lsb_first(s, pitch);
        s += pitch;
        if ((row & 7) == 0) yield();
    }
    SPI.endTransaction();
    gpio_set_level((gpio_num_t)st->u8CS, HIGH);

    it8951WriteCmdCode(st, IT8951_TCON_LD_IMG_END);
    if (gray) {
        it8951DisplayArea(st, 0, 0, w, h, IT8951_MODE_2);
    } else {
        it8951DisplayArea1Bit(st, 0, 0, w, h, IT8951_MODE_2, 0x00, 0xff);
    }
    it8951WaitForReady(st);
    g_epd.einkPower(0);
}

// REFRESH_FULL: parallel panels flash-clear to flush ghosting; IT8951 clears
// internally (ignores the mode) so it stays CLEAR_NONE.
static void fastepd_full_refresh_impl(void) {
    FASTEPDSTATE* st = g_epd.state();
    if (!fastepd_is_parallel() && !(st->iFlags & BB_PANEL_FLAG_MIRROR_X)) {
        it8951_full_write_unmirrored();
        return;
    }
    int clear = CLEAR_NONE;
    if (fastepd_is_parallel()) {
        // The PaperS3 needs the library-default 10-pass clear: CLEAR_FAST's
        // 8 passes leave residual banding on this panel (verified on
        // hardware 2026-08-31, library-only sketch clean with CLEAR_SLOW).
        // Inkplates keep the faster clear they were tuned with.
        clear = (globalConfig.displays[0].panel_ic_type == OD_PANEL_IC_M5PAPERS3_960X540)
                    ? CLEAR_SLOW
                    : CLEAR_FAST;
    }
    g_epd.fullUpdate(clear, true, NULL);
}

void fastepd_full_update(void) {
    if (!g_epd.currentBuffer()) return;
    fastepd_full_refresh_impl();
    g_epd.backupPlane();
}

bool fastepd_wait_refresh(int timeout_sec) {
    (void)timeout_sec;
    return !s_init_failed;
}

void fastepd_sleep_after_refresh(void) {
    g_epd.einkPower(0);
    fastepd_inkplate_wakeup_off();
    g_epd.deInit();
}

void fastepd_boot_write_row(uint16_t y, const uint8_t* row, unsigned pitch) {
    uint8_t* p = g_epd.currentBuffer();
    if (!p || !row) return;
    unsigned rp = row_pitch_bytes();
    if (pitch < rp) return;
    memcpy(p + (size_t)y * rp, row, rp);
}

void fastepd_boot_skip_planes(void) {
}

void fastepd_direct_write_reset(void) {
    fastepd_prepare_hardware();
    if (!s_hw_initialized) {
        fastepd_epaper_begin();
    } else {
        g_epd.einkPower(1);
        fastepd_apply_mode();
    }
    s_direct_offset = 0;
    uint8_t* p = g_epd.currentBuffer();
    if (p) {
        memset(p, 0xFF, fb_byte_size());
    }
}

void fastepd_direct_write_chunk(const uint8_t* data, uint32_t len) {
    if (!data || len == 0) return;
    uint8_t* base = g_epd.currentBuffer();
    if (!base) return;
    size_t maxb = fb_byte_size();
    size_t room = (s_direct_offset < maxb) ? (maxb - s_direct_offset) : 0;
    size_t n = (len > room) ? room : (size_t)len;
    if (n) {
        memcpy(base + s_direct_offset, data, n);
        s_direct_offset += n;
    }
}

void fastepd_direct_refresh(int refresh_mode) {
    if (!g_epd.currentBuffer()) return;
    if (refresh_mode == 1) {
        // FAST: parallel 1bpp uses native diff; IT8951 uses its DU waveform.
        // 4bpp has no fast path (fastUpdate and it8951_fullscreen_du() are
        // 1bpp-only) -> fall back to full.
        if (fastepd_panel_is_4gray()) fastepd_full_refresh_impl();
        else if (fastepd_is_parallel()) g_epd.fastUpdate(true);
        else it8951_fullscreen_du();
    } else {
        fastepd_full_refresh_impl();
    }
    g_epd.backupPlane();
}

// FastEPD's Inkplate einkPower(0) drops the rails but leaves the TPS65186 WAKEUP
// asserted, so the PMIC stays awake and the board idles high.
static void fastepd_inkplate_wakeup_off(void) {
    if (globalConfig.display_count < 1) return;
    const uint16_t ic = globalConfig.displays[0].panel_ic_type;
    if (ic != OD_PANEL_IC_INKPLATE5V2_1280X720 && ic != OD_PANEL_IC_INKPLATE10_1200X825) return;
    FASTEPDSTATE* st = g_epd.state();
    if (st) bbepPCALDigitalWrite(st->panelDef.ioShiftSTR, 0); // WAKEUP low -> PMIC shutdown
}

void fastepd_direct_sleep(void) {
    g_epd.einkPower(0);
    fastepd_inkplate_wakeup_off();
    g_epd.deInit();
}

void fastepd_mark_hw_deinitialized(void) {
    s_hw_initialized = false;
}

void fastepd_partial_prepare(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    fastepd_prepare_hardware();
    if (!s_hw_initialized) {
        fastepd_epaper_begin();
    } else {
        g_epd.einkPower(1);
        fastepd_apply_mode();
    }
    s_partial_x = x;
    s_partial_y = y;
    s_partial_w = w;
    s_partial_h = h;
    s_partial_plane_size = ((uint32_t)(w + 7u) / 8u) * h;
    s_partial_expected = s_partial_plane_size * 2u;
    s_partial_bytes_written = 0;
}

bool fastepd_partial_write_chunk(const uint8_t* data, uint32_t len) {
    if (!data || len == 0) return true;
    if (s_partial_bytes_written >= s_partial_expected) return false;
    if (len > s_partial_expected - s_partial_bytes_written) return false;

    uint8_t* prev = g_epd.previousBuffer();
    uint8_t* cur = g_epd.currentBuffer();
    if (!cur) return false;

    uint32_t remaining = len;
    const uint8_t* p = data;

    if (s_partial_bytes_written < s_partial_plane_size) {
        uint32_t into_old = s_partial_plane_size - s_partial_bytes_written;
        if (into_old > remaining) into_old = remaining;
        if (prev) {
            uint32_t off = s_partial_bytes_written;
            blit_1bpp_rect(prev, p, into_old, s_partial_x, s_partial_y, s_partial_w, s_partial_h, &off);
        }
        s_partial_bytes_written += into_old;
        p += into_old;
        remaining -= into_old;
    }

    if (remaining > 0 && s_partial_bytes_written >= s_partial_plane_size) {
        uint32_t new_off = s_partial_bytes_written - s_partial_plane_size;
        uint32_t off = new_off;
        blit_1bpp_rect(cur, p, remaining, s_partial_x, s_partial_y, s_partial_w, s_partial_h, &off);
        s_partial_bytes_written += remaining;
    }

    return true;
}

bool fastepd_partial_refresh(int refresh_mode) {
    if (s_partial_bytes_written != s_partial_expected) return false;
    if (!g_epd.currentBuffer()) return false;

    if (refresh_mode == 0) {
        fastepd_full_refresh_impl();
    } else if (fastepd_panel_is_4gray()) {
        // Rect already blitted into the framebuffer; 4bpp has no fast path.
        fastepd_full_refresh_impl();
    } else if (fastepd_is_parallel()) {
        g_epd.fastUpdate(true);
    } else {
        // Region rect is still applied to the framebuffer via blit; refresh is
        // full-screen DU (IT8951 region DU had persistent edge alignment issues).
        if (!it8951_fullscreen_du()) return false;
    }
    g_epd.backupPlane();
    return true;
}

#endif