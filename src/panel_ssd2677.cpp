#include "panel_ssd2677.h"

#include <Arduino.h>
#include <SPI.h>
#include <bb_epaper.h>
#include <string.h>
#include "structs.h"
#include "od_log.h"
#include "watchdog.h"

#ifdef TARGET_ESP32
#include "esp_heap_caps.h"
#endif

extern struct GlobalConfig globalConfig;
extern BBEPDISP bbep;

// Forward-declared by callers, as in split_panel.cpp: bb_epaper's public header
// does not export these.
void bbepWaitBusy(BBEPDISP *pBBEP);
void bbepStartWrite(BBEPDISP *pBBEP, int iPlane);
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen);
int bbepRefresh(BBEPDISP *pBBEP, int iMode);

static const uint8_t SSD2677_CHIP_ID = 0x07;
static const uint8_t PANEL_IC_EP397 = 69;
static const uint8_t PANEL_IC_EP397_4GRAY = 70;

static const uint8_t PIN_STICKY_CLK = 13;
static const uint8_t PIN_STICKY_DATA = 14;
static const uint8_t PIN_STICKY_CS = 15;
static const uint8_t PIN_STICKY_SD_CS = 8;
static const uint8_t PIN_STICKY_SD_EN = 10;

static const uint32_t PLANE_BYTES = 800u / 8u * 480u;   // 48000: one 1-bit plane

enum PanelVariant : uint8_t { VARIANT_UNRESOLVED, VARIANT_LEGACY, VARIANT_SSD2677 };
static PanelVariant s_variant = VARIANT_UNRESOLVED;

// Mirrors Seeed's init for this controller. The reset is done by bbepWakeUp().
// TRES 0x0320/0x02A8 (800x680) is the controller's source/gate count, not the
// visible area: the same value appears in bb_epaper's EP397YR init.
static const uint8_t ssd2677_init[] PROGMEM = {
    // Seeed's PSR is 0x2F. UD (bit 3) is cleared to match the Sticky's panel mounting.
    0x03, 0x00, 0x27, 0x0E,
    BUSY_WAIT,
    0x05, 0x06, 0x0F, 0x8B, 0x93, 0xC1,
    0x02, 0xE7, 0xC1,
    0x02, 0x30, 0x08,
    0x02, 0x50, 0x77,
    0x09, 0x62, 0x76, 0x76, 0x76, 0x5A, 0x9D, 0x8A, 0x76, 0x62,
    0x05, 0x61, 0x03, 0x20, 0x02, 0xA8,
    0x02, 0xE0, 0x10,
    0x05, 0x65, 0x00, 0x00, 0x00, 0x00,
    0x02, 0xE9, 0x01,
    BUSY_WAIT,
    0
};

#ifdef TARGET_ESP32
static bool configHasStickyPins() {
    if (globalConfig.display_count == 0) return false;
    const DisplayConfig& d = globalConfig.displays[0];
    return d.clk_pin == PIN_STICKY_CLK && d.data_pin == PIN_STICKY_DATA && d.cs_pin == PIN_STICKY_CS;
}
#endif

void panelStickySdPark() {
#ifdef TARGET_ESP32
    if (!configHasStickyPins()) return;
    pinMode(PIN_STICKY_SD_CS, OUTPUT);
    digitalWrite(PIN_STICKY_SD_CS, HIGH);  // SD_CS idle
    pinMode(PIN_STICKY_SD_EN, OUTPUT);
    digitalWrite(PIN_STICKY_SD_EN, LOW);   // SD_EN off
#endif
}

bool panelSsd2677Active() {
    return s_variant == VARIANT_SSD2677;
}

#ifdef TARGET_ESP32
// SPI mode 0, about 100 kHz. The clock is deliberately slow: this is one byte per
// boot, and the controller's read timing is not documented in what we have.
static const uint32_t BITBANG_HALF_PERIOD_US = 5;

static void bitBangWrite8(uint8_t clk, uint8_t data, uint8_t value) {
    pinMode(data, OUTPUT);
    for (int bit = 7; bit >= 0; bit--) {
        digitalWrite(data, (value >> bit) & 1);
        delayMicroseconds(BITBANG_HALF_PERIOD_US);
        digitalWrite(clk, HIGH);
        delayMicroseconds(BITBANG_HALF_PERIOD_US);
        digitalWrite(clk, LOW);
    }
}

static uint8_t bitBangRead8(uint8_t clk, uint8_t data) {
    pinMode(data, INPUT);
    delayMicroseconds(BITBANG_HALF_PERIOD_US);
    uint8_t value = 0;
    for (int bit = 7; bit >= 0; bit--) {
        delayMicroseconds(BITBANG_HALF_PERIOD_US);
        digitalWrite(clk, HIGH);
        delayMicroseconds(BITBANG_HALF_PERIOD_US);
        value = (uint8_t)((value << 1) | (digitalRead(data) ? 1 : 0));
        digitalWrite(clk, LOW);
    }
    return value;
}

// Reset, then read register 0x70 on raw pins. Caller has released the SPI peripheral.
static uint8_t probeReadChipId(const DisplayConfig& d) {
    digitalWrite(d.reset_pin, LOW);
    delay(20);
    digitalWrite(d.reset_pin, HIGH);
    delay(10);

    digitalWrite(d.cs_pin, HIGH);
    digitalWrite(d.dc_pin, LOW);
    digitalWrite(d.cs_pin, LOW);
    bitBangWrite8(d.clk_pin, d.data_pin, 0x70);
    digitalWrite(d.cs_pin, HIGH);
    digitalWrite(d.dc_pin, HIGH);
    digitalWrite(d.cs_pin, LOW);
    const uint8_t id = bitBangRead8(d.clk_pin, d.data_pin);
    digitalWrite(d.cs_pin, HIGH);
    return id;
}

// After a reset an idle SSD1677 holds BUSY low. High for this whole window means
// no panel, or a panel that is not an SSD1677.
static bool legacyBusyGoesIdle(uint8_t busyPin) {
    const uint32_t start = millis();
    do {
        if (digitalRead(busyPin) == LOW) return true;
        delay(5);
    } while (millis() - start < 500);
    return false;
}
#endif

void panelVariantResolve() {
#ifdef TARGET_ESP32
    if (s_variant != VARIANT_UNRESOLVED) return;
    if (!configHasStickyPins()) {
        s_variant = VARIANT_LEGACY;
        return;
    }
    const DisplayConfig& d = globalConfig.displays[0];
    if (d.panel_ic_type != PANEL_IC_EP397 && d.panel_ic_type != PANEL_IC_EP397_4GRAY) {
        s_variant = VARIANT_LEGACY;
        return;
    }
    if (d.reset_pin == 0xFF || d.busy_pin == 0xFF || d.dc_pin == 0xFF) {
        s_variant = VARIANT_LEGACY;
        return;
    }

    panelStickySdPark();
    SPI.end();   // no-op if not started; bb_epaper's bus would otherwise own these pins
    pinMode(d.clk_pin, OUTPUT);
    digitalWrite(d.clk_pin, LOW);
    pinMode(d.cs_pin, OUTPUT);
    digitalWrite(d.cs_pin, HIGH);
    pinMode(d.dc_pin, OUTPUT);
    pinMode(d.reset_pin, OUTPUT);
    pinMode(d.busy_pin, INPUT);

    const uint32_t t0 = millis();
    const uint8_t id1 = probeReadChipId(d);
    const uint8_t id2 = probeReadChipId(d);
    // Only needed to classify a non-SSD2677 result, and it costs its whole timeout
    // on an SSD2677, whose idle BUSY level is high.
    const bool isSsd2677 = (id1 == SSD2677_CHIP_ID && id2 == SSD2677_CHIP_ID);
    const bool busyIdleLow = !isSsd2677 && legacyBusyGoesIdle(d.busy_pin);
    const uint8_t busyLevel = (uint8_t)digitalRead(d.busy_pin);

    const char* result;
    if (isSsd2677) {
        s_variant = VARIANT_SSD2677;
        result = "SSD2677";
    } else if (id1 == id2 && busyIdleLow) {
        s_variant = VARIANT_LEGACY;
        result = "legacy SSD1677";
    } else {
        // Stays unresolved so the next cold acquire tries again. The legacy path
        // runs meanwhile and its BUSY gate skips the boot screen if nothing answers.
        result = "UNRESOLVED";
    }
    od_log_info("[panel probe] id=0x%02X/0x%02X busy=%u busyIdleLow=%d %lums -> %s",
                id1, id2, busyLevel, (int)busyIdleLow, (unsigned long)(millis() - t0), result);
#endif
}

void panelSsd2677ApplyOverrides() {
    if (!panelSsd2677Active()) return;
    // Leave bbep.type off EP397_*: firmware and bb_epaper both send SSD16xx-only
    // RAM setup for that type. EP397YR is the same size on a UC81xx command set.
    bbep.type = EP397YR_800x480;
    bbep.chip_type = BBEP_CHIP_UC81xx;   // BUSY idle-high, UC81xx sleep sequence
    // BBEP_4COLOR gives the "POF 00, wait, DSLP A5" sleep and makes the address
    // window a no-op; RED_SWAPPED sends PLANE_0 to DTM1 (0x10), the only image RAM.
    bbep.iFlags = BBEP_NEEDS_EXTRA_INIT | BBEP_4COLOR | BBEP_RED_SWAPPED;
    bbep.pInitFull = ssd2677_init;
    bbep.pInitFast = NULL;
    bbep.pInitPart = NULL;
}

#ifdef TARGET_ESP32
// Read the controller's temperature register. bb_epaper's bus has no read path, so
// the peripheral is released for one byte and restored as bbepInitIO configures it.
static uint8_t readTemperatureRaw() {
    const DisplayConfig& d = globalConfig.displays[0];
    bbepWriteCmd(&bbep, 0x40);
    bbepWaitBusy(&bbep);

    SPI.end();
    pinMode(d.clk_pin, OUTPUT);
    digitalWrite(d.clk_pin, LOW);
    digitalWrite(d.dc_pin, HIGH);
    digitalWrite(d.cs_pin, LOW);
    const uint8_t temp = bitBangRead8(d.clk_pin, d.data_pin);
    digitalWrite(d.cs_pin, HIGH);

    SPI.begin(d.clk_pin, -1, d.data_pin, -1);
    SPI.beginTransaction(SPISettings(bbep.iSpeed, MSBFIRST, SPI_MODE0));
    SPI.endTransaction();
    return temp;
}
#endif

// Waveform selectors from Seeed's driver ("EPD_Slow_Display_1BIT" and
// "EPD_Four_Gray_Display"). An implausible reading falls to the coldest bucket.
static uint8_t monoWaveformForTemperature(uint8_t temp) {
    if (temp <= 10) return 0xE8;
    if (temp <= 20) return 0xEB;
    if (temp <= 30) return 0xEE;
    if (temp <= 127) return 0xF1;
    return 0xE8;
}

static uint8_t gray4WaveformForTemperature(uint8_t temp) {
    if (temp <= 15) return 0xF1;
    if (temp <= 127) return 0xFA;
    return 0xF1;
}

static bool gray4Mode() {
    return globalConfig.displays[0].color_scheme == OD_COLOR_SCHEME_GRAY4;
}

void panelSsd2677AfterInit() {
    if (!panelSsd2677Active()) return;
#ifdef TARGET_ESP32
    const uint8_t temp = readTemperatureRaw();
#else
    const uint8_t temp = 25;
#endif
    const uint8_t waveform = gray4Mode() ? gray4WaveformForTemperature(temp)
                                         : monoWaveformForTemperature(temp);
    od_log_info("[SSD2677] temp=%u waveform=0x%02X", temp, waveform);
    bbepCMD2(&bbep, 0xE0, 0x12);
    bbepCMD2(&bbep, 0xE6, waveform);
    bbepWriteCmd(&bbep, 0xA5);
    bbepWaitBusy(&bbep);
    delay(10);
}

void panelSsd2677AfterRefresh() {
    if (!panelSsd2677Active()) return;
    bbepCMD2(&bbep, 0x02, 0x00);   // POF
    odWatchdogFeed();
    bbepWaitBusy(&bbep);
}

// --- image stream ---------------------------------------------------------------
// The panel has one RAM plane of packed 2-bit pixels, in DTM1.
// Mono: each input bit becomes 00 (black) or 11 (white).
// 4-gray: clients send two 1-bit planes one after the other, so the first has to
// be held until the second arrives.

static const uint8_t NIBBLE_TO_2BPP[16] = {
    0x00, 0x03, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F,
    0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF,
};

// Stored code (plane1:plane0, as sent by clients for panels 69/70) to panel pixel.
// Clients store black=3, dark=1, light=2, white=0. On the glass the panel shows
// black=0, dark=1, light=2, white=3; this was checked with a grey ramp, and differs
// from the mid-grey swap in Seeed's gray4_nibble_to_ssd2677_pixel.
static const uint8_t STORED_TO_PIXEL[4] = {3, 1, 2, 0};

static uint8_t* s_firstPlane = nullptr;
static uint32_t s_planeOffset = 0;
static bool s_discardWrite = false;
static bool s_bufferingFirstPlane = false;
static uint8_t s_out[512];

static bool ensureFirstPlaneBuffer() {
    if (s_firstPlane) return true;
#ifdef TARGET_ESP32
    s_firstPlane = (uint8_t*)heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif
    if (!s_firstPlane) od_log_error("[SSD2677] no memory for the 4-gray plane buffer");
    return s_firstPlane != nullptr;
}

// Open DTM1 and fill the rows ahead of the image. The controller addresses 680
// gate lines (TRES) and the glass shows 480; with UD cleared, the visible lines
// are the last 480 of RAM.
static void startImageRam() {
    bbepStartWrite(&bbep, PLANE_0);
    memset(s_out, 0xFF, sizeof(s_out));
    uint32_t remaining = 200u * 800u / 4u;   // 200 hidden rows, packed 2 bpp
    while (remaining > 0) {
        const int take = (int)min(remaining, (uint32_t)sizeof(s_out));
        bbepWriteData(&bbep, s_out, take);
        remaining -= (uint32_t)take;
    }
}

void epdStartWrite(int plane) {
    if (!panelSsd2677Active()) {
        bbepStartWrite(&bbep, plane);
        return;
    }
    s_planeOffset = 0;
    s_discardWrite = false;
    s_bufferingFirstPlane = false;
    if (gray4Mode()) {
        if (plane == PLANE_0) {
            s_bufferingFirstPlane = true;
            s_discardWrite = !ensureFirstPlaneBuffer();
        } else {
            s_discardWrite = (s_firstPlane == nullptr);
            if (!s_discardWrite) startImageRam();
        }
    } else if (plane == PLANE_0) {
        startImageRam();
    } else {
        s_discardWrite = true;   // mono: the second plane has no RAM to go to
    }
}

static void writeMono(const uint8_t* data, int len) {
    int off = 0;
    while (off < len) {
        const int take = min(len - off, (int)(sizeof(s_out) / 2));
        for (int i = 0; i < take; i++) {
            s_out[i * 2] = NIBBLE_TO_2BPP[data[off + i] >> 4];
            s_out[i * 2 + 1] = NIBBLE_TO_2BPP[data[off + i] & 0x0F];
        }
        bbepWriteData(&bbep, s_out, take * 2);
        off += take;
    }
}

static void writeGray4(const uint8_t* plane1, int len) {
    int off = 0;
    while (off < len && s_planeOffset < PLANE_BYTES) {
        int take = min(len - off, (int)(sizeof(s_out) / 2));
        if ((uint32_t)take > PLANE_BYTES - s_planeOffset) take = (int)(PLANE_BYTES - s_planeOffset);
        for (int i = 0; i < take; i++) {
            const uint8_t bit0 = s_firstPlane[s_planeOffset + i];
            const uint8_t bit1 = plane1[off + i];
            uint16_t packed = 0;
            for (int b = 7; b >= 0; b--) {
                const uint8_t code = (uint8_t)((((bit1 >> b) & 1) << 1) | ((bit0 >> b) & 1));
                packed = (uint16_t)((packed << 2) | STORED_TO_PIXEL[code]);
            }
            s_out[i * 2] = (uint8_t)(packed >> 8);
            s_out[i * 2 + 1] = (uint8_t)packed;
        }
        bbepWriteData(&bbep, s_out, take * 2);
        s_planeOffset += (uint32_t)take;
        off += take;
    }
}

void epdWriteData(const uint8_t* data, int len) {
    if (!panelSsd2677Active()) {
        bbepWriteData(&bbep, (uint8_t*)data, len);
        return;
    }
    if (s_discardWrite || len <= 0) return;
    if (s_bufferingFirstPlane) {
        uint32_t take = (uint32_t)len;
        if (take > PLANE_BYTES - s_planeOffset) take = PLANE_BYTES - s_planeOffset;
        memcpy(s_firstPlane + s_planeOffset, data, take);
        s_planeOffset += take;
    } else if (gray4Mode()) {
        writeGray4(data, len);
    } else {
        writeMono(data, len);
    }
}

void epdStartRefresh(int mode) {
    if (!panelSsd2677Active()) {
        bbepRefresh(&bbep, mode);
        return;
    }
    // No fast or partial waveform is implemented, so every mode is a full refresh.
    bbepWriteCmd(&bbep, 0x04);   // PON
    odWatchdogFeed();
    bbepWaitBusy(&bbep);
    bbepCMD2(&bbep, 0x12, 0x00); // DRF
}
