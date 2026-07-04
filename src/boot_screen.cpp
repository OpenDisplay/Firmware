#include "boot_screen.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "structs.h"
#include <bb_epaper.h>
#include "qr/qrcode.h"
#include "display_service.h"
#if __has_include("logo_bitmap.h")
#include "logo_bitmap.h"
#define BOOT_HAS_LOGO
#endif
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
#include "display_seeed_gfx.h"
#endif

extern struct GlobalConfig globalConfig;
extern struct SecurityConfig securityConfig;
extern BBEPDISP bbep;
extern uint8_t staticRowBuffer[BOOT_ROW_BUFFER_SIZE];

String getChipIdHex();
uint8_t getFirmwareMajor();
uint8_t getFirmwareMinor();
int getBitsPerPixel();
int getplane();
void writeSerial(String message, bool newLine);
void bbepSetAddrWindow(BBEPDISP *pBBEP, int x, int y, int cx, int cy);
void bbepStartWrite(BBEPDISP *pBBEP, int iPlane);
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen);

typedef struct { char c; uint8_t col[5]; } BootGlyph5x7;
static const BootGlyph5x7 BOOT_FONT5X7[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}}, {'!', {0x00,0x00,0x5F,0x00,0x00}},
    {'"', {0x00,0x07,0x00,0x07,0x00}}, {'#', {0x14,0x7F,0x14,0x7F,0x14}},
    {'%', {0x23,0x13,0x08,0x64,0x62}}, {'&', {0x36,0x49,0x55,0x22,0x50}},
    {'\'',{0x00,0x05,0x03,0x00,0x00}}, {'(', {0x00,0x1C,0x22,0x41,0x00}},
    {')', {0x00,0x41,0x22,0x1C,0x00}}, {'*', {0x14,0x08,0x3E,0x08,0x14}},
    {'+', {0x08,0x08,0x3E,0x08,0x08}}, {',', {0x00,0x50,0x30,0x00,0x00}},
    {'-', {0x08,0x08,0x08,0x08,0x08}}, {'.', {0x00,0x00,0x40,0x00,0x00}},
    {'/', {0x20,0x10,0x08,0x04,0x02}}, {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}}, {'2', {0x62,0x51,0x49,0x49,0x46}},
    {'3', {0x22,0x49,0x49,0x49,0x36}}, {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x2F,0x49,0x49,0x49,0x31}}, {'6', {0x3E,0x49,0x49,0x49,0x32}},
    {'7', {0x01,0x71,0x09,0x05,0x03}}, {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x26,0x49,0x49,0x49,0x3E}}, {':', {0x00,0x36,0x36,0x00,0x00}},
    {';', {0x00,0x56,0x36,0x00,0x00}}, {'<', {0x08,0x14,0x22,0x41,0x00}},
    {'=', {0x14,0x14,0x14,0x14,0x14}}, {'>', {0x00,0x41,0x22,0x14,0x08}},
    {'?', {0x02,0x01,0x51,0x09,0x06}}, {'@', {0x32,0x49,0x79,0x41,0x3E}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}}, {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}}, {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}}, {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}}, {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}}, {'J', {0x20,0x40,0x40,0x3F,0x00}},
    {'K', {0x7F,0x08,0x14,0x22,0x41}}, {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'M', {0x7F,0x02,0x04,0x02,0x7F}}, {'N', {0x7F,0x02,0x0C,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}}, {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'Q', {0x3E,0x41,0x51,0x21,0x5E}}, {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x26,0x49,0x49,0x49,0x32}}, {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}}, {'V', {0x07,0x18,0x20,0x18,0x07}},
    {'W', {0x3F,0x40,0x38,0x40,0x3F}}, {'X', {0x63,0x14,0x08,0x14,0x63}},
    {'Y', {0x07,0x08,0x70,0x08,0x07}}, {'Z', {0x61,0x51,0x49,0x45,0x43}},
    {'[', {0x00,0x7F,0x41,0x41,0x00}}, {'\\',{0x02,0x04,0x08,0x10,0x20}},
    {']', {0x00,0x41,0x41,0x7F,0x00}}, {'^', {0x04,0x02,0x01,0x02,0x04}},
    {'_', {0x40,0x40,0x40,0x40,0x40}}, {'`', {0x00,0x01,0x02,0x04,0x00}},
    {'a', {0x20,0x54,0x54,0x54,0x78}}, {'b', {0x7F,0x48,0x44,0x44,0x38}},
    {'c', {0x38,0x44,0x44,0x44,0x20}}, {'d', {0x38,0x44,0x44,0x48,0x7F}},
    {'e', {0x38,0x54,0x54,0x54,0x18}}, {'f', {0x08,0x7E,0x09,0x01,0x02}},
    {'g', {0x08,0x14,0x54,0x54,0x3C}}, {'h', {0x7F,0x08,0x04,0x04,0x78}},
    {'i', {0x00,0x44,0x7D,0x40,0x00}}, {'j', {0x20,0x40,0x44,0x3D,0x00}},
    {'k', {0x7F,0x10,0x28,0x44,0x00}}, {'l', {0x00,0x41,0x7F,0x40,0x00}},
    {'m', {0x7C,0x04,0x18,0x04,0x78}}, {'n', {0x7C,0x08,0x04,0x04,0x78}},
    {'o', {0x38,0x44,0x44,0x44,0x38}}, {'p', {0x7C,0x14,0x14,0x14,0x08}},
    {'q', {0x08,0x14,0x14,0x7C,0x40}}, {'r', {0x7C,0x08,0x04,0x04,0x08}},
    {'s', {0x48,0x54,0x54,0x54,0x20}}, {'t', {0x04,0x3F,0x44,0x40,0x20}},
    {'u', {0x3C,0x40,0x40,0x20,0x7C}}, {'v', {0x1C,0x20,0x40,0x20,0x1C}},
    {'w', {0x3C,0x40,0x30,0x40,0x3C}}, {'x', {0x44,0x28,0x10,0x28,0x44}},
    {'y', {0x0C,0x50,0x50,0x50,0x3C}}, {'z', {0x44,0x64,0x54,0x4C,0x44}},
    {'{', {0x00,0x08,0x36,0x41,0x00}}, {'|', {0x00,0x00,0x7F,0x00,0x00}},
    {'}', {0x00,0x41,0x36,0x08,0x00}}, {'~', {0x08,0x04,0x08,0x10,0x08}},
};

static const uint8_t* bootGlyph(char c) {
    for (unsigned i = 0; i < sizeof(BOOT_FONT5X7) / sizeof(BOOT_FONT5X7[0]); i++) {
        if (BOOT_FONT5X7[i].c == c) return BOOT_FONT5X7[i].col;
    }
    return BOOT_FONT5X7[0].col;
}

static uint16_t base64UrlEncode(const uint8_t* data, uint16_t len, char* out, uint16_t outSize) {
    static const char tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
    uint16_t outLen = 0;
    uint16_t i = 0;
    while (i + 3 <= len) {
        uint32_t v = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8) | data[i + 2];
        i += 3;
        if (outLen + 4 >= outSize) return 0;
        out[outLen++] = tbl[(v >> 18) & 63];
        out[outLen++] = tbl[(v >> 12) & 63];
        out[outLen++] = tbl[(v >> 6) & 63];
        out[outLen++] = tbl[v & 63];
    }
    uint16_t rem = (uint16_t)(len - i);
    if (rem == 1) {
        uint32_t v = ((uint32_t)data[i] << 16);
        if (outLen + 2 >= outSize) return 0;
        out[outLen++] = tbl[(v >> 18) & 63];
        out[outLen++] = tbl[(v >> 12) & 63];
    } else if (rem == 2) {
        uint32_t v = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8);
        if (outLen + 3 >= outSize) return 0;
        out[outLen++] = tbl[(v >> 18) & 63];
        out[outLen++] = tbl[(v >> 12) & 63];
        out[outLen++] = tbl[(v >> 6) & 63];
    }
    if (outLen >= outSize) return 0;
    out[outLen] = '\0';
    return outLen;
}

static void bytesToHex(const uint8_t* in, uint16_t len, char* out, uint16_t outSize) {
    static const char* H = "0123456789ABCDEF";
    if (!out || outSize == 0) return;
    if (outSize < (uint16_t)(len * 2 + 1)) {
        out[0] = '\0';
        return;
    }
    for (uint16_t i = 0; i < len; i++) {
        out[i * 2] = H[(in[i] >> 4) & 0x0F];
        out[i * 2 + 1] = H[in[i] & 0x0F];
    }
    out[len * 2] = '\0';
}

static bool bootKeyIsAllZero() {
    for (int i = 0; i < 16; i++) {
        if (securityConfig.encryption_key[i] != 0) return false;
    }
    return true;
}

static void formatBootKeyDisplay(char* out, uint16_t outSize) {
    const uint16_t hexLen = 32;
    if (!out || outSize < hexLen + 1) {
        if (out && outSize > 0) out[0] = '\0';
        return;
    }
    if (bootKeyIsAllZero()) {
        for (uint16_t i = 0; i < hexLen; i++) out[i] = '-';
        out[hexLen] = '\0';
    } else if (!(securityConfig.flags & SECURITY_FLAG_SHOW_KEY_ON_SCREEN)) {
        for (uint16_t i = 0; i < hexLen; i++) out[i] = 'X';
        out[hexLen] = '\0';
    } else {
        bytesToHex(securityConfig.encryption_key, 16, out, outSize);
    }
}

static void bootFormatKeyLine(char* out, size_t outSize, const char* label,
                              const uint8_t* keyBytes, uint8_t numBytes) {
    if (bootKeyIsAllZero()) {
        snprintf(out, outSize, "%s not set", label);
    } else if (!(securityConfig.flags & SECURITY_FLAG_SHOW_KEY_ON_SCREEN)) {
        snprintf(out, outSize, "%s hidden", label);
    } else {
        char hex[17];
        bytesToHex(keyBytes, numBytes, hex, sizeof(hex));
        snprintf(out, outSize, "%s %s", label, hex);
    }
}

static inline void setBootPixelBlack(uint8_t* row, uint16_t x, int pitch, int bitsPerPixel, uint8_t colorScheme) {
    if (bitsPerPixel == 1) {
        int bytePos = x / 8;
        int bitPos = 7 - (x % 8);
        if (bytePos < pitch) row[bytePos] &= ~(1 << bitPos);
    } else if (bitsPerPixel == 2) {
        int bytePos = x / 4;
        int shift = 6 - ((x % 4) * 2);
        if (bytePos < pitch) row[bytePos] &= ~(0x03 << shift);
    } else {
        int bytePos = x / 2;
        if (bytePos < pitch) {
            if ((x % 2) == 0) row[bytePos] = (row[bytePos] & 0x0F);
            else row[bytePos] = (row[bytePos] & 0xF0);
        }
    }
    (void)colorScheme;
}

static inline void setBootPixelCode(uint8_t* row, uint16_t x, int pitch, int bitsPerPixel, uint8_t code) {
    if (bitsPerPixel == 1) {
        int bytePos = x / 8;
        int bitPos = 7 - (x % 8);
        if (bytePos < pitch) {
            if (code & 1) row[bytePos] |= (uint8_t)(1 << bitPos);
            else row[bytePos] &= (uint8_t)~(1 << bitPos);
        }
    } else if (bitsPerPixel == 2) {
        int bytePos = x / 4;
        int shift = 6 - ((x % 4) * 2);
        if (bytePos < pitch) {
            row[bytePos] &= (uint8_t)~(0x03 << shift);
            row[bytePos] |= (uint8_t)((code & 0x03) << shift);
        }
    } else {
        int bytePos = x / 2;
        if (bytePos < pitch) {
            if ((x % 2) == 0) row[bytePos] = (uint8_t)((row[bytePos] & 0x0F) | ((code & 0x0F) << 4));
            else row[bytePos] = (uint8_t)((row[bytePos] & 0xF0) | (code & 0x0F));
        }
    }
}

static bool bootTextPixelBlack(uint16_t lx, uint16_t ly,
                               uint16_t x0, uint16_t y0,
                               const char* s, uint8_t scale,
                               uint16_t w_log, int maxX) {
    if (!s || scale == 0) return false;
    uint16_t clipRight = w_log;
    if (maxX >= 0 && (uint16_t)maxX < clipRight) clipRight = (uint16_t)maxX;
    if (lx >= clipRight || lx < x0) return false;
    if (ly < y0 || ly >= (uint16_t)(y0 + 7u * scale)) return false;
    uint8_t gy = (uint8_t)((ly - y0) / scale);
    uint16_t cursor = x0;
    for (const char* p = s; *p; p++) {
        uint16_t charEnd  = (uint16_t)(cursor + 5u * scale);
        uint16_t nextCursor = (uint16_t)(cursor + 6u * scale);
        if (lx < nextCursor) {
            if (lx >= charEnd) return false;  // inter-character gap
            uint8_t col = (uint8_t)((lx - cursor) / scale);
            return ((bootGlyph(*p)[col] >> gy) & 1) != 0;
        }
        cursor = nextCursor;
    }
    return false;
}

static bool bootQrPixelBlack(uint16_t lx, uint16_t ly,
                              int qrX, int qrY, int qrPx, int modulePx,
                              uint8_t quiet, uint8_t qrSize, QRCode* qr) {
    if (qrX < 0 || lx < (uint16_t)qrX || lx >= (uint16_t)(qrX + qrPx)) return false;
    if (qrY < 0 || ly < (uint16_t)qrY || ly >= (uint16_t)(qrY + qrPx)) return false;
    int16_t mx = (int16_t)((lx - (uint16_t)qrX) / (uint16_t)modulePx) - (int16_t)quiet;
    int16_t my = (int16_t)((ly - (uint16_t)qrY) / (uint16_t)modulePx) - (int16_t)quiet;
    if (mx < 0 || my < 0 || mx >= (int16_t)qrSize || my >= (int16_t)qrSize) return false;
    return qrcode_getModule(qr, (uint8_t)mx, (uint8_t)my);
}

static bool bootLogoPixelBlack(uint16_t lx, uint16_t ly,
                               int logoX, int logoY,
                               const uint8_t* bmp, int bmpW, int bmpH, int stride,
                               int maxX) {
    if ((int)lx < logoX || (int)lx >= logoX + bmpW) return false;
    if ((int)lx >= maxX) return false;
    if ((int)ly < logoY || (int)ly >= logoY + bmpH) return false;
    int bx = (int)lx - logoX;
    int by = (int)ly - logoY;
    return (bmp[by * stride + bx / 8] >> (7 - (bx & 7))) & 1;
}

static uint16_t bootTextWidth(const char* s, uint8_t scale) {
    return (!s || scale == 0) ? 0 : (uint16_t)(strlen(s) * 6U * scale);
}

static int bootLineStep(int scale) {
    return scale * 10;
}

// Returns the swatch index [0, numSwatches) containing logical pixel (lx, ly), or -1 if none.
static inline int bootSwatchIndex(uint16_t lx, uint16_t ly,
                                   int swatchY0, int swatchY1,
                                   int swatchW, int numSwatches, uint16_t w_log) {
    if (numSwatches <= 0 || swatchW <= 0) return -1;
    if ((int)ly < swatchY0 || (int)ly >= swatchY1) return -1;
    if (lx >= w_log) return -1;
    int idx = (int)lx / swatchW;
    if (idx >= numSwatches) idx = numSwatches - 1;  // rightmost swatch absorbs remainder
    return idx;
}

// Returns true if (lx, ly) lies on the 1-pixel border of its swatch box.
static inline bool bootSwatchIsBorder(uint16_t lx, uint16_t ly,
                                       int swatchY0, int swatchY1,
                                       int swatchW, int numSwatches, uint16_t w_log) {
    int idx = bootSwatchIndex(lx, ly, swatchY0, swatchY1, swatchW, numSwatches, w_log);
    if (idx < 0) return false;
    int boxX0 = idx * swatchW;
    int boxX1 = (idx == numSwatches - 1) ? (int)w_log : (idx + 1) * swatchW;
    return ((int)lx == boxX0 || (int)lx == boxX1 - 1 ||
            (int)ly == swatchY0 || (int)ly == swatchY1 - 1);
}


static uint16_t bootMaxTextWidth(const char* const* lines, unsigned n, int scale) {
    uint16_t maxW = 0;
    for (unsigned i = 0; i < n; i++) {
        uint16_t tw = bootTextWidth(lines[i], (uint8_t)scale);
        if (tw > maxW) maxW = tw;
    }
    return maxW;
}

static void formatBootColorSchemeText(uint8_t scheme, char* out, uint16_t outSize) {
    if (!out || outSize == 0) return;
    switch (scheme) {
        case COLOR_SCHEME_MONO:    snprintf(out, outSize, "BW"); break;
        case COLOR_SCHEME_BWR:     snprintf(out, outSize, "BWR"); break;
        case COLOR_SCHEME_BWY:     snprintf(out, outSize, "BWY"); break;
        case COLOR_SCHEME_BWRY:    snprintf(out, outSize, "BWRY"); break;
        case COLOR_SCHEME_BWGBRY:  snprintf(out, outSize, "6 color"); break;
        case COLOR_SCHEME_GRAY4:   snprintf(out, outSize, "4 gray"); break;
        case COLOR_SCHEME_GRAY16:  snprintf(out, outSize, "16 gray"); break;
        case COLOR_SCHEME_GRAY8:   snprintf(out, outSize, "7 color"); break;
        default:                   snprintf(out, outSize, "?"); break;
    }
}

// QR module pixel cap by panel size (qrModules=49 for v6 + quiet zone).
// 1872x1404: smaller text leaves more width for QR (~13 modules); height allows ~18.
static int bootQrModuleMax(uint16_t w, uint16_t h_full) {
    if (w >= 1872 && h_full >= 1404) return 18;
    if (w >= 1600 && h_full >= 1200) return 16;
    if (w >= 1200 && h_full >= 900)  return 12;
    if (w >= 800 && h_full >= 600)   return 8;
    if (w >= 800 && h_full >= 480)   return 7;
    if (w >= 600 && h_full >= 400 && h_full < 600) return 7;
    return 6;
}

// Max text scale for the middle zone (header/footer use their own logic).
static int bootMiddleScaleHi(uint16_t w_log, uint16_t h_log, bool useHighResLayout) {
    if (w_log >= 1600 && h_log >= 1200) return 10;
    if (w_log >= 800 && h_log >= 600) return 8;
    if (!useHighResLayout) return 2;
    if (h_log < 600) return (w_log >= 600) ? 3 : 2;
    return 6;
}

static int bootMiddlePad(int scale, uint16_t w_log, uint16_t h_log, bool useZoneLayout) {
    if (useZoneLayout && w_log < 800 && h_log >= 400 && h_log < 600) return 4 * scale;
    return 6 * scale;
}


static void bootFooterLayoutSegs(const char* const* segs, int numSegs, int scale,
                                 int bandLeft, int bandRight, int* xOut) {
    if (numSegs <= 0) return;
    int totalW = 0;
    for (int i = 0; i < numSegs; i++) totalW += (int)bootTextWidth(segs[i], (uint8_t)scale);
    const int bandW = bandRight - bandLeft;
    const int gap = (bandW > totalW) ? (bandW - totalW) / (numSegs + 1) : 0;
    int x = bandLeft + gap;
    for (int i = 0; i < numSegs; i++) {
        xOut[i] = x;
        x += (int)bootTextWidth(segs[i], (uint8_t)scale) + gap;
    }
}

static bool bootFooterTextPixelBlack(uint16_t lx, uint16_t ly, uint16_t footerInfoY,
                                     const char* const* segs, const int* segX, int numSegs,
                                     uint8_t scale, uint16_t w_log, int bandRight) {
    for (int i = 0; i < numSegs; i++) {
        if (bootTextPixelBlack(lx, ly, (uint16_t)segX[i], footerInfoY, segs[i], scale, w_log, bandRight)) {
            return true;
        }
    }
    return false;
}

static int bootHeaderLineGap(int manufScale) {
    int g = 4 * manufScale;
    return g < 4 ? 4 : g;
}

static int bootHeaderBlockH(const char* manuf, const char* model, int manufScale, int modelScale) {
    const int gap = bootHeaderLineGap(manufScale);
    if (manuf[0] && model[0]) return 7 * manufScale + gap + 7 * modelScale;
    if (manuf[0]) return 7 * manufScale;
    if (model[0]) return 7 * modelScale;
    return 0;
}

// Unused since bootPickHeaderScales switched to a direct formula instead of a search loop.
// Kept in case the search approach is revived.
// static bool bootHeaderTextFits(int headerH, const char* manuf, const char* model,
//                                int manufScale, int modelScale,
//                                int manufMaxW, int modelMaxW) {
//     const int blockH = bootHeaderBlockH(manuf, model, manufScale, modelScale);
//     if (blockH == 0) return true;
//     if (blockH > headerH - 8) return false;
//     if (manuf[0] && bootTextWidth(manuf, (uint8_t)manufScale) > (uint16_t)manufMaxW) return false;
//     if (model[0] && bootTextWidth(model, (uint8_t)modelScale) > (uint16_t)modelMaxW) return false;
//     return true;
// }

static void bootPickHeaderScales(int headerH, int headerMaxX, int pad,
                                 const char* manuf, const char* model,
                                 int* manufScaleOut, int* modelScaleOut) {
    *manufScaleOut = 1;
    *modelScaleOut = 1;
    if (!manuf[0] && !model[0]) return;

    const int availW = headerMaxX - pad;

    if (manuf[0] && model[0]) {
        int modelLen = (int)strlen(model);
        if (modelLen < 20) modelLen = 20;
        int modelS = availW / (modelLen * 6);
        if (modelS < 1) modelS = 1;
        int manufS = (modelS * 13 + 19) / 20;  // target: manuf scale = 65% of model scale, rounded up
        if (manufS < 1) manufS = 1;
        while (modelS > 1) {
            if (bootHeaderBlockH(manuf, model, manufS, modelS) <= headerH - 8) break;
            modelS--;
            manufS = (modelS * 13 + 19) / 20;  // target: manuf scale = 65% of model scale, rounded up
        }
        *manufScaleOut = manufS;
        *modelScaleOut = modelS;
        return;
    }

    if (model[0]) {
        int modelLen = (int)strlen(model);
        if (modelLen < 20) modelLen = 20;
        int modelS = availW / (modelLen * 6);
        if (modelS < 1) modelS = 1;
        while (modelS > 1 && bootHeaderBlockH("", model, 1, modelS) > headerH - 8) modelS--;
        *modelScaleOut = modelS;
        return;
    }

    int manufLen = (int)strlen(manuf);
    if (manufLen < 20) manufLen = 20;
    int manufS = availW / (manufLen * 6);
    if (manufS < 1) manufS = 1;
    while (manufS > 1 && bootHeaderBlockH(manuf, "", manufS, 1) > headerH - 8) manufS--;
    *manufScaleOut = manufS;
}

static bool bootLayoutFit(uint16_t w, uint16_t h, uint16_t h_full, int blockH, int pad, int qrModules, int* modulePxOut,
                          int* qrPxOut, bool* qrRightOut, int* qrXOut, int* qrYOut, int* availWOut,
                          int* textYOut, uint16_t maxTextW) {
    int textGap = pad;
    int sideGap = pad;
    int modulePx;
    int qrPx;
    int minDim = (int)((w < h) ? w : h);
    int moduleIdeal = (minDim - pad * 2) / qrModules;
    if (moduleIdeal < 1) moduleIdeal = 1;
    const int moduleMax = bootQrModuleMax(w, h_full);
    if (moduleIdeal > moduleMax) moduleIdeal = moduleMax;

    for (modulePx = moduleIdeal; modulePx >= 1; modulePx--) {
        qrPx = modulePx * qrModules;
        if (qrPx > (int)w - pad * 2) continue;
        // Landscape: text left, QR right — both vertically centered in the taller of the two
        if ((int)w >= pad * 2 + (int)maxTextW + sideGap + qrPx
            && (int)h >= pad * 2 + (blockH > qrPx ? blockH : qrPx)) {
            int availW = (int)w - pad * 2 - qrPx - sideGap;
            if ((int)maxTextW <= availW) {
                int combinedH = (blockH > qrPx) ? blockH : qrPx;
                int zoneY = ((int)h - combinedH) / 2;
                int qrX = (int)w - pad - qrPx;
                int qrY = zoneY;
                int textY = zoneY;
                if (blockH < qrPx) textY += (qrPx - blockH) / 2;
                else                qrY  += (blockH - qrPx) / 2;
                *modulePxOut = modulePx;
                *qrPxOut = qrPx;
                *qrRightOut = true;
                *qrXOut = qrX;
                *qrYOut = qrY;
                *availWOut = availW;
                *textYOut = textY;
                return true;
            }
        }
        // Portrait: text on top, QR below — both horizontally centered
        if ((int)w >= pad * 2 + (int)maxTextW && (int)h >= pad * 2 + blockH + textGap + qrPx) {
            int totalH = blockH + textGap + qrPx;
            int zoneY = ((int)h - totalH) / 2;
            *modulePxOut = modulePx;
            *qrPxOut = qrPx;
            *qrRightOut = false;
            *qrXOut = ((int)w - qrPx) / 2;
            *qrYOut = zoneY + blockH + textGap;
            *availWOut = (int)w - pad * 2;
            *textYOut = zoneY;
            return true;
        }
    }
    return false;
}

// 4-gray panels store each pixel's 2-bit code as two separate 1-bit controller
// planes (LSB -> PLANE_0, MSB -> PLANE_1) combined by the panel's gray LUT.
// De-interleave one bit of the packed 2bpp row into a 1bpp plane row (MSB-first)
// and stream it. planeRow scratch lives just past the 2bpp row in staticRowBuffer
// (200B + 100B for an 800px panel, well within the 680B buffer).
static void writeGray4PlaneRow(const uint8_t* row2bpp, int pitch2bpp, int planePitch, uint16_t w, int bitSel) {
    uint8_t* planeRow = staticRowBuffer + pitch2bpp;
    memset(planeRow, 0x00, planePitch);
    for (uint16_t x = 0; x < w; x++) {
        // SSD16xx 4-gray stores the inverted gray code (bb_epaper pColorLookup maps
        // level -> 3 - level); the boot image uses level 0=black, 3=white.
        uint8_t code = (uint8_t)((~(row2bpp[x >> 2] >> (6 - ((x & 3) << 1)))) & 0x03);
        if ((code >> bitSel) & 0x01) planeRow[x >> 3] |= (uint8_t)(0x80 >> (x & 7));
    }
    bbepWriteData(&bbep, planeRow, planePitch);
}

static const uint8_t kSchemeWhiteValue[] = {
    0xFF,  // 0: COLOR_SCHEME_MONO   — 1bpp, bit 1 = white
    0xFF,  // 1: COLOR_SCHEME_BWR    — 1bpp bitplane, all-ones = white plane
    0xFF,  // 2: COLOR_SCHEME_BWY    — 1bpp bitplane
    0x55,  // 3: COLOR_SCHEME_BWRY   — 2bpp, code 01b per pixel = white
    0x11,  // 4: COLOR_SCHEME_BWGBRY — 4bpp Spectra6, nibble 1 = white
    0xFF,  // 5: COLOR_SCHEME_GRAY4  — 2bpp gray, code 11b per pixel = white
    0xFF,  // 6: COLOR_SCHEME_GRAY16 — 4bpp Seeed 16-gray, nibble 15 = white
    0xFF,  // 7: COLOR_SCHEME_GRAY8  — 1bpp (default)
};

// Spectra 6 footer swatches: palette order black/white/yellow/red/blue/green →
// firmware nibbles 0,1,2,3,5,6 (4 unused). Same as py-opendisplay BWGBRY_MAP.
static const uint8_t kBwgbrySwatchCodes[] = {0, 1, 2, 3, 5, 6};

// 4-gray dither level (0=black..3=white) -> stored 2-bit panel code (bb_epaper /
// py-opendisplay get_gray4_codes). Boot 2bpp buffer inverts before plane split.
static const uint8_t kGray4StoredBase[4] = {3, 1, 2, 0};
static const uint8_t kGray4StoredV2[4]  = {3, 2, 1, 0};

static bool bootGray4PanelUsesLutV2(uint16_t panelIc) {
    return panelIc == 0x0028;  // EP426_800x480_4GRAY (u8Colors_4gray_v2)
}

static void bootGray4FillSwatchCodes(uint16_t panelIc, uint8_t out[4]) {
    const uint8_t* stored = bootGray4PanelUsesLutV2(panelIc) ? kGray4StoredV2 : kGray4StoredBase;
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    const bool direct2bpp = seeed_driver_used();
#else
    const bool direct2bpp = false;
#endif
    for (int level = 0; level < 4; level++) {
        out[level] = direct2bpp ? (uint8_t)level : (uint8_t)((~stored[level]) & 3);
    }
}

bool writeBootScreenWithQr() {
    const uint16_t w = globalConfig.displays[0].pixel_width;
    const uint16_t h = globalConfig.displays[0].pixel_height;
    const uint8_t rotation = globalConfig.displays[0].rotation; // 0=0°,1=90°,2=180°,3=270°
    const bool is90or270 = (rotation == 1 || rotation == 3);
    const uint16_t w_log = is90or270 ? h : w;  // logical (user-visible) width
    const uint16_t h_log = is90or270 ? w : h;  // logical (user-visible) height
    // Three-zone layout on medium+ screens only
    // Landscape: header 20% / middle 65% / footer 15% / footerInfo 4% (taken from footer)
    // Portrait:  header 15% / middle 73% / footer 12% / footerInfo 3% (taken from footer)
    const bool useZoneLayout = (w_log >= 400 && h_log >= 300);
    const bool useHighResLayout = useZoneLayout && (w_log >= 600 && h_log >= 400);
    const bool isLandscape = w_log > h_log;
    const int headerH  = useZoneLayout ? (int)h_log * (isLandscape ? 20 : 15) / 100 : 0;
    const int footerH  = useZoneLayout ? (int)h_log * (isLandscape ? 15 : 12) / 100 : 0;
    const int targetFooterInfoH = useZoneLayout ? (int)h_log * (isLandscape ? 4 : 3) / 100 : 0;
    const int footerInfoScale = (targetFooterInfoH - 4) / 7;
    const int footerInfoH = footerInfoScale * 7 + 4;
    const int footerY0 = (int)h_log - footerH;
    const int middleH  = footerY0 - headerH;
    const uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    const bool useBitplanes = (colorScheme == COLOR_SCHEME_BWR || colorScheme == COLOR_SCHEME_BWY);
    const int bitsPerPixel = getBitsPerPixel();
    int pitch;
    if (bitsPerPixel == 4) pitch = w / 2;
    else if (bitsPerPixel == 2) pitch = (w + 3) / 4;
    else pitch = (w + 7) / 8;
    uint8_t whiteValue = (colorScheme < sizeof(kSchemeWhiteValue))
        ? kSchemeWhiteValue[colorScheme]
        : 0xFF;

    // Footer color swatches: one box per color the scheme can produce, 1px black border.
    const int footerPadTop = useZoneLayout ? 8 : 0;
    int swatchY0 = 0;
    const int swatchY1 = (int)h_log;
    int swatchH = 0;
    int swatchW = 0;
    char footerSchemeStr[16] = "";
    char footerResStr[16] = "";
    const char* footerSegs[6];
    int footerSegX[6] = {0};
    int numFooterSegs = 0;
    uint8_t swatchCode[16] = {0};
    bool    swatchIsColor[16] = {false};
    int     numSwatches = 0;
    if (useZoneLayout && footerH > footerPadTop + 20) {
        switch (colorScheme) {
            case COLOR_SCHEME_MONO:
            case COLOR_SCHEME_GRAY8:
                swatchCode[0] = 0; swatchCode[1] = 1;
                numSwatches = 2;
                break;
            case COLOR_SCHEME_BWR:
            case COLOR_SCHEME_BWY:
                swatchCode[0] = 0; swatchIsColor[0] = false;
                swatchCode[1] = 1; swatchIsColor[1] = false;
                swatchCode[2] = 1; swatchIsColor[2] = true;  // red/yellow: PLANE_1 bit set
                numSwatches = 3;
                break;
            case COLOR_SCHEME_BWRY:
                swatchCode[0] = 0; swatchCode[1] = 1; swatchCode[2] = 2; swatchCode[3] = 3;
                numSwatches = 4;
                break;
            case COLOR_SCHEME_BWGBRY:
                for (int i = 0; i < 6; i++) swatchCode[i] = kBwgbrySwatchCodes[i];
                numSwatches = 6;
                break;
            case COLOR_SCHEME_GRAY4:
                bootGray4FillSwatchCodes(globalConfig.displays[0].panel_ic_type, swatchCode);
                numSwatches = 4;
                break;
            case COLOR_SCHEME_GRAY16:
                for (int i = 0; i < 16; i++) swatchCode[i] = (uint8_t)i;
                numSwatches = 16;
                break;
            default:
                break;
        }
    }
    const bool colorSwatchPlane1 = useBitplanes && numSwatches > 0;

    String chipId = getChipIdHex();
    if (chipId.length() < 6) {
        chipId = String("000000").substring(0, 6 - chipId.length()) + chipId;
    }
    String last6 = chipId.substring(chipId.length() - 6);
    for (size_t i = 0; i < last6.length(); i++) last6.setCharAt(i, (char)toupper(last6.charAt(i)));

    uint8_t payload[23] = {0};
    uint16_t res = globalConfig.displays[0].tag_type;
    payload[0] = (uint8_t)((res >> 8) & 0xFF);
    payload[1] = (uint8_t)(res & 0xFF);
    payload[2] = (uint8_t)strtoul(last6.substring(0, 2).c_str(), nullptr, 16);
    payload[3] = (uint8_t)strtoul(last6.substring(2, 4).c_str(), nullptr, 16);
    payload[4] = (uint8_t)strtoul(last6.substring(4, 6).c_str(), nullptr, 16);
    if (securityConfig.flags & SECURITY_FLAG_SHOW_KEY_ON_SCREEN) {
        memcpy(&payload[5], securityConfig.encryption_key, 16);
    } else {
        memset(&payload[5], 0, 16);
    }
    uint16_t mfg = globalConfig.manufacturer_data.manufacturer_id;
    payload[21] = (uint8_t)((mfg >> 8) & 0xFF);
    payload[22] = (uint8_t)(mfg & 0xFF);

    char payloadB64[64];
    if (!base64UrlEncode(payload, sizeof(payload), payloadB64, sizeof(payloadB64))) return false;

    char url[128];
    snprintf(url, sizeof(url), "https://opendisplay.org/l/?%s", payloadB64);

    QRCode qr;
    const uint8_t qrVersion = 6;
    uint8_t qrBuf[256];
    uint16_t qrBufSize = qrcode_getBufferSize(qrVersion);
    if (qrBufSize == 0 || qrBufSize > sizeof(qrBuf)) return false;
    if (qrcode_initText(&qr, qrBuf, qrVersion, ECC_MEDIUM, url) != 0) return false;

    const uint8_t qrSize = qr.size;
    const uint8_t quiet = 4;
    const uint16_t qrModules = (uint16_t)(qrSize + 2 * quiet);

// ARRANGING THESE IN ORDER OF HOW THEY SHOW UP.  
// ADDED HANDLING OF OPTIONAL EXTENDED DATA LINES FOR MANUFACTURER AND MODEL.
// IF NOT PRESENT, THEY WILL BE EMPTY STRINGS.
    char manufLine[32] = "";
    char modelLine[32] = "";
    if (globalConfig.data_extended_loaded) {
        snprintf(manufLine, sizeof(manufLine), "%s", (char*)globalConfig.data_extended.manufacturer_name);
        snprintf(modelLine, sizeof(modelLine), "%s", (char*)globalConfig.data_extended.model_name);
    }
    const char* domainLine = useZoneLayout ? "URL:  OpenDisplay.org" : "OpenDisplay.org";
    char nameLine[32];
    char fwLine[32];
    if (useZoneLayout) {
        snprintf(nameLine, sizeof(nameLine), "ID:   OD%s", last6.c_str());
        snprintf(fwLine, sizeof(fwLine), "FW:   OD ver %u.%u", (unsigned)getFirmwareMajor(), (unsigned)getFirmwareMinor());
    } else {
        snprintf(nameLine, sizeof(nameLine), "OD%s", last6.c_str());
        snprintf(fwLine, sizeof(fwLine), "FW:O %u.%u", (unsigned)getFirmwareMajor(), (unsigned)getFirmwareMinor());
    }
    char k1[24], k2[24];
    if (useZoneLayout) {
        bootFormatKeyLine(k1, sizeof(k1), "KEY1:", securityConfig.encryption_key, 8);
        bootFormatKeyLine(k2, sizeof(k2), "KEY2:", securityConfig.encryption_key + 8, 8);
    } else {
        char keyHex[33];
        formatBootKeyDisplay(keyHex, sizeof(keyHex));
        memcpy(k1, keyHex, 16);
        k1[16] = '\0';
        memcpy(k2, keyHex + 16, 16);
        k2[16] = '\0';
    }

    char vStr[8] = "";
    char tStr[8] = "";
    if (useZoneLayout) {
        float batV = readBatteryVoltage();
        float chipT = readChipTemperature();
        if (batV >= 0.0f) snprintf(vStr, sizeof(vStr), "%.2fV", batV);
        else snprintf(vStr, sizeof(vStr), "--V");
        if (chipT > -900.0f) snprintf(tStr, sizeof(tStr), "%.1fC", chipT);
        else snprintf(tStr, sizeof(tStr), "--C");
    }

    int middleScaleText;
    int pad;
    int fwKey1Gap = 0;
    int modulePx;
    int qrPx;
    bool qrRight;
    int qrX, qrY, availW, textY;
    const bool ultraHiResPanel = useZoneLayout && (w_log >= 1872 && h_log >= 1404);
    {
        const char* bootLinesFull[] = {
            domainLine, nameLine, fwLine, k1, k2,
        };
        const char* bootLinesReduced[] = {
            domainLine, nameLine, fwLine, k1, k2,
        };
        const unsigned numBootLines = useZoneLayout ? 5u : 5u;
        const char* const* bootLines = useZoneLayout ? bootLinesFull : bootLinesReduced;
        uint16_t maxTextW;
        bool layoutOk = false;
        int tryScale;
        fwKey1Gap = 0;

        const int scaleHi = bootMiddleScaleHi(w_log, h_log, useHighResLayout);
        for (tryScale = useZoneLayout ? scaleHi : 1; tryScale >= 1 && !layoutOk; tryScale--) {
            middleScaleText = ultraHiResPanel ? (tryScale > 2 ? tryScale - 2 : 1) : tryScale;
            pad = bootMiddlePad(middleScaleText, w_log, h_log, useZoneLayout);
            fwKey1Gap = useZoneLayout ? middleScaleText * 6 : 0;
            maxTextW = bootMaxTextWidth(bootLines, numBootLines, middleScaleText);
            int contentH = useZoneLayout
                ? (4 * bootLineStep(middleScaleText) + fwKey1Gap + 7 * middleScaleText)
                : (((int)numBootLines - 1) * bootLineStep(middleScaleText) + 7 * middleScaleText);
            layoutOk = bootLayoutFit(w_log, (uint16_t)middleH, h_log, contentH, pad, (int)qrModules, &modulePx, &qrPx, &qrRight, &qrX,
                                     &qrY, &availW, &textY, maxTextW);
        }
        if (!layoutOk) {
            middleScaleText = 1;
            pad = bootMiddlePad(middleScaleText, w_log, h_log, useZoneLayout);
            fwKey1Gap = useZoneLayout ? middleScaleText * 6 : 0;
            modulePx = 1;
            qrPx = modulePx * (int)qrModules;
            qrRight = false;
            qrX = ((int)w_log - qrPx) / 2;
            int contentH = useZoneLayout
                ? (4 * bootLineStep(middleScaleText) + fwKey1Gap + 7 * middleScaleText)
                : (((int)numBootLines - 1) * bootLineStep(middleScaleText) + 7 * middleScaleText);
            int totalH = contentH + pad + qrPx;
            int zoneY = (middleH - totalH) / 2;
            if (zoneY < pad) zoneY = pad;
            if (zoneY + totalH > middleH - pad) zoneY = middleH - pad - totalH;
            if (zoneY < pad) zoneY = pad;
            textY = zoneY;
            qrY = zoneY + contentH + pad;
            availW = (int)w_log - pad * 2;
        }
        // Translate layout coordinates into the middle zone
        qrY  += headerH;
        textY += headerH;
    }

    const int middleFwKey1Gap = useZoneLayout ? middleScaleText * 6 : 0;

    if (useZoneLayout && vStr[0] != '\0') {
        footerSegs[numFooterSegs++] = vStr;
        footerSegs[numFooterSegs++] = tStr;
        if (useHighResLayout) {
            formatBootColorSchemeText(colorScheme, footerSchemeStr, sizeof(footerSchemeStr));
            footerSegs[numFooterSegs++] = footerSchemeStr;
            if (globalConfig.displays[0].partial_update_support) {
                footerSegs[numFooterSegs++] = "+Partial Refresh";
            }
            snprintf(footerResStr, sizeof(footerResStr), "%ux%u",
                     (unsigned)w_log, (unsigned)h_log);
            footerSegs[numFooterSegs++] = footerResStr;
        }
    }

    const int contentRightX = (int)w_log - pad;

    const int footerTextY0 = footerY0 + footerPadTop;
    const int footerBandLeft = pad;
    const int footerBandRight = (int)w_log - pad;
    const int footerBandW = footerBandRight - footerBandLeft;
    if (numFooterSegs > 0) {
        bootFooterLayoutSegs(footerSegs, numFooterSegs, footerInfoScale,
                             footerBandLeft, footerBandRight, footerSegX);
    }
    swatchY0 = footerY0 + footerPadTop + (numFooterSegs > 0 ? footerInfoH : 0);
    swatchH = swatchY1 - swatchY0;
    if (numSwatches > 0 && swatchH > 2) {
        swatchW = (int)w_log / numSwatches;
    }

    int textOriginX = qrRight ? pad : ((int)w_log - availW) / 2;
    if (textOriginX < pad) textOriginX = pad;
    int textMaxX = (int)w_log;
    int domX   = textOriginX;
    int nameX  = textOriginX;
    int fwX    = textOriginX;
    int k1X    = textOriginX;
    int k2X    = textOriginX;

    int headerManufScale = 1;
    int headerModelScale = 1;
    int headerTextMaxX = (int)w_log - pad;
    int headerManufMaxX = (int)w_log;
    int headerModelMaxX = (int)w_log;
    int manufX = pad;
    int modelX = pad;
    uint16_t manufY = 0;
    uint16_t modelY = 0;

#ifdef BOOT_HAS_LOGO
    const uint8_t* logoBmp;
    int logoW = 0, logoH = 0, logoStride = 0;
    int logoX = pad;
    int logoY = 8;
    if (useZoneLayout) {
        // Logo scale: largest size whose height fits in headerH-16 and width is under 30% of logical width.
        const int maxLogoH = headerH - 16;
        const int maxLogoW = w_log * 3 / 10;
        int logoScale = 1;
        if (BOOT_LOGO_H_S3 <= maxLogoH && BOOT_LOGO_W_S3 < maxLogoW) logoScale = 3;
        else if (BOOT_LOGO_H_S2 <= maxLogoH && BOOT_LOGO_W_S2 < maxLogoW) logoScale = 2;
        if (logoScale >= 3) {
            logoBmp = BOOT_LOGO_BITMAP_S3; logoW = BOOT_LOGO_W_S3;
            logoH = BOOT_LOGO_H_S3; logoStride = BOOT_LOGO_STRIDE_S3;
        } else if (logoScale >= 2) {
            logoBmp = BOOT_LOGO_BITMAP_S2; logoW = BOOT_LOGO_W_S2;
            logoH = BOOT_LOGO_H_S2; logoStride = BOOT_LOGO_STRIDE_S2;
        } else {
            logoBmp = BOOT_LOGO_BITMAP_S1; logoW = BOOT_LOGO_W_S1;
            logoH = BOOT_LOGO_H_S1; logoStride = BOOT_LOGO_STRIDE_S1;
        }
        logoX = contentRightX - logoW;
        logoY = (headerH - logoH) / 2;
    }
#endif
    if (useZoneLayout && qrRight) {
        const int qrDataRightPx = modulePx * ((int)qrSize + (int)quiet);
        qrX = contentRightX - qrDataRightPx;
        textMaxX = qrX - pad;
    }
    if (useZoneLayout) {
#ifdef BOOT_HAS_LOGO
        headerTextMaxX = logoX - pad;
#endif
        if (manufLine[0] || modelLine[0]) {
            bootPickHeaderScales(headerH, headerTextMaxX, pad,
                                 manufLine, modelLine, &headerManufScale, &headerModelScale);
            headerManufMaxX = headerTextMaxX;
            headerModelMaxX = headerTextMaxX;
            const int headerGap = bootHeaderLineGap(headerManufScale);
            int headerBlockH = bootHeaderBlockH(manufLine, modelLine, headerManufScale, headerModelScale);
            const int headerBlockY0 = (headerH - headerBlockH) / 2;
            if (manufLine[0]) {
                manufY = (uint16_t)headerBlockY0;
                modelY = (uint16_t)(headerBlockY0 + 7 * headerManufScale + headerGap);
            } else {
                modelY = (uint16_t)headerBlockY0;
            }
        }
    }
    int textStartY = textY;
    const uint16_t footerInfoY = (uint16_t)(footerY0 + (footerPadTop + footerInfoH - 7 * footerInfoScale) / 2);

    uint8_t* row = staticRowBuffer;
    // bb_epaper 4-gray (scheme 5) needs the packed 2bpp image split into two
    // 1-bit controller planes, so render the frame once per plane and
    // de-interleave. Every other scheme writes a single packed plane in one pass.
    const bool gray4Split = (colorScheme == COLOR_SCHEME_GRAY4)
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        && !seeed_driver_used()
#endif
        ;
    const int planePitch = (w + 7) / 8;
    const size_t rowBytesNeeded = gray4Split ? (size_t)(pitch + planePitch) : (size_t)pitch;
    if (rowBytesNeeded > sizeof(staticRowBuffer)) {
        return false;
    }
    const int planePasses = (gray4Split || colorSwatchPlane1) ? 2 : 1;
    for (int pass = 0; pass < planePasses; pass++) {
        const int bitSel = pass;  // pass 0 -> LSB/PLANE_0, pass 1 -> MSB/PLANE_1
        const int targetPlane = gray4Split ? (pass == 0 ? PLANE_0 : PLANE_1)
                                           : (colorSwatchPlane1 ? (pass == 0 ? PLANE_0 : PLANE_1)
                                                                 : (useBitplanes ? PLANE_0 : getplane()));
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        if (!seeed_driver_used()) {
            bbepSetAddrWindow(&bbep, 0, 0, w, h);
            bbepStartWrite(&bbep, targetPlane);
        }
#else
        bbepSetAddrWindow(&bbep, 0, 0, w, h);
        bbepStartWrite(&bbep, targetPlane);
#endif
        const int ls = bootLineStep(middleScaleText);
        const uint16_t domY  = (uint16_t)textStartY;
        const uint16_t nameY = (uint16_t)(textStartY + ls);
        const uint16_t fwY   = (uint16_t)(textStartY + ls * 2);
        const uint16_t k1Y   = useZoneLayout
            ? (uint16_t)(fwY + ls + middleFwKey1Gap)
            : (uint16_t)(fwY + ls);
        const uint16_t k2Y   = (uint16_t)(k1Y + ls);
        const bool colorPlanePass = colorSwatchPlane1 && pass == 1;
        for (uint16_t y_native = 0; y_native < h; y_native++) {
            memset(row, colorPlanePass ? 0x00 : whiteValue, pitch);
            for (uint16_t x_native = 0; x_native < w; x_native++) {
                // Map native pixel to logical (user-visible) coordinates
                uint16_t lx, ly;
                switch (rotation) {
                    case 1:  lx = y_native; ly = (uint16_t)(h_log - 1u - x_native); break; // 90° CW
                    case 2:  lx = (uint16_t)(w_log - 1u - x_native); ly = (uint16_t)(h_log - 1u - y_native); break; // 180°
                    case 3:  lx = (uint16_t)(w_log - 1u - y_native); ly = x_native; break; // 270° CW
                    default: lx = x_native; ly = y_native; break;
                }

                if (colorPlanePass) {
                    // BWR/BWY color plane: only colored swatch interiors get bit=1.
                    int si = bootSwatchIndex(lx, ly, swatchY0, swatchY1, swatchW, numSwatches, w_log);
                    bool isBorder = bootSwatchIsBorder(lx, ly, swatchY0, swatchY1, swatchW, numSwatches, w_log);
                    if (si >= 0 && !isBorder && swatchIsColor[si]) {
                        setBootPixelCode(row, x_native, pitch, 1, 1);
                    }
                    continue;
                }

                bool isSwatchBorder = bootSwatchIsBorder(lx, ly, swatchY0, swatchY1, swatchW, numSwatches, w_log);
                bool black =
                    isSwatchBorder ||
                    (useZoneLayout && (
                        ly == (uint16_t)headerH || ly == (uint16_t)(headerH + 1) ||
                        ly == (uint16_t)footerY0 || ly == (uint16_t)(footerY0 + 1))) ||
                    (useZoneLayout && manufLine[0] && bootTextPixelBlack(lx, ly, (uint16_t)manufX, manufY, manufLine, (uint8_t)headerManufScale, w_log, headerManufMaxX)) ||
                    (useZoneLayout && modelLine[0] && bootTextPixelBlack(lx, ly, (uint16_t)modelX, modelY, modelLine, (uint8_t)headerModelScale, w_log, headerModelMaxX)) ||
                    bootTextPixelBlack(lx, ly, (uint16_t)domX,   domY,   domainLine, (uint8_t)middleScaleText, w_log, textMaxX) ||
                    bootTextPixelBlack(lx, ly, (uint16_t)nameX,  nameY,  nameLine,   (uint8_t)middleScaleText, w_log, textMaxX) ||
                    bootTextPixelBlack(lx, ly, (uint16_t)fwX,    fwY,    fwLine,     (uint8_t)middleScaleText, w_log, textMaxX) ||
                    (numFooterSegs > 0 && bootFooterTextPixelBlack(lx, ly, footerInfoY, footerSegs, footerSegX,
                        numFooterSegs, (uint8_t)footerInfoScale, w_log, footerBandRight)) ||
                    bootTextPixelBlack(lx, ly, (uint16_t)k1X,    k1Y,    k1,         (uint8_t)middleScaleText, w_log, textMaxX) ||
                    bootTextPixelBlack(lx, ly, (uint16_t)k2X,    k2Y,    k2,         (uint8_t)middleScaleText, w_log, textMaxX) ||
                    bootQrPixelBlack(lx, ly, qrX, qrY, qrPx, modulePx, quiet, qrSize, &qr)
#ifdef BOOT_HAS_LOGO
                    || (useZoneLayout && bootLogoPixelBlack(lx, ly, logoX, logoY, logoBmp, logoW, logoH, logoStride, (int)w_log))
#endif
                    ;
                if (black) {
                    setBootPixelBlack(row, x_native, pitch, bitsPerPixel, colorScheme);
                } else if (!useBitplanes) {
                    int si = bootSwatchIndex(lx, ly, swatchY0, swatchY1, swatchW, numSwatches, w_log);
                    if (si >= 0) setBootPixelCode(row, x_native, pitch, bitsPerPixel, swatchCode[si]);
                }
            }
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
            if (seeed_driver_used()) {
                seeed_gfx_boot_write_row(y_native, row, pitch);
            } else if (gray4Split) {
                writeGray4PlaneRow(row, pitch, planePitch, w, bitSel);
            } else {
                bbepWriteData(&bbep, row, pitch);
            }
#else
            if (gray4Split) {
                writeGray4PlaneRow(row, pitch, planePitch, w, bitSel);
            } else {
                bbepWriteData(&bbep, row, pitch);
            }
#endif
        }
    }

#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        seeed_gfx_boot_skip_planes();
    } else
#endif
    {
        if (useBitplanes && !colorSwatchPlane1) {
            memset(row, 0x00, pitch);
            bbepSetAddrWindow(&bbep, 0, 0, w, h);
            bbepStartWrite(&bbep, PLANE_1);
            for (uint16_t y = 0; y < h; y++) bbepWriteData(&bbep, row, pitch);
        }
    }
    writeSerial("Boot screen with QR rendered", true);
    return true;
}
