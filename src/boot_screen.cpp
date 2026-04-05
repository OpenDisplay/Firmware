#include "boot_screen.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "structs.h"
#include <bb_epaper.h>
#include "qr/qrcode.h"
#include "display_service.h"
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
#include "display_seeed_gfx.h"
#endif

extern struct GlobalConfig globalConfig;
extern struct SecurityConfig securityConfig;
extern BBEPDISP bbep;
extern uint8_t staticRowBuffer[680];

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
    {' ', {0,0,0,0,0}}, {'.', {0,0,0x40,0,0}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}}, {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x62,0x51,0x49,0x49,0x46}}, {'3', {0x22,0x49,0x49,0x49,0x36}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}}, {'5', {0x2F,0x49,0x49,0x49,0x31}},
    {'6', {0x3E,0x49,0x49,0x49,0x32}}, {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}}, {'9', {0x26,0x49,0x49,0x49,0x3E}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}}, {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}}, {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}}, {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}}, {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}}, {'N', {0x7F,0x02,0x0C,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}}, {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}}, {'S', {0x26,0x49,0x49,0x49,0x32}},
    {'W', {0x3F,0x40,0x38,0x40,0x3F}}, {'Y', {0x07,0x08,0x70,0x08,0x07}},
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

static void drawBootTextRow(uint8_t* row, uint16_t y, uint16_t x0, uint16_t y0, const char* s, uint8_t scale, uint16_t displayW, int pitch, int bitsPerPixel, uint8_t colorScheme) {
    if (!s || scale == 0) return;
    uint16_t cursor = x0;
    for (const char* p = s; *p; p++) {
        const uint8_t* g = bootGlyph(*p);
        for (uint8_t col = 0; col < 5; col++) {
            uint8_t bits = g[col];
            for (uint8_t gy = 0; gy < 7; gy++) {
                if (((bits >> gy) & 1) == 0) continue;
                uint16_t py = (uint16_t)(y0 + gy * scale);
                if (y < py || y >= (uint16_t)(py + scale)) continue;
                uint16_t px = (uint16_t)(cursor + col * scale);
                for (uint8_t sx = 0; sx < scale; sx++) {
                    for (uint8_t sy = 0; sy < scale; sy++) {
                        if (y == (uint16_t)(py + sy) && (uint16_t)(px + sx) < displayW) {
                            setBootPixelBlack(row, (uint16_t)(px + sx), pitch, bitsPerPixel, colorScheme);
                        }
                    }
                }
            }
        }
        cursor = (uint16_t)(cursor + 6 * scale);
    }
}

static uint16_t bootTextWidth(const char* s, uint8_t scale) {
    return (!s || scale == 0) ? 0 : (uint16_t)(strlen(s) * 6U * scale);
}

bool writeBootScreenWithQr() {
    const uint16_t w = globalConfig.displays[0].pixel_width;
    const uint16_t h = globalConfig.displays[0].pixel_height;
    const uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    const bool useBitplanes = (colorScheme == 1 || colorScheme == 2);
    const int bitsPerPixel = getBitsPerPixel();
    int pitch;
    uint8_t whiteValue;
    if (bitsPerPixel == 4) {
        pitch = w / 2;
        whiteValue = 0xFF;
    } else if (bitsPerPixel == 2) {
        pitch = (w + 3) / 4;
        whiteValue = (colorScheme == 5) ? 0xFF : 0x55;
    } else {
        pitch = (w + 7) / 8;
        whiteValue = 0xFF;
    }

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
    uint8_t scaleText = 1;
    if (w >= 560 && h >= 360) scaleText = 3;
    else if (w >= 360 && h >= 240) scaleText = 2;
    uint16_t pad = (uint16_t)(scaleText * 6);
    uint16_t qrPxMax = (uint16_t)((h > w ? w : h) - pad * 2);
    uint16_t modulePx = (uint16_t)(qrPxMax / qrModules);
    if (modulePx < 1) modulePx = 1;
    if (modulePx > 8) modulePx = 8;
    uint16_t qrPx = (uint16_t)(modulePx * qrModules);
    bool qrRight = (w >= (uint16_t)(qrPx + (120 * scaleText)));
    uint16_t qrX = qrRight ? (uint16_t)(w - pad - qrPx) : (uint16_t)((w - qrPx) / 2);
    uint16_t qrY = qrRight ? pad : (uint16_t)(h - pad - qrPx);

    char nameLine[16];
    snprintf(nameLine, sizeof(nameLine), "OD%s", last6.c_str());
    char fwLine[16];
    snprintf(fwLine, sizeof(fwLine), "FW:%d.%d", getFirmwareMajor(), getFirmwareMinor());
    const char* domainLine = "OPENDISPLAY.ORG";
    char keyHex[33];
    bytesToHex(&payload[5], 16, keyHex, sizeof(keyHex));
    char k1[17], k2[17];
    memcpy(k1, keyHex, 16); k1[16] = '\0';
    memcpy(k2, keyHex + 16, 16); k2[16] = '\0';

    uint8_t* row = staticRowBuffer;
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (!seeed_driver_used()) {
        bbepSetAddrWindow(&bbep, 0, 0, w, h);
        bbepStartWrite(&bbep, getplane());
    }
#else
    bbepSetAddrWindow(&bbep, 0, 0, w, h);
    bbepStartWrite(&bbep, getplane());
#endif
    for (uint16_t y = 0; y < h; y++) {
        memset(row, whiteValue, pitch);
        uint16_t availW = qrRight ? qrX : w;
        uint16_t textY = pad;
        if (qrRight) {
            uint16_t lineH = (uint16_t)(scaleText * 10);
            uint16_t blockH = (uint16_t)(4 * lineH + 7 * scaleText);
            textY = (qrPx > blockH) ? (uint16_t)(qrY + (qrPx - blockH) / 2) : qrY;
        }
        uint16_t dW = bootTextWidth(domainLine, scaleText);
        uint16_t nW = bootTextWidth(nameLine, scaleText);
        uint16_t fW = bootTextWidth(fwLine, scaleText);
        uint16_t k1W = bootTextWidth(k1, scaleText);
        uint16_t k2W = bootTextWidth(k2, scaleText);
        uint16_t domX = (dW < availW) ? (uint16_t)((availW - dW) / 2) : pad;
        uint16_t nameX = (nW < availW) ? (uint16_t)((availW - nW) / 2) : pad;
        uint16_t fwX = (fW < availW) ? (uint16_t)((availW - fW) / 2) : pad;
        uint16_t k1X = (k1W < availW) ? (uint16_t)((availW - k1W) / 2) : pad;
        uint16_t k2X = (k2W < availW) ? (uint16_t)((availW - k2W) / 2) : pad;
        drawBootTextRow(row, y, domX, textY, domainLine, scaleText, w, pitch, bitsPerPixel, colorScheme);
        drawBootTextRow(row, y, nameX, (uint16_t)(textY + scaleText * 10), nameLine, scaleText, w, pitch, bitsPerPixel, colorScheme);
        drawBootTextRow(row, y, fwX, (uint16_t)(textY + scaleText * 20), fwLine, scaleText, w, pitch, bitsPerPixel, colorScheme);
        drawBootTextRow(row, y, k1X, (uint16_t)(textY + scaleText * 30), k1, scaleText, w, pitch, bitsPerPixel, colorScheme);
        drawBootTextRow(row, y, k2X, (uint16_t)(textY + scaleText * 40), k2, scaleText, w, pitch, bitsPerPixel, colorScheme);

        if (y >= qrY && y < (uint16_t)(qrY + qrPx)) {
            uint16_t localY = (uint16_t)(y - qrY);
            uint16_t my = (uint16_t)(localY / modulePx);
            if (my < qrModules) {
                int16_t qy = (int16_t)my - quiet;
                for (uint16_t mx = 0; mx < qrModules; mx++) {
                    int16_t qx = (int16_t)mx - quiet;
                    bool on = false;
                    if (qx >= 0 && qy >= 0 && qx < qrSize && qy < qrSize) on = qrcode_getModule(&qr, (uint8_t)qx, (uint8_t)qy);
                    if (!on) continue;
                    uint16_t px0 = (uint16_t)(qrX + mx * modulePx);
                    for (uint16_t px = px0; px < (uint16_t)(px0 + modulePx) && px < w; px++) {
                        setBootPixelBlack(row, px, pitch, bitsPerPixel, colorScheme);
                    }
                }
            }
        }
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        if (seeed_driver_used()) {
            seeed_gfx_boot_write_row(y, row, pitch);
        } else {
            bbepWriteData(&bbep, row, pitch);
        }
#else
        bbepWriteData(&bbep, row, pitch);
#endif
    }

#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        seeed_gfx_boot_skip_planes();
    } else
#endif
    {
        if (useBitplanes) {
            memset(row, 0x00, pitch);
            bbepSetAddrWindow(&bbep, 0, 0, w, h);
            bbepStartWrite(&bbep, PLANE_1);
            for (uint16_t y = 0; y < h; y++) bbepWriteData(&bbep, row, pitch);
        }
        if (colorScheme == 5) {
            int otherPlane = (getplane() == PLANE_0) ? PLANE_1 : PLANE_0;
            int otherPlanePitch = (w + 7) / 8;
            memset(row, 0x00, otherPlanePitch);
            bbepSetAddrWindow(&bbep, 0, 0, w, h);
            bbepStartWrite(&bbep, otherPlane);
            for (uint16_t y = 0; y < h; y++) bbepWriteData(&bbep, row, otherPlanePitch);
        }
    }
    writeSerial("Boot screen with QR rendered", true);
    return true;
}
