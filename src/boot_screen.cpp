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

static void drawBootTextRow(uint8_t* row, uint16_t y, uint16_t x0, uint16_t y0, const char* s, uint8_t scale, uint16_t displayW, int pitch, int bitsPerPixel, uint8_t colorScheme, int maxX) {
    if (!s || scale == 0) return;
    uint16_t clipW = displayW;
    if (maxX >= 0 && (uint16_t)maxX < clipW) clipW = (uint16_t)maxX;
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
                        if (y == (uint16_t)(py + sy) && (uint16_t)(px + sx) < clipW) {
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

static int bootLineStep(int scale) {
    return scale * 10;
}

static int bootBlockH(int scale) {
    return 4 * bootLineStep(scale) + 7 * scale;
}

static uint16_t bootMaxTextWidth(const char* const* lines, unsigned n, int scale) {
    uint16_t maxW = 0;
    for (unsigned i = 0; i < n; i++) {
        uint16_t tw = bootTextWidth(lines[i], (uint8_t)scale);
        if (tw > maxW) maxW = tw;
    }
    return maxW;
}

static bool bootLayoutFit(uint16_t w, uint16_t h, int scale, int pad, int qrModules, int* modulePxOut,
                          int* qrPxOut, bool* qrRightOut, int* qrXOut, int* qrYOut, int* availWOut,
                          int* textYOut, uint16_t maxTextW) {
    int blockH = bootBlockH(scale);
    int textGap = pad;
    int sideGap = pad;
    int modulePx;
    int qrPx;
    int minDim = (int)((w < h) ? w : h);
    int moduleIdeal = (minDim - pad * 2) / qrModules;
    if (moduleIdeal < 1) moduleIdeal = 1;
    if (moduleIdeal > 6) moduleIdeal = 6;

    for (modulePx = moduleIdeal; modulePx >= 1; modulePx--) {
        qrPx = modulePx * qrModules;
        if (qrPx > (int)w - pad * 2) continue;
        if ((int)w >= pad * 2 + (int)maxTextW + sideGap + qrPx
            && (int)h >= pad * 2 + (blockH > qrPx ? blockH : qrPx)) {
            int availW = (int)w - pad * 2 - qrPx - sideGap;
            if ((int)maxTextW <= availW) {
                int qrX = (int)w - pad - qrPx;
                int qrY = pad;
                int textY = pad;
                if (blockH < qrPx) textY = pad + (qrPx - blockH) / 2;
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
        if ((int)w >= pad * 2 + (int)maxTextW && (int)h >= pad * 2 + blockH + textGap + qrPx) {
            *modulePxOut = modulePx;
            *qrPxOut = qrPx;
            *qrRightOut = false;
            *qrXOut = ((int)w - qrPx) / 2;
            *qrYOut = (int)h - pad - qrPx;
            *availWOut = (int)w - pad * 2;
            *textYOut = pad;
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

bool writeBootScreenWithQr() {
    const uint16_t w = globalConfig.displays[0].pixel_width;
    const uint16_t h = globalConfig.displays[0].pixel_height;
    const uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    const bool useBitplanes = (colorScheme == COLOR_SCHEME_BWR || colorScheme == COLOR_SCHEME_BWY);
    const int bitsPerPixel = getBitsPerPixel();
    int pitch;
    uint8_t whiteValue;
    if (bitsPerPixel == 4) {
        pitch = w / 2;
        whiteValue = 0x11; // for 4bpp spectra 6 panel, white is 0x11.  It MAY be 0xFF for other panels, so in production need to VERIFY THIS!!!
    } else if (bitsPerPixel == 2) {
        pitch = (w + 3) / 4;
        whiteValue = (colorScheme == COLOR_SCHEME_GRAY4) ? 0xFF : 0x55;
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

    char nameLine[16];
    snprintf(nameLine, sizeof(nameLine), "OD%s", last6.c_str());
    char fwLine[16];
    snprintf(fwLine, sizeof(fwLine), "FW:O %u.%u", (unsigned)getFirmwareMajor(), (unsigned)getFirmwareMinor());
    const char* domainLine = "OPENDISPLAY.ORG";
    char keyHex[33];
    bytesToHex(&payload[5], 16, keyHex, sizeof(keyHex));
    char k1[17], k2[17];
    memcpy(k1, keyHex, 16); k1[16] = '\0';
    memcpy(k2, keyHex + 16, 16); k2[16] = '\0';

    int scaleText;
    int pad;
    int modulePx;
    int qrPx;
    bool qrRight;
    int qrX, qrY, availW, textY;
    {
        static const char* bootLines[] = {
            domainLine, nameLine, fwLine, k1, k2,
        };
        uint16_t maxTextW;
        bool layoutOk = false;
        int tryScale;

        for (tryScale = (w >= 400 && h >= 300) ? 2 : 1; tryScale >= 1 && !layoutOk; tryScale--) {
            scaleText = tryScale;
            pad = 6 * scaleText;
            maxTextW = bootMaxTextWidth(bootLines, 5, scaleText);
            layoutOk = bootLayoutFit(w, h, scaleText, pad, (int)qrModules, &modulePx, &qrPx, &qrRight, &qrX,
                                     &qrY, &availW, &textY, maxTextW);
        }
        if (!layoutOk) {
            scaleText = 1;
            pad = 6;
            modulePx = 1;
            qrPx = modulePx * (int)qrModules;
            qrRight = false;
            qrX = ((int)w - qrPx) / 2;
            qrY = (int)h - pad - qrPx;
            if (qrY < pad) qrY = pad;
            availW = (int)w - pad * 2;
            textY = pad;
        }
    }

    uint16_t dW = bootTextWidth(domainLine, (uint8_t)scaleText);
    uint16_t nW = bootTextWidth(nameLine, (uint8_t)scaleText);
    uint16_t fW = bootTextWidth(fwLine, (uint8_t)scaleText);
    uint16_t k1W = bootTextWidth(k1, (uint8_t)scaleText);
    uint16_t k2W = bootTextWidth(k2, (uint8_t)scaleText);
    int textOriginX = qrRight ? pad : ((int)w - availW) / 2;
    if (textOriginX < pad) textOriginX = pad;
    int textMaxX = qrRight ? (qrX - pad) : (int)w;
    int domX = textOriginX + ((availW - (int)dW) / 2);
    int nameX = textOriginX + ((availW - (int)nW) / 2);
    int fwX = textOriginX + ((availW - (int)fW) / 2);
    int k1X = textOriginX + ((availW - (int)k1W) / 2);
    int k2X = textOriginX + ((availW - (int)k2W) / 2);
    if (domX < pad) domX = pad;
    if (nameX < pad) nameX = pad;
    if (fwX < pad) fwX = pad;
    if (k1X < pad) k1X = pad;
    if (k2X < pad) k2X = pad;

    uint8_t* row = staticRowBuffer;
    // bb_epaper 4-gray (scheme 5) needs the packed 2bpp image split into two
    // 1-bit controller planes, so render the frame once per plane and
    // de-interleave. Every other scheme writes a single packed plane in one pass.
    const bool gray4Split = (colorScheme == 5)
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        && !seeed_driver_used()
#endif
        ;
    const int planePitch = (w + 7) / 8;
    const int planePasses = gray4Split ? 2 : 1;
    for (int pass = 0; pass < planePasses; pass++) {
        const int bitSel = pass;  // pass 0 -> LSB/PLANE_0, pass 1 -> MSB/PLANE_1
        const int targetPlane = gray4Split ? (pass == 0 ? PLANE_0 : PLANE_1)
                                           : (useBitplanes ? PLANE_0 : getplane());
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        if (!seeed_driver_used()) {
            bbepSetAddrWindow(&bbep, 0, 0, w, h);
            bbepStartWrite(&bbep, targetPlane);
        }
#else
        bbepSetAddrWindow(&bbep, 0, 0, w, h);
        bbepStartWrite(&bbep, targetPlane);
#endif
        for (uint16_t y = 0; y < h; y++) {
            memset(row, whiteValue, pitch);
            drawBootTextRow(row, y, (uint16_t)domX, (uint16_t)textY, domainLine, (uint8_t)scaleText, w, pitch, bitsPerPixel, colorScheme, textMaxX);
            drawBootTextRow(row, y, (uint16_t)nameX, (uint16_t)(textY + bootLineStep(scaleText)), nameLine, (uint8_t)scaleText, w, pitch, bitsPerPixel, colorScheme, textMaxX);
            drawBootTextRow(row, y, (uint16_t)fwX, (uint16_t)(textY + bootLineStep(scaleText) * 2), fwLine, (uint8_t)scaleText, w, pitch, bitsPerPixel, colorScheme, textMaxX);
            drawBootTextRow(row, y, (uint16_t)k1X, (uint16_t)(textY + bootLineStep(scaleText) * 3), k1, (uint8_t)scaleText, w, pitch, bitsPerPixel, colorScheme, textMaxX);
            drawBootTextRow(row, y, (uint16_t)k2X, (uint16_t)(textY + bootLineStep(scaleText) * 4), k2, (uint8_t)scaleText, w, pitch, bitsPerPixel, colorScheme, textMaxX);

            if (y >= (uint16_t)qrY && y < (uint16_t)(qrY + qrPx)) {
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
        if (useBitplanes) {
            memset(row, 0x00, pitch);
            bbepSetAddrWindow(&bbep, 0, 0, w, h);
            bbepStartWrite(&bbep, PLANE_1);
            for (uint16_t y = 0; y < h; y++) bbepWriteData(&bbep, row, pitch);
        }
    }
    writeSerial("Boot screen with QR rendered", true);
    return true;
}
