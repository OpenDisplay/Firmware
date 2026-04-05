
#ifndef EPD_WIDTH
#define EPD_WIDTH 1872
#endif

#ifndef EPD_HEIGHT
#define EPD_HEIGHT 1404
#endif

#ifndef TFT_WIDTH
#define TFT_WIDTH EPD_WIDTH
#endif

#ifndef TFT_HEIGHT
#define TFT_HEIGHT EPD_HEIGHT
#endif

#define EPD_COLOR_DEPTH 1

#define USE_PARTIAL_EPAPER
#define USE_MUTIGRAY_EPAPER
#define GRAY_LEVEL16 16

#define EPD_PSR         0x00
#define EPD_PWRR        0x01
#define EPD_POF         0x02
#define EPD_POFS        0x03
#define EPD_PON         0x04
#define EPD_BTST1       0x05
#define EPD_BTST2       0x06
#define EPD_DSLP        0x07
#define EPD_BTST3       0x08
#define EPD_DTM         0x10
#define EPD_DRF         0x12
#define EPD_PLL         0x30
#define EPD_CDI         0x50
#define EPD_TCON        0x60
#define EPD_TRES        0x61
#define EPD_REV         0x70
#define EPD_VDCS        0x82
#define EPD_T_VDCS      0x84
#define EPD_PWS         0xE3

#define TFT_SWRST 0xFF
#define TFT_CASET 0xFF
#define TFT_PASET 0xFF
#define TFT_RAMWR 0xFF
#define TFT_RAMRD 0xFF
#define TFT_INVON EPD_PON
#define TFT_INVOFF EPD_POF

#define TFT_INIT_DELAY 0

#ifdef TFT_BUSY
#define CHECK_BUSY()               \
    do                             \
    {                              \
        tconWaitForReady();          \
    } while (true)
#else
#define CHECK_BUSY()
#endif

#define EPD_UPDATE_PARTIAL() \
    do                      \
    {                       \
        tconDisplayArea1bpp(_imgAreaInfo.usX,_imgAreaInfo.usY, _imgAreaInfo.usWidth, _imgAreaInfo.usHeight, 0x01, 0x00, 0xff); \
    } while (0);

#define EPD_UPDATE()        \
    do                      \
    {                       \
                            \
        tconDisplayArea1bpp(_imgAreaInfo.usX,_imgAreaInfo.usY, _imgAreaInfo.usWidth, _imgAreaInfo.usHeight, 0x02, 0x00, 0xff); \
    } while (0)

#define EPD_UPDATE_GRAY()        \
    do                      \
    {                       \
                            \
        tconDisplayArea(_imgAreaInfo.usX,_imgAreaInfo.usY, _imgAreaInfo.usWidth, _imgAreaInfo.usHeight, 0x02); \
    } while (0)

#define EPD_SLEEP()         \
    do                      \
    {                       \
        tconSleep();    \
    } while (0)

#if defined(OPENDISPLAY_SEEED_GFX_RUNTIME_PINS)
#include "OpenDisplay/opendisplay_runtime_pins.h"
#define OD_EPD_RST_TOGGLE()                                    \
    do {                                                       \
        if (opnd_seeed_runtime_rst >= 0) {                     \
            digitalWrite(opnd_seeed_runtime_rst, LOW);         \
            delay(10);                                         \
            digitalWrite(opnd_seeed_runtime_rst, HIGH);        \
            delay(10);                                         \
        }                                                      \
    } while (0)
#else
#define OD_EPD_RST_TOGGLE()                                    \
    do {                                                       \
        digitalWrite(TFT_RST, LOW);                            \
        delay(10);                                             \
        digitalWrite(TFT_RST, HIGH);                           \
        delay(10);                                             \
    } while (0)
#endif

#define EPD_INIT()                 \
    do                              \
    {                               \
        OD_EPD_RST_TOGGLE();        \
        CHECK_BUSY();               \
    } while (0)

#define EPD_WAKEUP()   \
    do                  \
    {                       \
        tconWake();    \
        if(_entemp)     \
            EPD_SET_TEMP(_temp);        \
    } while (0)

#define EPD_WAKEUP_GRAY EPD_WAKEUP
#define EPD_WAKEUP_PARTIAL()   \
    do                  \
    {                       \
        tconWake();    \
    } while (0)


#define EPD_SET_WINDOW(x1, y1, x2, y2)                  \
    do                                                  \
    {                                                   \
        setTconWindowsData(x1,y1, x2, y2); \
    } while (0)

#define EPD_COLOR_INVERT()                 \
    do                              \
    {                               \
    } while (0)

#define EPD_PUSH_NEW_COLORS(w, h, colors)   \
    do                                      \
    {                                       \
        tconLoad1bppImage(colors, _imgAreaInfo.usX, _imgAreaInfo.usY , w ,h, false);                 \
    } while (0)

#define EPD_PUSH_NEW_COLORS_FLIP(w, h, colors)                         \
    do                                                                 \
    {  tconLoad1bppImage(colors, _imgAreaInfo.usX, _imgAreaInfo.usY , w ,h, true);                 \
    } while (0)

#define EPD_PUSH_NEW_GRAY_COLORS(w, h, colors)                       \
    do                                                                  \
    {                                                                   \
        tconLoadImage(colors, _imgAreaInfo.usX, _imgAreaInfo.usY , w ,h, false);                 \
    } while (0)

#define EPD_PUSH_NEW_GRAY_COLORS_FLIP(w, h, colors)                    \
    do                                                                 \
    {                                                                  \
        tconLoadImage(colors, _imgAreaInfo.usX, _imgAreaInfo.usY , w ,h, true);                 \
    } while (0)    

#define EPD_PUSH_OLD_COLORS_FILP(w, h, colors)\
    do                                      \
    {                                       \
    } while (0)
    
#define EPD_PUSH_OLD_COLORS(w, h, colors)   \
    do                                      \
    {                                       \
    } while (0)

#define EPD_SET_TEMP(temp)              \
    do                                  \
    {                                   \
        setTconTemp((uint16_t)(temp));   \
    } while (0)
