
#ifndef EPD_WIDTH
#define EPD_WIDTH 800
#endif

#ifndef EPD_HEIGHT
#define EPD_HEIGHT 480
#endif

#ifndef TFT_WIDTH
#define TFT_WIDTH EPD_WIDTH
#endif

#ifndef TFT_HEIGHT
#define TFT_HEIGHT EPD_HEIGHT
#endif

#define EPD_COLOR_DEPTH 4

#define USE_COLORFULL_EPAPER

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
        delay(10);                 \
        if (digitalRead(TFT_BUSY)) \
            break;                 \
    } while (true)
#else
#define CHECK_BUSY()
#endif

#define EPD_UPDATE()        \
    do                      \
    {                       \
        writecommand(0x12); \
        writedata(0x00);    \
        delay(1);           \
        CHECK_BUSY();       \
    } while (0)

#define EPD_SLEEP()         \
    do                      \
    {                       \
        writecommand(0x02);          \
        writedata(0x00);             \
        delay(1);                    \
        CHECK_BUSY();                \
    } while (0)

 #define EPD_INIT()                 \
    do                              \
    {                               \
        digitalWrite(TFT_RST, LOW); \
        delay(10);                  \
        digitalWrite(TFT_RST, HIGH);\
        delay(10);                  \
        writecommand(0xAA);\
        writedata(0x49);\
        writedata(0x55);\
        writedata(0x20);\
        writedata(0x08);\
        writedata(0x09);\
        writedata(0x18);\
        writecommand(EPD_PWRR);\
        writedata(0x3F);\
        writecommand(EPD_PSR);  \
        writedata(0x5F);\
        writedata(0x69);\
        writecommand(EPD_POFS);\
        writedata(0x00);\
        writedata(0x54);\
        writedata(0x00);\
        writedata(0x44);\
        writecommand(EPD_BTST1);\
        writedata(0x40);\
        writedata(0x1F);\
        writedata(0x1F);\
        writedata(0x2C);\
        writecommand(EPD_BTST2);\
        writedata(0x6F);\
        writedata(0x1F);\
        writedata(0x17);\
        writedata(0x49);\
        writecommand(EPD_BTST3);\
        writedata(0x6F);\
        writedata(0x1F);\
        writedata(0x1F);\
        writedata(0x22);\
        writecommand(EPD_PLL);\
        writedata(0x08);\
        writecommand(EPD_CDI);\
        writedata(0x3F);\
        writecommand(EPD_TCON);\
        writedata(0x02);\
        writedata(0x00);\
        writecommand(EPD_TRES);\
        writedata(0x03);\
        writedata(0x20);\
        writedata(0x01); \
        writedata(0xE0);\
        writecommand(EPD_T_VDCS);\
        writedata(0x01);\
        writecommand(EPD_PWS);\
        writedata(0x2F);\
        writecommand(0x04);  \
        CHECK_BUSY();  \
    } while (0)

#define EPD_WAKEUP()   \
    do                  \
    {                       \
        writecommand(0x04);          \
        delay(1);                    \
        CHECK_BUSY();                \
    } while (0)

#define EPD_SET_WINDOW(x1, y1, x2, y2)                  \
    do                                                  \
    {                                                   \
    } while (0)



#define COLOR_GET(color) ( \
    (color) == 0x0F ? 0x00 : \
    (color) == 0x00 ? 0x01 : \
    (color) == 0x0D ? 0x05 : \
    (color) == 0x02 ? 0x06 : \
    (color) == 0x0B ? 0x02 : \
    (color) == 0x06 ? 0x03 : \
    0x00 \
)


#define EPD_PUSH_NEW_COLORS(w, h, colors)   \
    do                                      \
    {                                       \
        uint16_t bytes_per_row = (w) / 2;   \
        uint8_t temp1,temp2 ;               \
        writecommand(0x10);                 \
        for (uint16_t row = 0; row < (h) ; row++)        \
        {                                   \
            for(uint16_t col = 0; col < bytes_per_row; col++)   \
            {                               \
                uint8_t b = (colors[bytes_per_row *row+col]) ;   \
                temp1 =  (b >> 4) & 0x0F;\
                temp2 =   b & 0x0F;\
                writedata(((COLOR_GET(temp1) <<4)|( COLOR_GET(temp2))));\
            }                               \
        }                                   \
    } while (0)

#define EPD_PUSH_NEW_COLORS_FLIP(w, h, colors)                         \
    do                                                                 \
    {                                                                  \
        uint16_t bytes_per_row = (w) / 2;                              \
        uint8_t temp1,temp2 ;                                          \
        writecommand(0x10);                                            \
        for (uint16_t row = 0; row < (h); row++)                       \
        {                                                              \
            uint16_t start = row * bytes_per_row;                      \
            for (uint16_t col = 0; col < bytes_per_row; col++)         \
            {                                                          \
                uint8_t b = (colors[bytes_per_row *row + (bytes_per_row - 1 - col)]) ;\
                temp1 =  (b >> 4) & 0x0F;                              \
                temp2 =  b & 0x0F;                                     \
                writedata(((COLOR_GET(temp2) <<4)|( COLOR_GET(temp1))));\
            }                                                          \
        }                                                              \
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
do                                      \
{                                       \
} while (0)
    