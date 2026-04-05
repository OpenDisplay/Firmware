
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

#define EPD_COLOR_DEPTH 1

#define USE_PARTIAL_EPAPER
#define USE_MUTIGRAY_EPAPER
#define GRAY_LEVEL4 4

#define EPD_NOP 0xFF
#define EPD_PNLSET 0x00
#define EPD_DISPON 0x04
#define EPD_DISPOFF 0x03
#define EPD_SLPIN 0x07
#define EPD_SLPOUT 0xFF
#define EPD_PTLIN 0x91  // Partial display in
#define EPD_PTLOUT 0x92 // Partial display out
#define EPD_PTLW 0x90

#define TFT_SWRST 0xFF
#define TFT_CASET 0xFF
#define TFT_PASET 0xFF
#define TFT_RAMWR 0xFF
#define TFT_RAMRD 0xFF
#define TFT_INVON EPD_DISPON
#define TFT_INVOFF EPD_DISPOFF

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
        delay(1);           \
        CHECK_BUSY();       \
    } while (0)

#define EPD_UPDATE_GRAY EPD_UPDATE 
#define EPD_UPDATE_PARTIAL EPD_UPDATE 

#define EPD_SLEEP()         \
    do                      \
    {                       \
        writecommand(0X50); \
        writedata(0xf7);    \
        writecommand(0x02); \
        CHECK_BUSY();       \
        writecommand(0x07); \
        writedata(0xA5);    \
    } while (0)

#define EPD_INIT_FULL()          \
    do                           \
    {                            \
        writecommand(0x01);      \
        writedata(0x07);         \
        writedata(0x07);         \
        writedata(0x3f);         \
        writedata(0x3f);         \
        writecommand(0x06);      \
        writedata(0x17);         \
        writedata(0x17);         \
        writedata(0x28);         \
        writedata(0x17);         \
        writecommand(0x04);      \
        delay(100);              \
        CHECK_BUSY();            \
        writecommand(0x00);      \
        writedata(0x1F);         \
        writecommand(0x61);      \
        writedata(EPD_WIDTH >> 8); \
        writedata(EPD_WIDTH & 0xFF); \
        writedata(EPD_HEIGHT >> 8); \
        writedata(EPD_HEIGHT & 0xFF); \
        writecommand(0x50);      \
        writedata(0x10);         \
        writedata(0x07);         \
    } while (0)

#define EPD_INIT_FAST()     \
    do                      \
    {                       \
        writecommand(0x01);      \
        writedata(0x07);         \
        writedata(0x07);         \
        writedata(0x3f);         \
        writedata(0x3f);         \
        writecommand(0x06);      \
        writedata(0x17);         \
        writedata(0x17);         \
        writedata(0x28);         \
        writedata(0x17);         \
        writecommand(0x04);      \
        delay(100);              \
        CHECK_BUSY();            \
        writecommand(0x00);      \
        writedata(0x1F);         \
        writecommand(0x61);      \
        writedata(EPD_WIDTH >> 8); \
        writedata(EPD_WIDTH & 0xFF); \
        writedata(EPD_HEIGHT >> 8); \
        writedata(EPD_HEIGHT & 0xFF); \
        writecommand(0x50);      \
        writedata(0x10);         \
        writedata(0x07);         \
        writecommand(0xE0); \
        writedata(0x02);    \
        writecommand(0xE5); \
        writedata(0x55);    \
    } while (0)

#define EPD_INIT_GRAY()     \
    do                      \
    {                       \
        digitalWrite(TFT_RST, LOW);  \
        delay(10);                   \
        digitalWrite(TFT_RST, HIGH); \
        delay(10);                   \
        CHECK_BUSY();                \
        writecommand(0X00); \
        writedata(0x1F);    \
        writecommand(0X50); \
        writedata(0x10);    \
        writedata(0x07);    \
        writecommand(0x04); \
        delay(10);         \
        CHECK_BUSY();       \
        writecommand(0x06); \
        writedata(0x27);    \
        writedata(0x27);    \
        writedata(0x28);    \
        writedata(0x17);    \
        writecommand(0xE0); \
        writedata(0x02);    \
        writecommand(0xE5); \
        writedata(0x5F);    \
    } while (0)
    
#define EPD_INIT_PARTIAL()           \
    do                               \
    {                                \
        writecommand(0x00);          \
        writedata(0x1F);             \
        writecommand(0x04);          \
        delay(100);                  \
        CHECK_BUSY();                \
        writecommand(0xE0);          \
        writedata(0x02);             \
        writecommand(0xE5);          \
        writedata(0x6E);             \
    } while (0)

#define EPD_WAKEUP()                 \
    do                               \
    {                                \
        digitalWrite(TFT_RST, LOW);  \
        delay(10);                   \
        digitalWrite(TFT_RST, HIGH); \
        delay(10);                   \
        CHECK_BUSY();                \
        EPD_INIT_FAST();             \
    } while (0)

#define EPD_WAKEUP_GRAY EPD_WAKEUP

#define EPD_WAKEUP_PARTIAL()        \
    do                              \
    {                               \
        digitalWrite(TFT_RST, LOW);  \
        delay(10);                   \
        digitalWrite(TFT_RST, HIGH); \
        delay(10);                   \
        CHECK_BUSY();                \
        EPD_INIT_PARTIAL();          \
    } while (0);
    

#define EPD_SET_WINDOW(x1, y1, x2, y2)                  \
    do                                                  \
    {                                                   \
        writecommand(0x50);     \
        writedata(0xA9);        \
        writedata(0x07);        \
        writecommand(0x91);         \
        writecommand(0x90);         \
        writedata (x1 >> 8);     \
        writedata (x1& 0xFF);             \
        writedata (x2 >> 8);         \
        writedata ((x2& 0xFF)-1);       \
        writedata (y1 >> 8);         \
        writedata (y1& 0xFF);             \
        writedata (y2 >> 8);         \
        writedata ((y2& 0xFF)-1);       \
        writedata (0x01);       \
        } while (0)

#define EPD_PUSH_NEW_COLORS(w, h, colors)   \
    do                                      \
    {                                       \
        writecommand(0x13);                 \
        for (int i = 0; i < w * h / 8; i++) \
        {                                   \
            writedata(colors[i]);           \
        }                                   \
    } while (0)

#define EPD_PUSH_NEW_COLORS_FLIP(w, h, colors)                         \
    do                                                                 \
    {                                                                  \
        writecommand(0x13);                                            \
        uint16_t bytes_per_row = (w) / 8;                              \
        for (uint16_t row = 0; row < (h); row++)                       \
        {                                                              \
            uint16_t start = row * bytes_per_row;                      \
            for (uint16_t col = 0; col < bytes_per_row; col++)         \
            {                                                          \
                uint8_t b = colors[start + (bytes_per_row - 1 - col)]; \
                b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);             \
                b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);             \
                b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);             \
                writedata(b);                                          \
            }                                                          \
        }                                                              \
    } while (0)

#define EPD_PUSH_NEW_GRAY_COLORS(w, h, colors)                       \
    do                                                                  \
    {                                                                   \
        EPD_INIT_GRAY();                                                \
        uint16_t i, j, k;                                               \
        uint8_t temp1, temp2, temp3;                                    \
        writecommand(0x10);                                             \
        for(i = 0; i < 48000; i++)                                      \
        {                                                               \
            /* Read 4 input bytes = 8 pixels */                         \
            uint8_t c0 = colors[i * 4 + 0];                             \
            uint8_t c1 = colors[i * 4 + 1];                             \
            uint8_t c2 = colors[i * 4 + 2];                             \
            uint8_t c3 = colors[i * 4 + 3];                             \
            /* Extract 8 pixels from bit5-4 and bit1-0 of each byte */  \
            uint8_t p0 = (c0 >> 4) & 0x03;                              \
            uint8_t p1 = (c0 >> 0) & 0x03;                              \
            uint8_t p2 = (c1 >> 4) & 0x03;                              \
            uint8_t p3 = (c1 >> 0) & 0x03;                              \
            uint8_t p4 = (c2 >> 4) & 0x03;                              \
            uint8_t p5 = (c2 >> 0) & 0x03;                              \
            uint8_t p6 = (c3 >> 4) & 0x03;                              \
            uint8_t p7 = (c3 >> 0) & 0x03;                              \
            /* Pack into original-style packed bytes (high 2-bit per pixel) */ \
            uint8_t packed_byte0 = (p0 << 6) | (p1 << 4) | (p2 << 2) | p3; \
            uint8_t packed_byte1 = (p4 << 6) | (p5 << 4) | (p6 << 2) | p7; \
                                                                        \
            temp3 = 0;                                                  \
            for(j = 0; j < 2; j++)                                      \
            {                                                           \
                temp1 = (j == 0) ? packed_byte0 : packed_byte1;         \
                for(k = 0; k < 4; k++)                                  \
                {                                                       \
                    temp2 = temp1 & 0xC0;                               \
                    if(temp2 == 0xC0)                                   \
                        temp3 |= 0x01;                                  \
                    else if(temp2 == 0x00)                              \
                        temp3 |= 0x00;                                  \
                    else if((temp2 >= 0x80) && (temp2 < 0xC0))          \
                        temp3 |= 0x00;                                  \
                    else if(temp2 == 0x40)                              \
                        temp3 |= 0x01;                                  \
                    if((j == 0 && k <= 3) || (j == 1 && k <= 2))        \
                    {                                                   \
                        temp3 <<= 1;                                    \
                        temp1 <<= 2;                                    \
                    }                                                   \
                }                                                       \
            }                                                           \
            writedata(~temp3);                                          \
        }                                                               \
                                                                        \
        writecommand(0x13);                                             \
        for(i = 0; i < 48000; i++)                                      \
        {                                                               \
            uint8_t c0 = colors[i * 4 + 0];                             \
            uint8_t c1 = colors[i * 4 + 1];                             \
            uint8_t c2 = colors[i * 4 + 2];                             \
            uint8_t c3 = colors[i * 4 + 3];                             \
            uint8_t p0 = (c0 >> 4) & 0x03;                              \
            uint8_t p1 = (c0 >> 0) & 0x03;                              \
            uint8_t p2 = (c1 >> 4) & 0x03;                              \
            uint8_t p3 = (c1 >> 0) & 0x03;                              \
            uint8_t p4 = (c2 >> 4) & 0x03;                              \
            uint8_t p5 = (c2 >> 0) & 0x03;                              \
            uint8_t p6 = (c3 >> 4) & 0x03;                              \
            uint8_t p7 = (c3 >> 0) & 0x03;                              \
            uint8_t packed_byte0 = (p0 << 6) | (p1 << 4) | (p2 << 2) | p3; \
            uint8_t packed_byte1 = (p4 << 6) | (p5 << 4) | (p6 << 2) | p7; \
                                                                        \
            temp3 = 0;                                                  \
            for(j = 0; j < 2; j++)                                      \
            {                                                           \
                temp1 = (j == 0) ? packed_byte0 : packed_byte1;         \
                for(k = 0; k < 4; k++)                                  \
                {                                                       \
                    temp2 = temp1 & 0xC0;                               \
                    if(temp2 == 0xC0)                                   \
                        temp3 |= 0x01;                                  \
                    else if(temp2 == 0x00)                              \
                        temp3 |= 0x00;                                  \
                    else if((temp2 >= 0x80) && (temp2 < 0xC0))          \
                        temp3 |= 0x01;                                  \
                    else if(temp2 == 0x40)                              \
                        temp3 |= 0x00;                                  \
                    if((j == 0 && k <= 3) || (j == 1 && k <= 2))        \
                    {                                                   \
                        temp3 <<= 1;                                    \
                        temp1 <<= 2;                                    \
                    }                                                   \
                }                                                       \
            }                                                           \
            writedata(~temp3);                                          \
        }                                                               \
    } while (0)

#define EPD_PUSH_NEW_GRAY_COLORS_FLIP(w, h, colors)                    \
    do                                                                 \
    {                                                                  \
        EPD_INIT_GRAY();                                               \
        writecommand(0x13);                                            \
        uint16_t bytes_per_row = (w) / 8;                              \
        for (uint16_t row = 0; row < (h); row++)                       \
        {                                                              \
            uint16_t start = row * bytes_per_row;                      \
            for (uint16_t col = 0; col < bytes_per_row; col++)         \
            {                                                          \
                uint8_t b = colors[start + (bytes_per_row - 1 - col)]; \
                b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);             \
                b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);             \
                b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);             \
                writedata(b);                                          \
            }                                                          \
        }                                                              \
    } while (0)

#define EPD_PUSH_OLD_COLORS(w, h, colors)   \
    do                                      \
    {                                       \
        writecommand(0x10);                 \
        for (int i = 0; i < w * h / 8; i++) \
        {                                   \
            writedata(colors[i]);           \
        }                                   \
    } while (0)

#define EPD_PUSH_OLD_COLORS_FLIP(w, h, colors)                         \
    do                                                                 \
    {                                                                  \
        writecommand(0x10);                                            \
        uint16_t bytes_per_row = (w) / 8;                              \
        for (uint16_t row = 0; row < (h); row++)                       \
        {                                                              \
            uint16_t start = row * bytes_per_row;                      \
            for (uint16_t col = 0; col < bytes_per_row; col++)         \
            {                                                          \
                uint8_t b = colors[start + (bytes_per_row - 1 - col)]; \
                b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);             \
                b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);             \
                b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);             \
                writedata(b);                                          \
            }                                                          \
        }                                                              \
    } while (0)

#define EPD_SET_TEMP(temp)                  \
    do                                      \
    {                                       \
    } while (0)    
