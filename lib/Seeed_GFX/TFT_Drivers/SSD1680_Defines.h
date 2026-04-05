#ifndef __SSD1680_DEFINES_H__
#define __SSD1680_DEFINES_H__

// Define screen resolution
#ifndef EPD_WIDTH
#define EPD_WIDTH 128
#endif

#ifndef EPD_HEIGHT
#define EPD_HEIGHT 296
#endif

#ifndef TFT_WIDTH
#define TFT_WIDTH EPD_WIDTH
#endif

#ifndef TFT_HEIGHT
#define TFT_HEIGHT EPD_HEIGHT
#endif

// Define color depth (1 bit for e-ink display)
#define EPD_COLOR_DEPTH 1

#define USE_PARTIAL_EPAPER
#define USE_MUTIGRAY_EPAPER
#define GRAY_LEVEL4 4

// Define no operation command
#define EPD_NOP 0xFF

// Define common command macros
#define EPD_PNLSET 0x01  // Driver output control
#define EPD_DISPON 0xFF  // No direct display on command
#define EPD_DISPOFF 0xFF // No direct display off command
#define EPD_SLPIN 0x10   // Enter deep sleep
#define EPD_SLPOUT 0xFF  // No direct sleep out command
#define EPD_PTLIN 0x3C   // Partial display in (BorderWaveform)
#define EPD_PTLOUT 0xFF  // No direct partial display out command
#define EPD_PTLW 0x4E    // Set RAM address counter

#define TFT_SWRST 0x12  // Software reset
#define TFT_CASET 0x44  // Set RAM X address start/end position
#define TFT_PASET 0x45  // Set RAM Y address start/end position
#define TFT_RAMWR 0x24  // Write to RAM
#define TFT_RAMRD 0xFF  // No direct read RAM command
#define TFT_INVON 0xFF  // No inversion on command
#define TFT_INVOFF 0xFF // No inversion off command

#define TFT_INIT_DELAY 0

// Macro to check BUSY signal
#ifdef TFT_BUSY
#define CHECK_BUSY()                  \
    do                                \
    {                                 \
        while (digitalRead(TFT_BUSY)) \
            ;                         \
    } while (0)
#else
#define CHECK_BUSY()
#endif

// Macro to update display
#define EPD_UPDATE()        \
    do                      \
    {                       \
        writecommand(0x22); \
        writedata(0xF7);    \
        writecommand(0x20); \
        CHECK_BUSY();       \
    } while (0)

#define EPD_UPDATE_FAST()        \
    do                      \
    {                       \
        writecommand(0x22); \
        writedata(0xC7);    \
        writecommand(0x20); \
        CHECK_BUSY();       \
    } while (0)

#define EPD_UPDATE_GRAY()   EPD_UPDATE_FAST()

#define EPD_UPDATE_PARTIAL() \
    do                      \
    {                       \
        writecommand(0x22); \
        writedata(0xFF);    \
        writecommand(0x20); \
    } while (0);

// Macro to enter deep sleep
#define EPD_SLEEP()         \
    do                      \
    {                       \
        writecommand(0x10); \
        writedata(0x01);    \
        delay(100);         \
    } while (0)

 #define EPD_INIT()                         \
    do                                     \
    {                                      \
        digitalWrite(TFT_RST, LOW);        \
        delay(10);                         \
        digitalWrite(TFT_RST, HIGH);       \
        delay(10);                         \
        CHECK_BUSY();                      \
        writecommand(0x12);                \
        CHECK_BUSY();                      \
        writecommand(0x3C);                \
        writedata(0x05);                   \
        writecommand(0x01);                \
        writedata((EPD_HEIGHT - 1) % 256);  \
        writedata((EPD_HEIGHT - 1) / 256);  \
        writedata(0x00);                   \
        writecommand(0x11);                \
        writedata(0x03);                   \
        writecommand(0x44);                \
        writedata(0x00);                   \
        writedata((EPD_WIDTH / 8) - 1); \
        writecommand(0x45);                \
        writedata(0x00);                   \
        writedata(0x00);                   \
        writedata((EPD_HEIGHT - 1) % 256);  \
        writedata((EPD_HEIGHT - 1) / 256);  \
        writecommand(0x18);                \
        writedata(0x80);                   \
        writecommand(0x4E);                \
        writedata(0x00);    \
        writecommand(0x4F);                \
        writedata(0x00);  \
        writedata(0x00);  \
        CHECK_BUSY();                      \
    } while (0) 

#define EPD_INIT_PARTIAL()          \
    do                              \
    {                               \
    writecommand(0x18);             \
    writedata(0x80);                \
    writecommand(0x3C);             \
    writedata(0x80);               \
    } while (0);    

#define EPD_INIT_GRAY()         \
  do                            \
  {                             \
    writecommand(0x18); \
	writedata(0x80);	\
	writecommand(0x22); \
	writedata(0xB1);	\
    writecommand(0x20);	\
    CHECK_BUSY();  \
	writecommand(0x1A); \
	writedata(0x5A);		\
    writedata(0x00);		\
	writecommand(0x22); \
	writedata(0x91);		\
    writecommand(0x20);	\
	CHECK_BUSY();  \
  } while (0)

// Macro to wake up device
#define EPD_WAKEUP()  EPD_INIT()

#define EPD_WAKEUP_GRAY() EPD_WAKEUP()

#define EPD_WAKEUP_PARTIAL()        \
    do                              \
    {                               \
        EPD_INIT();                 \
        EPD_INIT_PARTIAL();         \
    } while (0);

// Macro to set display window
#define EPD_SET_WINDOW(x1, y1, x2, y2) \
    do                                 \
    {                                  \
            writecommand(0x44);\
            writedata((x1 + 1) / 8); \
            writedata((x2 + 1) / 8 - 1);\
            writecommand(0x45); \
            writedata(y1 & 0xFF);\
            writedata(y1 >> 8);\
            writedata((y2) & 0xFF);  \
            writedata((y2) >> 8); \
            writecommand(0x4E);   \
            writedata((x1 + 1) / 8);\
            writecommand(0x4F);   \
            writedata(y1 & 0xFF);\
            writedata(y1 >> 8);\
    } while (0)

// Macro to push new color data
#define EPD_PUSH_NEW_COLORS(w, h, colors)        \
    do                                           \
    {                                            \
        writecommand(0x24);                      \
        for (uint16_t i = 0; i < w * h / 8; i++) \
        {                                        \
            writedata(colors[i]);                \
        }                                        \
    } while (0)

#define EPD_PUSH_NEW_COLORS_FLIP(w, h, colors)                         \
    do                                                                 \
    {                                                                  \
        writecommand(0x24);                                            \
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
        writecommand(0x24);                                             \
        for(i = 0; i < (TFT_WIDTH * TFT_HEIGHT) / 8; i++)                                      \
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
        writecommand(0x26);                                             \
        for(i = 0; i < (TFT_WIDTH * TFT_HEIGHT) / 8; i++)                                      \
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
        EPD_PUSH_NEW_GRAY_COLORS(w, h, colors);                        \
    } while (0)


// Macro to push old color data
#define EPD_PUSH_OLD_COLORS(w, h, colors) \
    do                                    \
    {                                     \
        writecommand(0x26);                 \
        for (int i = 0; i < w * h / 8; i++) \
        {                                   \
            writedata(colors[i]);           \
        }                                   \
    } while (0)

#define EPD_PUSH_OLD_COLORS_FLIP(w, h, colors) \
    do                                         \
    {                                          \
        writecommand(0x26);                                            \
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
        }                                                               \
    } while (0)

#define EPD_SET_TEMP(temp)                  \
    do                                      \
    {                                       \
    } while (0)
#endif 