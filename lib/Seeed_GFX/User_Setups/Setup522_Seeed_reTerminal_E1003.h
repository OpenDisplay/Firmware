#include <Wire.h>

#define USER_SETUP_ID 522

#define ED103TC2_DRIVER


#define EPAPER_ENABLE

#define TFT_WIDTH 1872
#define TFT_HEIGHT 1404

#define EPD_WIDTH TFT_WIDTH
#define EPD_HEIGHT TFT_HEIGHT

// #define EPD_HORIZONTAL_MIRROR


#define TFT_SCLK 7
#define TFT_MISO 8
#define TFT_MOSI 9
#define TFT_CS 10  
#define TFT_DC -1  
#define TFT_BUSY 13 
#define TFT_RST 12 
#define TFT_ENABLE 11 
#define ITE_ENABLE 21 



#define LOAD_GLCD  // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2 // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4 // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6 // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7 // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
#define LOAD_FONT8 // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
// #define LOAD_FONT8N // Font 8. Alternative to Font 8 above, slightly narrower, so 3 digits fit a 160 pixel TFT
#define LOAD_GFXFF // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

#define SMOOTH_FONT

#ifdef CONFIG_IDF_TARGET_ESP32S3
#define USE_HSPI_PORT
#endif
#include "XIAO_SPI_Frequency.h"