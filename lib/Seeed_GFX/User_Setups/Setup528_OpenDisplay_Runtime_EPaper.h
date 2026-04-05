// OpenDisplay: ED103TC2 + IT8951 + TCON. SPI/control pins come from device config at runtime
// (see display_seeed_gfx.cpp). Values here satisfy TFT_eSPI #if checks only.

#include <Wire.h>

#define USER_SETUP_ID 528

#define DISABLE_ALL_LIBRARY_WARNINGS

#define TCON_ENABLE
#define ED103TC2_DRIVER
#define EPAPER_ENABLE

#define TFT_WIDTH 1872
#define TFT_HEIGHT 1404

#define EPD_WIDTH TFT_WIDTH
#define EPD_HEIGHT TFT_HEIGHT

#define TFT_SCLK 7
#define TFT_MISO 8
#define TFT_MOSI 9
#define TFT_CS 10
#define TFT_DC (-1)
#define TFT_BUSY 13
#define TFT_RST 12
#define TFT_ENABLE 11
#define ITE_ENABLE 21

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define USE_HSPI_PORT
#endif

#include "XIAO_SPI_Frequency.h"
