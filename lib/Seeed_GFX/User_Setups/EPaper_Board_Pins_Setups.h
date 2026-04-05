
#if defined(USE_XIAO_EPAPER_DRIVER_BOARD)
#define TFT_SCLK D8
#define TFT_MISO D9
#define TFT_MOSI D10
#define TFT_CS D1
#define TFT_DC D3
#define TFT_BUSY D2
#define TFT_RST D0
#elif defined(USE_XIAO_EPAPER_BREAKOUT_BOARD)
#define TFT_SCLK D8
#define TFT_MISO D9
#define TFT_MOSI D10
#define TFT_CS D1
#define TFT_DC D3
#define TFT_BUSY D5
#define TFT_RST D0
#elif defined(USE_XIAO_EPAPER_DISPLAY_BOARD_EE02)
#define TFT_SCLK D8
#define TFT_MISO -1
#define TFT_MOSI D10
#define TFT_CS 44  // D7
#define TFT_CS1 41  // D8
#define TFT_DC 10  // D16
#define TFT_BUSY 4 // D3
#define TFT_RST 38 // D11
#define TFT_ENABLE 43
#elif defined(USE_XIAO_EPAPER_DISPLAY_BOARD_EE03)
#define TFT_SCLK D8
#define TFT_MISO D9
#define TFT_MOSI D10
#define TFT_CS 44  // D7
#define TFT_DC -1  // D16
#define TFT_BUSY 4 // D3
#define TFT_RST 38 // D11
#define TFT_ENABLE 43
#elif defined(USE_XIAO_EPAPER_DISPLAY_BOARD_EE04)
#define TFT_SCLK D8
#define TFT_MISO -1
#define TFT_MOSI D10
#define TFT_CS 44  // D7
#define TFT_DC 10  // D16
#define TFT_BUSY 4 // D3
#define TFT_RST 38 // D11
#define TFT_ENABLE 43 //D6
#elif defined(USE_XIAO_EPAPER_DISPLAY_BOARD_EE05)
#define TFT_SCLK D8
#define TFT_MISO -1
#define TFT_MOSI D10
#define TFT_CS 44  // D7
#define TFT_DC 10  // D16
#define TFT_BUSY 4 // D3
#define TFT_RST 38 // D11
#define TFT_ENABLE 43 //D6
#elif defined(USE_XIAO_EPAPER_DISPLAY_BOARD_EN04)
#define TFT_SCLK D8
#define TFT_MISO -1
#define TFT_MOSI D10
#define TFT_CS D7  // D7
#define TFT_DC D16  // D16
#define TFT_BUSY D3 // D3
#define TFT_RST D11 // D11
#define TFT_ENABLE D6 //D6
#elif defined(USE_XIAO_EPAPER_DISPLAY_BOARD_EN05)
#define TFT_SCLK D8
#define TFT_MISO -1
#define TFT_MOSI D10
#define TFT_CS D7  // D7
#define TFT_DC D16  // D16
#define TFT_BUSY D3 // D3
#define TFT_RST D11 // D11
#define TFT_ENABLE D6 //D6
#endif