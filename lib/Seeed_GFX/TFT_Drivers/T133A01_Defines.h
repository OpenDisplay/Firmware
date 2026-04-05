
#ifndef EPD_WIDTH
#define EPD_WIDTH 1600
#endif

#ifndef EPD_HEIGHT
#define EPD_HEIGHT 1200
#endif

#ifndef TFT_WIDTH
#define TFT_WIDTH EPD_WIDTH
#endif

#ifndef TFT_HEIGHT
#define TFT_HEIGHT EPD_HEIGHT
#endif



#define EPD_COLOR_DEPTH 4

#define USE_COLORFULL_EPAPER

#define MASTER_0         0x00
#define SLAVE_1          0x01
#define MAS_SLA          0x10

#define R00_PSR             0x00
#define R01_PWR             0x01
#define R02_POF             0x02
#define R04_PON             0x04
#define R05_BTST_N          0x05
#define R06_BTST_P          0x06
#define R10_DTM             0x10
#define R12_DRF             0x12
#define R20_LUT0            0x20
#define R30_PLL             0x30
#define R40_TSC             0x40
#define R50_CDI             0x50
#define R61_TRES            0x61
#define R80_AMV             0x80
#define R81_VV              0x81
#define R82_VDCS            0x82
#define R90_PGM             0x90
#define R91_APG             0x91
#define R92_ROTP            0x92
#define RE0_CCSET           0xE0
#define RE3_PWS             0xE3
#define RE5_TSSET           0xE5
#define RA5_DCDC            0xA5

#define TFT_SWRST 0xFF
#define TFT_CASET 0xFF
#define TFT_PASET 0xFF
#define TFT_RAMWR 0xFF
#define TFT_RAMRD 0xFF
#define TFT_INVON R04_PON
#define TFT_INVOFF R02_POF

const unsigned char PSR_V[2] = {
	0xDF, 0x69
};
const unsigned char PWR_V[6] = {
	0x0F, 0x00, 0x28, 0x2C, 0x28, 0x38
};
const unsigned char POF_V[1] = {
	0x00
};
const unsigned char DRF_V[1] = {
	0x01
};
const unsigned char CDI_V[1] = {
	0x37
};

const unsigned char TRES_V[4] = {
	0x04, 0xB0, 0x03, 0x20
};
const unsigned char AMV_V[2] = {
	0x01, 0x00
};

const unsigned char CCSET_V_CUR[1] = {
	0x01
};

const unsigned char CCSET_V_LOCK[1] = {
	0x03
};

const unsigned char PWS_V[1] = {
	0x22
};

const unsigned char DCDC_V[3] = {
	0x44, 0x54, 0x00
};

const unsigned char BTST_P_V[2] = {
	0xE0, 0x20
};
const unsigned char BTST_N_V[2] = {
	0xE0, 0x20
};
const unsigned char Sleep_V[1] = {
	0xa5
};

const unsigned char r74DataBuf[9]={
    0x00, 0x0C, 0x0C, 0xD9, 0xDD, 0xDD, 0x15, 0x15, 0x55
};
const unsigned char rf0DataBuf[6]={
    0x49, 0x55, 0x13, 0x5D, 0x05, 0x10
};
const unsigned char r60DataBuf[2]={
    0x03, 0x03
};
const unsigned char r86DataBuf[1]={
    0x10
};
const unsigned char rb6DataBuf[1]={
    0x07
};
const unsigned char rb7DataBuf[1]={
    0x01
};
const unsigned char rb0DataBuf[1]={
    0x01
};
const unsigned char rb1DataBuf[1]={
    0x02
};

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
        digitalWrite(TFT_CS1, LOW);     \
        writecommand(R04_PON);          \
        CHECK_BUSY();       \
        digitalWrite(TFT_CS1, HIGH);  delay(30);                    \
        digitalWrite(TFT_CS1, LOW);                            \
        writecommanddata(R12_DRF,DRF_V,sizeof(DRF_V));          \
        CHECK_BUSY();       \
        digitalWrite(TFT_CS1, HIGH); delay(30);                    \
        digitalWrite(TFT_CS1, LOW);                            \
        writecommanddata(R02_POF,POF_V,sizeof(POF_V));          \
        CHECK_BUSY();       \
        digitalWrite(TFT_CS1, HIGH); delay(30);                    \
    } while (0)

#define EPD_SLEEP()         \
    do                      \
    {                       \
        writecommanddata(0x07,Sleep_V,sizeof(Sleep_V));          \
        delay(1);                    \
        CHECK_BUSY();                \
    } while (0)

 #define EPD_INIT()                 \
    do                              \
    {                               \
        digitalWrite(TFT_RST, LOW); \
        delay(20);                  \
        digitalWrite(TFT_RST, HIGH);\
        delay(20);                  \
        CHECK_BUSY();                \
        writecommanddata(0x74,r74DataBuf,sizeof(r74DataBuf));          \
        digitalWrite(TFT_CS1, LOW);  delay(10);                  \
        writecommanddata(0xF0,rf0DataBuf,sizeof(rf0DataBuf));          \
        digitalWrite(TFT_CS1, HIGH); delay(10);                \
        digitalWrite(TFT_CS1, LOW);                             \
        writecommanddata(R00_PSR,PSR_V,sizeof(PSR_V));          \
        digitalWrite(TFT_CS1, HIGH); delay(10);                  \
        writecommanddata(RA5_DCDC,DCDC_V,sizeof(DCDC_V));          \
        delay(10);                  \
        digitalWrite(TFT_CS1, LOW);                             \
        writecommanddata(R50_CDI,CDI_V,sizeof(CDI_V));          \
        digitalWrite(TFT_CS1, HIGH); delay(10);                  \
        digitalWrite(TFT_CS1, LOW);                             \
        writecommanddata(0x60,r60DataBuf,sizeof(r60DataBuf));          \
        digitalWrite(TFT_CS1, HIGH); delay(10);                  \
        digitalWrite(TFT_CS1, LOW);                             \
        writecommanddata(0x86,r86DataBuf,sizeof(r86DataBuf));          \
        digitalWrite(TFT_CS1, HIGH); delay(10);                  \
        digitalWrite(TFT_CS1, LOW);                             \
        writecommanddata(RE3_PWS,PWS_V,sizeof(PWS_V));          \
        digitalWrite(TFT_CS1, HIGH); delay(10);                  \
        digitalWrite(TFT_CS1, LOW);                             \
        writecommanddata(R61_TRES,TRES_V,sizeof(TRES_V));          \
        digitalWrite(TFT_CS1, HIGH); delay(10);                  \
        writecommanddata(R01_PWR,PWR_V,sizeof(PWR_V));          \
        delay(10);                  \
        writecommanddata(0xB6,rb6DataBuf,sizeof(rb6DataBuf));          \
        delay(10);                  \
        writecommanddata(R06_BTST_P,BTST_P_V,sizeof(BTST_P_V));          \
        delay(10);                  \
        writecommanddata(0xB7,rb7DataBuf,sizeof(rb7DataBuf));          \
        delay(10);                  \
        writecommanddata(R05_BTST_N,BTST_N_V,sizeof(BTST_N_V));          \
        delay(10);                  \
        writecommanddata(0xB0,rb0DataBuf,sizeof(rb0DataBuf));          \
        delay(10);                  \
        writecommanddata(0xB1,rb1DataBuf,sizeof(rb1DataBuf));          \
        delay(10);                  \
    } while (0)

#define EPD_WAKEUP()    EPD_INIT()  

#define EPD_SET_WINDOW(x1, y1, x2, y2)                  \
    do                                                  \
    {                                                   \
    } while (0)



#define COLOR_GET(color) ( \
    (color) == 0x0F ? 0x00 : \
    (color) == 0x00 ? 0x01 : \
    (color) == 0x02 ? 0x06 : \
    (color) == 0x0B ? 0x02 : \
    (color) == 0x0D ? 0x05 : \
    (color) == 0x06 ? 0x03 : \
    0x01 \
)


#define EPD_PUSH_NEW_COLORS(w, h, colors)   \
    do                                      \
    {                                       \
        digitalWrite(TFT_CS1, LOW);             \
        writecommanddata(RE0_CCSET, CCSET_V_CUR, sizeof(CCSET_V_CUR));  \
        digitalWrite(TFT_CS1, HIGH);     \
        CHECK_BUSY();       \
        delay(10);                    \
        uint16_t bytes_per_blcok_row = (w) / 4;   \
        uint8_t temp1,temp2 ;               \
        digitalWrite(TFT_CS1, HIGH);             \
        digitalWrite(TFT_CS, LOW);             \
        spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE));   \
        DC_C; \
        spi.transfer(R10_DTM);                 \
        DC_D; \
        for (uint16_t row = 0; row < (h) ; row++)        \
        {                                   \
            for(uint16_t col = 0; col < bytes_per_blcok_row ; col++)   \
            {                               \
                uint8_t b = (colors[(bytes_per_blcok_row * 2) *row+col]) ;   \
                temp1 =  (b >> 4) & 0x0F;\
                temp2 =   b & 0x0F;\
                spi.transfer(((COLOR_GET(temp1) <<4)|( COLOR_GET(temp2))));\
            }                               \
        }                                   \
        spi.endTransaction();               \
        digitalWrite(TFT_CS, HIGH);             \
        digitalWrite(TFT_CS1, LOW);             \
        spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE));   \
        DC_C; \
        spi.transfer(R10_DTM);                 \
        DC_D; \
        for (uint16_t row = 0; row < (h) ; row++)        \
        {                                   \
            for(uint16_t col = 0; col < bytes_per_blcok_row ; col++)   \
            {                               \
                uint8_t b = (colors[(bytes_per_blcok_row * 2) *row+col + bytes_per_blcok_row]) ;   \
                temp1 =  (b >> 4) & 0x0F;\
                temp2 =   b & 0x0F;\
                spi.transfer(((COLOR_GET(temp1) <<4)|( COLOR_GET(temp2))));\
            }                               \
        }                                   \
        spi.endTransaction();               \
        digitalWrite(TFT_CS1, HIGH);             \
    } while (0)

#define EPD_PUSH_NEW_COLORS_FLIP(w, h, colors)                         \
    do                                      \
    {                                       \
        digitalWrite(TFT_CS1, LOW);             \
        writecommanddata(RE0_CCSET, CCSET_V_CUR, sizeof(CCSET_V_CUR));  \
        digitalWrite(TFT_CS1, HIGH);     \
        CHECK_BUSY();       \
        delay(10);                    \
        uint16_t bytes_per_blcok_row = (w) / 4;   \
        uint8_t temp1,temp2 ;               \
        digitalWrite(TFT_CS1, HIGH);             \
        digitalWrite(TFT_CS, LOW);             \
        spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE));   \
        DC_C; \
        spi.transfer(R10_DTM);                 \
        DC_D; \
        for (uint16_t row = 0; row < (h) ; row++)        \
        {                                   \
            for(uint16_t col = 0; col < bytes_per_blcok_row ; col++)   \
            {                               \
                uint8_t b = (colors[(bytes_per_blcok_row * 2) * row + (bytes_per_blcok_row - col - 1) + bytes_per_blcok_row]) ;   \
                temp1 =  (b >> 4) & 0x0F;\
                temp2 =   b & 0x0F;\
                spi.transfer(((COLOR_GET(temp1) <<4)|( COLOR_GET(temp2))));\
            }                               \
        }                                   \
        spi.endTransaction();               \
        digitalWrite(TFT_CS, HIGH);             \
        digitalWrite(TFT_CS1, LOW);             \
        spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE));   \
        DC_C; \
        spi.transfer(R10_DTM);                 \
        DC_D; \
        for (uint16_t row = 0; row < (h) ; row++)        \
        {                                   \
            for(uint16_t col = 0; col < bytes_per_blcok_row ; col++)   \
            {                               \
                uint8_t b = (colors[(bytes_per_blcok_row * 2) * row + (bytes_per_blcok_row - col - 1)]) ;   \
                temp1 =  (b >> 4) & 0x0F;\
                temp2 =   b & 0x0F;\
                spi.transfer(((COLOR_GET(temp1) <<4)|( COLOR_GET(temp2))));\
            }                               \
        }                                   \
        spi.endTransaction();               \
        digitalWrite(TFT_CS1, HIGH);             \
    } while (0)

#define EPD_PUSH_OLD_COLORS_FILP(w, h, colors)\
    do                                      \
    {                                       \
    } while (0)
    
#define EPD_PUSH_OLD_COLORS(w, h, colors)   \
    do                                      \
    {                                       \
    } while (0)


#define EPD_SET_TEMP(temp)                  \
    do                                      \
    {                                       \
        uint8_t _temp_val = (uint8_t)(temp); \
        writecommanddata(RE5_TSSET, &_temp_val, 1); \
    } while (0)    
