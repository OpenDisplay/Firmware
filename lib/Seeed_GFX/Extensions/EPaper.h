#include "TFT_eSPI.h"

class EPaper : public TFT_eSprite
{
public:
    explicit EPaper();

    void begin(uint8_t wake = 0);
    void drawBufferPixel(int32_t x, int32_t y, uint32_t color, uint8_t bpp);
    void update();
    void update(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *data);
    void updataPartial(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    
#ifdef  USE_MUTIGRAY_EPAPER
    void initGrayMode(uint8_t grayLevel);
    void deinitGrayMode();
#endif
    void sleep();
    void wake();
    
    typedef float (*GetTempCallback)();
    typedef float (*GetHumiCallback)();
    void  setTemp(GetTempCallback callback);
    float getTemp();
    void  setHumi(GetHumiCallback callback);
    float getHumi();

    
private:
    uint8_t _grayLevel;
    bool _sleep;
    bool _entemp;
    float _temp;
    float _humi;

    typedef struct 
    {
       uint16_t x1;
       uint16_t x2;
       uint16_t y1;
       uint16_t y2;
    } freshArea_t;
    freshArea_t _freshArea;
};

