EPaper::EPaper() : TFT_eSprite(this), _sleep(false), _entemp(true), _temp(16.00), _humi(50.00)
{
    setColorDepth(EPD_COLOR_DEPTH);
    createSprite(_width, _height, 1);
    //createPalette(cmap, 16);
}

void EPaper::begin(uint8_t wake)
{
    setBitmapColor(1, 0);
    setTextFont(1);
    setTextColor(TFT_BLACK, TFT_WHITE, true);
    if(wake)
    {
        initFromSleep();
    }
    else
    {
        init();
        EPD_WAKEUP();
    } 
//     fillSprite(1);
// #ifdef EPD_HORIZONTAL_MIRROR
//     EPD_PUSH_OLD_COLORS_FLIP(_width, _height, _img8);
//     fillSprite(0);
//     EPD_PUSH_NEW_COLORS_FLIP(_width, _height, _img8);
// #else
//     EPD_PUSH_OLD_COLORS(_width, _height, _img8);
//     fillSprite(0);
//     EPD_PUSH_NEW_COLORS(_width, _height, _img8);
// #endif
//     EPD_UPDATE();
    
}

 void EPaper::drawBufferPixel(int32_t x, int32_t y, uint32_t color, uint8_t bpp)
 {
    _img8[y * (_width / (8 / bpp)) + (x / (8 / bpp))] = color;
 }

void EPaper::update()
{
    wake();
    EPD_SET_WINDOW(0, 0, (_width - 1), (_height - 1));
    if(!_grayLevel)
    {
        #ifdef EPD_HORIZONTAL_MIRROR
            EPD_PUSH_OLD_COLORS_FLIP(_width, _height, _img8);
            EPD_PUSH_NEW_COLORS_FLIP(_width, _height, _img8);
        #else
            EPD_PUSH_OLD_COLORS(_width, _height, _img8);
            EPD_PUSH_NEW_COLORS(_width, _height, _img8);
        #endif
            EPD_UPDATE();
    }    
    else
    {
      #ifdef USE_MUTIGRAY_EPAPER  
        #ifdef EPD_HORIZONTAL_MIRROR
            EPD_PUSH_NEW_GRAY_COLORS_FLIP(_width, _height, _img8);
        #else
            EPD_PUSH_NEW_GRAY_COLORS(_width, _height, _img8);
        #endif
            EPD_UPDATE_GRAY();
      #endif  
    }
    sleep();
}

#ifdef USE_PARTIAL_EPAPER
void EPaper::updataPartial(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{

    int32_t bx = x;
    int32_t by = y;
    int32_t bw = w;
    int32_t bh = h;
    // Map the rotated coordinate space back to the backing buffer.
    switch (rotation & 3)
    {
    case 1:
        bx = (int32_t)_width - y - h;
        by = x;
        bw = h;
        bh = w;
        break;
    case 2:
        bx = (int32_t)_width - x - w;
        by = (int32_t)_height - y - h;
        break;
    case 3:
        bx = y;
        by = (int32_t)_height - x - w;
        bw = h;
        bh = w;
        break;
    default:
        break;
    }

    if (bx < 0) { bw += bx; bx = 0; }
    if (by < 0) { bh += by; by = 0; }
    if ((bx + bw) > (int32_t)_width)  bw = (int32_t)_width - bx;
    if ((by + bh) > (int32_t)_height) bh = (int32_t)_height - by;
    if (bw < 1 || bh < 1) return;

    uint16_t align_px = 8;
#ifdef TCON_ENABLE
    align_px = 16;
    wake();
    _sleep = false;
#else
    if (_sleep) { EPD_WAKEUP_PARTIAL(); _sleep = false; }
#endif

    uint16_t x0 = ((uint16_t)bx) & ~(align_px - 1);
    uint16_t x1 = ((uint16_t)(bx + bw + (align_px - 1))) & ~(align_px - 1);
    uint16_t w_aligned = x1 - x0;
    uint16_t yy = (uint16_t)by;
    uint16_t hh = (uint16_t)bh;

    uint16_t stride = _width >> 3;           
    uint16_t win_bytes_per_row = w_aligned >> 3;

    const uint8_t* src0 = _img8 + (yy * stride) + (x0 >> 3);

    size_t win_size = (size_t)win_bytes_per_row * hh;
    uint8_t* winbuf = (uint8_t*)malloc(win_size);
    if (!winbuf) return;


    for (uint16_t row = 0; row < hh; row++) {
        memcpy(winbuf + row * win_bytes_per_row,
               src0  + row * stride,
               win_bytes_per_row);
    }



    #ifdef EPD_HORIZONTAL_MIRROR
    uint16_t x_end = x0 + w_aligned - 1;
    uint16_t mx0 = (_width - 1) - x_end;
    uint16_t mx1 = (_width - 1) - x0;

    EPD_SET_WINDOW(mx0, yy, mx1, yy + hh - 1);
    EPD_PUSH_NEW_COLORS_FLIP(w_aligned, hh, winbuf);
    #else
    EPD_SET_WINDOW(x0, yy, x0 + w_aligned - 1, yy + hh - 1);
    EPD_PUSH_NEW_COLORS(w_aligned, hh, winbuf);
    #endif
    EPD_UPDATE_PARTIAL();

    free(winbuf);
    sleep();
}

#endif

void EPaper::update(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *data)
{
    if (_sleep)
    {
        EPD_WAKEUP();
        _sleep = false;
    }
    uint8_t *p = (uint8_t *)data;
    // only support x, y multiple of 8 (floor)
    x = (x / 8) * 8;
    y = (y / 8) * 8;


    if(_bpp == 4)
     pushImage(x, y, w, h / 2 , (uint16_t *)p);
    else
     pushImage(x, y, w, h , (uint16_t *)p);
#ifdef EPD_HORIZONTAL_MIRROR
    EPD_PUSH_NEW_COLORS_FLIP(w, h, p);
#else
    EPD_PUSH_NEW_COLORS(w, h, p);
#endif
    EPD_UPDATE();
    sleep();
}

#ifdef USE_MUTIGRAY_EPAPER
void EPaper::initGrayMode(uint8_t grayLevel)
{
    if (grayLevel != 4 && grayLevel != 16) {
        return;
    }
    if (grayLevel == _grayLevel) {
        return;
    }
    else
    {
        _grayLevel =  grayLevel;
    }
    if (_created) {
        deleteSprite();
    }
    setColorDepth(4); 
    createSprite(_width, _height, 1);
    fillSprite(grayLevel - 1); 
    setTextColor(TFT_GRAY_0, grayLevel - 1, true);
}


void EPaper::deinitGrayMode()
{
    if (_bpp == EPD_COLOR_DEPTH) return;

    if (_created) {
        deleteSprite();
        _grayLevel = 0;
    }
    setColorDepth(EPD_COLOR_DEPTH);
    createSprite(_width, _height, 1);
    fillSprite(TFT_WHITE); 
    setTextColor(TFT_BLACK, TFT_WHITE, true);
}
#endif 


void EPaper::sleep()
{
    if (_sleep)
        return;
    EPD_SLEEP();
    _sleep = true;
}

void EPaper::wake()
{
    if (!_sleep)
        return;
    if(_entemp)
        EPD_SET_TEMP(_temp);
    if(!_grayLevel)
        EPD_WAKEUP();
    else
    {
        #ifdef USE_MUTIGRAY_EPAPER  
        EPD_WAKEUP_GRAY();
        #endif
    }    
        _sleep = false;

}

void  EPaper::setTemp(GetTempCallback callback)
{
    _temp = callback();
    EPD_SET_TEMP(_temp);
}
float EPaper::getTemp()
{
    return _temp;
}
void  EPaper::setHumi(GetHumiCallback callback)
{
    _humi = callback();
}
float EPaper::getHumi()
{
    return _humi;
}
