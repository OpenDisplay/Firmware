#if defined(OPENDISPLAY_SEEED_GFX_RUNTIME_PINS)
#include <Arduino.h>
#include "OpenDisplay/opendisplay_runtime_pins.h"
#endif

TWord reverse_bits_16(TWord x) {
    x = (x & 0xAAAA) >> 1 | (x & 0x5555) << 1;   
    x = (x & 0xCCCC) >> 2 | (x & 0x3333) << 2;   
    x = (x & 0xF0F0) >> 4 | (x & 0x0F0F) << 4;   
    x = (x & 0xFF00) >> 8 | (x & 0x00FF) << 8;   
    return x;
}


TByte reverse_bits_8(TByte x) {
    x = (x & 0xAA) >> 1 | (x & 0x55) << 1;   
    x = (x & 0xCC) >> 2 | (x & 0x33) << 2;   
    x = (x & 0xF0) >> 4 | (x & 0x0F) << 4;   
    return x;
}
 
void TFT_eSPI::tconWaitForReady()
{
#if defined(OPENDISPLAY_SEEED_GFX_RUNTIME_PINS)
    if (opnd_seeed_tcon_busy_timed_out) {
        return;
    }
    TByte ulData = digitalRead(opnd_seeed_runtime_busy);
    uint32_t deadline = millis() + OPND_SEEED_TCON_BUSY_TIMEOUT_MS;
    while (ulData == 0) {
        if ((int32_t)(millis() - deadline) >= 0) {
            opnd_seeed_tcon_busy_timed_out = true;
            break;
        }
        ulData = digitalRead(opnd_seeed_runtime_busy);
    }
#else
    TByte ulData = digitalRead(TFT_BUSY);
	while(ulData == 0)
	{
		ulData = digitalRead(TFT_BUSY);
	}
#endif
}



inline void TFT_eSPI::tconSendWord(TWord data)
{
    spi.transfer16(data);
}

inline TWord TFT_eSPI::tconReceiveWord()
{
    TWord rxData = 0;
    rxData =   spi.transfer16(0); 
    return rxData;   
}

void TFT_eSPI::tconWriteCmdCode(TWord usCmdCode)
{
    //Set Preamble for Write Command
	TWord wPreamble = 0x6000; 

    //begin_tft_write();
	spi_begin();  

	tconWaitForReady();

	tconSendWord(wPreamble);		
	
	tconWaitForReady();
	tconSendWord(usCmdCode);		
    
    //end_tft_write();
	spi_end();
	//delay_nop();


}

void TFT_eSPI::tconWirteData(TWord usData)
{
    TWord wPreamble	= 0x0000;
    spi_begin(); 

	tconWaitForReady();
	tconSendWord(wPreamble);		
	
	
	tconWaitForReady();
	tconSendWord(usData);	

	spi_end();
	//delay_nop();


}

void TFT_eSPI::tconWirteNData(TWord* pwBuf, TDWord ulSizeWordCnt)
{
    TWord wPreamble	= 0x0000;
    TDWord i, b = 0;
    TDWord chunk_size = 16384;  
	TWord *EPD_temp = NULL;
    spi_begin(); 
	tconWaitForReady();
	tconSendWord(wPreamble);		
	tconWaitForReady();
	//Send Data

	for (i = 0; i < (ulSizeWordCnt / chunk_size); i++)
    {


		EPD_temp = &pwBuf[b];
		if (DMA_Enabled) 
			pushPixelsDMA(EPD_temp, chunk_size);
		else
			pushPixels(EPD_temp, chunk_size);
       // writenBytes(EPD_temp, chunk_size, 32768);
        b += chunk_size;
    }

    TDWord remainder = ulSizeWordCnt % chunk_size;
    if (remainder > 0)
    {
		EPD_temp = &pwBuf[b];
		if (DMA_Enabled) 
			pushPixelsDMA(EPD_temp, remainder);
		else
			pushPixels(EPD_temp, remainder);
        //writenBytes(EPD_temp, remainder, 32768);
    }
	// for(i=0;i<ulSizeWordCnt;i++)
	// {
	// 	tconSendWord((pwBuf[i]));
	// }
	
	//delay_nop();
    spi_end();
}

TWord TFT_eSPI::tconReadData()
{
    TWord wRData; 
	spi_begin_read();
	//set type and direction
	TWord wPreamble	= 0x1000;
	
	tconWaitForReady();
	tconSendWord(wPreamble);		
	
	tconReceiveWord();//Dummy
	tconWaitForReady();
	
	//Read Data
	wRData = tconReceiveWord();

	spi_end_read();  
	return wRData;
}

void TFT_eSPI::tconReadNData(TWord* pwBuf, TDWord ulSizeWordCnt)
{
    TDWord i;
    spi_begin_read();
	//set type and direction
	TWord wPreamble	= 0x1000;
	
	//Send Preamble before reading data
	
	tconWaitForReady();
	tconSendWord(wPreamble);		

	tconWaitForReady();
	tconReceiveWord();//Dummy
	
	tconWaitForReady();
	for(i=0;i<ulSizeWordCnt;i++)
	{
		//Read Data  
		pwBuf[i] = (tconReceiveWord());
	}
	spi_end_read(); 
}

void TFT_eSPI::tconSendCmdArg(TWord usCmdCode,TWord* pArg, TWord usNumArg)
{
    TWord i;
     //Send Cmd code
     tconWriteCmdCode(usCmdCode);
     //Send Data
     for(i=0;i<usNumArg;i++)
     {
         tconWirteData(pArg[i]);
     }

}

TWord TFT_eSPI::tconReadReg(TWord usRegAddr)
{
    TWord usData;
    //----------I80 Mode-------------
    //Send Cmd and Register Address
    tconWriteCmdCode(IT8951_TCON_REG_RD);
    tconWirteData(usRegAddr);
    //Read data from Host Data bus
    usData = tconReadData();
    return usData;
}

void TFT_eSPI::tconWriteReg(TWord usRegAddr,TWord usValue)
{
    //I80 Mode
    //Send Cmd , Register Address and Write Value
    tconWriteCmdCode(IT8951_TCON_REG_WR);
    tconWirteData(usRegAddr);
    tconWirteData(usValue);

}

void TFT_eSPI::tconLoadImgStart(TCONLdImgInfo* pstLdImgInfo)
{
    TWord usArg;
    //Setting Argument for Load image start
    usArg = (pstLdImgInfo->usEndianType << 8 )
    |(pstLdImgInfo->usPixelFormat << 4)
    |(pstLdImgInfo->usRotate);
    //Send Cmd
    tconWriteCmdCode(IT8951_TCON_LD_IMG);
    //Send Arg
    tconWirteData(usArg);

}

void TFT_eSPI::tconSetImgRotation(TDWord rotation)
{
	TWord arg = {(IT8951_LDIMG_B_ENDIAN << 8)|(IT8951_8BPP << 4)| rotation};
	tconWriteCmdCode(IT8951_TCON_LD_IMG);
	tconWirteData((arg));
	
}

void TFT_eSPI::tconLoadImgAreaStart(TCONLdImgInfo* pstLdImgInfo ,TCONAreaImgInfo* pstAreaImgInfo)
{
    TWord usArg[5];
    //Setting Argument for Load image start
    usArg[0] = (pstLdImgInfo->usEndianType << 8 )
    |(pstLdImgInfo->usPixelFormat << 4)
    |(pstLdImgInfo->usRotate);

    usArg[1] = pstAreaImgInfo->usX;
    usArg[2] = pstAreaImgInfo->usY;
    usArg[3] = pstAreaImgInfo->usWidth;
    usArg[4] = pstAreaImgInfo->usHeight;
    //Send Cmd and Args
    tconSendCmdArg(IT8951_TCON_LD_IMG_AREA , usArg , 5);
}

void TFT_eSPI::tconLoadImgEnd(void)
{
    tconWriteCmdCode(IT8951_TCON_LD_IMG_END);
}

void TFT_eSPI::tconSetImgBufBaseAddr(TDWord ulImgBufAddr)
{
    TWord usWordH = (TWord)((ulImgBufAddr >> 16) & 0x0000FFFF);
	TWord usWordL = (TWord)( ulImgBufAddr & 0x0000FFFF);
	//Write LISAR Reg
	tconWriteReg(LISAR + 2 ,usWordH);
	tconWriteReg(LISAR ,usWordL);
}


void TFT_eSPI::tconHostAreaPackedPixelWrite(TCONLdImgInfo* pstLdImgInfo,TCONAreaImgInfo* pstAreaImgInfo)
{

    TDWord i,j;
	//Source buffer address of Host
	TWord* pusFrameBuf = (TWord*)pstLdImgInfo->ulStartFBAddr;
	//Set Image buffer(IT8951) Base address
	tconSetImgBufBaseAddr(pstLdImgInfo->ulImgBufBaseAddr);
	//Send Load Image start Cmd
	tconLoadImgAreaStart(pstLdImgInfo , pstAreaImgInfo);	
	//printf("---IT8951 Host Area Packed Pixel Write begin---\r\n");
	//Host Write Data


    uint16_t height = pstAreaImgInfo->usHeight;
    uint16_t width = pstAreaImgInfo->usWidth; 
    if(pstLdImgInfo->usPixelFormat == IT8951_4BPP)
            width = (width + 3) / 4;
    else if(pstLdImgInfo->usPixelFormat == IT8951_8BPP)
            width = (width + 1) / 2;

	TWord *mirroredFrameBuf = (TWord *)malloc(width * height * sizeof(TWord));
    if(mirroredFrameBuf == NULL) {
        return;
    }

    for (uint16_t j = 0; j < height; j++) {
        for (uint16_t i = 0; i < width; i++) {
			if(pstLdImgInfo->usFilp)
				mirroredFrameBuf[j * width + i] = reverse_bits_16(pusFrameBuf[j * width + i]);
			else
				mirroredFrameBuf[j * width + i] = pusFrameBuf[j * width + (width - 1 - i)];
        }
    }
	tconWirteNData(mirroredFrameBuf, height * width );
	free(mirroredFrameBuf);
	// for(j=0;j< pstAreaImgInfo->usHeight;j++)
	// {
	// 	 for(i=0;i< pstAreaImgInfo->usWidth/2;i++)
	// 		{
	// 			if(pstLdImgInfo->usFilp)
	// 				//Write a Word(2-Bytes) for each time
	// 				tconWirteData(reverse_bits_16(pusFrameBuf[j * (pstAreaImgInfo->usWidth/2) + (pstAreaImgInfo->usWidth/2 + i)]));
	// 			else
	// 				tconWirteData((pusFrameBuf[j * (pstAreaImgInfo->usWidth/2) + (pstAreaImgInfo->usWidth/2 - i - 1)]));
	// 		}
	// }
	//printf("---IT8951 Host Area Packed Pixel Write end---\r\n");
	//Send Load Img End Command
	tconLoadImgEnd();
	//memset(pusFrameBufTemp, 0xf, pstAreaImgInfo->usHeight * pstAreaImgInfo->usWidth);
}



void TFT_eSPI::tconDisplayArea(TWord usX, TWord usY, TWord usW, TWord usH, TWord usDpyMode)
{
    //Send I80 Display Command (User defined command of IT8951)
	tconWriteCmdCode(USDEF_I80_CMD_DPY_AREA); //0x0034
	//Write arguments
	tconWirteData(usX);
	tconWirteData(usY);
	tconWirteData(usW);
	tconWirteData(usH);

	tconWirteData(usDpyMode);


}

void TFT_eSPI::tconDisplayArea1bpp(TWord usX, TWord usY, TWord usW, TWord usH, TWord usDpyMode, TByte ucBGGrayVal, TByte ucFGGrayVal)
{
    usX = (_gstI80DevInfo.usPanelW - 1) - usX - usW + 1;
    //Set Display mode to 1 bpp mode - Set 0x18001138 Bit[18](0x1800113A Bit[2])to 1
    tconWriteReg(UP1SR+2, tconReadReg(UP1SR+2) | (1<<2));

    //Set BitMap color table 0 and 1 , => Set Register[0x18001250]:
    //Bit[7:0]: ForeGround Color(G0~G15)  for 1
    //Bit[15:8]:Background Color(G0~G15)  for 0
    tconWriteReg(BGVR, (ucBGGrayVal<<8) | ucFGGrayVal);
        
    //Display
    tconDisplayArea( usX, usY, usW, usH, usDpyMode);
		
    tconWaitForDisplayReady();
    
    //Restore to normal mode
    tconWriteReg(UP1SR+2, tconReadReg(UP1SR+2) & ~(1<<2));

} 



void TFT_eSPI::tconLoad1bppImage(const TByte* p1bppImgBuf, TWord usX, TWord usY, TWord usW, TWord usH, TByte enFilp)
{
    usX = (_gstI80DevInfo.usPanelW - 1) - usX - usW + 1;
	TCONLdImgInfo stLdImgInfo;
    TCONAreaImgInfo stAreaImgInfo;
	
    //Setting Load image information
    stLdImgInfo.ulStartFBAddr    = (TDWord) p1bppImgBuf;
    stLdImgInfo.usEndianType     = IT8951_LDIMG_L_ENDIAN;
    stLdImgInfo.usPixelFormat    = IT8951_8BPP; //we use 8bpp because IT8951 dose not support 1bpp mode for load image�Aso we use Load 8bpp mode ,but the transfer size needs to be reduced to Size/8
    stLdImgInfo.usRotate         = IT8951_ROTATE_0;
    stLdImgInfo.ulImgBufBaseAddr = _gulImgBufAddr;
	stLdImgInfo.usFilp           = enFilp;
    //Set Load Area
    stAreaImgInfo.usX      = (usX + 7)/8;
    stAreaImgInfo.usY      = usY;
    stAreaImgInfo.usWidth  = (usW + 7)/8;//1bpp, Chaning Transfer size setting to 1/8X of 8bpp mode 
    stAreaImgInfo.usHeight = usH;
    //printf("IT8951HostAreaPackedPixelWrite [wait]\n\r");
    //Load Image from Host to IT8951 Image Buffer
    tconHostAreaPackedPixelWrite(&stLdImgInfo, &stAreaImgInfo);//Display function 2
}


void TFT_eSPI::tconLoadImage(const TByte* pImgBuf, TWord usX, TWord usY, TWord usW, TWord usH, TByte enFilp)
{
	TCONLdImgInfo stLdImgInfo;
    TCONAreaImgInfo stAreaImgInfo;
	
    //Setting Load image information
    stLdImgInfo.ulStartFBAddr    = (TDWord) pImgBuf;
    stLdImgInfo.usEndianType     = IT8951_LDIMG_L_ENDIAN;
    stLdImgInfo.usPixelFormat    = IT8951_4BPP; 
    stLdImgInfo.usRotate         = IT8951_ROTATE_0;
    stLdImgInfo.ulImgBufBaseAddr = _gulImgBufAddr;
	stLdImgInfo.usFilp           = enFilp;
    //Set Load Area
    stAreaImgInfo.usX      = usX ;
    stAreaImgInfo.usY      = usY;
    stAreaImgInfo.usWidth  = usW ;
    stAreaImgInfo.usHeight = usH;
    //printf("IT8951HostAreaPackedPixelWrite [wait]\n\r");
    //Load Image from Host to IT8951 Image Buffer
    tconHostAreaPackedPixelWrite(&stLdImgInfo, &stAreaImgInfo);//Display function 2
}


void TFT_eSPI::getTconInfo(void* pBuf)
{
    TWord* pusWord = (TWord*)pBuf;
	I80TCONDevInfo* pstDevInfo;

	tconWriteCmdCode(USDEF_I80_CMD_GET_DEV_INFO);
 
	//Burst Read Request for SPI interface only
	tconReadNData(pusWord, sizeof(I80TCONDevInfo) / 2);//Polling HRDY for each words(2-bytes) if possible
	
	//Show Device information of IT8951
	pstDevInfo = (I80TCONDevInfo*)pBuf;
	//printf("Panel(W,H) = (%d,%d)\r\n",
	//pstDevInfo->usPanelW, pstDevInfo->usPanelH );
	//printf("Image Buffer Address = %X\r\n",
	//pstDevInfo->usImgBufAddrL | (pstDevInfo->usImgBufAddrH << 16));
	//Show Firmware and LUT Version
	//printf("FW Version = %s\r\n", (TByte*)pstDevInfo->usFWVersion);
	//printf("LUT Version = %s\r\n", (TByte*)pstDevInfo->usLUTVersion);
}

void TFT_eSPI::hostTconInit()
{

	setTconVcom(1400);      //SET VCOM
	//setTconTemp(14); 		//SET TEMP
    getTconInfo(&_gstI80DevInfo);
    if (_gstI80DevInfo.usPanelW == 0 || _gstI80DevInfo.usPanelH == 0) {
        println("Invalid panel size! Communication with IT8951 may have failed.");
        return;
    }

    _gulImgBufAddr = _gstI80DevInfo.usImgBufAddrL | (_gstI80DevInfo.usImgBufAddrH << 16);
    //Set to Enable I80 Packed mode
	tconWriteReg(I80CPCR, 0x0001);  
}

void TFT_eSPI::hostTconInitFast()
{
    //setTconVcom(1400); 
    getTconInfo(&_gstI80DevInfo);
    if (_gstI80DevInfo.usPanelW == 0 || _gstI80DevInfo.usPanelH == 0) {
        println("Invalid panel size! Communication with IT8951 may have failed.");
        return;
    }

    _gulImgBufAddr = _gstI80DevInfo.usImgBufAddrL | (_gstI80DevInfo.usImgBufAddrH << 16);
    //tconWriteReg(I80CPCR, 0x0001);  
}

void TFT_eSPI::setTconWindowsData(TWord x1, TWord y1, TWord x2, TWord y2)
{
    _imgAreaInfo.usX = x1;
	_imgAreaInfo.usY = y1;
	_imgAreaInfo.usWidth = x2 - x1 + 1;
	_imgAreaInfo.usHeight = y2 - y1 + 1;
}

TWord TFT_eSPI::getTconTemp()
{
    tconWriteCmdCode(0x0040);
	tconWirteData(0x00);
	return (TByte)tconReadData();

}

 void TFT_eSPI::setTconTemp(TWord temp)
 {
    tconWriteCmdCode(0x0040);
	tconWirteData(0x01);
	tconWirteData(temp);

 }


TWord TFT_eSPI::getTconVcom()
 {
    tconWriteCmdCode(0x0039);
	tconWirteData(0x00);
	return tconReadData();
 }

 void TFT_eSPI::setTconVcom(TWord vcom)
 {
    tconWriteCmdCode(0x0039);
	tconWirteData(0x02);
	tconWirteData(vcom);
 }


void TFT_eSPI::tconSleep()
{
    tconWriteCmdCode(IT8951_TCON_SLEEP);
}

void TFT_eSPI::tconWake()
{
    tconWriteCmdCode(IT8951_TCON_SYS_RUN);
}

void TFT_eSPI::tconStandby()
{
    tconWriteCmdCode(IT8951_TCON_STANDBY);
}

void TFT_eSPI::tconWaitForDisplayReady()
{
    while(tconReadReg(LUTAFSR));
}