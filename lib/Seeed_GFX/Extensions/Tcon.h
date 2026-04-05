public:

    typedef struct TCONLdImgInfo
    {
        TWord usEndianType; //little or Big Endian
        TWord usPixelFormat; //bpp
        TWord usRotate; //Rotate mode
        TWord usFilp; //Filp mode
        TDWord ulStartFBAddr; //Start address of source Frame buffer
        TDWord ulImgBufBaseAddr;//Base address of target image buffer
        
    }TCONLdImgInfo;

    //structure prototype 2
    typedef struct TCONAreaImgInfo
    {
        TWord usX;
        TWord usY;
        TWord usWidth;
        TWord usHeight;
        
    }TCONAreaImgInfo;

    typedef struct I80TCON1DevInfo
    {
        TWord usPanelW;
        TWord usPanelH;
        TWord usImgBufAddrL;
        TWord usImgBufAddrH;
        TWord usFWVersion[8]; 	//16 Bytes String
        TWord usLUTVersion[8]; 	//16 Bytes String
    }I80TCONDevInfo;

    I80TCONDevInfo _gstI80DevInfo;
    TDWord _gulImgBufAddr; //IT8951 Image buffer address

    TCONAreaImgInfo _imgAreaInfo;

    

    void tconWaitForReady();

    void tconSendWord(TWord data);
    TWord tconReceiveWord(void);

    void tconWriteCmdCode(TWord usCmdCode);
    void tconWirteData(TWord usData);
    void tconWirteNData(TWord* pwBuf, TDWord ulSizeWordCnt);

    void tconSendCmdArg(TWord usCmdCode,TWord* pArg, TWord usNumArg);
    TWord tconReadData();
    void tconReadNData(TWord* pwBuf, TDWord ulSizeWordCnt);

    TWord tconReadReg(TWord usRegAddr);
    void tconWriteReg(TWord usRegAddr,TWord usValue);

    void tconLoadImgStart(TCONLdImgInfo* pstLdImgInfo);
    void tconLoadImgAreaStart(TCONLdImgInfo* pstLdImgInfo ,TCONAreaImgInfo* pstAreaImgInfo);
    void tconLoadImgEnd(void);

    void tconSetImgBufBaseAddr(TDWord ulImgBufAddr);
    void tconSetImgRotation(TDWord rotation);

    void tconHostAreaPackedPixelWrite(TCONLdImgInfo* pstLdImgInfo,TCONAreaImgInfo* pstAreaImgInfo);
    

    void tconDisplayArea(TWord usX, TWord usY, TWord usW, TWord usH, TWord usDpyMode);
    void tconDisplayArea1bpp(TWord usX, TWord usY, TWord usW, TWord usH, TWord usDpyMode, TByte ucBGGrayVal, TByte ucFGGrayVal);    

    void tconLoad1bppImage(const TByte* p1bppImgBuf, TWord usX, TWord usY, TWord usW, TWord usH, TByte enFilp);
    void tconLoadImage(const TByte* pImgBuf, TWord usX, TWord usY, TWord usW, TWord usH, TByte enFilp);

    TWord getTconTemp();
    void setTconTemp(TWord temp);

    TWord getTconVcom();
    void setTconVcom(TWord vcom);

    void getTconInfo(void* pBuf);
    void setTconWindowsData(TWord x1, TWord y1, TWord x2, TWord y2);
    void hostTconInit();
    void hostTconInitFast();

    void tconSleep();
    void tconWake();
    void tconStandby();
    void tconWaitForDisplayReady();
    





