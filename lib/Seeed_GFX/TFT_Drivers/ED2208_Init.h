
// This is the command sequence that initialises the  ED2208 driver

{

#ifdef TFT_BUSY
    pinMode(TFT_BUSY, INPUT);
#endif
#ifdef TFT_ENABLE
  pinMode(TFT_ENABLE, OUTPUT);
  digitalWrite(TFT_ENABLE, HIGH);
#endif  
  digitalWrite(TFT_RST, LOW);  
  delay(20);                   
  digitalWrite(TFT_RST, HIGH); 
  delay(10);                   
  writecommand(0xAA);    // CMDH
  writedata(0x49);
  writedata(0x55);
  writedata(0x20);
  writedata(0x08);
  writedata(0x09);
  writedata(0x18);

  writecommand(EPD_PWRR);
  writedata(0x3F);
  writedata(0x00);
  writedata(0x32);
  writedata(0x2A);
  writedata(0x0E);
  writedata(0x2A);
  
  writecommand(EPD_PSR);  
  writedata(0x5F);
  writedata(0x69);
  
  writecommand(EPD_POFS);
  writedata(0x00);
  writedata(0x54);
  writedata(0x00);
  writedata(0x44); 

  writecommand(EPD_BTST1);
  writedata(0x40);
  writedata(0x1F);
  writedata(0x1F);
  writedata(0x2C);
  
  writecommand(EPD_BTST2);
  writedata(0x6F);
  writedata(0x1F);
  writedata(0x16);
  writedata(0x25);

  writecommand(EPD_BTST3);
  writedata(0x6F);
  writedata(0x1F);
  writedata(0x1F);
  writedata(0x22);
 
  writecommand(0x13);    // IPC
  writedata(0x00);
  writedata(0x04);
  
  writecommand(EPD_PLL);
  writedata(0x02);
  
  writecommand(0x41);     // TSE
  writedata(0x00);
  
  writecommand(EPD_CDI);
  writedata(0x3F);
  
  writecommand(EPD_TCON);
  writedata(0x02);
  writedata(0x00);
  
  writecommand(EPD_TRES);
  writedata((TFT_WIDTH >> 8) & 0xFF);
  writedata(TFT_WIDTH & 0xFF);
  writedata((TFT_HEIGHT >> 8) & 0xFF);
  writedata(TFT_HEIGHT & 0xFF);
  

  writecommand(EPD_VDCS);
  writedata(0x1E); 

  writecommand(EPD_T_VDCS);
  writedata(0x01);

  writecommand(0x86);    // AGID
  writedata(0x00);
  
  writecommand(EPD_PWS);
  writedata(0x2F);
 
  writecommand(0xE0);   // CCSET
  writedata(0x00); 
  
  writecommand(0xE6);   // TSSET
  writedata(0x00);  

  writecommand(0x04);     //PWR on    
  CHECK_BUSY();          //waiting for the electronic paper IC to release the idle signal

}