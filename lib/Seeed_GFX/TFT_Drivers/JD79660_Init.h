// JD79660_Init.h
{
#ifdef TFT_BUSY
    pinMode(TFT_BUSY, INPUT);
#endif
#ifdef TFT_ENABLE
    pinMode(TFT_ENABLE, OUTPUT);
    digitalWrite(TFT_ENABLE, HIGH);
#endif  
    digitalWrite(TFT_RST, LOW);
    delay(50);
    digitalWrite(TFT_RST, HIGH);
    delay(100);
	writecommand(0x4D);
	writedata(0x78);

	writecommand(0x00);	//PSR
	writedata(0x0F);
	writedata(0x29);

	writecommand(0X06); //BTST_P
	writedata(0x0D);  //47uH
	writedata(0x12);
	writedata(0x30);
	writedata(0x20);
	writedata(0x19);
	writedata(0x2A);
	writedata(0x22);

	writecommand(0x50);	//CDI
	writedata(0x37);

	writecommand(0x61); //TRES
	writedata(TFT_WIDTH/256);		// Source_BITS_H
	writedata(TFT_WIDTH%256);		// Source_BITS_L
	writedata(TFT_HEIGHT/256);			// Gate_BITS_H
	writedata(TFT_HEIGHT%256); 		// Gate_BITS_L	

	writecommand(0xE9);
	writedata(0x01); 

	writecommand(0x30);
	writedata(0x08);  
		
	writecommand(0x04);
	CHECK_BUSY();  
}
