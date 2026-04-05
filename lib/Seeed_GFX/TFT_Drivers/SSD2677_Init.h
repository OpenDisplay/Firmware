// JD79686B_Init.h
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
    delay(50);
    CHECK_BUSY();

	writecommand(0x00);	//0x00
	writedata(0x2B);	
	writedata(0x29);	

	writecommand(0x06);	//0x06
	writedata(0x0F);	
	writedata(0x8B);	
	writedata(0x93);	
	writedata(0xC1);//0xC1

	writecommand(0x50);	//0x50
	writedata(0x37);	

	writecommand(0x30);	//0x30
	writedata(0x08);	

    writecommand(0x61); //TRES
    writedata(800/256);   // Source_BITS_H
    writedata(800%256);   // Source_BITS_L
    writedata(680/256);     // Gate_BITS_H
    writedata(680%256);     // Gate_BITS_L  

	writecommand(0x62);
	writedata(0x76); 
	writedata(0x76);
	writedata(0x76); 
	writedata(0x5A);
	writedata(0x9D); 
	writedata(0x8A);	
	writedata(0x76); 
	writedata(0x62); 

	writecommand(0x65);	//0x65
	writedata(0x00);	
	writedata(0x00);	
	writedata(0x00);	
	writedata(0x00);	
	
	writecommand(0xE0);	//0xE3
	writedata(0x10);	
	
	writecommand(0xE7);	//0xE7
	writedata(0xA4);	

	writecommand(0xE9);	
	writedata(0x01);
	
    writecommand(0x04); //Power on
    CHECK_BUSY();  
}