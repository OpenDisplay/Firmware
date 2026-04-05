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
    writecommand(0x4D);
    writedata(0x78);

    writecommand(0x00); //PSR
    writedata(0x0F);
    writedata(0x29);

    writecommand(0x01); //PWRR
    writedata(0x07);
    writedata(0x00);

    writecommand(0x03); //POFS
    writedata(0x10);
    writedata(0x54);
    writedata(0x44);

    writecommand(0x06); //BTST_P
    writedata(0x05);
    writedata(0x00);
    writedata(0x3F);
    writedata(0x0A);
    writedata(0x25);
    writedata(0x12);
    writedata(0x1A); 

    writecommand(0x50); //CDI
    writedata(0x37);

    writecommand(0x60); //TCON
    writedata(0x02);
    writedata(0x02);

    writecommand(0x61); //TRES
    writedata(128/256);   // Source_BITS_H
    writedata(128%256);   // Source_BITS_L
    writedata(296/256);     // Gate_BITS_H
    writedata(296%256);     // Gate_BITS_L  

    writecommand(0xE7);
    writedata(0x1C);

    writecommand(0xE3); 
    writedata(0x22);

    writecommand(0xB4);
    writedata(0xD0);
    writecommand(0xB5);
    writedata(0x03);

    writecommand(0xE9);
    writedata(0x01); 

    writecommand(0x30);
    writedata(0x08);  

    writecommand(0x04);
    CHECK_BUSY();  
}