#ifdef TFT_BUSY
    pinMode(TFT_BUSY, INPUT);
#endif
#ifdef TFT_ENABLE
    pinMode(TFT_ENABLE, OUTPUT);
    digitalWrite(TFT_ENABLE, HIGH);
#endif  
    pinMode(TFT_CS1, OUTPUT); 
    digitalWrite(TFT_CS1, HIGH); 
    digitalWrite(TFT_RST, LOW);
    delay(20);                 
    digitalWrite(TFT_RST, HIGH);
    delay(20);   