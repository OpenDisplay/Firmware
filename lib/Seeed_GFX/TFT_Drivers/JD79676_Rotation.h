rotation = m % 4;

writecommand(EPD_PNLSET);
switch (rotation) {
    case 0: // Portrait
        writedata(0x0F); // 0b00011111
        _width = _init_width;
        _height = _init_height;
        break;
    case 1: // Landscape (Portrait + 90)
        writedata(0x0B); // 0b00011011
        _width = _init_height;
        _height = _init_width;
        break;
    case 2: // Inverted Portrait
        writedata(0x03); // 0b00010011
        _width = _init_width;
        _height = _init_height;
        break;
    case 3: // Landscape (Portrait + 270)
        writedata(0x07); // 0b00010111
        _width = _init_height;
        _height = _init_width;
        break;
}