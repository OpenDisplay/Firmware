#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>

void sendResponseUnencrypted(uint8_t* response, uint8_t len);
void sendResponse(uint8_t* response, uint8_t len);
uint16_t calculateCRC16CCITT(uint8_t* data, uint32_t len);
uint8_t getFirmwareMajor();
uint8_t getFirmwareMinor();
const char* getFirmwareShaString();
void handleFirmwareVersion();
void handleReadMSD();
void handleReadConfig();
void handleWriteConfig(uint8_t* data, uint16_t len);
void handleWriteConfigChunk(uint8_t* data, uint16_t len);
void sendConnectionNotification(uint8_t status);
void sendDisplayAnnouncement();
void sendImageRequest();

#endif
