#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <stdint.h>

#define CONFIG_FILE_PATH "/config.bin"
#define MAX_CONFIG_SIZE 4096

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t crc;
    uint32_t data_len;
    uint8_t data[MAX_CONFIG_SIZE];
} config_storage_t;

bool initConfigStorage();
void formatConfigStorage();
bool saveConfig(uint8_t* configData, uint32_t len);
bool loadConfig(uint8_t* configData, uint32_t* len);
uint32_t calculateConfigCRC(uint8_t* data, uint32_t len);
bool loadGlobalConfig();
void printConfigSummary();
void full_config_init();

#endif
