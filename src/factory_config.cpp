#include "factory_config.h"
#include "communication.h"
#include <Arduino.h>
#include <string.h>

void writeSerial(String message, bool newLine = true);

static uint16_t toolboxOuterCrc(const uint8_t* data, uint32_t bodyLen) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < bodyLen; i++) {
        const uint8_t byte = (i < 2) ? 0 : data[i];
        crc ^= (uint16_t)(byte << 8);
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
            } else {
                crc = (crc << 1) & 0xFFFF;
            }
        }
    }
    return crc;
}

static bool factoryPacketValid(const uint8_t* data, uint32_t len) {
    if (len < 4 || len > MAX_CONFIG_SIZE) {
        return false;
    }
    const uint16_t declared = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    if (declared != len) {
        return false;
    }
    const uint16_t calc = toolboxOuterCrc(data, len - 2);
    const uint16_t given = (uint16_t)data[len - 2] | ((uint16_t)data[len - 1] << 8);
    return given == calc;
}

static bool factoryEmbedPresent(const factory_flash_cfg_t* fc) {
    if (!fc || fc->magic != FACTORY_CFG_MAGIC) {
        return false;
    }
    if (fc->len < 4 || fc->len > MAX_CONFIG_SIZE) {
        return false;
    }
    return factoryPacketValid(fc->data, fc->len);
}

bool tryProvisionFactoryEmbed(void) {
#ifdef FACTORY_HAS_EMBED
    if (!factoryEmbedPresent(&g_factory_embed)) {
        return false;
    }

    writeSerial("No valid stored config; provisioning from factory embed...");
    if (saveConfig(const_cast<uint8_t*>(g_factory_embed.data), g_factory_embed.len)) {
        writeSerial("Factory config saved to filesystem");
        return true;
    }

    writeSerial("ERROR: Factory embed present but saveConfig failed");
#endif
    return false;
}
