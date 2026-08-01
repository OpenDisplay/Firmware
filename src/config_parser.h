#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <stdint.h>

#define CONFIG_FILE_PATH "/config.bin"
#define MAX_CONFIG_SIZE 4096

// Sentinel in the first 4 bytes of /config.bin. Cheap provenance gate checked
// before data_len is trusted or the CRC is computed: erased flash, a truncated
// write, or a file from other firmware fails here in 4 bytes.
#define CONFIG_STORAGE_MAGIC   0xDEADBEEFu
#define CONFIG_STORAGE_VERSION 1u

// On-flash header, written verbatim ahead of the payload. This was previously
// the front of a config_storage_t that also carried a uint8_t[MAX_CONFIG_SIZE]
// payload array, so save/load each kept a private 4 KB staging copy of the whole
// blob purely to have somewhere contiguous to read/write it from. The payload is
// now streamed straight to/from the caller's buffer; only the header is staged.
// The on-flash byte layout is unchanged -- header(16) followed by data_len bytes.
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t crc;
    uint32_t data_len;
} config_header_t;

// The on-flash layout depends on this exact size: saveConfig() writes the header
// verbatim and loadConfig() reads it back verbatim, so any padding introduced here
// silently shifts the payload offset and orphans every previously stored config.
static_assert(sizeof(config_header_t) == 16, "config_header_t must stay 16 bytes on flash");

// Reassembly state for a multi-chunk BLE config write (CMD_CONFIG_WRITE START +
// CMD_CONFIG_CHUNK continuations). Defined here, NOT in main.h: communication.cpp
// used to carry its own copy of this struct with a hardcoded 4096 in place of
// MAX_CONFIG_SIZE, so the bound checked in handleWriteConfigChunk() and the slot
// it guards were sized in two different files -- two definitions of one type (an
// ODR violation) that had to be edited in lockstep or the guard would admit more
// than the buffer holds. Same footgun as the old ResponseQueueItem duplication.
typedef struct {
    bool active;
    uint32_t totalSize;
    uint32_t receivedSize;
    uint8_t buffer[MAX_CONFIG_SIZE];
    uint32_t expectedChunks;
    uint32_t receivedChunks;
} chunked_write_state_t;

/**
 * Clear the chunked config-upload state.
 *
 * The single primitive for it. The three sites in communication.cpp that used to
 * clear it inline each zeroed a different subset -- one set only `active`, another
 * also the counters, none the totals -- so a teardown routed through the wrong one
 * left a partially-live upload. abortToKnownState() calls this too, which is what
 * gives session teardown any coverage of this state at all: it previously had no
 * reset function, so no disconnect path and no watchdog touched it.
 *
 * The payload buffer is deliberately not zeroed: `active = false` makes it
 * unreachable, and MAX_CONFIG_SIZE is large enough that clearing it on every
 * teardown would be pointless work.
 */
void resetChunkedWriteState(void);

bool initConfigStorage();
void formatConfigStorage();
bool saveConfig(uint8_t* configData, uint32_t len);
bool clearStoredConfig(void);
bool loadConfig(uint8_t* configData, uint32_t* len);
bool hasValidStoredConfig(void);
uint32_t calculateConfigCRC(uint8_t* data, uint32_t len);
bool loadGlobalConfig();
void printConfigSummary();
// Suppress the informational config dumps (parse-time detail + printConfigSummary)
// without touching ERROR/WARNING output. Used to keep a deep-sleep wake quiet.
void full_config_init();

// The one 4 KB config staging buffer, shared by every consumer that needs a
// whole config blob in RAM (loadGlobalConfig, handleReadConfig,
// hasValidStoredConfig). Safe to share because all config paths run
// synchronously on the loop task and none of them nest: no two callers ever
// hold it live at the same time. Callers must treat the contents as valid only
// until they return. Do NOT use this for chunkedWriteState -- that buffer stays
// live across BLE commands while other config work can run.
uint8_t* getConfigScratch(void);

#endif
