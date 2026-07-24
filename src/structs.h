#ifndef STRUCTS_H
#define STRUCTS_H

#include <stdint.h>
// Canonical wire contract: config + message payload structs, OD_-prefixed enums
// and flag macros, and (transitively) opendisplay_protocol.h framing constants
// (CMD_*, RESP_*, PIPE_*, config limits). This header is the single source of
// truth for every config-packet struct; the firmware-only runtime types below
// are the only definitions that remain local.
#include "opendisplay_structs.h"

#define BOOT_ROW_BUFFER_SIZE 960

// Image transfer state variables
struct ImageData {
    uint8_t* data;
    uint32_t size;
    uint32_t received;
    uint8_t dataType;
    bool isCompressed;
    uint32_t crc32;
    uint16_t width;
    uint16_t height;
    bool ready;
    uint32_t totalBlocks;
    uint32_t currentBlock;
    bool* blocksReceived;
    uint32_t* blockBytesReceived;  // Track bytes received per block
    uint32_t* blockPacketsReceived; // Track packets received per block
};

// PIPE_WRITE (0x0080-0x0082) sliding-window receive state. Out-of-order frames
// are held in a small reorder queue while the controller stream pauses at a hole;
// when the missing frame arrives in-order it is written and the contiguous run of
// queued successors drains. 33 slots = 32 (max window) + 1 safety; the sender's
// span-based window rule bounds occupancy to <=32 so overflow is a protocol
// violation, not an expected condition. Indexing by seq % PIPE_REORDER_SLOTS is
// collision-free because any live window spans <=W < PIPE_REORDER_SLOTS seqs.
//
// PIPE_SMALL_DRAM_WINDOW is set ONLY by the classic-ESP32 env:esp32-N4 (esp32dev,
// 320KB RAM). Its static DRAM is far tighter than the S3/C3/C6 parts, so the full
// 33-slot x 248 B queue (~8.3KB .bss) overflows dram0_0_seg by ~672 B at link.
// Cap that env to W=16 / 17 slots (~4.2KB); 17 = W+1 keeps seq%SLOTS collision-free
// (a live window spans <=16 < 17). All other ESP32 envs keep the full 32-deep window.
#ifdef PIPE_SMALL_DRAM_WINDOW
#define PIPE_REORDER_SLOTS      17
#define PIPE_MAX_W      16
#define PIPE_MAX_N      16
#else
#define PIPE_REORDER_SLOTS      33
#define PIPE_MAX_W      32
#define PIPE_MAX_N      32
#endif
#define PIPE_REORDER_SLOT_SIZE  248    // >= max plaintext data payload (241 @ frame 244; 212 encrypted)

// LOCAL link policy: the ATT MTU this device asks NimBLE to negotiate on the BLE
// transport (BLEDevice::setMTU -> ble_att_set_preferred_mtu). "Preferred" is literal --
// the central drives the exchange and may settle lower, so nothing may assume this
// value was granted. Deliberately distinct from OD_BLE_MAX_FRAME: that is the
// cross-repo WIRE ceiling used to size buffers and the GATT value, whereas this is
// one device's request on one physical link. They are equal today; the asserts below
// state the coupling that actually matters instead of leaving it to a shared symbol.
//
// NOT used on nRF: Bluefruit fixes the MTU at BLE_GATT_ATT_MTU_MAX (247) via
// configPrphBandwidth(BANDWIDTH_MAX) -> configPrphConn(247, ...) before the SoftDevice
// starts, and 256 exceeds that cap. See ble_init.cpp.
#define OD_BLE_PREFERRED_ATT_MTU  256u

// A single-PDU write carries OD_BLE_PREFERRED_ATT_MTU - 3 value bytes (ATT opcode 1 +
// handle 2). It must (a) still admit the largest legitimate inbound frame, and (b) never
// exceed the slot the payload is copied into.
static_assert(OD_BLE_PREFERRED_ATT_MTU - 3u >= PIPE_REORDER_SLOT_SIZE,
              "negotiated MTU too small for the largest pipe frame");
static_assert(OD_BLE_PREFERRED_ATT_MTU - 3u <= OD_BLE_MAX_FRAME,
              "a single-PDU write could overrun an OD_BLE_MAX_FRAME-sized slot");

#ifdef TARGET_ESP32
// BLE TX ring. Defined here, NOT in main.h: communication.cpp used to carry its own
// copy of this struct plus a MAX_RESPONSE_SIZE_LOCAL constant, so the bound checked
// before the memcpy in esp32_queue_ble_notify_copy() lived in a different file from
// the slot it guarded -- two definitions of one type (an ODR violation) that had to be
// edited in lockstep or the guard would admit a response larger than the slot.
#define RESPONSE_QUEUE_SIZE 10
#define MAX_RESPONSE_SIZE   OD_BLE_MAX_FRAME

struct ResponseQueueItem {
    uint8_t data[MAX_RESPONSE_SIZE];
    uint16_t len;
    bool pending;
};
#endif

// PIPE_WRITE protocol constants (PIPE_ACK_MASK_BITS, PIPE_MAX_FRAME, PIPE_VERSION,
// PIPE_FLAG_COMPRESSED, PIPE_FLAG_PARTIAL) come from the canonical opendisplay_protocol.h.
// PIPE_FLAG_PARTIAL bit1: partial-region refresh. START carries a 12-byte LE extension
// [old_etag:4][x:2][y:2][w:2][h:2]; geometry/etag validated like 0x76, refresh
// mode + new_etag ride the 0x0082 END. See PIPE_WRITE section in display_service.cpp.

struct PipeReorderSlot {
    bool     occupied;
    uint8_t  seq;
    uint16_t len;
    uint8_t  data[PIPE_REORDER_SLOT_SIZE];
};

struct PipeWriteState {
    bool     active;
    bool     error;             // fatal: silently discard 0x0081 until next 0x0080 / disconnect
    bool     compressed;
    bool     partial;           // partial-region transfer: route DATA to partialCtx, END drives REFRESH_PARTIAL
    bool     gap_open;          // true while a hole is outstanding (queue non-empty)
    uint8_t  window;            // W_eff
    uint8_t  ack_every;         // N_eff
    uint16_t max_frame;         // frame_eff
    uint8_t  expected_seq;      // next in-order seq (mod 256)
    bool     has_received;      // false until first accepted-or-queued frame (highest_seen valid)
    uint8_t  highest_seen;      // highest received seq (accepted or queued), mod 256
    uint32_t received_count;    // accepted+queued distinct frames (diagnostics)
    uint8_t  frames_since_ack;  // cadence counter (in-order accepts)
    uint8_t  ooo_acks_since_gap;// rate-limit counter for out-of-order / duplicate gap ACKs
    uint32_t total_size;        // negotiated decompressed panel byte total
    uint8_t  queued_count;      // reorder-queue occupancy
    uint8_t  queue_high_water;  // diagnostics: max occupancy seen this transfer
};

// Global configuration structure. Members are the canonical config structs from
// opendisplay_structs.h; only the aggregation, per-type instance arrays/counts,
// and load metadata are firmware-local.
struct GlobalConfig {
    // Required packets (single instances)
    struct SystemConfig system_config;
    struct ManufacturerData manufacturer_data;
    struct PowerOption power_option;

    // Optional repeatable packets (max 4 instances each)
    struct DisplayConfig displays[4];
    uint8_t display_count;      // Number of display instances loaded

    struct LedConfig leds[4];
    uint8_t led_count;          // Number of LED instances loaded

    struct SensorData sensors[4];
    uint8_t sensor_count;       // Number of sensor instances loaded

    struct DataBus data_buses[4];
    uint8_t data_bus_count;     // Number of data bus instances loaded

    struct BinaryInputs binary_inputs[4];
    uint8_t binary_input_count; // Number of binary input instances loaded

    struct TouchController touch_controllers[4];
    uint8_t touch_controller_count;

    struct BuzzerConfig passive_buzzers[4];
    uint8_t passive_buzzer_count;

    struct FlashConfig flash_configs[2];
    uint8_t flash_config_count;

    struct DataExtended data_extended;
    bool data_extended_loaded;

    // Config metadata
    uint8_t version;            // Protocol version
    uint8_t minor_version;      // Protocol minor version
    bool loaded;                // True if config was successfully loaded
};

#define MAX_BUTTONS 32  // Up to 4 instances * 8 pins = 32 buttons max
struct ButtonState {
    uint8_t button_id;          // Button ID (0-7, from instance_number + pin offset)
    uint8_t press_count;         // Press count (0-15)
    volatile uint8_t current_state;       // Current button state (0=released, 1=pressed, updated in ISR)
    uint8_t byte_index;          // Byte index in dynamicreturndata
    uint8_t pin;                 // GPIO pin number
    uint8_t instance_index;      // BinaryInputs instance index
    bool initialized;          // Whether this button is initialized
    uint8_t pin_offset;         // Pin offset within instance (0-7) for faster ISR lookup
    bool inverted;              // Inverted flag for this pin (cached for ISR)
    bool power_off;             // Whether this button triggers a power-off
    uint16_t power_off_hold_ms; // Hold duration (ms) required to trigger power-off
};

#endif
