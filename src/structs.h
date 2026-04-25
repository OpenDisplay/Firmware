#ifndef STRUCTS_H
#define STRUCTS_H

#include <stdint.h>

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

// 0x01: system_config
struct SystemConfig {
    uint16_t ic_type;           // IC used in this device
    uint8_t communication_modes; // Supported communication modes (bitfield)
    uint8_t device_flags;       // Misc device flags (bitfield)
    uint8_t pwr_pin;            // Power pin number (0xFF if not present)
    uint8_t reserved[15];       // Reserved bytes for future use
    uint8_t pwr_pin_2;          // Optional 2nd power/enable (e.g. Seeed ED103 TFT_ENABLE); 0 or 0xFF = default (11)
    uint8_t pwr_pin_3;          // Optional 3rd power/enable (e.g. ITE_ENABLE); 0 or 0xFF = default (21)
} __attribute__((packed));

// 0x02: manufacturer_data
struct ManufacturerData {
    uint16_t manufacturer_id;   // Defines the manufacturer
    uint8_t board_type;         // Board identifier
    uint8_t board_revision;     // Board revision number
    uint8_t reserved[18];       // Reserved bytes for future use
} __attribute__((packed));

// 0x04: power_option
struct PowerOption {
    uint8_t power_mode;         // Power source type enum
    uint8_t battery_capacity_mah[3]; // Battery capacity in mAh (3 bytes)
    uint16_t sleep_timeout_ms;  // Nominal awake time in milliseconds (advertising timeout)
    uint8_t tx_power;           // Transmit power setting
    uint8_t sleep_flags;        // Sleep-related flags (bitfield)
    uint8_t battery_sense_pin;  // Pin used to measure battery voltage (0xFF if none)
    uint8_t battery_sense_enable_pin; // Pin that enables battery sense circuit (0xFF if none)
    uint8_t battery_sense_flags; // Battery sense flags (bitfield)
    uint8_t capacity_estimator; // Battery chemistry estimator enum
    uint16_t voltage_scaling_factor; // Voltage scaling / divider factor
    uint32_t deep_sleep_current_ua; // Deep sleep current in microamperes
    uint16_t deep_sleep_time_seconds; // Deep sleep duration in seconds (0 if not used)
    uint8_t reserved[10];        // Reserved bytes for future use
} __attribute__((packed));

// Panel IDs must match web/firmware/toolbox/config.yaml display.panel_ic_type enum values.
// Decimal 3000–3999 = Seeed_GFX / OpenDisplay runtime epaper (add new IDs here as panels ship).
#define PANEL_IC_SEEED_ED103TC2_1872X1404 3000u
#define PANEL_IC_SEEED_ED103TC2_1872X1404_4GRAY 3001u

// display.color_scheme (config.yaml); use with matching panel (e.g. gray16 + panel_ic 3001).
#define COLOR_SCHEME_GRAY16 6u

// display.transmission_modes (config.yaml bitfield). ZIPXL extends ZIP on builds with a larger buffer.
#if defined(TARGET_ESP32) && defined(TARGET_LARGE_MEMORY) && defined(BOARD_HAS_PSRAM)
#define MAX_COMPRESSED_BUFFER_BYTES (512u * 1024u)
#elif defined(TARGET_ESP32) && defined(TARGET_LARGE_MEMORY)
#define MAX_COMPRESSED_BUFFER_BYTES (256u * 1024u)
#else
#define MAX_COMPRESSED_BUFFER_BYTES (54u * 1024u)
#endif
#define TRANSMISSION_MODE_ZIPXL          (1u << 0)
#define TRANSMISSION_MODE_ZIP            (1u << 1)
#define TRANSMISSION_MODE_G5             (1u << 2)
#define TRANSMISSION_MODE_DIRECT_WRITE   (1u << 3)
#define TRANSMISSION_MODE_CLEAR_ON_BOOT  (1u << 7)

// 0x20: display (repeatable, max 4 instances)
struct DisplayConfig {
    uint8_t instance_number;    // Unique index for multiple display blocks (0-based)
    uint8_t display_technology; // Display technology enum
    uint16_t panel_ic_type;     // Display controller / panel type
    uint16_t pixel_width;       // Pixel width of panel
    uint16_t pixel_height;      // Pixel height of panel
    uint16_t active_width_mm;   // Active width of panel in millimeters
    uint16_t active_height_mm;  // Active height of panel in millimeters
    uint16_t tag_type;          // Legacy tag type (optional)
    uint8_t rotation;           // Physical rotation in degrees (enum)
    uint8_t reset_pin;          // Pin number for panel reset (0xFF if none)
    uint8_t busy_pin;           // Pin number to read panel busy status (0xFF if none)
    uint8_t dc_pin;             // SPI MISO for Seeed ED103/IT8951 (OpenDisplay); else data/command if used
    uint8_t cs_pin;             // SPI chip select pin (0xFF if none)
    uint8_t data_pin;           // Data out pin (MOSI / data line)
    uint8_t partial_update_support; // Partial update capability (enum)
    uint8_t color_scheme;       // Color scheme supported by the display
    uint8_t transmission_modes; // Supported image/data transmission modes (bitfield)
    uint8_t clk_pin;            // SPI SCLK (Seeed ePaper)
    uint8_t reserved_pin_2;     // Spare GPIO (Seeed enables use system_config.pwr_pin_2/3)
    uint8_t reserved_pin_3;     // Spare GPIO
    uint8_t reserved_pin_4;     // Reserved / spare pin 4
    uint8_t reserved_pin_5;     // Reserved / spare pin 5
    uint8_t reserved_pin_6;     // Reserved / spare pin 6
    uint8_t reserved_pin_7;     // Reserved / spare pin 7
    uint8_t reserved_pin_8;     // Reserved / spare pin 8
    uint16_t full_update_mC;    // Energy for full refresh in millicoulombs (0 = unknown)
    uint8_t reserved[13];       // Reserved bytes for future use
} __attribute__((packed));

// 0x21: led (repeatable, max 4 instances)
struct LedConfig {
    uint8_t instance_number;    // Unique index for multiple LED blocks (0-based)
    uint8_t led_type;           // LED type enum (RGB, single, RY, etc.)
    uint8_t led_1_r;            // LED channel 1 (red) pin number
    uint8_t led_2_g;            // LED channel 2 (green) pin number
    uint8_t led_3_b;            // LED channel 3 (blue) pin number
    uint8_t led_4;              // LED channel 4 pin number (if present)
    uint8_t led_flags;          // LED flags (bitfield)
    uint8_t reserved[15];       // Reserved bytes for future use
} __attribute__((packed));

// 0x29: passive_buzzer (repeatable, max 4 instances)
// Frequency in 0x0075 payload: 0 = silence/rest; 1–255 maps linearly to firmware-defined Hz range (not stored in config).
#define BUZZER_FLAG_ENABLE_ACTIVE_HIGH (1u << 0)

struct PassiveBuzzerConfig {
    uint8_t instance_number;
    uint8_t drive_pin;            // PWM / square wave to buzzer (+ transistor)
    uint8_t enable_pin;           // Optional enable (e.g. FET); 0xFF = unused
    uint8_t flags;                // BUZZER_FLAG_*
    uint8_t duty_percent;         // 1–100 PWM duty; 0 = default 50
    uint8_t reserved[27];
} __attribute__((packed));

// 0x23: sensor_data (repeatable, max 4 instances)
#define SENSOR_TYPE_TEMPERATURE 0x0001u
#define SENSOR_TYPE_HUMIDITY    0x0002u
#define SENSOR_TYPE_AXP2101     0x0003u
#define SENSOR_TYPE_SHT40       0x0004u

struct SensorData {
    uint8_t instance_number;    // Unique index for multiple sensor blocks (0-based)
    uint16_t sensor_type;       // Sensor type enum (SENSOR_TYPE_*)
    uint8_t bus_id;             // Instance id of the bus to use for this sensor
    uint8_t i2c_addr_7bit;      // I2C 7-bit address; 0 or 0xFF = default per sensor (SHT40: 0x44)
    uint8_t msd_data_start_byte; // SHT40: first index in dynamicreturndata for 3-byte MSD block; 0 or 0xFF = default (7)
    uint8_t reserved[24];       // Reserved for future use
} __attribute__((packed));

// 0x24: data_bus (repeatable, max 4 instances)
struct DataBus {
    uint8_t instance_number;    // Unique index for multiple bus blocks (0-based)
    uint8_t bus_type;           // Bus type enum
    uint8_t pin_1;              // Pin 1 (SCL for I2C)
    uint8_t pin_2;              // Pin 2 (SDA for I2C)
    uint8_t pin_3;              // Pin 3 (aux)
    uint8_t pin_4;              // Pin 4 (aux)
    uint8_t pin_5;              // Pin 5 (aux)
    uint8_t pin_6;              // Pin 6 (aux)
    uint8_t pin_7;              // Pin 7 (aux)
    uint32_t bus_speed_hz;      // Bus speed in Hz (32-bit value)
    uint8_t bus_flags;          // Bus flags (bitfield)
    uint8_t pullups;            // Internal pullup resistors (bit per pin)
    uint8_t pulldowns;          // Internal pulldown resistors (bit per pin)
    uint8_t reserved[14];       // Reserved bytes for future use
} __attribute__((packed));

// 0x25: binary_inputs (repeatable, max 4 instances)
struct BinaryInputs {
    uint8_t instance_number;    // Unique index for multiple input blocks (0-based)
    uint8_t input_type;         // Input type enum
    uint8_t display_as;         // How input should be represented in systems (enum)
    uint8_t reserved_pin_1;     // Reserved / spare pin 1
    uint8_t reserved_pin_2;     // Reserved / spare pin 2
    uint8_t reserved_pin_3;     // Reserved / spare pin 3
    uint8_t reserved_pin_4;     // Reserved / spare pin 4
    uint8_t reserved_pin_5;     // Reserved / spare pin 5
    uint8_t reserved_pin_6;     // Reserved / spare pin 6
    uint8_t reserved_pin_7;     // Reserved / spare pin 7
    uint8_t reserved_pin_8;     // Reserved / spare pin 8
    uint8_t input_flags;        // Input flags (bitfield)
    uint8_t invert;             // Invert flags per pin (bitfield)
    uint8_t pullups;            // Internal pullup resistors per pin (bitfield)
    uint8_t pulldowns;          // Internal pulldown resistors per pin (bitfield)
    uint8_t button_data_byte_index;  // Byte index in dynamicreturndata (0-10) for button data
    uint8_t reserved[14];       // Reserved bytes for future use
} __attribute__((packed));

// 0x28: touch_controller (repeatable, max 4 instances)
// touch_ic_type: 0 = disabled / none, 1 = GT911
#define TOUCH_IC_NONE   0u
#define TOUCH_IC_GT911  1u
#define TOUCH_FLAG_INVERT_X  (1u << 0)
#define TOUCH_FLAG_INVERT_Y  (1u << 1)
#define TOUCH_FLAG_SWAP_XY   (1u << 2)

struct TouchController {
    uint8_t instance_number;
    uint16_t touch_ic_type;
    uint8_t bus_id;             // data_bus index, or 0xFF if I2C already up (e.g. after display init)
    uint8_t i2c_addr_7bit;      // GT911: 0x5D or 0x14; 0 or 0xFF = auto (try both after reset)
    uint8_t int_pin;            // GT911 INT, 0xFF = poll only
    uint8_t rst_pin;            // GT911 RST, 0xFF = skip hardware reset
    uint8_t display_instance;   // Clip/scale to displays[instance] pixel size
    uint8_t flags;              // TOUCH_FLAG_*
    uint8_t poll_interval_ms;   // 0 = default 25 ms
    uint8_t touch_data_start_byte; // First of 5 bytes in MSD dynamicreturndata (0–6): byte0 low nibble = contacts 1–5 (down) or 6 (released, last xy kept); high nibble = track id
    uint8_t reserved[21];
} __attribute__((packed));

// Global configuration structure
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

    struct PassiveBuzzerConfig passive_buzzers[4];
    uint8_t passive_buzzer_count;

    // Config metadata
    uint8_t version;            // Protocol version
    uint8_t minor_version;      // Protocol minor version
    bool loaded;                // True if config was successfully loaded
};

// 0x26 (decimal 38): wifi_config — matches web/firmware/toolbox/config.yaml packet_types
struct WifiConfig {
    uint8_t ssid[32];
    uint8_t password[32];
    uint8_t encryption_type;
    uint8_t reserved[95];
} __attribute__((packed));

// 0x27: security_config
struct SecurityConfig {
    uint8_t encryption_enabled;     // 0 = disabled, 1 = enabled
    uint8_t encryption_key[16];    // AES-128 master key (16 bytes)
    uint16_t session_timeout_seconds; // Session timeout (0 = no timeout)
    // Bitfield flags (reserved[0]):
    // Bit 0: rewrite_allowed - Allow unauthenticated config writes when encryption is enabled
    // Bit 1: show_key_on_screen - Show encryption key on screen (future feature)
    // Bit 2: reset_pin_enabled - Reset pin enabled (must be set for reset pin to work)
    // Bit 3: reset_pin_polarity - Reset pin polarity (0 = LOW triggers, 1 = HIGH triggers)
    // Bit 4: reset_pin_pullup - Enable pullup on reset pin
    // Bit 5: reset_pin_pulldown - Enable pulldown on reset pin
    // Bits 6-7: Reserved
    uint8_t flags;                  // Security flags bitfield
    uint8_t reset_pin;               // Reset pin number
    uint8_t reserved[43];           // Reserved bytes for future use
} __attribute__((packed));

// Security flags bitfield definitions
#define SECURITY_FLAG_REWRITE_ALLOWED     (1 << 0)
#define SECURITY_FLAG_SHOW_KEY_ON_SCREEN  (1 << 1)
#define SECURITY_FLAG_RESET_PIN_ENABLED   (1 << 2)
#define SECURITY_FLAG_RESET_PIN_POLARITY  (1 << 3)
#define SECURITY_FLAG_RESET_PIN_PULLUP    (1 << 4)
#define SECURITY_FLAG_RESET_PIN_PULLDOWN  (1 << 5)

// ---------------------------------------------------------------------------
// Partial-rendering protocol constants (§1 of partial-rendering-plan.md)
// ---------------------------------------------------------------------------

// Error codes for partial-rendering NACK responses: {0xFF, opcode, error, 0x00}.
// Mirror these values in py-opendisplay so the client can distinguish cases.
#define ERR_ETAG_MISMATCH   0x01u  // 0x70: old_etag supplied but does not match stored etag
#define ERR_MIXED_DATA      0x02u  // 0x76/0x71: mixed partial/full data in same transfer
#define ERR_SEGMENT_OOB     0x03u  // 0x76: segment x+w > W or y+h > H

// Transfer data-kind: tracks whether the current transfer has received full
// (0x71) or partial (0x76) data. Lives only for the duration of one transfer.
#define DATA_KIND_NONE      0u  // No data received yet
#define DATA_KIND_FULL      1u  // Full-image data (0x71) in progress
#define DATA_KIND_PARTIAL   2u  // Partial-segment data (0x76) in progress

#endif