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
    uint8_t reserved[17];       // Reserved bytes for future use
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
    uint8_t dc_pin;             // Data/Command select pin (0xFF if none)
    uint8_t cs_pin;             // SPI chip select pin (0xFF if none)
    uint8_t data_pin;           // Data out pin (MOSI / data line)
    uint8_t partial_update_support; // Partial update capability (enum)
    uint8_t color_scheme;       // Color scheme supported by the display
    uint8_t transmission_modes; // Supported image/data transmission modes (bitfield)
    uint8_t clk_pin;     // Reserved / spare pin 1
    uint8_t reserved_pin_2;     // Reserved / spare pin 2
    uint8_t reserved_pin_3;     // Reserved / spare pin 3
    uint8_t reserved_pin_4;     // Reserved / spare pin 4
    uint8_t reserved_pin_5;     // Reserved / spare pin 5
    uint8_t reserved_pin_6;     // Reserved / spare pin 6
    uint8_t reserved_pin_7;     // Reserved / spare pin 7
    uint8_t reserved_pin_8;     // Reserved / spare pin 8
    uint8_t reserved[15];       // Reserved bytes for future use
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

// 0x23: sensor_data (repeatable, max 4 instances)
struct SensorData {
    uint8_t instance_number;    // Unique index for multiple sensor blocks (0-based)
    uint16_t sensor_type;       // Sensor type enum
    uint8_t bus_id;             // Instance id of the bus to use for this sensor
    uint8_t reserved[26];       // Reserved bytes for future use
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
    
    // Config metadata
    uint8_t version;            // Protocol version
    uint8_t minor_version;      // Protocol minor version
    bool loaded;                // True if config was successfully loaded
};

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

#endif