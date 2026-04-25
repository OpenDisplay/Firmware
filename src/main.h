#include <Arduino.h>
#include "structs.h"
#include "uzlib.h"
#include <bb_epaper.h>
#include <SPI.h>
#include "encryption_state.h"
#include "config_parser.h"
#include "ble_init.h"
#include "wifi_service.h"

#ifndef BUILD_VERSION
#define BUILD_VERSION "0.0"
#endif
#ifndef SHA
#define SHA ""
#endif

#define STRINGIFY(x) #x
#define XSTRINGIFY(x) STRINGIFY(x)
#define SHA_STRING XSTRINGIFY(SHA)

#ifdef TARGET_NRF
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#endif

#ifdef TARGET_ESP32
#include <LittleFS.h>
#endif

#include <Wire.h>

#define DECOMP_CHUNK 512
#define DECOMP_CHUNK_SIZE 4096
#define MAX_DICT_SIZE 32768
#define MAX_BLOCKS 64
// Text rendering constants
#define FONT_CHAR_WIDTH 16  // 7 font columns + 1 blank column, each doubled (8*2)
#define FONT_CHAR_HEIGHT 16 // 8 pixels tall, doubled (8*2)
#define FONT_BASE_WIDTH 8   // Base font width (7 columns + 1 spacing)
#define FONT_BASE_HEIGHT 8  // Base font height (8 rows)
#define FONT_SMALL_THRESHOLD 264  // Use 1x scale for displays narrower than this
// Config chunked write constants
#define CONFIG_CHUNK_SIZE 200  // Maximum size of a config chunk
#define CONFIG_CHUNK_SIZE_WITH_PREFIX 202  // Chunk size with 2-byte size prefix
#define MAX_CONFIG_CHUNKS 20  // Maximum number of chunks allowed
// Response buffer constants
#define MAX_RESPONSE_DATA_SIZE 100  // Maximum data size in response buffer

// BLE response codes (second byte only, first byte is always 0x00 for success, 0xFF for error)
#define RESP_DIRECT_WRITE_START_ACK  0x70  // Direct write start acknowledgment
#define RESP_DIRECT_WRITE_DATA_ACK   0x71  // Direct write data acknowledgment
#define RESP_DIRECT_WRITE_END_ACK    0x72  // Direct write end acknowledgment
#define RESP_DIRECT_WRITE_REFRESH_SUCCESS 0x73  // Display refresh completed successfully
#define RESP_DIRECT_WRITE_REFRESH_TIMEOUT 0x74  // Display refresh timed out
#define RESP_PARTIAL_WRITE_DATA_ACK  0x76  // Partial image data acknowledgment
#define RESP_DIRECT_WRITE_ERROR      0xFF  // Direct write error response
#define RESP_CONFIG_READ             0x40  // Config read response
#define RESP_CONFIG_WRITE             0x41  // Config write response
#define RESP_CONFIG_CHUNK             0x42  // Config chunk response
#define RESP_MSD_READ                 0x44  // MSD (Manufacturer Specific Data) read response

// Communication mode bit definitions (for system_config.communication_modes)
#define COMM_MODE_BLE           (1 << 0)  // Bit 0: BLE transfer supported
#define COMM_MODE_OEPL          (1 << 1)  // Bit 1: OEPL based transfer supported
#define COMM_MODE_WIFI          (1 << 2)  // Bit 2: WiFi transfer supported

// Device flags bit definitions (for system_config.device_flags)
#define DEVICE_FLAG_PWR_PIN      (1 << 0)  // Bit 0: Device has external power management pin
#define DEVICE_FLAG_XIAOINIT     (1 << 1)  // Bit 1: Call xiaoinit() after config load (nRF52840 only)
#define DEVICE_FLAG_WS_PP_INIT   (1 << 2)  // Bit 2: Call ws_pp_init() after config load (Waveshare Photo Printer)

#ifdef TARGET_NRF
#include <bluefruit.h>
extern BLEDfu bledfu;
extern BLEService imageService;
extern BLECharacteristic imageCharacteristic;
// Forward declaration for SoftDevice temperature API
extern "C" uint32_t sd_temp_get(int32_t *p_temp);
extern "C" {
    #include "nrf_soc.h"   // for sd_app_evt_wait()
  }
#endif

#ifdef TARGET_ESP32
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <esp_system.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include "esp_sleep.h"
#include <WiFi.h>
#include <ESPmDNS.h>

extern BLEServer* pServer;
extern BLEService* pService;
extern BLECharacteristic* pTxCharacteristic;
extern BLECharacteristic* pRxCharacteristic;
extern BLEAdvertisementData* advertisementData;  // Pointer to global advertisementData object

// RTC memory variables for deep sleep state tracking (declared in main.cpp)
extern bool advertising_timeout_active;
extern uint32_t advertising_start_time;
#endif

BBEPDISP bbep;

// Forward declarations for bbep functions
void bbepInitIO(BBEPDISP *pBBEP, uint8_t u8DC, uint8_t u8RST, uint8_t u8BUSY, uint8_t u8CS, uint8_t u8MOSI, uint8_t u8SCK, uint32_t u32Speed);
int bbepSetPanelType(BBEPDISP *pBBEP, int iPanel);
void bbepSetRotation(BBEPDISP *pBBEP, int iRotation);
void bbepStartWrite(BBEPDISP *pBBEP, int iPlane);
int bbepRefresh(BBEPDISP *pBBEP, int iMode);
bool bbepIsBusy(BBEPDISP *pBBEP);
void bbepWakeUp(BBEPDISP *pBBEP);
void bbepSleep(BBEPDISP *pBBEP, int deepSleep);
void bbepSendCMDSequence(BBEPDISP *pBBEP, const uint8_t *pSeq);
void bbepSetAddrWindow(BBEPDISP *pBBEP, int x, int y, int cx, int cy);
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen);

extern uint8_t* compressedDataBuffer;
void allocCompressedDataBuffer(void);

uint8_t decompressionChunk[DECOMP_CHUNK_SIZE];
uint8_t dictionaryBuffer[MAX_DICT_SIZE];
uint8_t bleResponseBuffer[94];
uint8_t mloopcounter = 0;
uint8_t rebootFlag = 1;  // Set to 1 after reboot, cleared to 0 after BLE connection
uint8_t connectionRequested = 0;  // Reserved for future features (connection requested flag)
uint8_t dynamicreturndata[11] = {0};  // Dynamic return data blocks (bytes 2-12 in advertising payload)
uint8_t msd_payload[16] = {0};  // Manufacturer Specific Data payload (public, updated by updatemsdata())

// Button state tracking structure
struct ButtonState {
    uint8_t button_id;          // Button ID (0-7, from instance_number + pin offset)
    uint8_t press_count;         // Press count (0-15)
    volatile uint32_t last_press_time;    // Timestamp of last press (millis, updated in ISR)
    volatile uint8_t current_state;       // Current button state (0=released, 1=pressed, updated in ISR)
    uint8_t byte_index;          // Byte index in dynamicreturndata
    uint8_t pin;                 // GPIO pin number
    uint8_t instance_index;      // BinaryInputs instance index
    bool initialized;          // Whether this button is initialized
    uint8_t pin_offset;         // Pin offset within instance (0-7) for faster ISR lookup
    bool inverted;              // Inverted flag for this pin (cached for ISR)
};

#define MAX_BUTTONS 32  // Up to 4 instances * 8 pins = 32 buttons max
ButtonState buttonStates[MAX_BUTTONS] = {0};  // Button state tracking
uint8_t buttonStateCount = 0;  // Number of initialized buttons
volatile bool buttonEventPending = false;  // Flag set by ISR to indicate button event
volatile uint8_t lastChangedButtonIndex = 0xFF;  // Index of button that last changed (set by ISR)
uint8_t ledFlashPosition = 0;  // Current position in LED flash pattern group
uint8_t activeLedInstance = 0xFF;  // LED instance index for flashing (0xFF = none configured)
bool ledFlashActive = false;  // Flag to indicate if LED flashing is active (set by command)

// Static buffers for writeTextAndFill() to avoid dynamic allocation
// Maximum pitch: 1360px / 2 = 680 bytes (4BPP), line buffer: 256 bytes
uint8_t staticWhiteRow[680];
uint8_t staticRowBuffer[680];
char staticLineBuffer[256];

char wifiSsid[33] = {0};  // 32 bytes + null terminator
char wifiPassword[33] = {0};  // 32 bytes + null terminator
uint8_t wifiEncryptionType = 0;  // 0x00=none, 0x01=WEP, 0x02=WPA, 0x03=WPA2, 0x04=WPA3
bool wifiConfigured = false;  // True if WiFi config packet (0x26) was received and parsed
#ifdef TARGET_ESP32
#include <WiFi.h>
bool wifiConnected = false;
bool wifiInitialized = false;
char wifiServerUrl[65] = {0};
uint16_t wifiServerPort = 2446;
bool wifiServerConfigured = false;
WiFiServer wifiServer;
WiFiClient wifiClient;
bool wifiServerConnected = false;
uint8_t tcpReceiveBuffer[8192];
uint32_t tcpReceiveBufferPos = 0;
#endif

// Direct write mode state (bufferless display writing)
bool directWriteActive = false;  // True when direct write mode is active
bool directWriteCompressed = false;  // True if using compressed direct write
bool directWriteBitplanes = false;  // True if using bitplanes (BWR/BWY - 2 planes)
bool directWritePlane2 = false;  // True when writing plane 2 (R/Y) for bitplanes
uint32_t directWriteBytesWritten = 0;  // Total bytes written to current plane
uint32_t directWriteDecompressedTotal = 0;  // Expected decompressed size
uint16_t directWriteWidth = 0;  // Display width in pixels
uint16_t directWriteHeight = 0;  // Display height in pixels
uint32_t directWriteTotalBytes = 0;  // Total bytes expected per plane (for bitplanes) or total (for others)
uint8_t directWriteRefreshMode = 0;  // 0 = FULL (default), 1 = FAST/PARTIAL (if supported)
uint8_t directWriteDataKind = DATA_KIND_NONE;  // 0x71 (FULL) vs 0x76 (PARTIAL) tracking for current transfer

// Direct write compressed mode: use same buffer as regular image upload
uint32_t directWriteCompressedSize = 0;  // Total compressed size expected
uint32_t directWriteCompressedReceived = 0;  // Total compressed bytes received
uint8_t* directWriteCompressedBuffer = nullptr;  // Points at compressedDataBuffer when compressed direct-write is active
uint32_t directWriteStartTime = 0;  // Timestamp when direct write started (for timeout detection)
bool displayPowerState = false;  // Track display power state (true = powered on, false = powered off)

bool waitforrefresh(int timeout);
void pwrmgm(bool onoff);
bool powerDownExternalFlash(uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin, uint8_t csPin, uint8_t wpPin, uint8_t holdPin);
void xiaoinit();
void ws_pp_init();
void writeSerial(String message, bool newLine = true);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
#ifdef TARGET_ESP32
void minimalSetup();
void fullSetupAfterConnection();
void enterDeepSleep();
extern bool advertising_timeout_active;
extern uint32_t advertising_start_time;
#endif
String getChipIdHex();

// Platform-specific type aliases for BLE callback
#ifdef TARGET_NRF
    typedef uint16_t BLEConnHandle;
    typedef BLECharacteristic* BLECharPtr;
#else
    typedef void* BLEConnHandle;
    typedef void* BLECharPtr;
#endif

void imageDataWritten(BLEConnHandle conn_hdl, BLECharPtr chr, uint8_t* data, uint16_t len);
void sendResponse(uint8_t* response, uint8_t len);
void sendResponseUnencrypted(uint8_t* response, uint8_t len);
void secureEraseConfig();
void checkResetPin();
void reboot();
void enterDFUMode();
void initio();
void initDataBuses();
void initButtons();
void handleButtonPress(uint8_t buttonIndex);
void processButtonEvents();  // Process button events and update BLE data
void idleDelay(uint32_t delayMs);  // Non-blocking delay that processes buttons at 100ms intervals
void flashLed(uint8_t color, uint8_t brightness);  // Flash LED with color and brightness
void ledFlashLogic();  // LED flashing logic with pattern support (runs indefinitely)
void handleLedActivate(uint8_t* data, uint16_t len);  // Handle LED activation command
#ifdef TARGET_ESP32
void handleButtonISR(uint8_t buttonIndex);  // Shared ISR handler (IRAM_ATTR in implementation)
#else
void handleButtonISR(uint8_t buttonIndex);  // Shared ISR handler
#endif
void scanI2CDevices();
void initSensors();
void initAXP2101(uint8_t busId);
void updatemsdata();
uint16_t calculateCRC16CCITT(uint8_t* data, uint32_t len);
void handleReadConfig();
void handleWriteConfig(uint8_t* data, uint16_t len);
void handleWriteConfigChunk(uint8_t* data, uint16_t len);
void handleFirmwareVersion();
void handleReadMSD();  // Read Manufacturer Specific Data (MSD) payload
const char* getFirmwareShaString();
void cleanupDirectWriteState(bool refreshDisplay);
void handleDirectWriteStart(uint8_t* data, uint16_t len);
void handleDirectWriteData(uint8_t* data, uint16_t len);
void handleDirectWriteEnd(uint8_t* data = nullptr, uint16_t len = 0);
void handleDirectWriteCompressedData(uint8_t* data, uint16_t len);
void handlePartialWriteData(uint8_t* data, uint16_t len);
int mapEpd(int id);
uint8_t getFirmwareMajor();
uint8_t getFirmwareMinor();
float readBatteryVoltage();  // Returns battery voltage in volts, or -1.0 if not configured
float readChipTemperature();  // Returns chip temperature in degrees Celsius
int getplane();
int getBitsPerPixel();

// Encryption functions
bool isEncryptionEnabled();
bool isAuthenticated();
void clearEncryptionSession();
bool checkEncryptionSessionTimeout();
void updateEncryptionSessionActivity();
bool handleAuthenticate(uint8_t* data, uint16_t len);
bool decryptCommand(uint8_t* ciphertext, uint16_t ciphertext_len, uint8_t* plaintext, uint16_t* plaintext_len, uint8_t* nonce, uint8_t* auth_tag, uint16_t command_header);
bool encryptResponse(uint8_t* plaintext, uint16_t plaintext_len, uint8_t* ciphertext, uint16_t* ciphertext_len, uint8_t* nonce, uint8_t* auth_tag);
bool verifyNonceReplay(uint8_t* nonce);
void incrementNonceCounter();
void getCurrentNonce(uint8_t* nonce);

typedef struct {
    bool active;
    uint32_t totalSize;
    uint32_t receivedSize;
    uint8_t buffer[MAX_CONFIG_SIZE];
    uint32_t expectedChunks;
    uint32_t receivedChunks;
} chunked_write_state_t;

extern chunked_write_state_t chunkedWriteState;
chunked_write_state_t chunkedWriteState = {false, 0, 0, {0}, 0, 0};
struct GlobalConfig globalConfig = {0};
uint8_t configReadResponseBuffer[128];

// Security configuration
struct SecurityConfig securityConfig = {0};
EncryptionSession encryptionSession = {0};
bool encryptionInitialized = false;

#ifndef RTC_DATA_ATTR
// nRF52 has no equivalent of ESP32 RTC slow RAM; the variable becomes a
// regular global and resets on boot — partial-rendering then degrades to
// always-full-upload, which is the safe fallback.
#define RTC_DATA_ATTR
#endif

// 0x00000000 = "not set". Persists across deep sleep on ESP32.
RTC_DATA_ATTR uint32_t displayed_etag = 0;

#ifdef TARGET_ESP32
// RTC memory variables for deep sleep state tracking
RTC_DATA_ATTR bool woke_from_deep_sleep = false;
RTC_DATA_ATTR uint32_t deep_sleep_count = 0;

// Advertising timeout state variables
bool advertising_timeout_active = false;
uint32_t advertising_start_time = 0;

// First-boot holdoff before allowing deep sleep
static bool firstBootDelayInitialized = false;
static uint32_t firstBootDelayStart = 0;
#endif

#define AXP2101_SLAVE_ADDRESS 0x34
#define AXP2101_REG_POWER_STATUS 0x00
#define AXP2101_REG_POWER_ON_STATUS 0x01
#define AXP2101_REG_POWER_OFF_STATUS 0x02
#define AXP2101_REG_DC_ONOFF_DVM_CTRL 0x80
#define AXP2101_REG_LDO_ONOFF_CTRL0 0x90
#define AXP2101_REG_DC_VOL0_CTRL 0x82  // DCDC1 voltage
#define AXP2101_REG_LDO_VOL2_CTRL 0x94  // ALDO3 voltage
#define AXP2101_REG_LDO_VOL3_CTRL 0x95  // ALDO4 voltage
#define AXP2101_REG_POWER_WAKEUP_CTL 0x26
#define AXP2101_REG_ADC_CHANNEL_CTRL 0x30
#define AXP2101_REG_ADC_DATA_BAT_VOL_H 0x34
#define AXP2101_REG_ADC_DATA_BAT_VOL_L 0x35
#define AXP2101_REG_ADC_DATA_VBUS_VOL_H 0x36
#define AXP2101_REG_ADC_DATA_VBUS_VOL_L 0x37
#define AXP2101_REG_ADC_DATA_SYS_VOL_H 0x38
#define AXP2101_REG_ADC_DATA_SYS_VOL_L 0x39
#define AXP2101_REG_BAT_PERCENT_DATA 0xA4
#define AXP2101_REG_PWRON_STATUS 0x20
#define AXP2101_REG_BAT_DETECTION_CTRL 0x68  // Battery detection control
#define AXP2101_REG_IRQ_ENABLE1 0x40  // IRQ enable register 1
#define AXP2101_REG_IRQ_ENABLE2 0x41  // IRQ enable register 2
#define AXP2101_REG_IRQ_ENABLE3 0x42  // IRQ enable register 3
#define AXP2101_REG_IRQ_ENABLE4 0x43  // IRQ enable register 4
#define AXP2101_REG_IRQ_STATUS1 0x44  // IRQ status register 1
#define AXP2101_REG_IRQ_STATUS2 0x45  // IRQ status register 2
#define AXP2101_REG_IRQ_STATUS3 0x46  // IRQ status register 3
#define AXP2101_REG_IRQ_STATUS4 0x47  // IRQ status register 4
#define AXP2101_REG_LDO_ONOFF_CTRL1 0x91  // LDO control register 1 (BLDO1-2, CPUSLDO, DLDO1-2)

#ifdef TARGET_NRF
BLEDfu bledfu;
BLEService imageService("2446");
BLECharacteristic imageCharacteristic("2446", BLEWrite | BLEWriteWithoutResponse | BLENotify, 512);
#endif

#ifdef TARGET_ESP32
// Define queue sizes and structures first
#define RESPONSE_QUEUE_SIZE 10
#define MAX_RESPONSE_SIZE 512
#define COMMAND_QUEUE_SIZE 5
#define MAX_COMMAND_SIZE 512

struct ResponseQueueItem {
    uint8_t data[MAX_RESPONSE_SIZE];
    uint16_t len;
    bool pending;
};

ResponseQueueItem responseQueue[RESPONSE_QUEUE_SIZE];
uint8_t responseQueueHead = 0;
uint8_t responseQueueTail = 0;

#include "esp32_ble_callbacks.h"

CommandQueueItem commandQueue[COMMAND_QUEUE_SIZE];
uint8_t commandQueueHead = 0;
uint8_t commandQueueTail = 0;

BLEServer* pServer = nullptr;
BLEService* pService = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
BLECharacteristic* pRxCharacteristic = nullptr;
BLEAdvertisementData globalAdvertisementData;  // Global object, not pointer
BLEAdvertisementData* advertisementData = &globalAdvertisementData;  // Pointer to global object
MyBLEServerCallbacks staticServerCallbacks;  // Static callback object (no dynamic allocation)
MyBLECharacteristicCallbacks staticCharCallbacks;  // Static callback object (no dynamic allocation)
#endif

extern const uint8_t writelineFont[] PROGMEM;