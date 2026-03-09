#include <Arduino.h>
#include "structs.h"
#include "uzlib.h"
#include <bb_epaper.h>
#include <Group5.h>
#include <SPI.h>

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
#define MAX_IMAGE_SIZE (54 * 1024)
#define MAX_BLOCKS 64
#define CONFIG_FILE_PATH "/config.bin"
#define MAX_CONFIG_SIZE 4096
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

// Transmission mode bit definitions (for display.transmission_modes)
#define TRANSMISSION_MODE_RAW          (1 << 0)  // Bit 0: Raw transfer
#define TRANSMISSION_MODE_ZIP          (1 << 1)  // Bit 1: ZIP compressed transfer
#define TRANSMISSION_MODE_G5           (1 << 2)  // Bit 2: Group 5 compression
#define TRANSMISSION_MODE_DIRECT_WRITE (1 << 3)  // Bit 3: Direct write mode (bufferless)
#define TRANSMISSION_MODE_CLEAR_ON_BOOT (1 << 7) // Bit 7: Clear screen at bootup (writeTextAndFill with empty string)

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

uint8_t compressedDataBuffer[MAX_IMAGE_SIZE];  // Static buffer for compressed image data
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
#include <WiFiClient.h>
bool wifiConnected = false;  // True if WiFi is currently connected
bool wifiInitialized = false;  // True if WiFi initialization was attempted
char wifiServerUrl[65] = {0};  // 64 bytes + null terminator for server URL/hostname
uint16_t wifiServerPort = 2446;  // Default server port
bool wifiServerConfigured = false;  // True if server URL is configured
WiFiClient wifiClient;  // TCP client connection
bool wifiServerConnected = false;  // True if TCP connection to server is established
uint32_t wifiServerLastConnectAttempt = 0;  // Timestamp of last connection attempt
const uint32_t WIFI_SERVER_RECONNECT_DELAY = 30000;  // 30 seconds between reconnect attempts
uint8_t tcpReceiveBuffer[8192];  // TCP receive buffer
uint32_t tcpReceiveBufferPos = 0;  // Current position in receive buffer
uint32_t wifiNextImageRequestTime = 0;  // Timestamp when next Image Request should be sent (0 = send immediately)
uint32_t wifiPollInterval = 60;  // Poll interval in seconds (default: 60 seconds)
bool wifiImageRequestPending = false;  // True when Image Request has been sent and we're waiting for response
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

// Direct write compressed mode: use same buffer as regular image upload
uint32_t directWriteCompressedSize = 0;  // Total compressed size expected
uint32_t directWriteCompressedReceived = 0;  // Total compressed bytes received
uint8_t* directWriteCompressedBuffer = nullptr;  // Pointer to compressedDataBuffer (static allocation only)
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
void initDisplay();
#ifdef TARGET_ESP32
void minimalSetup();
void fullSetupAfterConnection();
void enterDeepSleep();
void ble_init_esp32(bool update_manufacturer_data = true);
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
void readAXP2101Data();
void powerDownAXP2101();
void updatemsdata();
void ble_init();
#ifdef TARGET_ESP32
void initWiFi();
void connectWiFiServer();
void discoverAndConnectWiFiServer();
void handleWiFiServer();
void sendConnectionNotification(uint8_t status);
void disconnectWiFiServer();
void sendDisplayAnnouncement();
void sendImageRequest();
#endif
void full_config_init();
void formatConfigStorage();
bool initConfigStorage();
bool saveConfig(uint8_t* configData, uint32_t len);
bool loadConfig(uint8_t* configData, uint32_t* len);
uint32_t calculateConfigCRC(uint8_t* data, uint32_t len);
uint16_t calculateCRC16CCITT(uint8_t* data, uint32_t len);
void handleReadConfig();
void handleWriteConfig(uint8_t* data, uint16_t len);
void handleWriteConfigChunk(uint8_t* data, uint16_t len);
void handleFirmwareVersion();
void handleReadMSD();  // Read Manufacturer Specific Data (MSD) payload
void cleanupDirectWriteState(bool refreshDisplay);
void handleDirectWriteStart(uint8_t* data, uint16_t len);
void handleDirectWriteData(uint8_t* data, uint16_t len);
void handleDirectWriteEnd(uint8_t* data = nullptr, uint16_t len = 0);
void handleDirectWriteCompressedData(uint8_t* data, uint16_t len);
void printConfigSummary();
void reboot();
bool loadGlobalConfig();
int mapEpd(int id);
uint8_t getFirmwareMajor();
uint8_t getFirmwareMinor();
float readBatteryVoltage();  // Returns battery voltage in volts, or -1.0 if not configured
float readChipTemperature();  // Returns chip temperature in degrees Celsius
int getplane();
int getBitsPerPixel();
void writeTextAndFill(const char* text);
void drawBootScreen();

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
uint8_t* getCurrentNonce();

typedef struct {
    bool active;
    uint32_t totalSize;
    uint32_t receivedSize;
    uint8_t buffer[MAX_CONFIG_SIZE];
    uint32_t expectedChunks;
    uint32_t receivedChunks;
} chunked_write_state_t;

typedef struct {
    uint32_t magic;        
    uint32_t version;
    uint32_t crc;
    uint32_t data_len;
    uint8_t data[MAX_CONFIG_SIZE];
} config_storage_t;

extern chunked_write_state_t chunkedWriteState;
chunked_write_state_t chunkedWriteState = {false, 0, 0, {0}, 0, 0};
struct GlobalConfig globalConfig = {0};
uint8_t configReadResponseBuffer[128];

// Encryption session state
struct EncryptionSession {
    bool authenticated;              // Session is authenticated
    uint8_t session_key[16];         // Derived session key (zeroized on clear)
    uint8_t session_id[8];          // Session identifier (8 bytes, derived via CMAC)
    uint64_t nonce_counter;         // Per-session counter (64-bit, prevents wrap)
    uint64_t last_seen_counter;     // Last accepted counter value
    uint64_t replay_window[64];     // Sliding window for replay protection (64 packets)
    uint32_t last_activity;         // Timestamp of last activity
    uint8_t integrity_failures;     // Consecutive integrity failures (max 3)
    uint32_t session_start_time;    // Session establishment time
    uint8_t auth_attempts;          // Authentication attempts (for rate limiting)
    uint32_t last_auth_time;        // Last authentication attempt time
    uint8_t client_nonce[16];       // Client nonce from auth (16 bytes)
    uint8_t server_nonce[16];       // Server nonce from auth (16 bytes)
    uint8_t pending_server_nonce[16]; // Pending server nonce (expires after 30 seconds)
    uint32_t server_nonce_time;     // Time when server nonce was generated
};

// Security configuration
struct SecurityConfig securityConfig = {0};
EncryptionSession encryptionSession = {0};
bool encryptionInitialized = false;

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

struct CommandQueueItem {
    uint8_t data[MAX_COMMAND_SIZE];
    uint16_t len;
    bool pending;
};

ResponseQueueItem responseQueue[RESPONSE_QUEUE_SIZE];
uint8_t responseQueueHead = 0;
uint8_t responseQueueTail = 0;

CommandQueueItem commandQueue[COMMAND_QUEUE_SIZE];
uint8_t commandQueueHead = 0;
uint8_t commandQueueTail = 0;

class MyBLEServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        writeSerial("=== BLE CLIENT CONNECTED (ESP32) ===");
        rebootFlag = 0;  // Clear reboot flag after BLE connection established
        updatemsdata();  // Update advertising data with cleared flag
    }
    void onDisconnect(BLEServer* pServer) {
        writeSerial("=== BLE CLIENT DISCONNECTED (ESP32) ===");
        cleanupDirectWriteState(true);
        writeSerial("Waiting before restarting advertising...");
        delay(500);
        if (pServer->getConnectedCount() == 0) {
            BLEDevice::startAdvertising();
            writeSerial("Advertising restarted");
        } else {
            writeSerial("not restarting advertising");
        }
    }
};

class MyBLECharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* pCharacteristic) {
        writeSerial("=== BLE WRITE RECEIVED (ESP32) ===");
        String value = pCharacteristic->getValue();
        writeSerial("Received data length: " + String(value.length()) + " bytes");
        if (value.length() > 0 && value.length() <= MAX_COMMAND_SIZE) {
            uint8_t* data = (uint8_t*)value.c_str();
            uint16_t len = value.length();
            // Log first few bytes
            String hexDump = "Data: ";
            for (int i = 0; i < len && i < 16; i++) {
                if (data[i] < 16) hexDump += "0";
                hexDump += String(data[i], HEX) + " ";
            }
            writeSerial(hexDump);
            uint8_t nextHead = (commandQueueHead + 1) % COMMAND_QUEUE_SIZE;
            if (nextHead != commandQueueTail) {
                memcpy(commandQueue[commandQueueHead].data, data, len);
                commandQueue[commandQueueHead].len = len;
                commandQueue[commandQueueHead].pending = true;
                commandQueueHead = nextHead;
                writeSerial("ESP32: Command queued for processing");
            } else {
                writeSerial("ERROR: Command queue full, dropping command");
            }
        } else if (value.length() > MAX_COMMAND_SIZE) {
            writeSerial("WARNING: Command too large, dropping");
        } else {
            writeSerial("WARNING: Empty data received");
        }
    }
};

BLEServer* pServer = nullptr;
BLEService* pService = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
BLECharacteristic* pRxCharacteristic = nullptr;
BLEAdvertisementData globalAdvertisementData;  // Global object, not pointer
BLEAdvertisementData* advertisementData = &globalAdvertisementData;  // Pointer to global object
MyBLEServerCallbacks staticServerCallbacks;  // Static callback object (no dynamic allocation)
MyBLECharacteristicCallbacks staticCharCallbacks;  // Static callback object (no dynamic allocation)
#endif

const uint8_t writelineFont[] PROGMEM = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00, // space (32)
    0x00,0x06,0x5f,0x5f,0x06,0x00,0x00, // !
    0x00,0x07,0x07,0x00,0x07,0x07,0x00, // "
    0x14,0x7f,0x7f,0x14,0x7f,0x7f,0x14, // #
    0x24,0x2e,0x2a,0x6b,0x6b,0x3a,0x12, // $
    0x46,0x66,0x30,0x18,0x0c,0x66,0x62, // %
    0x30,0x7a,0x4f,0x5d,0x37,0x7a,0x48, // &
    0x00,0x04,0x07,0x03,0x00,0x00,0x00, // '
    0x00,0x1c,0x3e,0x63,0x41,0x00,0x00, // (
    0x00,0x41,0x63,0x3e,0x1c,0x00,0x00, // )
    0x08,0x2a,0x3e,0x1c,0x3e,0x2a,0x08, // *
    0x00,0x08,0x08,0x3e,0x3e,0x08,0x08, // +
    0x00,0x00,0x80,0xe0,0x60,0x00,0x00, // ,
    0x00,0x08,0x08,0x08,0x08,0x08,0x08, // -
    0x00,0x00,0x00,0x60,0x60,0x00,0x00, // .
    0x60,0x30,0x18,0x0c,0x06,0x03,0x01, // /
    0x3e,0x7f,0x59,0x4d,0x47,0x7f,0x3e, // 0
    0x40,0x42,0x7f,0x7f,0x40,0x40,0x00, // 1
    0x62,0x73,0x59,0x49,0x6f,0x66,0x00, // 2
    0x22,0x63,0x49,0x49,0x7f,0x36,0x00, // 3
    0x18,0x1c,0x16,0x53,0x7f,0x7f,0x50, // 4
    0x27,0x67,0x45,0x45,0x7d,0x39,0x00, // 5
    0x3c,0x7e,0x4b,0x49,0x79,0x30,0x00, // 6
    0x03,0x03,0x71,0x79,0x0f,0x07,0x00, // 7
    0x36,0x7f,0x49,0x49,0x7f,0x36,0x00, // 8
    0x06,0x4f,0x49,0x69,0x3f,0x1e,0x00, // 9
    0x00,0x00,0x00,0x66,0x66,0x00,0x00, // :
    0x00,0x00,0x80,0xe6,0x66,0x00,0x00, // ;
    0x08,0x1c,0x36,0x63,0x41,0x00,0x00, // <
    0x00,0x14,0x14,0x14,0x14,0x14,0x14, // =
    0x00,0x41,0x63,0x36,0x1c,0x08,0x00, // >
    0x02,0x03,0x59,0x5d,0x07,0x02,0x00, // ?
    0x3e,0x7f,0x41,0x5d,0x5d,0x5f,0x0e, // @
    0x7c,0x7e,0x13,0x13,0x7e,0x7c,0x00, // A
    0x41,0x7f,0x7f,0x49,0x49,0x7f,0x36, // B
    0x1c,0x3e,0x63,0x41,0x41,0x63,0x22, // C
    0x41,0x7f,0x7f,0x41,0x63,0x3e,0x1c, // D
    0x41,0x7f,0x7f,0x49,0x5d,0x41,0x63, // E
    0x41,0x7f,0x7f,0x49,0x1d,0x01,0x03, // F
    0x1c,0x3e,0x63,0x41,0x51,0x33,0x72, // G
    0x7f,0x7f,0x08,0x08,0x7f,0x7f,0x00, // H
    0x00,0x41,0x7f,0x7f,0x41,0x00,0x00, // I
    0x30,0x70,0x40,0x41,0x7f,0x3f,0x01, // J
    0x41,0x7f,0x7f,0x08,0x1c,0x36,0x63, // K
    0x41,0x7f,0x7f,0x40,0x40,0x40,0x40, // L
    0x7f,0x7f,0x06,0x0c,0x06,0x7f,0x7f, // M
    0x7f,0x7f,0x06,0x0c,0x18,0x7f,0x7f, // N
    0x1c,0x3e,0x63,0x41,0x63,0x3e,0x1c, // O
    0x41,0x7f,0x7f,0x49,0x09,0x0f,0x06, // P
    0x1e,0x3f,0x21,0x71,0x7f,0x5e,0x00, // Q
    0x41,0x7f,0x7f,0x49,0x19,0x7f,0x66, // R
    0x26,0x6f,0x4d,0x59,0x73,0x32,0x00, // S
    0x03,0x41,0x7f,0x7f,0x41,0x03,0x00, // T
    0x7f,0x7f,0x40,0x40,0x7f,0x7f,0x00, // U
    0x1f,0x3f,0x60,0x60,0x3f,0x1f,0x00, // V
    0x7f,0x7f,0x30,0x18,0x30,0x7f,0x7f, // W
    0x63,0x36,0x1c,0x08,0x1c,0x36,0x63, // X
    0x07,0x4f,0x78,0x78,0x4f,0x07,0x00, // Y
    0x47,0x63,0x71,0x59,0x4d,0x67,0x73, // Z
    0x00,0x00,0x7f,0x7f,0x41,0x41,0x00, // [
    0x01,0x03,0x06,0x0c,0x18,0x30,0x60, // backslash
    0x00,0x41,0x41,0x7f,0x7f,0x00,0x00, // ]
    0x08,0x0c,0x06,0x03,0x06,0x0c,0x08, // ^
    0x80,0x80,0x80,0x80,0x80,0x80,0x80, // _
    0x00,0x00,0x03,0x07,0x04,0x00,0x00, // `
    0x20,0x74,0x54,0x54,0x3c,0x78,0x40, // a
    0x41,0x7f,0x3f,0x48,0x48,0x78,0x30, // b
    0x38,0x7c,0x44,0x44,0x6c,0x28,0x00, // c
    0x30,0x78,0x48,0x49,0x3f,0x7f,0x40, // d
    0x38,0x7c,0x54,0x54,0x5c,0x18,0x00, // e
    0x48,0x7e,0x7f,0x49,0x03,0x02,0x00, // f
    0x98,0xbc,0xa4,0xa4,0xf8,0x7c,0x04, // g
    0x41,0x7f,0x7f,0x08,0x04,0x7c,0x78, // h
    0x00,0x44,0x7d,0x7d,0x40,0x00,0x00, // i
    0x40,0xc4,0x84,0xfd,0x7d,0x00,0x00, // j
    0x41,0x7f,0x7f,0x10,0x38,0x6c,0x44, // k
    0x00,0x41,0x7f,0x7f,0x40,0x00,0x00, // l
    0x7c,0x7c,0x18,0x38,0x1c,0x7c,0x78, // m
    0x7c,0x7c,0x04,0x04,0x7c,0x78,0x00, // n
    0x38,0x7c,0x44,0x44,0x7c,0x38,0x00, // o
    0x84,0xfc,0xf8,0xa4,0x24,0x3c,0x18, // p
    0x18,0x3c,0x24,0xa4,0xf8,0xfc,0x84, // q
    0x44,0x7c,0x78,0x4c,0x04,0x1c,0x18, // r
    0x48,0x5c,0x54,0x54,0x74,0x24,0x00, // s
    0x00,0x04,0x3e,0x7f,0x44,0x24,0x00, // t
    0x3c,0x7c,0x40,0x40,0x3c,0x7c,0x40, // u
    0x1c,0x3c,0x60,0x60,0x3c,0x1c,0x00, // v
    0x3c,0x7c,0x70,0x38,0x70,0x7c,0x3c, // w
    0x44,0x6c,0x38,0x10,0x38,0x6c,0x44, // x
    0x9c,0xbc,0xa0,0xa0,0xfc,0x7c,0x00, // y
    0x4c,0x64,0x74,0x5c,0x4c,0x64,0x00, // z
    0x00,0x08,0x36,0x7f,0x41,0x00,0x00, // {
    0x00,0x00,0x00,0x7f,0x7f,0x00,0x00, // |
    0x00,0x41,0x7f,0x36,0x08,0x00,0x00, // }
    0x08,0x0c,0x06,0x0c,0x08,0x0c,0x06, // ~
};