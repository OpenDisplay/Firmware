/*
 * Standalone nRF52840 InternalFS recovery utility.
 *
 * This target deliberately does not include, mount, or parse LittleFS and does
 * not link any OpenDisplay firmware source. It erases the exact internal-flash
 * range reserved by Adafruit InternalFileSystem. Normal firmware will create a
 * fresh LittleFS volume when InternalFS.begin() sees the blank region.
 *
 * Layout authority in the pinned framework:
 *   libraries/InternalFileSytem/src/InternalFileSystem.cpp
 *     LFS_FLASH_ADDR       = 0xED000 (NRF52840)
 *     LFS_FLASH_TOTAL_SIZE = 7 * 4096
 *   libraries/InternalFileSytem/src/flash/flash_nrf5x.c
 *     BOOTLOADER_ADDR      = 0xF4000 (NRF52840)
 *
 * The framework exposes no partition-table API for InternalFS; these addresses
 * are hard-coded in that library. Keep the assertions below synchronized if the
 * framework's InternalFileSystem layout changes.
 */

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <nrf.h>
#include <nrf_nvmc.h>

namespace {

constexpr uint32_t kPageSize = 0x1000u;
constexpr uint32_t kInternalFsStart = 0xED000u;
constexpr uint32_t kInternalFsPageCount = 7u;
constexpr uint32_t kInternalFsEnd =
    kInternalFsStart + kInternalFsPageCount * kPageSize;
constexpr uint32_t kFrameworkBootloaderStart = 0xF4000u;
constexpr uint32_t kSerialConnectTimeoutMs = 30000u;
constexpr uint32_t kPreErasePauseMs = 5000u;

static_assert(kInternalFsStart % kPageSize == 0, "InternalFS start must be page-aligned");
static_assert(kInternalFsEnd == kFrameworkBootloaderStart,
              "InternalFS must end exactly where the bootloader begins");

enum class RecoveryResult {
    Success,
    UnexpectedHardwareLayout,
    EraseVerificationFailed,
};

RecoveryResult recoveryResult = RecoveryResult::UnexpectedHardwareLayout;
bool serialWasConnected = false;

void printHex32(uint32_t value) {
    Serial.print("0x");
    Serial.print(value, HEX);
}

void waitForNvmc() {
    while (!nrf_nvmc_ready_check(NRF_NVMC)) {
    }
}

bool hardwareLayoutIsSafe() {
    const uint32_t hardwarePageSize = NRF_FICR->CODEPAGESIZE;
    const uint32_t flashSize = hardwarePageSize * NRF_FICR->CODESIZE;
    const uint32_t configuredBootloader = NRF_UICR->NRFFW[0];

    if (hardwarePageSize != kPageSize || kInternalFsEnd > flashSize) {
        return false;
    }

    // An erased UICR bootloader field is allowed because some bootloaders use
    // the MBR settings page instead. If UICR names a bootloader, never erase
    // across its start address.
    if (configuredBootloader != 0xFFFFFFFFu &&
        kInternalFsEnd > configuredBootloader) {
        return false;
    }

    return true;
}

void eraseInternalFsPages() {
    waitForNvmc();
    nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_ERASE);
    waitForNvmc();

    for (uint32_t address = kInternalFsStart;
         address < kInternalFsEnd;
         address += kPageSize) {
        Serial.print("Erasing page ");
        printHex32(address);
        Serial.println("...");
        Serial.flush();
        nrf_nvmc_page_erase_start(NRF_NVMC, address);
        waitForNvmc();
    }

    nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_READONLY);
    waitForNvmc();
}

bool internalFsIsBlank() {
    for (uint32_t address = kInternalFsStart;
         address < kInternalFsEnd;
         address += sizeof(uint32_t)) {
        if (*reinterpret_cast<volatile const uint32_t*>(address) != 0xFFFFFFFFu) {
            return false;
        }
    }
    return true;
}

RecoveryResult recoverInternalFs() {
    if (!hardwareLayoutIsSafe()) {
        return RecoveryResult::UnexpectedHardwareLayout;
    }

    eraseInternalFsPages();
    return internalFsIsBlank()
               ? RecoveryResult::Success
               : RecoveryResult::EraseVerificationFailed;
}

void setStatusLed(RecoveryResult result, bool on) {
    const uint8_t pin =
        result == RecoveryResult::Success ? LED_GREEN : LED_RED;
    digitalWrite(pin, on ? LED_STATE_ON : !LED_STATE_ON);
}

void printFinalResult() {
    if (recoveryResult == RecoveryResult::Success) {
        Serial.println("SUCCESS: every InternalFS word verified as 0xFFFFFFFF.");
        Serial.println("Flash normal firmware now; it will create a fresh LittleFS volume.");
    } else if (recoveryResult == RecoveryResult::UnexpectedHardwareLayout) {
        Serial.println("ERROR: hardware or bootloader layout mismatch; nothing was erased.");
    } else {
        Serial.println("ERROR: one or more words failed erase verification.");
    }
    Serial.flush();
}

}  // namespace

void setup() {
    Serial.begin(115200);

    // USB CDC writes made before a host opens the port may be discarded. Wait
    // up to 30 seconds for a monitor, but do not make recovery permanently
    // depend on a serial connection.
    const uint32_t serialWaitStartMs = millis();
    while (!Serial && millis() - serialWaitStartMs < kSerialConnectTimeoutMs) {
        delay(10);
    }
    serialWasConnected = static_cast<bool>(Serial);

    Serial.println();
    Serial.println("nRF52840 InternalFS recovery utility");
    Serial.println("No LittleFS mount or OpenDisplay initialization will run.");
    Serial.println("Serial connected; erase starts in 5 seconds.");
    Serial.flush();
    delay(kPreErasePauseMs);

    const uint32_t hardwarePageSize = NRF_FICR->CODEPAGESIZE;
    const uint32_t flashSize = hardwarePageSize * NRF_FICR->CODESIZE;
    const uint32_t configuredBootloader = NRF_UICR->NRFFW[0];

    Serial.print("Hardware page size: ");
    Serial.println(hardwarePageSize);
    Serial.print("Hardware flash size: ");
    Serial.println(flashSize);
    Serial.print("UICR bootloader address: ");
    printHex32(configuredBootloader);
    Serial.println();
    Serial.print("InternalFS erase range: ");
    printHex32(kInternalFsStart);
    Serial.print("-");
    printHex32(kInternalFsEnd - 1u);
    Serial.println();
    Serial.flush();

    recoveryResult = recoverInternalFs();

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_RED, !LED_STATE_ON);
    digitalWrite(LED_GREEN, !LED_STATE_ON);

    printFinalResult();
}

void loop() {
    static bool ledOn = false;
    static uint32_t lastToggleMs = 0;

    if (millis() - lastToggleMs >= 500u) {
        lastToggleMs = millis();
        ledOn = !ledOn;
        // Green blink means success; red blink means failure.
        setStatusLed(recoveryResult, ledOn);
    }

    const bool serialConnected = static_cast<bool>(Serial);
    if (serialConnected && !serialWasConnected) {
        Serial.println();
        Serial.println("Serial monitor connected after recovery completed.");
        Serial.print("InternalFS erase range: ");
        printHex32(kInternalFsStart);
        Serial.print("-");
        printHex32(kInternalFsEnd - 1u);
        Serial.println();
        printFinalResult();
    }
    serialWasConnected = serialConnected;
}
