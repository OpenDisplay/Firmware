#include "encryption.h"
#include "encryption_state.h"

#ifdef TARGET_NRF
#include <Arduino.h>
#include <Adafruit_nRFCrypto.h>
#include "nrf_cc310/include/crys_aesccm.h"
#include "nrf_cc310/include/ssi_aes.h"
#include "nrf_cc310/include/ssi_aes_defs.h"
#endif
#ifdef TARGET_ESP32
#include <Arduino.h>
#include "mbedtls/aes.h"
#include "mbedtls/ccm.h"
#include "mbedtls/cmac.h"
#include "esp_random.h"
#endif
#include <string.h>

#ifdef TARGET_NRF
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#endif
#ifdef TARGET_ESP32
#include <LittleFS.h>
#include <esp_system.h>
#endif

void sendResponse(uint8_t* response, uint8_t len);
bool aes_cmac(const uint8_t* key, const uint8_t* message, size_t message_len, uint8_t* mac);
bool aes_ecb_encrypt(const uint8_t* key, const uint8_t* input, uint8_t* output);
bool aes_ccm_encrypt(const uint8_t* key, const uint8_t* nonce, size_t nonce_len,
                     const uint8_t* ad, size_t ad_len,
                     const uint8_t* plaintext, size_t plaintext_len,
                     uint8_t* ciphertext, uint8_t* tag, size_t tag_len);
bool aes_ccm_decrypt(const uint8_t* key, const uint8_t* nonce, size_t nonce_len,
                     const uint8_t* ad, size_t ad_len,
                     const uint8_t* ciphertext, size_t ciphertext_len,
                     uint8_t* plaintext, const uint8_t* tag, size_t tag_len);
bool constantTimeCompare(const uint8_t* a, const uint8_t* b, size_t len);
void secure_random(uint8_t* output, size_t len);
void writeSerial(String message, bool newLine = true);

void getAuthDeviceIdBytes(uint8_t* device_id) {
    if (device_id == nullptr) return;
#ifdef TARGET_NRF
    uint32_t id = NRF_FICR->DEVICEID[0];
#elif defined(TARGET_ESP32)
    uint64_t mac = ESP.getEfuseMac();
    uint32_t id = (uint32_t)(mac >> 16);
#else
    uint32_t id = 0x00000001;
#endif
    device_id[0] = (uint8_t)(id >> 24);
    device_id[1] = (uint8_t)(id >> 16);
    device_id[2] = (uint8_t)(id >> 8);
    device_id[3] = (uint8_t)(id);
}

bool deriveSessionKey(const uint8_t* master_key, const uint8_t* client_nonce,
                      const uint8_t* server_nonce, uint8_t* session_key) {
    uint8_t device_id[4];
    getAuthDeviceIdBytes(device_id);
    uint8_t cmac_input[64];
    size_t offset = 0;
    const char* label = "OpenDisplay session";
    memcpy(cmac_input + offset, label, strlen(label));
    offset += strlen(label);
    cmac_input[offset++] = 0x00;
    memcpy(cmac_input + offset, device_id, 4);
    offset += 4;
    memcpy(cmac_input + offset, client_nonce, 16);
    offset += 16;
    memcpy(cmac_input + offset, server_nonce, 16);
    offset += 16;
    cmac_input[offset++] = 0x00;
    cmac_input[offset++] = 0x80;
    uint8_t intermediate[16];
    if (!aes_cmac(master_key, cmac_input, offset, intermediate)) {
        return false;
    }
    uint8_t final_input[16];
    uint64_t counter_be = 1;
    for (int i = 0; i < 8; i++) {
        final_input[i] = (counter_be >> (56 - i * 8)) & 0xFF;
    }
    memcpy(final_input + 8, intermediate, 8);
    if (!aes_ecb_encrypt(master_key, final_input, session_key)) {
        return false;
    }
    return true;
}

void deriveSessionId(const uint8_t* session_key, const uint8_t* client_nonce,
                     const uint8_t* server_nonce, uint8_t* session_id) {
    uint8_t input[32];
    memcpy(input, client_nonce, 16);
    memcpy(input + 16, server_nonce, 16);
    uint8_t cmac_output[16];
    if (aes_cmac(session_key, input, 32, cmac_output)) {
        for (int i = 0; i < 8; i++) {
            session_id[i] = cmac_output[i];
        }
    } else {
        memset(session_id, 0, 8);
    }
}

bool verifyNonceReplay(uint8_t* nonce) {
    if (!encryptionSession.authenticated) return false;
    uint8_t nonce_session_id[8];
    uint64_t nonce_counter = 0;
    memcpy(nonce_session_id, nonce, 8);
    for (int i = 0; i < 8; i++) {
        nonce_counter = (nonce_counter << 8) | nonce[8 + i];
    }
    if (!constantTimeCompare(nonce_session_id, encryptionSession.session_id, 8)) {
        char buf[256];
        snprintf(buf, sizeof(buf), "ERROR: Nonce session_id mismatch\n  Nonce ID: %02X%02X%02X%02X%02X%02X%02X%02X\n  Expected: %02X%02X%02X%02X%02X%02X%02X%02X",
                 nonce_session_id[0], nonce_session_id[1], nonce_session_id[2], nonce_session_id[3],
                 nonce_session_id[4], nonce_session_id[5], nonce_session_id[6], nonce_session_id[7],
                 encryptionSession.session_id[0], encryptionSession.session_id[1], encryptionSession.session_id[2], encryptionSession.session_id[3],
                 encryptionSession.session_id[4], encryptionSession.session_id[5], encryptionSession.session_id[6], encryptionSession.session_id[7]);
        writeSerial(buf, true);
        return false;
    }
    int64_t counter_diff = (int64_t)nonce_counter - (int64_t)encryptionSession.last_seen_counter;
    if (counter_diff < -32 || counter_diff > 32) {
        char buf[128];
        snprintf(buf, sizeof(buf), "ERROR: Nonce counter outside replay window (counter=%llu, last_seen=%llu, diff=%lld)",
                 (unsigned long long)nonce_counter, (unsigned long long)encryptionSession.last_seen_counter, (long long)counter_diff);
        writeSerial(buf, true);
        return false;
    }
    if (nonce_counter <= encryptionSession.last_seen_counter && counter_diff != 0) {
        bool already_seen = false;
        for (int i = 0; i < 64; i++) {
            if (encryptionSession.replay_window[i] == nonce_counter) {
                already_seen = true;
                break;
            }
        }
        if (already_seen) {
            writeSerial("ERROR: Nonce counter already seen (replay detected)", true);
            return false;
        }
    }
    if (nonce_counter > encryptionSession.last_seen_counter) {
        encryptionSession.last_seen_counter = nonce_counter;
    }
    static uint8_t replay_window_index = 0;
    encryptionSession.replay_window[replay_window_index] = nonce_counter;
    replay_window_index = (replay_window_index + 1) % 64;
    return true;
}

void getCurrentNonce(uint8_t* nonce) {
    if (!encryptionSession.authenticated) {
        memset(nonce, 0, 16);
        return;
    }
    memcpy(nonce, encryptionSession.session_id, 8);
    uint64_t counter = encryptionSession.nonce_counter;
    for (int i = 0; i < 8; i++) {
        nonce[8 + i] = (counter >> (56 - i * 8)) & 0xFF;
    }
}

void incrementNonceCounter() {
    if (encryptionSession.authenticated) {
        encryptionSession.nonce_counter++;
    }
}

bool isEncryptionEnabled() {
    return (securityConfig.encryption_enabled == 1) &&
           (securityConfig.encryption_key[0] != 0 ||
            memcmp(securityConfig.encryption_key, securityConfig.encryption_key + 1, 15) != 0);
}

bool isAuthenticated() {
    return encryptionSession.authenticated &&
           (encryptionSession.session_start_time > 0) &&
           checkEncryptionSessionTimeout();
}

void clearEncryptionSession() {
    memset(encryptionSession.session_key, 0, 16);
    memset(encryptionSession.client_nonce, 0, 16);
    memset(encryptionSession.server_nonce, 0, 16);
    memset(encryptionSession.pending_server_nonce, 0, 16);
    encryptionSession.authenticated = false;
    encryptionSession.nonce_counter = 0;
    encryptionSession.last_seen_counter = 0;
    encryptionSession.integrity_failures = 0;
    encryptionSession.session_start_time = 0;
    encryptionSession.last_activity = 0;
    encryptionSession.auth_attempts = 0;
    encryptionSession.server_nonce_time = 0;
    memset(encryptionSession.replay_window, 0, sizeof(encryptionSession.replay_window));
    writeSerial("Encryption session cleared");
}

bool checkEncryptionSessionTimeout() {
    if (!encryptionSession.authenticated) return false;
    if (securityConfig.session_timeout_seconds == 0) return true;
    uint32_t currentTime = millis() / 1000;
    uint32_t sessionAge = currentTime - (encryptionSession.session_start_time / 1000);
    if (sessionAge >= securityConfig.session_timeout_seconds) {
        writeSerial("Encryption session timeout (" + String(sessionAge) + "s >= " +
                   String(securityConfig.session_timeout_seconds) + "s)");
        clearEncryptionSession();
        return false;
    }
    return true;
}

void updateEncryptionSessionActivity() {
    if (encryptionSession.authenticated) {
        encryptionSession.last_activity = millis();
    }
}

bool constantTimeCompare(const uint8_t* a, const uint8_t* b, size_t len) {
    uint8_t result = 0;
    for (size_t i = 0; i < len; i++) {
        result |= a[i] ^ b[i];
    }
    return result == 0;
}

#ifdef TARGET_ESP32
bool aes_cmac(const uint8_t* key, const uint8_t* message, size_t message_len, uint8_t* mac) {
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t* cipher_info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
    if (cipher_info == NULL) {
        writeSerial("ERROR: Failed to get cipher info for AES-128-ECB");
        return false;
    }
    mbedtls_cipher_init(&ctx);
    if (mbedtls_cipher_setup(&ctx, cipher_info) != 0) {
        writeSerial("ERROR: Failed to setup cipher");
        mbedtls_cipher_free(&ctx);
        return false;
    }
    if (mbedtls_cipher_cmac_starts(&ctx, key, 128) != 0) {
        writeSerial("ERROR: Failed to start CMAC");
        mbedtls_cipher_free(&ctx);
        return false;
    }
    if (mbedtls_cipher_cmac_update(&ctx, message, message_len) != 0) {
        writeSerial("ERROR: Failed to update CMAC");
        mbedtls_cipher_free(&ctx);
        return false;
    }
    if (mbedtls_cipher_cmac_finish(&ctx, mac) != 0) {
        writeSerial("ERROR: Failed to finish CMAC");
        mbedtls_cipher_free(&ctx);
        return false;
    }
    mbedtls_cipher_free(&ctx);
    return true;
}

bool aes_ecb_encrypt(const uint8_t* key, const uint8_t* input, uint8_t* output) {
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    if (mbedtls_aes_setkey_enc(&aes, key, 128) != 0) {
        writeSerial("ERROR: Failed to set AES key");
        mbedtls_aes_free(&aes);
        return false;
    }
    if (mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, input, output) != 0) {
        writeSerial("ERROR: Failed to encrypt with AES-ECB");
        mbedtls_aes_free(&aes);
        return false;
    }
    mbedtls_aes_free(&aes);
    return true;
}

bool aes_ccm_encrypt(const uint8_t* key, const uint8_t* nonce, size_t nonce_len,
                     const uint8_t* ad, size_t ad_len,
                     const uint8_t* plaintext, size_t plaintext_len,
                     uint8_t* ciphertext, uint8_t* tag, size_t tag_len) {
    mbedtls_ccm_context ccm;
    mbedtls_ccm_init(&ccm);
    if (mbedtls_ccm_setkey(&ccm, MBEDTLS_CIPHER_ID_AES, key, 128) != 0) {
        writeSerial("ERROR: Failed to set CCM key");
        mbedtls_ccm_free(&ccm);
        return false;
    }
    int ret = mbedtls_ccm_encrypt_and_tag(&ccm, plaintext_len, nonce, nonce_len,
                                          ad, ad_len, plaintext, ciphertext, tag, tag_len);
    mbedtls_ccm_free(&ccm);
    if (ret != 0) {
        writeSerial("ERROR: CCM encrypt failed: " + String((int)ret));
        return false;
    }
    return true;
}

bool aes_ccm_decrypt(const uint8_t* key, const uint8_t* nonce, size_t nonce_len,
                     const uint8_t* ad, size_t ad_len,
                     const uint8_t* ciphertext, size_t ciphertext_len,
                     uint8_t* plaintext, const uint8_t* tag, size_t tag_len) {
    mbedtls_ccm_context ccm;
    mbedtls_ccm_init(&ccm);
    if (mbedtls_ccm_setkey(&ccm, MBEDTLS_CIPHER_ID_AES, key, 128) != 0) {
        writeSerial("ERROR: Failed to set CCM key");
        mbedtls_ccm_free(&ccm);
        return false;
    }
    if (nonce_len < 7 || nonce_len > 13) {
        writeSerial("ERROR: Invalid CCM nonce length (must be 7-13 bytes)");
        mbedtls_ccm_free(&ccm);
        return false;
    }
    if (tag_len != 4 && tag_len != 6 && tag_len != 8 && tag_len != 10 && tag_len != 12 && tag_len != 14 && tag_len != 16) {
        writeSerial("ERROR: Invalid CCM tag length (must be 4, 6, 8, 10, 12, 14, or 16 bytes)");
        mbedtls_ccm_free(&ccm);
        return false;
    }
    if (ciphertext_len == 0) {
        writeSerial("ERROR: CCM ciphertext length is 0 (must be at least 1 byte)");
        mbedtls_ccm_free(&ccm);
        return false;
    }
    int ret = mbedtls_ccm_auth_decrypt(&ccm, ciphertext_len, nonce, nonce_len,
                                       ad, ad_len, ciphertext, plaintext, tag, tag_len);
    mbedtls_ccm_free(&ccm);
    if (ret != 0) {
        char err_buf[128];
        snprintf(err_buf, sizeof(err_buf), "ERROR: CCM decrypt failed: %d (ciphertext_len=%zu, nonce_len=%zu, tag_len=%zu)",
                 ret, ciphertext_len, nonce_len, tag_len);
        writeSerial(err_buf);
        if (ret == -15) {
            writeSerial("ERROR: MBEDTLS_ERR_CCM_BAD_INPUT - invalid input parameters");
        }
        return false;
    }
    return true;
}

void secure_random(uint8_t* output, size_t len) {
    esp_fill_random(output, len);
}
#else
static bool cc310_initialized = false;

static bool init_cc310() {
    if (cc310_initialized) return true;
    if (!nRFCrypto.begin()) {
        writeSerial("ERROR: Failed to initialize CryptoCell CC310");
        return false;
    }
    cc310_initialized = true;
    writeSerial("CryptoCell CC310 initialized successfully");
    return true;
}

bool aes_cmac(const uint8_t* key, const uint8_t* message, size_t message_len, uint8_t* mac) {
    if (!init_cc310()) return false;
    SaSiAesUserContext_t ctx;
    SaSiAesUserKeyData_t keyData;
    keyData.pKey = (uint8_t*)key;
    keyData.keySize = 16;
    SaSiError_t err = SaSi_AesInit(&ctx, SASI_AES_ENCRYPT, SASI_AES_MODE_CMAC, SASI_AES_PADDING_NONE);
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesInit (CMAC) failed: 0x" + String((unsigned long)err, HEX));
        return false;
    }
    err = SaSi_AesSetKey(&ctx, SASI_AES_USER_KEY, &keyData, sizeof(keyData));
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesSetKey (CMAC) failed: 0x" + String((unsigned long)err, HEX));
        SaSi_AesFree(&ctx);
        return false;
    }
    size_t block_len = 0;
    size_t finish_offset = 0;
    size_t finish_len = message_len;
    if (message_len > 16) {
        if (message_len % 16 == 0) block_len = message_len - 16;
        else block_len = (message_len / 16) * 16;
        finish_offset = block_len;
        finish_len = message_len - block_len;
    }
    if (block_len > 0) {
        err = SaSi_AesBlock(&ctx, (uint8_t*)message, block_len, NULL);
        if (err != SASI_OK) {
            writeSerial("ERROR: SaSi_AesBlock (CMAC) failed: 0x" + String((unsigned long)err, HEX));
            SaSi_AesFree(&ctx);
            return false;
        }
    }
    size_t mac_size = 16;
    err = SaSi_AesFinish(&ctx, finish_len, (uint8_t*)message + finish_offset, finish_len, mac, &mac_size);
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesFinish (CMAC) failed: 0x" + String((unsigned long)err, HEX));
        SaSi_AesFree(&ctx);
        return false;
    }
    SaSi_AesFree(&ctx);
    return true;
}

bool aes_ecb_encrypt(const uint8_t* key, const uint8_t* input, uint8_t* output) {
    if (!init_cc310()) return false;
    SaSiAesUserContext_t ctx;
    SaSiAesUserKeyData_t keyData;
    keyData.pKey = (uint8_t*)key;
    keyData.keySize = 16;
    SaSiError_t err = SaSi_AesInit(&ctx, SASI_AES_ENCRYPT, SASI_AES_MODE_ECB, SASI_AES_PADDING_NONE);
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesInit (ECB) failed: 0x" + String((unsigned long)err, HEX));
        return false;
    }
    err = SaSi_AesSetKey(&ctx, SASI_AES_USER_KEY, &keyData, sizeof(keyData));
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesSetKey (ECB) failed: 0x" + String((unsigned long)err, HEX));
        SaSi_AesFree(&ctx);
        return false;
    }
    size_t out_size = 16;
    err = SaSi_AesFinish(&ctx, 16, (uint8_t*)input, 16, output, &out_size);
    if (err != SASI_OK) {
        writeSerial("ERROR: SaSi_AesFinish (ECB) failed: 0x" + String((unsigned long)err, HEX));
        SaSi_AesFree(&ctx);
        return false;
    }
    SaSi_AesFree(&ctx);
    return true;
}

bool aes_ccm_encrypt(const uint8_t* key, const uint8_t* nonce, size_t nonce_len,
                     const uint8_t* ad, size_t ad_len,
                     const uint8_t* plaintext, size_t plaintext_len,
                     uint8_t* ciphertext, uint8_t* tag, size_t tag_len) {
    if (!init_cc310()) return false;
    if (nonce_len < 7 || nonce_len > 13) {
        writeSerial("ERROR: Invalid CCM nonce length: " + String((int)nonce_len));
        return false;
    }
    if (tag_len < 4 || tag_len > 16 || (tag_len % 2 != 0)) {
        writeSerial("ERROR: Invalid CCM tag length: " + String((int)tag_len));
        return false;
    }
    CRYS_AESCCM_Key_t ccmKey;
    memset(ccmKey, 0, sizeof(ccmKey));
    memcpy(ccmKey, key, 16);
    CRYS_AESCCM_Mac_Res_t macRes;
    memset(macRes, 0, sizeof(macRes));
    CRYSError_t err = CRYS_AESCCM(
        SASI_AES_ENCRYPT, ccmKey, CRYS_AES_Key128BitSize,
        (uint8_t*)nonce, (uint8_t)nonce_len,
        (uint8_t*)ad, (uint32_t)ad_len,
        (uint8_t*)plaintext, (uint32_t)plaintext_len,
        ciphertext, (uint8_t)tag_len, macRes);
    if (err != CRYS_OK) {
        writeSerial("ERROR: CRYS_AESCCM (encrypt) failed: 0x" + String((unsigned long)err, HEX));
        return false;
    }
    memcpy(tag, macRes, tag_len);
    return true;
}

bool aes_ccm_decrypt(const uint8_t* key, const uint8_t* nonce, size_t nonce_len,
                     const uint8_t* ad, size_t ad_len,
                     const uint8_t* ciphertext, size_t ciphertext_len,
                     uint8_t* plaintext, const uint8_t* tag, size_t tag_len) {
    if (!init_cc310()) return false;
    if (nonce_len < 7 || nonce_len > 13) {
        writeSerial("ERROR: Invalid CCM nonce length: " + String((int)nonce_len));
        return false;
    }
    if (tag_len < 4 || tag_len > 16 || (tag_len % 2 != 0)) {
        writeSerial("ERROR: Invalid CCM tag length: " + String((int)tag_len));
        return false;
    }
    CRYS_AESCCM_Key_t ccmKey;
    memset(ccmKey, 0, sizeof(ccmKey));
    memcpy(ccmKey, key, 16);
    CRYS_AESCCM_Mac_Res_t macRes;
    memset(macRes, 0, sizeof(macRes));
    memcpy(macRes, tag, tag_len);
    CRYSError_t err = CRYS_AESCCM(
        SASI_AES_DECRYPT, ccmKey, CRYS_AES_Key128BitSize,
        (uint8_t*)nonce, (uint8_t)nonce_len,
        (uint8_t*)ad, (uint32_t)ad_len,
        (uint8_t*)ciphertext, (uint32_t)ciphertext_len,
        plaintext, (uint8_t)tag_len, macRes);
    if (err != CRYS_OK) {
        writeSerial("ERROR: CRYS_AESCCM (decrypt) failed: 0x" + String((unsigned long)err, HEX));
        return false;
    }
    return true;
}

void secure_random(uint8_t* output, size_t len) {
    if (!init_cc310()) {
        writeSerial("WARNING: CC310 not initialized, using non-secure random");
        for (size_t i = 0; i < len; i++) output[i] = random(256);
        return;
    }
    if (!nRFCrypto.Random.generate(output, (uint16_t)len)) {
        writeSerial("ERROR: CC310 RNG failed, using non-secure fallback");
        for (size_t i = 0; i < len; i++) output[i] = random(256);
    }
}
#endif

bool handleAuthenticate(uint8_t* data, uint16_t len) {
    if (!isEncryptionEnabled()) {
        uint8_t response[] = {0x00, 0x50, 0x03};
        sendResponse(response, sizeof(response));
        return false;
    }
    uint32_t currentTime = millis();
    if (encryptionSession.last_auth_time > 0) {
        uint32_t timeSinceLastAuth = (currentTime - encryptionSession.last_auth_time) / 1000;
        if (timeSinceLastAuth < 60) {
            if (encryptionSession.auth_attempts >= 10) {
                uint8_t response[] = {0x00, 0x50, 0x04};
                sendResponse(response, sizeof(response));
                return false;
            }
        } else {
            encryptionSession.auth_attempts = 0;
        }
    }
    encryptionSession.auth_attempts++;
    encryptionSession.last_auth_time = currentTime;
    if (len == 1 && data[0] == 0x00) {
        if (encryptionSession.authenticated && checkEncryptionSessionTimeout()) {
            writeSerial("New authentication requested, clearing existing session");
            clearEncryptionSession();
        }
        secure_random(encryptionSession.pending_server_nonce, 16);
        encryptionSession.server_nonce_time = currentTime;
        uint8_t device_id[4];
        getAuthDeviceIdBytes(device_id);
        uint8_t response[2 + 1 + 16 + 4];
        response[0] = 0x00; response[1] = 0x50; response[2] = 0x00;
        memcpy(response + 3, encryptionSession.pending_server_nonce, 16);
        memcpy(response + 19, device_id, 4);
        sendResponse(response, sizeof(response));
        writeSerial("Authentication challenge sent");
        return false;
    }
    if (len == 32) {
        uint8_t client_nonce[16];
        uint8_t challenge_response[16];
        memcpy(client_nonce, data, 16);
        memcpy(challenge_response, data + 16, 16);
        if (currentTime - encryptionSession.server_nonce_time > 30000) {
            writeSerial("ERROR: Server nonce expired");
            uint8_t response[] = {0x00, 0x50, 0xFF};
            sendResponse(response, sizeof(response));
            return false;
        }
        uint8_t device_id[4];
        getAuthDeviceIdBytes(device_id);
        uint8_t challenge_input[36];
        memcpy(challenge_input, encryptionSession.pending_server_nonce, 16);
        memcpy(challenge_input + 16, client_nonce, 16);
        memcpy(challenge_input + 32, device_id, 4);
        uint8_t expected_response[16];
        if (!aes_cmac(securityConfig.encryption_key, challenge_input, 36, expected_response)) {
            writeSerial("ERROR: Failed to compute expected CMAC");
            uint8_t response[] = {0x00, 0x50, 0xFF};
            sendResponse(response, sizeof(response));
            return false;
        }
        if (!constantTimeCompare(challenge_response, expected_response, 16)) {
            writeSerial("ERROR: Authentication failed (wrong key)");
            uint8_t response[] = {0x00, 0x50, 0x01};
            sendResponse(response, sizeof(response));
            memset(encryptionSession.pending_server_nonce, 0, 16);
            return false;
        }
        memcpy(encryptionSession.client_nonce, client_nonce, 16);
        memcpy(encryptionSession.server_nonce, encryptionSession.pending_server_nonce, 16);
        if (!deriveSessionKey(securityConfig.encryption_key, client_nonce,
                              encryptionSession.pending_server_nonce, encryptionSession.session_key)) {
            writeSerial("ERROR: Failed to derive session key");
            uint8_t response[] = {0x00, 0x50, 0xFF};
            sendResponse(response, sizeof(response));
            return false;
        }
        deriveSessionId(encryptionSession.session_key, client_nonce,
                        encryptionSession.server_nonce, encryptionSession.session_id);
        bool session_id_valid = false;
        for (int i = 0; i < 8; i++) {
            if (encryptionSession.session_id[i] != 0) { session_id_valid = true; break; }
        }
        if (!session_id_valid) {
            writeSerial("ERROR: Session ID is invalid (all zeros)!");
            uint8_t response[] = {0x00, 0x50, 0xFF};
            sendResponse(response, sizeof(response));
            return false;
        }
        encryptionSession.authenticated = true;
        encryptionSession.nonce_counter = 0;
        encryptionSession.last_seen_counter = 0;
        encryptionSession.integrity_failures = 0;
        encryptionSession.session_start_time = currentTime;
        encryptionSession.last_activity = currentTime;
        memset(encryptionSession.replay_window, 0, sizeof(encryptionSession.replay_window));
        memset(encryptionSession.pending_server_nonce, 0, 16);
        encryptionSession.server_nonce_time = 0;
        uint8_t server_response[16];
        uint8_t server_input[36];
        memcpy(server_input, encryptionSession.server_nonce, 16);
        memcpy(server_input + 16, client_nonce, 16);
        memcpy(server_input + 32, device_id, 4);
        if (!aes_cmac(encryptionSession.session_key, server_input, 36, server_response)) {
            writeSerial("ERROR: Failed to compute server response");
            clearEncryptionSession();
            uint8_t response[] = {0x00, 0x50, 0xFF};
            sendResponse(response, sizeof(response));
            return false;
        }
        uint8_t response[2 + 1 + 16];
        response[0] = 0x00; response[1] = 0x50; response[2] = 0x00;
        memcpy(response + 3, server_response, 16);
        sendResponse(response, sizeof(response));
        writeSerial("Authentication successful, session established");
        return true;
    }
    writeSerial("ERROR: Invalid authentication request format (len=" + String(len) + ")");
    uint8_t response[] = {0x00, 0x50, 0xFF};
    sendResponse(response, sizeof(response));
    return false;
}

bool decryptCommand(uint8_t* ciphertext, uint16_t ciphertext_len, uint8_t* plaintext,
                    uint16_t* plaintext_len, uint8_t* nonce_full, uint8_t* auth_tag, uint16_t command_header) {
    if (!isAuthenticated()) return false;
    if (!verifyNonceReplay(nonce_full)) {
        encryptionSession.integrity_failures++;
        if (encryptionSession.integrity_failures >= 3) {
            writeSerial("Too many integrity failures, clearing session");
            clearEncryptionSession();
        }
        return false;
    }
    uint16_t encrypted_len = ciphertext_len;
    if (encrypted_len > 512) {
        writeSerial("ERROR: Encrypted payload too large");
        return false;
    }
    uint8_t nonce[13];
    memcpy(nonce, nonce_full + 3, 13);
    uint8_t ad[2];
    ad[0] = (command_header >> 8) & 0xFF;
    ad[1] = command_header & 0xFF;
    if (encrypted_len == 0) {
        writeSerial("ERROR: Encrypted payload is 0 bytes (should include length byte)");
        return false;
    }
    static uint8_t decrypted_with_length[512];
    bool success = aes_ccm_decrypt(encryptionSession.session_key, nonce, 13,
                                   ad, 2, ciphertext, encrypted_len,
                                   decrypted_with_length, auth_tag, 12);
    if (success) {
        uint8_t payload_length = decrypted_with_length[0];
        if (payload_length > encrypted_len - 1) {
            writeSerial("ERROR: Invalid payload length in decrypted data");
            return false;
        }
        if (payload_length > 0) memcpy(plaintext, decrypted_with_length + 1, payload_length);
        *plaintext_len = payload_length;
        encryptionSession.integrity_failures = 0;
        updateEncryptionSessionActivity();
        return true;
    }
    encryptionSession.integrity_failures++;
    if (encryptionSession.integrity_failures >= 3) {
        writeSerial("Too many integrity failures, clearing session");
        clearEncryptionSession();
    }
    return false;
}

bool encryptResponse(uint8_t* plaintext, uint16_t plaintext_len, uint8_t* ciphertext,
                     uint16_t* ciphertext_len, uint8_t* nonce, uint8_t* auth_tag) {
    if (!isAuthenticated()) return false;
    getCurrentNonce(nonce);
    incrementNonceCounter();
    uint8_t nonce_ccm[13];
    memcpy(nonce_ccm, nonce + 3, 13);
    uint8_t ad[2] = {plaintext[0], plaintext[1]};
    static uint8_t payload_with_length[513];
    uint16_t payload_len = plaintext_len - 2;
    payload_with_length[0] = payload_len & 0xFF;
    if (payload_len > 0) memcpy(payload_with_length + 1, plaintext + 2, payload_len);
    uint16_t total_payload_len = 1 + payload_len;
    bool success = aes_ccm_encrypt(encryptionSession.session_key, nonce_ccm, 13,
                                   ad, 2, payload_with_length, total_payload_len,
                                   ciphertext + 2 + 16, auth_tag, 12);
    if (!success) return false;
    uint8_t nonce_copy[16];
    memcpy(nonce_copy, nonce, 16);
    ciphertext[0] = plaintext[0];
    ciphertext[1] = plaintext[1];
    memcpy(ciphertext + 2, nonce_copy, 16);
    memcpy(ciphertext + 2 + 16 + total_payload_len, auth_tag, 12);
    *ciphertext_len = 2 + 16 + total_payload_len + 12;
    updateEncryptionSessionActivity();
    return true;
}

static constexpr const char* CONFIG_FILE_PATH_LOCAL = "/config.bin";

void reboot();

String getChipIdHex() {
#ifdef TARGET_NRF
    uint32_t id1 = NRF_FICR->DEVICEID[0];
    uint32_t id2 = NRF_FICR->DEVICEID[1];
    uint32_t last3Bytes = id2 & 0xFFFFFF;
    String hexId = String(last3Bytes, HEX);
    hexId.toUpperCase();
    while (hexId.length() < 6) {
        hexId = "0" + hexId;
    }
    writeSerial("Chip ID: " + String(id1, HEX) + String(id2, HEX), true);
    writeSerial("Using last 3 bytes: " + hexId, true);
    return hexId;
#endif
#ifdef TARGET_ESP32
    uint64_t macAddress = ESP.getEfuseMac();
    uint32_t chipId = (uint32_t)(macAddress >> 24) & 0xFFFFFF;
    String hexId = String(chipId, HEX);
    hexId.toUpperCase();
    while (hexId.length() < 6) {
        hexId = "0" + hexId;
    }
    writeSerial("Chip ID: " + String(chipId, HEX), true);
    writeSerial("Using chip ID: " + hexId, true);
    return hexId;
#endif
    return "";
}

void secureEraseConfig() {
    writeSerial("=== SECURE ERASE CONFIG ===", true);
    static uint8_t zeroBuffer[512];
    memset(zeroBuffer, 0, sizeof(zeroBuffer));

#ifdef TARGET_NRF
    if (InternalFS.exists(CONFIG_FILE_PATH_LOCAL)) {
        File file = InternalFS.open(CONFIG_FILE_PATH_LOCAL, FILE_O_WRITE);
        if (file) {
            size_t fileSize = file.size();
            file.seek(0);
            size_t written = 0;
            while (written < fileSize) {
                size_t toWrite = (fileSize - written < sizeof(zeroBuffer)) ? (fileSize - written) : sizeof(zeroBuffer);
                file.write(zeroBuffer, toWrite);
                written += toWrite;
            }
            file.close();
            writeSerial("Config file securely erased (" + String(written) + " bytes)", true);
        }
        InternalFS.remove(CONFIG_FILE_PATH_LOCAL);
    }
#elif defined(TARGET_ESP32)
    if (LittleFS.exists(CONFIG_FILE_PATH_LOCAL)) {
        File file = LittleFS.open(CONFIG_FILE_PATH_LOCAL, FILE_WRITE);
        if (file) {
            size_t fileSize = file.size();
            file.seek(0);
            size_t written = 0;
            while (written < fileSize) {
                size_t toWrite = (fileSize - written < sizeof(zeroBuffer)) ? (fileSize - written) : sizeof(zeroBuffer);
                file.write(zeroBuffer, toWrite);
                written += toWrite;
            }
            file.close();
            writeSerial("Config file securely erased (" + String(written) + " bytes)", true);
        }
        LittleFS.remove(CONFIG_FILE_PATH_LOCAL);
    }
#endif
    writeSerial("Config securely erased", true);
}

void checkResetPin() {
    if (!(securityConfig.flags & SECURITY_FLAG_RESET_PIN_ENABLED)) {
        return;
    }

    uint8_t pin = securityConfig.reset_pin;
    bool polarity = (securityConfig.flags & SECURITY_FLAG_RESET_PIN_POLARITY) != 0;
    bool pullup = (securityConfig.flags & SECURITY_FLAG_RESET_PIN_PULLUP) != 0;
    bool pulldown = (securityConfig.flags & SECURITY_FLAG_RESET_PIN_PULLDOWN) != 0;

    writeSerial("Checking reset pin " + String(pin) + " (polarity: " + String(polarity ? "HIGH" : "LOW") +
                ", pullup: " + String(pullup) + ", pulldown: " + String(pulldown) + ")", true);

#ifdef TARGET_ESP32
    pinMode(pin, INPUT);
    if (pullup) {
        pinMode(pin, INPUT_PULLUP);
    } else if (pulldown) {
        pinMode(pin, INPUT_PULLDOWN);
    }
#elif defined(TARGET_NRF)
    pinMode(pin, INPUT);
    if (pullup) {
        pinMode(pin, INPUT_PULLUP);
    }
#endif

    delay(100);
    bool pinState = digitalRead(pin);

    if (pinState == polarity) {
        writeSerial("Reset pin triggered! Securely erasing config and rebooting...", true);
        secureEraseConfig();
        delay(100);
        reboot();
    } else {
        writeSerial("Reset pin not triggered (state: " + String(pinState ? "HIGH" : "LOW") + ")", true);
    }
}
