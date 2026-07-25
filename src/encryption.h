#ifndef ENCRYPTION_H
#define ENCRYPTION_H

#include <Arduino.h>
#include <stdint.h>

bool deriveSessionKey(const uint8_t* master_key, const uint8_t* client_nonce,
                      const uint8_t* server_nonce, uint8_t* session_key);
void deriveSessionId(const uint8_t* session_key, const uint8_t* client_nonce,
                     const uint8_t* server_nonce, uint8_t* session_id);
void getAuthDeviceIdBytes(uint8_t* device_id);
bool isEncryptionEnabled();
bool isAuthenticated();
void clearEncryptionSession();
bool checkEncryptionSessionTimeout();
void updateEncryptionSessionActivity();
bool verifyNonceReplay(uint8_t* nonce);
void getCurrentNonce(uint8_t* nonce);
void incrementNonceCounter();
bool handleAuthenticate(uint8_t* data, uint16_t len);
bool decryptCommand(uint8_t* ciphertext, uint16_t ciphertext_len, uint8_t* plaintext,
                    uint16_t* plaintext_len, uint8_t* nonce, uint8_t* auth_tag, uint16_t command_header);
bool encryptResponse(uint8_t* plaintext, uint16_t plaintext_len, uint8_t* ciphertext,
                     uint16_t* ciphertext_len, uint8_t* nonce, uint8_t* auth_tag);
/// Derive the 16-byte TLS-PSK for the LAN TLS channel from the configured master
/// key via AES-CMAC over a fixed KDF label. Returns false if encryption is not
/// configured (no usable master key). The matching PSK IDENTITY is "opendisplay".
bool deriveTlsPsk(uint8_t* psk_out16);

String getChipIdHex();
void secureEraseConfig();
void checkResetPin();

#endif
