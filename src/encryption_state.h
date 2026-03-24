#ifndef ENCRYPTION_STATE_H
#define ENCRYPTION_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "structs.h"

struct EncryptionSession {
    bool authenticated;
    uint8_t session_key[16];
    uint8_t session_id[8];
    uint64_t nonce_counter;
    uint64_t last_seen_counter;
    uint64_t replay_window[64];
    uint32_t last_activity;
    uint8_t integrity_failures;
    uint32_t session_start_time;
    uint8_t auth_attempts;
    uint32_t last_auth_time;
    uint8_t client_nonce[16];
    uint8_t server_nonce[16];
    uint8_t pending_server_nonce[16];
    uint32_t server_nonce_time;
};

extern struct SecurityConfig securityConfig;
extern EncryptionSession encryptionSession;
extern bool encryptionInitialized;

#endif
