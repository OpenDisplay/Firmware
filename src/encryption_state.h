#ifndef ENCRYPTION_STATE_H
#define ENCRYPTION_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "structs.h"
#include "nonce_window.h"
#ifdef TARGET_ESP32
#include "mbedtls/ccm.h"
#endif

struct EncryptionSession {
    bool authenticated;
    uint8_t session_key[16];
    uint8_t session_id[8];
#ifdef TARGET_ESP32
    mbedtls_ccm_context ccm_ctx;
    bool is_ccm_ready;
#endif
    uint64_t nonce_counter;
    uint64_t last_seen_counter;
    // Anti-replay: bit i == "counter (last_seen_counter - i) has been consumed".
    // Bit 0 is last_seen_counter itself. The backward window is implicitly
    // OD_NONCE_BACKWARD_BITS - 1; there is no separate window constant to keep
    // in step, and no insertion index to reset. Replaces a uint64_t[64] value
    // ring (512 B -> 32 B). See src/nonce_window.h and
    // docs/PLAN_PHASE1_NONCE_REPLAY_2026-07-26.md Step 1 / Decision B.
    uint64_t replay_bitmap[OD_NONCE_BITMAP_WORDS];
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
