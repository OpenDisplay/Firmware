#include "wifi_service.h"

#ifdef OPENDISPLAY_HAS_WIFI

#include "communication.h"
#include "encryption.h"
#include "structs.h"
#include "od_log.h"
#include "ble_transport.h"
#include "link_owner.h"
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_event.h>       // raw handler for WIFI_EVENT_STA_BSS_RSSI_LOW (no Arduino event id)
#include <esp_heap_caps.h>
#include <string.h>

#include "mbedtls/ssl.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/ssl_ciphersuites.h"
#include "mbedtls/platform.h"   // mbedtls_platform_set_calloc_free (MBEDTLS_PLATFORM_MEMORY is on via esp_config.h)

#ifndef COMM_MODE_WIFI
#define COMM_MODE_WIFI (1 << 2)
#endif

extern struct GlobalConfig globalConfig;
extern char wifiSsid[33];
extern char wifiPassword[33];
extern uint8_t wifiEncryptionType;
extern bool wifiConfigured;
extern bool wifiConnected;
extern bool wifiInitialized;
extern uint16_t wifiServerPort;
extern WiFiServer wifiServer;
extern WiFiClient wifiClient;
extern bool wifiServerConnected;
// tcpReceiveBuffer / tcpReceiveBufferPos are declared in wifi_service.h (included
// above) so the pointer type is checked against its definition in main.h.
extern uint8_t msd_payload[16];

// Logging here is printf-style through od_log_*, like the rest of the firmware.
// The String-taking lanLog adapter this replaces existed only to avoid reflowing
// the call sites; with them reflowed it bought nothing and cost a heap allocation
// per line (Arduino String has SSO up to 14 chars, and essentially every line here
// is longer), plus a startsWith() level sniff. Calling the macros directly also
// makes OD_LOG_LEVEL a real compile-time gate: a gated-out line no longer formats
// its arguments. The EVENT-CONTEXT RULE below still applies -- these are cheaper
// now, not free.

// Command origin marker (F4): the shared dispatcher (imageDataWritten) reads this
// to decide whether to run the app-layer AES-CCM gate. Defined in communication.cpp;
// enum CommandOrigin comes from communication.h.
// ORIGIN_LAN_TLS frames are already secured by TLS, so CCM MUST be bypassed.
extern volatile uint8_t g_commandOrigin;

// Roam gating: never re-associate (or full-channel scan, which steals the shared radio
// from BLE under coex) while a transfer is in flight. transferActive() covers all three
// transfer types and is declared in display_service.h.
bool transferActive(void);

String getChipIdHex();
static void lanBeginConnect(void);   // defined with the roaming / RTC AP-cache block below
uint8_t getFirmwareMajor();
uint8_t getFirmwareMinor();

// imageDataWritten + its opaque parameter typedefs come from communication.h.

// ------------------------------------------------------------------ TLS-PSK ---
// One TLS session at a time, driven cooperatively from handleWiFiServer(). The
// PSK identity string "opendisplay" must match the py-opendisplay client; the
// PSK bytes come from deriveTlsPsk() (AES-CMAC over the master key).
static const char* kTlsPskIdentity = "opendisplay";
static const int kTlsCiphersuites[] = { MBEDTLS_TLS_ECDHE_PSK_WITH_AES_128_CBC_SHA256, 0 };

static bool tlsMode = false;            // true when the ACTIVE channel is TLS-PSK
static bool tlsInited = false;          // mbedTLS config objects built once
static bool tlsSessionActive = false;   // an mbedtls_ssl_context is live for wifiClient
static bool tlsHandshakeDone = false;
static uint8_t tlsPsk[16];

static mbedtls_ssl_context   tlsSsl;
static mbedtls_ssl_config    tlsConf;
static mbedtls_ctr_drbg_context tlsDrbg;
static mbedtls_entropy_context  tlsEntropy;

// lastLanActivityMs is GONE. LAN used to keep its own activity clock, stamped at
// connect, handshake completion, raw bytes read and frame dispatch, and checked
// inline in handleWiFiServer(). R4 requires the SAME definition of activity on
// every transport -- a recognised command from the owner, excluding refresh -- and
// two clocks implementing one rule is how they drift. The shared clock in
// link_owner.cpp is now the only one; serviceIdleTimeout() in main.cpp enforces it
// for both transports, with each keeping its own constant (BLE 120 s local,
// LAN OD_LAN_READ_TIMEOUT_S 30 s from the wire header).
// Epoch of the current LAN session's ownership claim, 0 when this session does not
// own the slot. Kept so the release matches the FULL identity: releasing on
// transport alone would let a stale LAN teardown free a slot a newer session (or a
// BLE client) had since taken.
static uint16_t s_lanEpoch = 0;

// WiFi.BSSIDstr() returns a String by value; format the raw bytes instead. Matches
// its output, including the empty string when the driver has no BSSID yet.
static void lanFormatBssid(char* out, size_t outSize, const uint8_t* b) {
    if (b == nullptr) { if (outSize) out[0] = '\0'; return; }
    snprintf(out, outSize, "%02X:%02X:%02X:%02X:%02X:%02X",
             b[0], b[1], b[2], b[3], b[4], b[5]);
}

static uint16_t lanBasePort(void) {
    return (wifiServerPort != 0) ? wifiServerPort : (uint16_t)OD_LAN_TCP_PORT;
}
// Active LAN port: plaintext -> base; TLS -> base+1 (derived, no config field).
uint16_t lanActivePort(void) {
    uint16_t base = lanBasePort();
    return isEncryptionEnabled() ? (uint16_t)(base + 1) : base;
}

bool lanTlsEnabled(void) { return isEncryptionEnabled(); }

// mbedTLS BIO shims over the accepted WiFiClient (non-blocking cooperative model).
static int tls_bio_send(void* ctx, const unsigned char* buf, size_t len) {
    WiFiClient* c = static_cast<WiFiClient*>(ctx);
    if (c == nullptr || !c->connected()) return MBEDTLS_ERR_NET_CONN_RESET;
    int w = c->write(buf, len);
    if (w <= 0) return MBEDTLS_ERR_SSL_WANT_WRITE;
    return w;
}
static int tls_bio_recv(void* ctx, unsigned char* buf, size_t len) {
    WiFiClient* c = static_cast<WiFiClient*>(ctx);
    if (c == nullptr || !c->connected()) return MBEDTLS_ERR_NET_CONN_RESET;
    if (c->available() <= 0) return MBEDTLS_ERR_SSL_WANT_READ;
    int r = c->read(buf, len);
    if (r <= 0) return MBEDTLS_ERR_SSL_WANT_READ;
    return r;
}

// mbedTLS on this SDK is built with CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC=y, so every TLS
// allocation comes from internal DRAM only -- PSRAM is never eligible, however much of
// it the board has. ssl_setup alone needs two ~16.4 KB contiguous blocks there
// (CONFIG_MBEDTLS_SSL_MAX_CONTENT_LEN=16384, asymmetric length not set), which is the
// dominant failure mode once WiFi + BLE coex and the static buffers have taken their cut.
// Append this to any TLS failure so the log says whether it was OOM and by how much.
// Writes into a caller-supplied buffer rather than returning a String: these are
// failure paths, so one stack buffer at the call site costs nothing and keeps the
// allocator out of a path that may be failing *because* of the allocator.
// OD_TLS_FAIL_NOTE_LEN covers the longest form: two 10-digit sizes plus the text.
#define OD_TLS_FAIL_NOTE_LEN 80
static void tlsFailNote(int ret, char* out, size_t outSize) {
    const unsigned freeB = (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const unsigned maxB  = (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    if (ret < 0) {
        snprintf(out, outSize, " (ret=-0x%x, internal free=%u, largest block=%u)",
                 (unsigned)(-ret), freeB, maxB);
    } else {
        snprintf(out, outSize, " (ret=%d, internal free=%u, largest block=%u)",
                 ret, freeB, maxB);
    }
}

// -------------------------------------------- TLS record pre-reservation ---
// mbedtls_ssl_setup() allocates TWO record buffers of CONFIG_MBEDTLS_SSL_MAX_CONTENT_LEN
// (16384) + record overhead, i.e. ~16.7 KB EACH, and CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC=y
// forces both into internal DRAM. Both values are baked into the prebuilt SDK libs, so
// neither the size nor the region can be changed from here.
//
// That fails in the field even with plenty of free RAM, because what matters is the
// LARGEST CONTIGUOUS block, not the total. Observed on hardware:
//   ssl_setup failed (ret=-0x7f00, internal free=51144, largest block=31732)
// 51 KB free, yet the first 16.7 KB allocation leaves only ~15 KB in that block and the
// second has nowhere to go -- short by under 2 KB.
//
// Worse, ssl_setup runs on EVERY LAN client connect and ssl_free releases on every
// disconnect, so TLS cycles 33 KB through the heap per connection: the odds degrade
// the longer the device stays up. This file's String-based logging used to churn
// small blocks in between, widening the window; that is gone as of the printf-style
// conversion, but the 33 KB cycle itself is what the reservation below removes.
//
// Fix: reserve the two slots ONCE at boot -- from setup(), right after full_config_init()
// and BEFORE BleTransport::begin()/initWiFi() take their ~100 KB -- when the heap is still
// contiguous, then serve mbedTLS from them via its allocator hook. This does not raise
// peak usage (the buffers are needed whenever TLS runs); it moves the allocation to the
// one moment where success is guaranteed, and recycling the slots stops TLS fragmenting
// the heap for lwIP/BLE.
//
// Only record-sized requests come from the pool. Small handshake/bignum allocations (and
// encryption.cpp's CCM/CMAC contexts) still go to the normal internal heap, so their
// behavior and placement are unchanged.
//
// Single-task by design: every mbedTLS call here runs on the loop task (handleWiFiServer
// drives handshake + reads), so the busy flags need no locking.
#define OD_TLS_RECORD_SLOT_SIZE  17408u   /* 17 KB: 16384 content + record overhead margin */
#define OD_TLS_RECORD_SLOTS      2        /* ssl_setup takes exactly one in + one out */
#define OD_TLS_POOL_MIN_ALLOC    8192u    /* only record-sized requests use the pool */

static uint8_t* s_tlsSlot[OD_TLS_RECORD_SLOTS];
static bool     s_tlsSlotBusy[OD_TLS_RECORD_SLOTS];
static bool     s_tlsAllocHooked = false;

static void* od_tls_calloc(size_t n, size_t size) {
    const size_t total = n * size;
    if (total >= OD_TLS_POOL_MIN_ALLOC) {
        for (int i = 0; i < OD_TLS_RECORD_SLOTS; i++) {
            if (s_tlsSlot[i] != nullptr && !s_tlsSlotBusy[i] && total <= OD_TLS_RECORD_SLOT_SIZE) {
                s_tlsSlotBusy[i] = true;
                memset(s_tlsSlot[i], 0, total);   /* calloc semantics */
                return s_tlsSlot[i];
            }
        }
        // The pool could not serve a record-sized request: either the slot is too small
        // or both are in use. This is exactly the case that reintroduces the original
        // failure, so name it once -- the size tells us whether the slot needs raising.
        static bool warned = false;
        if (!warned) {
            warned = true;
            od_log_warn("WARNING: TLS alloc %u B not served from the reserved pool "
                        "(slot %u B) -- falling back to heap",
                        (unsigned)total, (unsigned)OD_TLS_RECORD_SLOT_SIZE);
        }
    }
    return heap_caps_calloc(n, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

static void od_tls_free(void* p) {
    if (p == nullptr) return;
    for (int i = 0; i < OD_TLS_RECORD_SLOTS; i++) {
        if (p == s_tlsSlot[i]) {
            s_tlsSlotBusy[i] = false;   /* recycle, never return it to the heap */
            return;
        }
    }
    heap_caps_free(p);
}

void od_tls_reserve_records(void) {
    if (s_tlsAllocHooked) return;
    // Gate on config: a device without encryption must not hold 34 KB it never uses.
    if (!isEncryptionEnabled()) {
        od_log_info("TLS: encryption disabled, no record buffers reserved");
        return;
    }
    int got = 0;
    for (int i = 0; i < OD_TLS_RECORD_SLOTS; i++) {
        s_tlsSlot[i] = (uint8_t*)heap_caps_malloc(OD_TLS_RECORD_SLOT_SIZE,
                                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        s_tlsSlotBusy[i] = false;
        if (s_tlsSlot[i] != nullptr) got++;
    }
    // Install the hook even on a partial reservation: whatever was reserved still helps,
    // and od_tls_calloc falls back to the heap for the rest.
    mbedtls_platform_set_calloc_free(od_tls_calloc, od_tls_free);
    s_tlsAllocHooked = true;
    od_log_info("TLS: reserved %d/%d record slots of %u B"
                ", internal free=%u, largest block=%u",
                got, (int)OD_TLS_RECORD_SLOTS, (unsigned)OD_TLS_RECORD_SLOT_SIZE,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    if (got < OD_TLS_RECORD_SLOTS) {
        od_log_warn("WARNING: TLS record reservation incomplete -- ssl_setup may still fail");
    }
}

void odLanReserveRxBuffer(void) {
    if (tcpReceiveBuffer != nullptr) return;   // idempotent, per od_tls_reserve_records
    // calloc, not malloc: this was a .bss array and callers may read ahead of the write
    // cursor on a partial frame. PSRAM first -- it is 16 KB of internal DRAM otherwise,
    // on a part where mbedTLS alone needs 34 KB contiguous internal.
    tcpReceiveBuffer = (uint8_t*)heap_caps_calloc(1, OD_LAN_RX_BUFFER_SIZE,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const bool inPsram = (tcpReceiveBuffer != nullptr);
    if (!inPsram) {
        tcpReceiveBuffer = (uint8_t*)heap_caps_calloc(1, OD_LAN_RX_BUFFER_SIZE,
                                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (tcpReceiveBuffer == nullptr) {
        od_log_error("ERROR: LAN RX buffer reservation failed -- LAN transport will not start");
        return;
    }
    od_log_info("LAN: reserved RX buffer %u B in %s"
                ", internal free=%u, largest block=%u",
                (unsigned)OD_LAN_RX_BUFFER_SIZE, inPsram ? "PSRAM" : "DRAM",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    if (!inPsram) {
        // The only signal that this board's PSRAM is absent or dead:
        // CONFIG_SPIRAM_IGNORE_NOTFOUND=1 lets it boot silently. No reclaim here.
        od_log_warn("WARNING: LAN RX buffer fell back to internal DRAM -- no PSRAM on this board?");
    }
}

// Build the shared server config once (RNG + PSK + one ECDHE-PSK ciphersuite).
static bool tlsEnsureConfig(void) {
    // Late fallback: encryption can be turned on by a runtime config write, long after
    // the boot-time reservation window has passed. Try anyway -- it may fail on a churned
    // heap, but the log then says so instead of leaving it silently unreserved.
    od_tls_reserve_records();
    if (tlsInited) return true;
    if (!deriveTlsPsk(tlsPsk)) {
        od_log_error("ERROR: TLS PSK derivation failed (no master key)");
        return false;
    }
    mbedtls_ssl_config_init(&tlsConf);
    mbedtls_ctr_drbg_init(&tlsDrbg);
    mbedtls_entropy_init(&tlsEntropy);
    const char* pers = "opendisplay-tls";
    int rc = mbedtls_ctr_drbg_seed(&tlsDrbg, mbedtls_entropy_func, &tlsEntropy,
                                   reinterpret_cast<const unsigned char*>(pers), strlen(pers));
    if (rc != 0) {
        char note[OD_TLS_FAIL_NOTE_LEN];
        tlsFailNote(rc, note, sizeof(note));
        od_log_error("ERROR: TLS RNG seed failed%s", note);
        return false;
    }
    rc = mbedtls_ssl_config_defaults(&tlsConf, MBEDTLS_SSL_IS_SERVER,
                                     MBEDTLS_SSL_TRANSPORT_STREAM,
                                     MBEDTLS_SSL_PRESET_DEFAULT);
    if (rc != 0) {
        char note[OD_TLS_FAIL_NOTE_LEN];
        tlsFailNote(rc, note, sizeof(note));
        od_log_error("ERROR: TLS config defaults failed%s", note);
        return false;
    }
    mbedtls_ssl_conf_rng(&tlsConf, mbedtls_ctr_drbg_random, &tlsDrbg);
    mbedtls_ssl_conf_ciphersuites(&tlsConf, kTlsCiphersuites);
    rc = mbedtls_ssl_conf_psk(&tlsConf, tlsPsk, sizeof(tlsPsk),
                              reinterpret_cast<const unsigned char*>(kTlsPskIdentity),
                              strlen(kTlsPskIdentity));
    if (rc != 0) {
        char note[OD_TLS_FAIL_NOTE_LEN];
        tlsFailNote(rc, note, sizeof(note));
        od_log_error("ERROR: TLS conf_psk failed%s", note);
        return false;
    }
    tlsInited = true;
    return true;
}

static void tlsCloseSession(void) {
    if (tlsSessionActive) {
        mbedtls_ssl_close_notify(&tlsSsl);
        mbedtls_ssl_free(&tlsSsl);
    }
    tlsSessionActive = false;
    tlsHandshakeDone = false;
}

static bool tlsBeginSession(void) {
    if (!tlsEnsureConfig()) return false;
    mbedtls_ssl_init(&tlsSsl);
    int rc = mbedtls_ssl_setup(&tlsSsl, &tlsConf);
    if (rc != 0) {
        // -0x7F00 == MBEDTLS_ERR_SSL_ALLOC_FAILED: the record buffers did not fit in
        // internal DRAM. Compare "largest block" against ~16.4 KB in the note above.
        char note[OD_TLS_FAIL_NOTE_LEN];
        tlsFailNote(rc, note, sizeof(note));
        od_log_error("ERROR: TLS ssl_setup failed%s", note);
        mbedtls_ssl_free(&tlsSsl);
        return false;
    }
    mbedtls_ssl_set_bio(&tlsSsl, &wifiClient, tls_bio_send, tls_bio_recv, nullptr);
    tlsSessionActive = true;
    tlsHandshakeDone = false;
    return true;
}

// Staging buffer so a frame leaves as ONE write. Two writes cost an extra packet
// (and, before setNoDelay(), a 40-200 ms Nagle/delayed-ACK stall on every frame);
// on TLS they also cost a second record header + MAC, which on a 2-byte ACK is more
// overhead than payload. Sized past the largest response the device produces
// (encrypted_response[600] in communication.cpp); anything larger falls back to the
// two-write path rather than growing static RAM for a case that does not occur.
static uint8_t lanTxFrame[2 + 640];

// Write one [len:2 LE][payload] frame over the active LAN channel (TLS or plain).
// Called by communication.cpp for LAN-origin responses (send_tls_lan_frame / plain).
void opendisplay_lan_send_frame(const uint8_t* payload, uint16_t len) {
    if (!wifiServerConnected || !wifiClient.connected() || len == 0) {
        return;
    }
    if (tlsMode && (!tlsSessionActive || !tlsHandshakeDone)) return;

    if ((uint32_t)len + 2u <= sizeof(lanTxFrame)) {
        lanTxFrame[0] = (uint8_t)(len & 0xFF);
        lanTxFrame[1] = (uint8_t)((len >> 8) & 0xFF);
        memcpy(lanTxFrame + 2, payload, len);
        const uint16_t total = (uint16_t)(len + 2u);
        if (tlsMode) {
            if (mbedtls_ssl_write(&tlsSsl, lanTxFrame, total) < 0) {
                od_log_error("ERROR: TLS LAN response write failed");
            }
        } else if (wifiClient.write(lanTxFrame, total) != total) {
            od_log_error("ERROR: LAN response write incomplete");
        }
        return;
    }

    // Oversized fallback: header then payload. A failure between the two leaves the
    // peer waiting on a length prefix whose payload never arrives.
    uint8_t hdr[2] = { (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
    if (tlsMode) {
        if (mbedtls_ssl_write(&tlsSsl, hdr, 2) < 0 ||
            mbedtls_ssl_write(&tlsSsl, payload, len) < 0) {
            od_log_error("ERROR: TLS LAN response write failed");
        }
        return;
    }
    if (wifiClient.write(hdr, 2) != 2 || wifiClient.write(payload, len) != len) {
        od_log_error("ERROR: LAN response write incomplete");
    }
}

bool wifiLanClientConnected(void) {
    return wifiServerConnected && wifiClient.connected();
}

static void hex14_lower(const uint8_t* src, char* out29) {
    static const char* h = "0123456789abcdef";
    for (int i = 0; i < 14; i++) {
        out29[i * 2] = h[(src[i] >> 4) & 0x0F];
        out29[i * 2 + 1] = h[src[i] & 0x0F];
    }
    out29[28] = '\0';
}

void opendisplay_mdns_update_msd_txt(void) {
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        return;
    }
    static uint8_t last_msd[14];
    static uint32_t last_ms = 0;
    static bool have_last = false;
    uint8_t cur[14];
    memcpy(cur, &msd_payload[2], sizeof(cur));
    uint32_t now = millis();
    if (have_last && memcmp(cur, last_msd, sizeof(cur)) == 0 && (now - last_ms) < 400) {
        return;
    }
    have_last = true;
    memcpy(last_msd, cur, sizeof(cur));
    last_ms = now;
    char hex[29];
    hex14_lower(cur, hex);
    // const char* overload (void); char* overload (bool) — avoid ambiguous resolution with char hex[].
    MDNS.addServiceTxt("opendisplay", "tcp", "msd", static_cast<const char*>(hex));
}

// The advertised BLE address, lowercase colon-separated (SECTION 9 rule 6, key
// `mac`). Prior identity used getChipIdHex() (eFuse), which is NOT what HA
// stores as the device unique_id. The lowercasing and the hardware-validation
// caveat now live in BleTransport::addressString().
static String advertisedBleMacLower(void) {
    return String(ble.addressString());
}

static void restartLanService(void) {
    String deviceName = "OD" + getChipIdHex();
    if (!MDNS.begin(deviceName.c_str())) {
        od_log_error("ERROR: mDNS responder failed");
        return;
    }
    uint16_t port = lanActivePort();
    od_log_info("mDNS: %s.local", deviceName.c_str());
    MDNS.addService("opendisplay", "tcp", port);
    // F1 -- identity/capability TXT keys (SECTION 9 rule 6, ADDITIVE to `msd`).
    String mac = advertisedBleMacLower();
    MDNS.addServiceTxt("opendisplay", "tcp", "mac", mac.c_str());        // REQUIRED
    MDNS.addServiceTxt("opendisplay", "tcp", "tls", isEncryptionEnabled() ? "1" : "0"); // REQUIRED
    // const char* overload (value) vs char* overload (key/value) — cast char[] to
    // const char* to disambiguate, matching opendisplay_mdns_update_msd_txt().
    char fw[12];
    snprintf(fw, sizeof(fw), "%u.%u", (unsigned)getFirmwareMajor(), (unsigned)getFirmwareMinor());
    MDNS.addServiceTxt("opendisplay", "tcp", "fw", static_cast<const char*>(fw));  // RECOMMENDED
    char cm[3];
    snprintf(cm, sizeof(cm), "%02x", (unsigned)globalConfig.system_config.communication_modes);
    MDNS.addServiceTxt("opendisplay", "tcp", "cm", static_cast<const char*>(cm));  // RECOMMENDED
    uint8_t did[4];
    getAuthDeviceIdBytes(did);
    char idhex[9];
    snprintf(idhex, sizeof(idhex), "%02x%02x%02x%02x", did[0], did[1], did[2], did[3]);
    MDNS.addServiceTxt("opendisplay", "tcp", "id", static_cast<const char*>(idhex));  // OPTIONAL
    MDNS.addServiceTxt("opendisplay", "tcp", "pv", OD_PROTOCOL_VERSION_STR); // OPTIONAL
    od_log_info("mDNS: _opendisplay._tcp port %u tls=%s mac=%s",
                (unsigned)port, isEncryptionEnabled() ? "1" : "0", mac.c_str());
    opendisplay_mdns_update_msd_txt();
}

static void startLanServer(void) {
    // No RX buffer, no listener. Refusing here is the whole degrade path: it covers
    // every caller, and it is better than accepting a socket the parser cannot serve.
    // BLE and the display path are unaffected.
    if (tcpReceiveBuffer == nullptr) {
        od_log_error("ERROR: LAN RX buffer unavailable -- LAN transport disabled");
        return;
    }
    tlsMode = isEncryptionEnabled();
    uint16_t port = lanActivePort();
    wifiServer.begin(port);
    od_log_info("%s LAN server listening on port %u",
                tlsMode ? "TLS-PSK" : "Plaintext", (unsigned)port);
    restartLanService();
}

// WiFi.status() collapses every association failure into WL_DISCONNECTED (6), which
// is useless for field diagnosis. Log the 802.11/ESP reason code instead: 201
// (NO_AP_FOUND) means the SSID was never seen -- typically a 5 GHz-only or hidden
// network; 15 (4WAY_HANDSHAKE_TIMEOUT) / 202 (AUTH_FAIL) mean a bad password; 200
// (BEACON_TIMEOUT) / 4 (ASSOC_EXPIRE) point at range or BLE coexistence.
static bool wifiDiagEventsRegistered = false;

// ------------------------------------------------------------------ roaming ---
// Move to a stronger AP on the same SSID when the current link degrades. Two halves
// that only work together:
//
//  (a) BEST-AP SELECTION. Arduino's default is scan_method = WIFI_FAST_SCAN, which
//      stops at the FIRST SSID match and associates with it however weak; the
//      sort_method it also defaults to (WIFI_CONNECT_AP_BY_SIGNAL) is INERT unless
//      the scan is WIFI_ALL_CHANNEL_SCAN. So on a multi-AP SSID (mesh/extenders) the
//      device can sit on a distant AP while a strong one exists on another channel,
//      non-deterministically across reboots. Switching to ALL_CHANNEL_SCAN makes the
//      RSSI sort actually apply, so every (re)connect picks the strongest AP.
//      Cost: a full-channel scan per connect (~1.5-2.5 s vs ~0.3-0.5 s) -- radio-on
//      time paid on each deep-sleep wake.
//
//  (b) THE TRIGGER. esp_wifi_set_rssi_threshold() posts WIFI_EVENT_STA_BSS_RSSI_LOW
//      once the averaged RSSI drops below the threshold. It is ONE-SHOT (see the API
//      note in esp_wifi.h) so it must be re-armed after every event and after every
//      re-association. Arduino does not surface this event in arduino_event_id_t, so
//      it needs a raw esp_event handler on WIFI_EVENT.
//
// Without (a), (b) is a footgun: a reconnect under FAST_SCAN can land on the SAME
// weak AP, giving a pointless disconnect/reconnect loop. Do not enable one alone.
//
// The roam itself is DEFERRED to an idle moment (serviceLanRoam) because
// re-association drops the TCP session and a full-channel scan steals the shared
// radio from BLE under coex -- doing that mid-transfer would abort it.
#ifndef OD_LAN_ROAM_RSSI_THRESHOLD
#define OD_LAN_ROAM_RSSI_THRESHOLD (-75)   /* dBm; valid range -100..10 */
#endif

static bool roamPending = false;            // set by the RSSI-low event, cleared on roam
static bool roamEventRegistered = false;

// Event handlers run on tiny stacks (see EVENT-CONTEXT RULE below), so they only set
// these and stash plain scalars; serviceWifiEventFollowUp() does the logging and the
// esp_* calls on the loop task.
static volatile bool assocNoticePending = false;
static volatile bool disconnectNoticePending = false;
static volatile bool gotIpPending = false;
static volatile bool rssiLowNoticePending = false;
static volatile bool cacheFailPending = false;
static volatile int  s_lastAssocChannel = 0;
static volatile int  s_lastDisconnectReason = 0;
static volatile int  s_lastRssiLowDbm = 0;
static volatile uint32_t s_lastGotIp = 0;

// ------------------------------------------------------- RTC-cached AP choice ---
// Deep sleep is a full reset, so without this every wake pays the all-channel scan
// (~1.5-2.5 s of radio-on time) just to rediscover the AP it used seconds earlier.
// Caching the winning BSSID + channel in RTC memory (which survives deep sleep) lets a
// wake connect DIRECTLY to that BSSID on a single channel: no scan at all, faster than
// even the old FAST_SCAN default, while still landing on the strongest AP because the
// cache is only ever written from a scan-selected association.
//
// Kept file-local rather than in main.h (where the other RTC_DATA_ATTR state lives)
// because nothing outside this translation unit uses it.
//
// Self-healing: any failure or degradation invalidates the cache and falls back to a
// full scan -- see lanCacheInvalidate() callers. In particular a cached connect that
// lands on a now-weak AP trips the RSSI threshold, which invalidates and rescans, so a
// stale "best" choice cannot persist across days.
RTC_DATA_ATTR static uint8_t s_cachedBssid[6];
RTC_DATA_ATTR static uint8_t s_cachedChannel;   // 1..14; 0 = unset
RTC_DATA_ATTR static bool    s_cachedValid;

static bool usingCachedAp = false;      // this attempt used the cache (drives fallback)
static bool rescanReconnectPending = false;  // cached BSSID failed; re-begin with a scan

// Make every (re)connect choose the strongest AP for the SSID. Must be called
// BEFORE WiFi.begin(); the setting is sticky for later auto-reconnects.
static void lanApplyBestApSelection(void) {
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);      // required for the sort below to apply
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);  // strongest RSSI wins
}

static void lanCacheInvalidate(const char* why) {
    if (s_cachedValid) {
        od_log_info("WiFi: AP cache cleared (%s) -- next connect will scan", why);
    }
    s_cachedValid = false;
    s_cachedChannel = 0;
}

// Record the AP we actually associated with, so the next wake can go straight to it.
static void lanCacheStoreCurrentAp(void) {
    const uint8_t* b = WiFi.BSSID();
    int32_t ch = WiFi.channel();
    if (b == nullptr || ch < 1 || ch > 14) return;
    memcpy(s_cachedBssid, b, 6);
    s_cachedChannel = (uint8_t)ch;
    s_cachedValid = true;
}

// Single entry point for association: cached BSSID when available, full scan otherwise.
static void lanBeginConnect(void) {
    if (s_cachedValid && s_cachedChannel >= 1 && s_cachedChannel <= 14) {
        usingCachedAp = true;
        char b[18];
        snprintf(b, sizeof(b), "%02X:%02X:%02X:%02X:%02X:%02X",
                 s_cachedBssid[0], s_cachedBssid[1], s_cachedBssid[2],
                 s_cachedBssid[3], s_cachedBssid[4], s_cachedBssid[5]);
        od_log_info("WiFi: connecting to cached AP %s ch %d (no scan)",
                    b, (int)s_cachedChannel);
        WiFi.begin(wifiSsid, wifiPassword, (int32_t)s_cachedChannel, s_cachedBssid);
        return;
    }
    usingCachedAp = false;
    od_log_info("WiFi: scanning all channels for the strongest AP");
    lanApplyBestApSelection();
    WiFi.begin(wifiSsid, wifiPassword);
}

static void onWifiRssiLow(void* arg, esp_event_base_t base, int32_t id, void* data);

// Register the raw WIFI_EVENT handler LAZILY, retrying until it takes.
//
// esp_event_handler_register() requires the default event loop, which Arduino creates
// while starting the STA -- not before. Doing this pre-WiFi.begin() (where the Arduino
// callback is registered) fails with ESP_ERR_INVALID_STATE and leaves roaming silently
// dead, which is exactly what happened on hardware. Called from the association path,
// by which point the loop and the WiFi task definitely exist; idempotent and self-
// retrying so a transient failure does not permanently disable roaming.
static void ensureRoamEventRegistered(void) {
    if (roamEventRegistered) return;
    esp_err_t rc = esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_BSS_RSSI_LOW,
                                              &onWifiRssiLow, NULL);
    if (rc == ESP_OK) {
        roamEventRegistered = true;
        od_log_info("WiFi: roaming armed (RSSI-low handler registered)");
        return;
    }
    // Always report the code -- "could not register" with no reason is undiagnosable.
    od_log_info("WiFi: RSSI-low handler register failed (%s); will retry on next association",
                esp_err_to_name(rc));
}

// One-shot: re-arm after every event and after every association.
static void lanArmRssiThreshold(void) {
    ensureRoamEventRegistered();   // no point arming a threshold nothing listens for
    esp_err_t rc = esp_wifi_set_rssi_threshold(OD_LAN_ROAM_RSSI_THRESHOLD);
    if (rc != ESP_OK) {
        od_log_info("WiFi: RSSI threshold arm failed (%d)", (int)rc);
    }
}

// EVENT-CONTEXT RULE (learned the hard way -- this handler previously panicked the
// device): callbacks here run on tiny stacks -- the raw esp_event handler on the system
// event task, and the Arduino callbacks on "arduino_events" with a 4096-byte stack
// (ARDUINO_NETWORK_EVENT_TASK_STACK_SIZE). Logging plus esp_wifi_*/esp_event_* calls
// is far too much for that budget. So handlers ONLY set flags; every log and esp_*
// call happens in serviceWifiEventFollowUp() on the loop task. Keep it that way.
// Note this rule predates the printf-style logging: the old String concatenation made
// it worse (a heap allocation per line), but od_log_* still formats into a 256-byte
// stack buffer, which alone is 6% of this task's stack. The rule stands.
static void onWifiRssiLow(void* arg, esp_event_base_t base, int32_t id, void* data) {
    (void)arg; (void)base; (void)id;
    const wifi_event_bss_rssi_low_t* e = (const wifi_event_bss_rssi_low_t*)data;
    s_lastRssiLowDbm = e ? (int)e->rssi : 0;   // plain int store; logged from loop()
    roamPending = true;
    rssiLowNoticePending = true;
}

// Flags only -- see the EVENT-CONTEXT RULE above. No logging and no esp_* calls:
// this runs on the 4096-byte "arduino_events" task.
static void onWiFiDiagEvent(arduino_event_id_t event, arduino_event_info_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            s_lastAssocChannel = (int)info.wifi_sta_connected.channel;
            assocNoticePending = true;
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            s_lastDisconnectReason = (int)info.wifi_sta_disconnected.reason;
            disconnectNoticePending = true;
            // A BSSID-pinned begin() leaves bssid_set = 1 in the driver config, so the
            // framework's auto-reconnect would retry that ONE AP forever -- fatal if it
            // moved, changed channel, or powered off. Ask the loop to drop the cache and
            // re-begin WITHOUT a BSSID, which both clears bssid_set and rescans.
            if (usingCachedAp) {
                usingCachedAp = false;
                cacheFailPending = true;      // loop() invalidates + logs
                rescanReconnectPending = true;
            }
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            s_lastGotIp = info.got_ip.ip_info.ip.addr;
            rescanReconnectPending = false;
            gotIpPending = true;              // loop() logs RSSI/ch/BSSID, caches, arms
            break;
        default:
            break;
    }
}

static void registerWiFiDiagEvents(void) {
    if (wifiDiagEventsRegistered) return;
    WiFi.onEvent(onWiFiDiagEvent);
    wifiDiagEventsRegistered = true;
    // The raw WIFI_EVENT handler is NOT registered here: this runs before WiFi.begin(),
    // and esp_event_handler_register() needs the default event loop, which Arduino only
    // creates while bringing the STA up. Registering here fails with ESP_ERR_INVALID_STATE
    // and silently disabled roaming. It is now done lazily -- see ensureRoamEventRegistered().
}

// Execute a queued roam, but only when nothing is in flight. Called from
// handleWiFiServer() each tick.
// Everything the event handlers deferred, run on the loop task where the stack is ample.
// Called from handleWiFiServer() each tick.
static void serviceWifiEventFollowUp(void) {
    if (assocNoticePending) {
        assocNoticePending = false;
        od_log_info("WiFi event: associated (channel %d)", (int)s_lastAssocChannel);
    }
    if (disconnectNoticePending) {
        disconnectNoticePending = false;
        od_log_info("WiFi event: disconnected, reason %d", (int)s_lastDisconnectReason);
    }
    if (cacheFailPending) {
        cacheFailPending = false;
        lanCacheInvalidate("cached BSSID did not associate");
    }
    if (rssiLowNoticePending) {
        rssiLowNoticePending = false;
        od_log_info("WiFi: RSSI %d dBm below %d dBm -- roam queued (deferred to idle)",
                    (int)s_lastRssiLowDbm, (int)OD_LAN_ROAM_RSSI_THRESHOLD);
        lanArmRssiThreshold();   // one-shot: re-arm so a deferred roam still re-triggers
    }
    if (gotIpPending) {
        gotIpPending = false;
        const IPAddress gotIp((uint32_t)s_lastGotIp);
        char bssid[18];
        lanFormatBssid(bssid, sizeof(bssid), WiFi.BSSID());
        od_log_info("WiFi event: got IP %u.%u.%u.%u, RSSI %d dBm, ch %d, BSSID %s",
                    gotIp[0], gotIp[1], gotIp[2], gotIp[3],
                    (int)WiFi.RSSI(), (int)WiFi.channel(), bssid);
        lanCacheStoreCurrentAp();   // remember this AP for the next deep-sleep wake
        lanArmRssiThreshold();      // also (re)registers the RSSI-low handler
    }
}

void serviceLanRoam(void) {
    serviceWifiEventFollowUp();
    // A cached BSSID that failed to associate (AP moved, changed channel, or powered
    // off). Re-begin WITHOUT a BSSID from loop context -- never from the event callback --
    // so the driver's bssid_set is cleared and a full scan picks a live AP.
    if (rescanReconnectPending && !wifiConnected) {
        rescanReconnectPending = false;
        od_log_info("WiFi: cached AP unreachable -- falling back to a full scan");
        WiFi.disconnect(false);
        lanBeginConnect();   // cache was invalidated on the disconnect -> scan path
        return;
    }
    if (!roamPending || !wifiConnected) return;
    // A LAN client implies a possible in-flight transfer; BLE transfers would also be
    // disrupted by a full-channel scan under coex. Wait for genuine idle. transferActive()
    // covers DIRECT/PIPE/PARTIAL -- the previous direct+pipe test let a BLE-origin partial
    // write (no LAN client attached, so the check above does not fire either) be
    // interrupted by the scan.
    if (wifiClient.connected() || wifiServerConnected) return;
    if (transferActive()) return;

    roamPending = false;
    od_log_info("WiFi: roaming -- re-associating to the strongest AP for this SSID");
    // MUST drop the cache first: the whole point of a roam is to leave this AP, and
    // lanBeginConnect() would otherwise pin us straight back to the one we are escaping.
    lanCacheInvalidate("roaming to a stronger AP");
    usingCachedAp = false;
    rescanReconnectPending = false;
    // Clearing wifiConnected lets handleWiFiServer()'s existing re-association path
    // restart the LAN server once the new link is up.
    wifiConnected = false;
    WiFi.disconnect(false);
    lanBeginConnect();   // cache now invalid -> full scan, strongest AP wins
}

void initWiFi(bool waitForConnection) {
    od_log_info("=== Initializing WiFi ===");

    // WiFi is NOT gated on power_mode: if COMM_MODE_WIFI is enabled the radio comes
    // up on battery too. Radio cost on battery is managed by the driver's default
    // power-save mode and by deep sleep, not by refusing to associate.
    if (!(globalConfig.system_config.communication_modes & COMM_MODE_WIFI)) {
        od_log_info("WiFi not enabled in communication_modes, skipping");
        wifiInitialized = false;
        return;
    }
    if (!wifiConfigured) {
        od_log_info("WiFi: system_config has WiFi mode on, but wifi_config TLV (0x26) is not in saved "
                    "configuration (or failed to parse). Enable Wi-Fi in config, set SSID, and write full "
                    "config to the device.");
        wifiInitialized = false;
        return;
    }
    if (wifiSsid[0] == '\0' || strlen(wifiSsid) == 0) {
        od_log_info("WiFi: wifi_config packet present but SSID field is empty.");
        wifiInitialized = false;
        return;
    }
    // Bound the WiFi driver's TX memory to a fixed pool. Arduino's default
    // (_wifiUseStaticBuffers = false) REPLACES the IDF buffer profile inside
    // wifiLowLevelInit() with a demand-driven one: static_tx_buf_num 0 /
    // dynamic_tx_buf_num 32 / cache_tx_buf_num 4 / static_rx_buf_num 4. Setting this
    // skips that override, so esp_wifi_init() gets WIFI_INIT_CONFIG_DEFAULT() --
    // static_tx 8, dynamic_tx 0, cache_tx 0, static_rx 8 (sdkconfig).
    //
    // At the documented ~1.6 KB per WiFi buffer that trades ~13 KB more resident
    // internal DRAM for ~38 KB off the peak: TX becomes 12.8 KB always instead of up
    // to 51.2 KB whenever traffic asks. The right trade here because this part dies on
    // transients, not on baseline -- a 2026-08-03 coredump caught the tcpip task with a
    // NULL from the mDNS allocator during an inbound query burst, and the OOM error log
    // then aborted in lock_init_generic. RX is unchanged: dynamic_rx_buf_num is 32 in
    // both profiles and Arduino exposes no knob for it.
    //
    // cache_tx_buf_num 0 is correct here, not the "can't be zero!" case in Arduino's
    // dynamic path: cache TX buffers exist to copy dynamic TX buffers out of PSRAM into
    // DMA-capable memory, and static TX buffers are already internal and DMA-capable.
    //
    // MUST precede every WiFi call -- wifiLowLevelInit() reads the flag once and latches
    // lowLevelInitDone. Set later it only logs a warning and takes effect after a
    // WiFi.mode(WIFI_MODE_NULL). Nothing touches the WiFi API before this point: the
    // WiFi.status() call sites in main.cpp/config_parser.cpp all run after setup().
    WiFi.useStaticBuffers(true);
    // Do not log the SSID or password (credentials); log only presence/length.
    od_log_info("WiFi: connecting to configured SSID (len %u)", (unsigned)strlen(wifiSsid));
    registerWiFiDiagEvents();
    WiFi.setAutoReconnect(true);
    wifiSsid[32] = '\0';
    wifiPassword[32] = '\0';
    od_log_info("Encryption type: 0x%x", (unsigned)wifiEncryptionType);
    wifiConnected = false;
    wifiInitialized = true;
    // Cached BSSID when RTC memory still holds one (the deep-sleep-wake fast path:
    // single channel, no scan), otherwise an all-channel scan for the strongest AP.
    lanBeginConnect();
    // Tx power can only be set once the STA is started, i.e. after begin(); the
    // pre-begin() call this replaces failed with ESP_ERR_WIFI_NOT_START.
    WiFi.setTxPower(WIFI_POWER_15dBm);
    if (!waitForConnection) {
        od_log_info("WiFi: STA started (non-blocking; LAN starts when associated)");
        return;
    }
    od_log_info("Waiting for WiFi connection...");
    const int maxRetries = 3;
    const unsigned long timeoutPerRetry = 10000;
    bool connected = false;
    for (int retry = 0; retry < maxRetries && !connected; retry++) {
        unsigned long startAttempt = millis();
        bool abortCurrentRetry = false;
        while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt < timeoutPerRetry)) {
            delay(500);
            wl_status_t status = WiFi.status();
            od_log_info("WiFi status: %d", (int)status);
            if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
                od_log_info("Connection failed immediately (Status: %d)", (int)status);
                abortCurrentRetry = true;
                break;
            }
        }
        if (WiFi.status() == WL_CONNECTED) {
            connected = true;
            break;
        }
        if (!abortCurrentRetry) {
            od_log_info("WiFi attempt %d timed out", retry + 1);
        }
        if (retry < maxRetries - 1) {
            delay(2000);
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        od_log_info("=== WiFi connected ===");
        {
            const IPAddress ip = WiFi.localIP();
            od_log_info("IP: %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
        }
        startLanServer();
    } else {
        wifiConnected = false;
        od_log_info("=== WiFi connection failed ===");
    }
}

void wifiLanDropOwnedSocket(void) {
    // The LAN arm of abortToKnownState()'s step 10. It exists as its own seam
    // because tlsCloseSession() is file-static, so session_guard.cpp cannot reach
    // the pieces directly.
    //
    // Deliberately the LAN-LOCAL subset of disconnectWiFiServer() below: it closes
    // the socket and clears this file's session bookkeeping, but does NOT call
    // clearEncryptionSession() or requestTransferSessionCleanup(), which are the
    // abort's own steps 8 and 3-5. Calling them here would nest the two teardowns.
    tlsCloseSession();
    if (wifiClient.connected()) {
        od_log_info("Closing LAN client (session abort)");
        wifiClient.stop();
    }
    wifiServerConnected = false;
    tcpReceiveBufferPos = 0;
    // Deliberately NO linkRelease() here: this is the abort's drop step, and the
    // abort releases the token itself as its final step (strictly after the drop).
    // Releasing here would free the slot before the drop had completed.
    s_lanEpoch = 0;
}

void disconnectWiFiServer() {
    tlsCloseSession();
    if (wifiClient.connected()) {
        od_log_info("Closing LAN client");
        clearEncryptionSession();
        wifiClient.stop();
    }
    wifiServerConnected = false;
    tcpReceiveBufferPos = 0;
    // The token is deliberately NOT released here, for the same reason the BLE
    // disconnect callbacks do not release: the socket is closed but this session's
    // transfer state is still live, and its teardown is DEFERRED to the loop below
    // (requestTransferSessionCleanup). Releasing now would let a BLE connect claim
    // the slot before that teardown runs -- and serviceBleDisconnectCleanup would
    // then see the new BLE owner live, skip the abort entirely, and leave the new
    // owner's frames executing against the departed LAN session's transfer state.
    //
    // Release stays the abort's final step (R3a), reached via the cleanup flag
    // raised below. s_lanEpoch is left intact so the identity remains valid until
    // then; a new accept cannot use it, because accept refuses while the slot is
    // held.
    // F4: abort any in-flight direct-write / pipe / partial transfer + tear down a
    // mid-transfer panel session, DEFERRED to loop() so cleanup never races an
    // in-progress EPD refresh. Shared with the BLE disconnect path -- main.cpp
    // works out which transport actually owned the transfer.
    requestTransferSessionCleanup();
}

// lanReadIntoBuffer() return codes. A graceful peer close is deliberately distinct from
// a fault: both end the session, but only one is worth an ERROR in the log.
#define OD_LAN_READ_ERROR   (-1)
#define OD_LAN_READ_CLOSED  (-2)
static int s_lastLanReadErr = 0;   // mbedTLS ret for the last OD_LAN_READ_ERROR

// Drain readable bytes from the active channel into tcpReceiveBuffer. Returns the number
// of bytes appended (>=0), OD_LAN_READ_CLOSED on a graceful peer close, or
// OD_LAN_READ_ERROR on a fatal channel error (caller drops in both cases).
static int lanReadIntoBuffer(void) {
    // Belt-and-braces: startLanServer() refuses to listen without the buffer, so no
    // socket should reach here, but never index a null on the read path.
    if (tcpReceiveBuffer == nullptr) return OD_LAN_READ_ERROR;
    // OD_LAN_RX_BUFFER_SIZE, not sizeof(): tcpReceiveBuffer is a pointer now, and
    // sizeof would silently yield 4/8 -- reads would collapse to a few bytes per tick
    // and read as a network fault rather than a code bug.
    int space = (int)OD_LAN_RX_BUFFER_SIZE - (int)tcpReceiveBufferPos;
    if (space <= 0) {
        od_log_info("LAN RX buffer full, dropping connection");
        return -1;
    }
    if (tlsMode) {
        int r = mbedtls_ssl_read(&tlsSsl, &tcpReceiveBuffer[tcpReceiveBufferPos], (size_t)space);
        if (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE) return 0;
        // A GRACEFUL close is not a fault. The client sends close_notify right after its
        // last frame (e.g. the 0x0073 ack at the end of a push), so every successful
        // transfer used to end with "channel read error" -- alarming and wrong. Separate
        // it from a real failure so the log distinguishes "done" from "broken".
        if (r == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY || r == 0) return OD_LAN_READ_CLOSED;
        if (r < 0) {
            s_lastLanReadErr = r;      // carried out so the caller can name the code
            return OD_LAN_READ_ERROR;
        }
        tcpReceiveBufferPos += (uint32_t)r;
        return r;
    }
    int available = wifiClient.available();
    if (available <= 0) return 0;
    int bytesToRead = (available > space) ? space : available;
    int bytesRead = wifiClient.read(&tcpReceiveBuffer[tcpReceiveBufferPos], bytesToRead);
    if (bytesRead > 0) tcpReceiveBufferPos += (uint32_t)bytesRead;
    return (bytesRead > 0) ? bytesRead : 0;
}

void wifiLanReapClosedSession(void) {
    // Notice a peer-closed socket EARLY in the pass, so the deferred cleanup that
    // this raises releases the token before handleWiFiServer()'s accept runs later
    // in the same pass. See the call site in loop() for why "early" is the whole
    // point: raising it from inside handleWiFiServer left the accept testing a
    // corpse's token and refusing an ordinary reconnect.
    if (wifiServerConnected && !wifiClient.connected()) {
        od_log_info("LAN: peer closed the socket, reaping the session");
        disconnectWiFiServer();
    }
}

// Decide a freshly accepted socket's fate: admit it as the owner, or refuse it.
//
// Split out of handleWiFiServer() so the caller can CONTINUE servicing an existing
// session in the same pass whatever the outcome. When refusal returned early from
// handleWiFiServer, a host reconnecting every pass starved the incumbent -- its TLS
// handshake never advanced and its buffered frames were never dispatched, so the
// shared activity clock stopped being stamped and the end-of-pass idle timeout
// eventually dropped it with valid commands still unread. Refusal must be inert
// (R3), and inbound traffic must be parsed before the idle check (R7d step 3 before
// step 4); an early return broke both.
static void admitOrRefuseLanClient(WiFiClient& incoming) {
    // REFUSE while the slot is held -- never evict. Two reasons, both concrete:
    //
    //  - The eviction path this replaces closed the previous socket without
    //    releasing its ownership epoch, and the replacement then lost its own
    //    claim, so the departed session's identity became unreachable and the slot
    //    stayed held until reboot.
    //  - Eviction is unauthenticated by design: LAN-TLS bypasses app-layer auth, so
    //    any host on the network could kill an in-flight display push by opening a
    //    socket, with no credentials.
    //
    // The incumbent is untouched: no teardown, no crypto clear, no cleanup flag.
    // That is R3's "refusal is inert", and it also removes a pre-existing bug in the
    // eviction it replaces -- that path cleared TLS and crypto but never requested
    // transfer cleanup, so an evicted client's in-flight transfer stayed live and
    // the new client's frames (same ORIGIN_LAN, so frameOwnsSession could not tell
    // them apart) landed in it.
    const LinkId held = linkOwnerId();
    if (held.who != OWNER_NONE) {
        od_log_info("LAN: refusing new client, slot held by %s",
                    held.who == OWNER_BLE ? "BLE" : "LAN");
        incoming.stop();
        return;
    }

    wifiClient = incoming;
    // TCP_NODELAY: every LAN write is a complete, self-delimited frame, so there is
    // never a following write for Nagle to coalesce it with -- it can only hold a
    // small frame until the peer's delayed ACK fires (40-200 ms). With per-chunk
    // direct-write ACKs that lands on every frame of a transfer.
    wifiClient.setNoDelay(true);
    wifiClient.setTimeout(30000);
    tcpReceiveBufferPos = 0;
    wifiServerConnected = true;

    // Claim at TCP ACCEPT, before the TLS handshake (7a row 2): the handshake is
    // driven incrementally across later passes, so deferring the claim until it
    // completes would leave the slot free for a BLE connect or a second socket in
    // the meantime. The accept is this transport's earliest hook, so it is the same
    // "claim at the earliest hook" rule the BLE connect callback follows -- and the
    // same CAS, which is what makes cross-transport arbitration the word itself
    // rather than loop ordering.
    //
    // The slot was free a moment ago, so this normally wins. It can still lose to a
    // BLE connect landing on the host task in between: the callback is the
    // authoritative arbitration point, not this loop-side test (R7d).
    s_lanEpoch = linkNextEpoch();
    if (!linkClaim((LinkId){OWNER_LAN, 0, s_lanEpoch})) {
        od_log_info("LAN: refusing new client, slot claimed concurrently");
        s_lanEpoch = 0;
        incoming.stop();
        wifiClient = WiFiClient();
        wifiServerConnected = false;
        return;
    }

    const IPAddress peer = wifiClient.remoteIP();
    od_log_info("LAN client connected from %u.%u.%u.%u", peer[0], peer[1], peer[2], peer[3]);
    if (tlsMode && !tlsBeginSession()) {
        od_log_info("LAN: TLS session start failed, dropping");
        disconnectWiFiServer();
    }
}

void handleWiFiServer() {
    // Execute a queued roam (RSSI dropped below OD_LAN_ROAM_RSSI_THRESHOLD) before any
    // other work; it self-gates on idle, so this is a no-op mid-transfer.
    serviceLanRoam();

    if (wifiInitialized && WiFi.status() == WL_CONNECTED && !wifiConnected) {
        wifiConnected = true;
        od_log_info("=== WiFi connected ===");
        {
            const IPAddress ip = WiFi.localIP();
            char bssid[18];
            lanFormatBssid(bssid, sizeof(bssid), WiFi.BSSID());
            od_log_info("IP: %u.%u.%u.%u, RSSI %d dBm, ch %d, BSSID %s",
                        ip[0], ip[1], ip[2], ip[3],
                        (int)WiFi.RSSI(), (int)WiFi.channel(), bssid);
        }
        startLanServer();
    }
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        if (wifiServerConnected || wifiClient.connected()) {
            od_log_info("WiFi lost, closing LAN session");
            disconnectWiFiServer();
        }
        return;
    }

    // Accept-side refusal must NOT return from this function. Everything below --
    // driving the incumbent's TLS handshake, reading its socket, dispatching its
    // frames -- is what stamps the shared activity clock, and the idle timeout is
    // evaluated at the end of this same pass. An early return on refusal therefore
    // lets a contender starve the incumbent: a host that reconnects every pass
    // stops the incumbent being serviced at all, and after OD_LAN_READ_TIMEOUT_S
    // the idle drop kills it -- with valid commands still sitting unread in its
    // socket. That is both an R3 violation (refusal must be inert) and an R7d one
    // (step 3 must precede step 4). So the refusal branch closes the contender and
    // falls through.
    WiFiClient incoming = wifiServer.accept();
    if (incoming) {
        admitOrRefuseLanClient(incoming);
        // Deliberately no return here on either outcome -- see the note above.
    }

    if (!wifiServerConnected || !wifiClient.connected()) {
        if (wifiServerConnected) {
            od_log_info("LAN client disconnected");
            disconnectWiFiServer();
        }
        return;
    }

    // Drive the TLS handshake incrementally; return until it completes.
    if (tlsMode && tlsSessionActive && !tlsHandshakeDone) {
        int hs = mbedtls_ssl_handshake(&tlsSsl);
        if (hs == 0) {
            tlsHandshakeDone = true;
            // The idle baseline starts HERE, not at TCP accept: handshake traffic
            // is not a command (R4/7a). A handshake that never completes therefore
            // leaves the clock at its accept-time stamp and the ordinary 30 s drop
            // reclaims the socket -- which is why no separate handshake deadline is
            // needed.
            linkStampOwnerCommand();
            od_log_info("LAN: TLS handshake complete");
        } else if (hs == MBEDTLS_ERR_SSL_WANT_READ || hs == MBEDTLS_ERR_SSL_WANT_WRITE) {
            // still handshaking; but honor the idle timeout below
        } else {
            // The handshake struct is another internal-DRAM allocation, so annotate the
            // heap here too -- an OOM mid-handshake looks like a protocol error otherwise.
            char note[OD_TLS_FAIL_NOTE_LEN];
            tlsFailNote(hs, note, sizeof(note));
            od_log_info("LAN: TLS handshake failed%s, dropping", note);
            disconnectWiFiServer();
            return;
        }
    }

    // Bounded drain: keep reading and dispatching until the channel is dry or a
    // per-tick byte budget (one full tcpReceiveBuffer) is spent. Parsing INSIDE
    // the loop frees buffer space before the next read -- essential on TLS, where
    // one mbedtls_ssl_read can return a whole max-size record. The budget bounds
    // LAN's hold on loop() so BLE drain, buttons/touch/buzzer, and the deep-sleep
    // gate still run under a saturating streaming client.
    uint32_t drainedBytes = 0;
    int got;
    do {
        got = lanReadIntoBuffer();
        if (got == OD_LAN_READ_CLOSED) {
            // Normal end of a push: the client finished and sent close_notify.
            od_log_info("LAN: client closed the connection");
            disconnectWiFiServer();
            return;
        }
        if (got < 0) {
            // Real fault -- name the mbedTLS code, otherwise this is undiagnosable.
            char note[OD_TLS_FAIL_NOTE_LEN];
            tlsFailNote(s_lastLanReadErr, note, sizeof(note));
            od_log_info("LAN: channel read error%s, dropping", note);
            disconnectWiFiServer();
            return;
        }
        if (got > 0) {
            // NOT an activity stamp. R4 defines activity as a RECOGNISED COMMAND
            // from the owner, and this site fires on any bytes read -- before
            // framing, opcode recognition, ownership or authentication -- so a
            // plain-mode flooder could hold the slot indefinitely with garbage and
            // defeat both the 30 s read timeout and anything built on this clock.
            // That is the same defect the BLE clock had in its intake-stamping
            // draft, and it is fixed the same way: the stamp moved to the dispatch
            // site below, which is reached only by a framed, recognised command.
            drainedBytes += (uint32_t)got;
        } else if (drainedBytes == 0) {
            // Nothing to read this tick. The idle DROP is not decided here any
            // more: serviceIdleTimeout() owns it for both transports, and it must
            // run after inbound traffic has been parsed (7d step 4) or a LAN client
            // is dropped with its command already sitting in the buffer.
            return;
        }

        while (tcpReceiveBufferPos >= 2) {
            uint16_t flen = (uint16_t)(tcpReceiveBuffer[0] | (tcpReceiveBuffer[1] << 8));
            if (flen == 0 || flen > OD_LAN_MAX_PAYLOAD) {
                od_log_info("LAN: invalid frame length, closing");
                disconnectWiFiServer();
                return;
            }
            if (tcpReceiveBufferPos < (uint32_t)(2 + flen)) {
                break;
            }
            // F4: tag the frame's origin so the dispatcher bypasses app-layer CCM on
            // TLS (already-secure) and routes the response back over LAN only.
            g_commandOrigin = tlsMode ? ORIGIN_LAN_TLS : ORIGIN_LAN_PLAIN;
            // Instance identity for the R4 activity test. LAN frames never traverse
            // the BLE ring, so there is no queued tag to carry: the socket is parsed
            // and dispatched within this pass, and its buffer dies with the session.
            // Publishing the live owner word directly is therefore exact -- if LAN
            // does not own the slot, the word will not match and the frame stamps
            // nothing.
            {
                const LinkId lanOwner = linkOwnerId();
                g_commandInstance = (lanOwner.who == OWNER_LAN) ? linkIdWord(lanOwner) : 0;
            }
            // No stamp here: imageDataWritten() stamps the shared clock itself,
            // and only for a RECOGNISED command from the owner -- which is the
            // whole of R4's definition and what this site could not enforce.
            imageDataWritten(NULL, NULL, tcpReceiveBuffer + 2, flen);
            g_commandInstance = 0;
            g_commandOrigin = ORIGIN_BLE;   // restore default for any subsequent BLE drain
            uint32_t consumed = 2u + (uint32_t)flen;
            uint32_t rem = tcpReceiveBufferPos - consumed;
            if (rem > 0) {
                memmove(tcpReceiveBuffer, tcpReceiveBuffer + consumed, rem);
            }
            tcpReceiveBufferPos = rem;
        }
        // A dispatched command may have torn the session down (reboot, power-off,
        // config-driven LAN restart). Never read from a dead client.
        if (!wifiServerConnected || !wifiClient.connected()) {
            return;
        }
    } while (got > 0 && drainedBytes < OD_LAN_RX_BUFFER_SIZE);
}

void restartWiFiLanAfterReconnect() {
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        return;
    }
    disconnectWiFiServer();
    startLanServer();
}

// F5 -- reboot teardown: drop the LAN client + TLS context, stop the TCP server,
// and disconnect WiFi so esp_restart() does not leave the radio/socket half-up.
// Extends PR #114's BLE-only teardown (device_control.cpp) to the WiFi surface.
void opendisplay_lan_teardown(void) {
    tlsCloseSession();
    if (wifiClient.connected()) {
        wifiClient.stop();
    }
    wifiServer.end();
    wifiServerConnected = false;
    tcpReceiveBufferPos = 0;
    if (tlsInited) {
        mbedtls_ssl_config_free(&tlsConf);
        mbedtls_ctr_drbg_free(&tlsDrbg);
        mbedtls_entropy_free(&tlsEntropy);
        tlsInited = false;
    }
    WiFi.disconnect(true);
    od_log_info("LAN/WiFi torn down before restart");
}

#endif  // OPENDISPLAY_HAS_WIFI
