#include "wifi_service.h"

#ifdef OPENDISPLAY_HAS_WIFI

#include "communication.h"
#include "encryption.h"
#include "structs.h"
#include "od_log.h"
#include "ble_init.h"   // NimBLE-Arduino + BLE* aliases (NimBLEDevice for the advertised MAC)
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
extern uint8_t tcpReceiveBuffer[16384];
extern uint32_t tcpReceiveBufferPos;
extern uint8_t msd_payload[16];

// This file builds its log lines with Arduino String concatenation, while the rest
// of the firmware logs printf-style through od_log_*. Rather than reflow 60-odd
// call sites into format strings, funnel them through one adapter and route by the
// message's own ERROR:/WARNING: prefix so the intended level survives. The String
// is built by the caller either way, so this adds no allocation the call sites did
// not already make -- and the EVENT-CONTEXT RULE below still forbids calling it
// from a WiFi event callback.
static void lanLog(const String& message, bool newLine = true) {
    (void)newLine;
    if (message.startsWith("ERROR")) {
        od_log_error("%s", message.c_str());
    } else if (message.startsWith("WARNING")) {
        od_log_warn("%s", message.c_str());
    } else {
        od_log_info("%s", message.c_str());
    }
}

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

typedef void* BLEConnHandle;
typedef void* BLECharPtr;
void imageDataWritten(BLEConnHandle conn_hdl, BLECharPtr chr, uint8_t* data, uint16_t len);

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

static uint32_t lastLanActivityMs = 0;  // for the OD_LAN_READ_TIMEOUT_S idle drop

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
static String tlsFailNote(int ret) {
    const String code = (ret < 0) ? ("-0x" + String((unsigned)(-ret), HEX)) : String(ret);
    return String(" (ret=") + code +
           ", internal free=" + String((unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL)) +
           ", largest block=" + String((unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)) + ")";
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
// disconnect, so TLS cycles 33 KB through the heap per connection while String logging
// churns small blocks in between: the odds degrade the longer the device stays up.
//
// Fix: reserve the two slots ONCE at boot -- from setup(), right after full_config_init()
// and BEFORE ble_init()/initWiFi() take their ~100 KB -- when the heap is still
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
            lanLog("WARNING: TLS alloc " + String((unsigned)total) +
                        " B not served from the reserved pool (slot " +
                        String((unsigned)OD_TLS_RECORD_SLOT_SIZE) + " B) -- falling back to heap");
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
        lanLog("TLS: encryption disabled, no record buffers reserved");
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
    lanLog("TLS: reserved " + String(got) + "/" + String((int)OD_TLS_RECORD_SLOTS) +
                " record slots of " + String((unsigned)OD_TLS_RECORD_SLOT_SIZE) + " B" +
                ", internal free=" + String((unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL)) +
                ", largest block=" + String((unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)));
    if (got < OD_TLS_RECORD_SLOTS) {
        lanLog("WARNING: TLS record reservation incomplete -- ssl_setup may still fail");
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
        lanLog("ERROR: TLS PSK derivation failed (no master key)");
        return false;
    }
    mbedtls_ssl_config_init(&tlsConf);
    mbedtls_ctr_drbg_init(&tlsDrbg);
    mbedtls_entropy_init(&tlsEntropy);
    const char* pers = "opendisplay-tls";
    int rc = mbedtls_ctr_drbg_seed(&tlsDrbg, mbedtls_entropy_func, &tlsEntropy,
                                   reinterpret_cast<const unsigned char*>(pers), strlen(pers));
    if (rc != 0) {
        lanLog("ERROR: TLS RNG seed failed" + tlsFailNote(rc));
        return false;
    }
    rc = mbedtls_ssl_config_defaults(&tlsConf, MBEDTLS_SSL_IS_SERVER,
                                     MBEDTLS_SSL_TRANSPORT_STREAM,
                                     MBEDTLS_SSL_PRESET_DEFAULT);
    if (rc != 0) {
        lanLog("ERROR: TLS config defaults failed" + tlsFailNote(rc));
        return false;
    }
    mbedtls_ssl_conf_rng(&tlsConf, mbedtls_ctr_drbg_random, &tlsDrbg);
    mbedtls_ssl_conf_ciphersuites(&tlsConf, kTlsCiphersuites);
    rc = mbedtls_ssl_conf_psk(&tlsConf, tlsPsk, sizeof(tlsPsk),
                              reinterpret_cast<const unsigned char*>(kTlsPskIdentity),
                              strlen(kTlsPskIdentity));
    if (rc != 0) {
        lanLog("ERROR: TLS conf_psk failed" + tlsFailNote(rc));
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
        lanLog("ERROR: TLS ssl_setup failed" + tlsFailNote(rc));
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
                lanLog("ERROR: TLS LAN response write failed", true);
            }
        } else if (wifiClient.write(lanTxFrame, total) != total) {
            lanLog("ERROR: LAN response write incomplete", true);
        }
        return;
    }

    // Oversized fallback: header then payload. A failure between the two leaves the
    // peer waiting on a length prefix whose payload never arrives.
    uint8_t hdr[2] = { (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
    if (tlsMode) {
        if (mbedtls_ssl_write(&tlsSsl, hdr, 2) < 0 ||
            mbedtls_ssl_write(&tlsSsl, payload, len) < 0) {
            lanLog("ERROR: TLS LAN response write failed", true);
        }
        return;
    }
    if (wifiClient.write(hdr, 2) != 2 || wifiClient.write(payload, len) != len) {
        lanLog("ERROR: LAN response write incomplete", true);
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
// `mac`). This is a genuinely new call: prior identity used getChipIdHex()
// (eFuse), which is NOT what HA stores as the device unique_id. HARDWARE
// VALIDATION REQUIRED: confirm NimBLEDevice::getAddress() == the advertised AdvA
// HA sees (public vs static-random) on BOTH S3 and C6.
static String advertisedBleMacLower(void) {
    auto s = NimBLEDevice::getAddress().toString();
    String out(s.c_str());
    out.toLowerCase();
    return out;
}

static void restartLanService(void) {
    String deviceName = "OD" + getChipIdHex();
    if (!MDNS.begin(deviceName.c_str())) {
        lanLog("ERROR: mDNS responder failed");
        return;
    }
    uint16_t port = lanActivePort();
    lanLog("mDNS: " + deviceName + ".local");
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
    lanLog("mDNS: _opendisplay._tcp port " + String(port) +
                " tls=" + (isEncryptionEnabled() ? "1" : "0") + " mac=" + mac);
    opendisplay_mdns_update_msd_txt();
}

static void startLanServer(void) {
    tlsMode = isEncryptionEnabled();
    uint16_t port = lanActivePort();
    wifiServer.begin(port);
    lanLog(String(tlsMode ? "TLS-PSK" : "Plaintext") +
                " LAN server listening on port " + String(port));
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
        lanLog(String("WiFi: AP cache cleared (") + why + ") -- next connect will scan");
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
        lanLog("WiFi: connecting to cached AP " + String(b) +
                    " ch " + String((int)s_cachedChannel) + " (no scan)");
        WiFi.begin(wifiSsid, wifiPassword, (int32_t)s_cachedChannel, s_cachedBssid);
        return;
    }
    usingCachedAp = false;
    lanLog("WiFi: scanning all channels for the strongest AP");
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
        lanLog("WiFi: roaming armed (RSSI-low handler registered)");
        return;
    }
    // Always report the code -- "could not register" with no reason is undiagnosable.
    lanLog(String("WiFi: RSSI-low handler register failed (") + esp_err_to_name(rc) +
                "); will retry on next association");
}

// One-shot: re-arm after every event and after every association.
static void lanArmRssiThreshold(void) {
    ensureRoamEventRegistered();   // no point arming a threshold nothing listens for
    esp_err_t rc = esp_wifi_set_rssi_threshold(OD_LAN_ROAM_RSSI_THRESHOLD);
    if (rc != ESP_OK) {
        lanLog("WiFi: RSSI threshold arm failed (" + String((int)rc) + ")");
    }
}

// EVENT-CONTEXT RULE (learned the hard way -- this handler previously panicked the
// device): callbacks here run on tiny stacks -- the raw esp_event handler on the system
// event task, and the Arduino callbacks on "arduino_events" with a 4096-byte stack
// (ARDUINO_NETWORK_EVENT_TASK_STACK_SIZE). Arduino String concatenation plus
// lanLog(String) BY VALUE, plus esp_wifi_*/esp_event_* calls, is far too much for
// that budget. So handlers ONLY set flags; every String, log, and esp_* call happens in
// serviceWifiEventFollowUp() on the loop task. Keep it that way.
static void onWifiRssiLow(void* arg, esp_event_base_t base, int32_t id, void* data) {
    (void)arg; (void)base; (void)id;
    const wifi_event_bss_rssi_low_t* e = (const wifi_event_bss_rssi_low_t*)data;
    s_lastRssiLowDbm = e ? (int)e->rssi : 0;   // plain int store; logged from loop()
    roamPending = true;
    rssiLowNoticePending = true;
}

// Flags only -- see the EVENT-CONTEXT RULE above. No String, no lanLog, no esp_*
// calls: this runs on the 4096-byte "arduino_events" task.
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
        lanLog("WiFi event: associated (channel " + String((int)s_lastAssocChannel) + ")");
    }
    if (disconnectNoticePending) {
        disconnectNoticePending = false;
        lanLog("WiFi event: disconnected, reason " + String((int)s_lastDisconnectReason));
    }
    if (cacheFailPending) {
        cacheFailPending = false;
        lanCacheInvalidate("cached BSSID did not associate");
    }
    if (rssiLowNoticePending) {
        rssiLowNoticePending = false;
        lanLog("WiFi: RSSI " + String((int)s_lastRssiLowDbm) + " dBm below " +
                    String(OD_LAN_ROAM_RSSI_THRESHOLD) + " dBm -- roam queued (deferred to idle)");
        lanArmRssiThreshold();   // one-shot: re-arm so a deferred roam still re-triggers
    }
    if (gotIpPending) {
        gotIpPending = false;
        lanLog("WiFi event: got IP " + IPAddress((uint32_t)s_lastGotIp).toString() +
                    ", RSSI " + String(WiFi.RSSI()) + " dBm, ch " + String(WiFi.channel()) +
                    ", BSSID " + WiFi.BSSIDstr());
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
        lanLog("WiFi: cached AP unreachable -- falling back to a full scan");
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
    lanLog("WiFi: roaming -- re-associating to the strongest AP for this SSID");
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
    lanLog("=== Initializing WiFi ===");

    // WiFi is NOT gated on power_mode: if COMM_MODE_WIFI is enabled the radio comes
    // up on battery too. Radio cost on battery is managed by the driver's default
    // power-save mode and by deep sleep, not by refusing to associate.
    if (!(globalConfig.system_config.communication_modes & COMM_MODE_WIFI)) {
        lanLog("WiFi not enabled in communication_modes, skipping");
        wifiInitialized = false;
        return;
    }
    if (!wifiConfigured) {
        lanLog("WiFi: system_config has WiFi mode on, but wifi_config TLV (0x26) is not in saved "
                    "configuration (or failed to parse). Enable Wi-Fi in config, set SSID, and write full "
                    "config to the device.");
        wifiInitialized = false;
        return;
    }
    if (wifiSsid[0] == '\0' || strlen(wifiSsid) == 0) {
        lanLog("WiFi: wifi_config packet present but SSID field is empty.");
        wifiInitialized = false;
        return;
    }
    // Do not log the SSID or password (credentials); log only presence/length.
    lanLog("WiFi: connecting to configured SSID (len " + String(strlen(wifiSsid)) + ")");
    registerWiFiDiagEvents();
    WiFi.setAutoReconnect(true);
    wifiSsid[32] = '\0';
    wifiPassword[32] = '\0';
    lanLog("Encryption type: 0x" + String(wifiEncryptionType, HEX));
    wifiConnected = false;
    wifiInitialized = true;
    // Cached BSSID when RTC memory still holds one (the deep-sleep-wake fast path:
    // single channel, no scan), otherwise an all-channel scan for the strongest AP.
    lanBeginConnect();
    // Tx power can only be set once the STA is started, i.e. after begin(); the
    // pre-begin() call this replaces failed with ESP_ERR_WIFI_NOT_START.
    WiFi.setTxPower(WIFI_POWER_15dBm);
    if (!waitForConnection) {
        lanLog("WiFi: STA started (non-blocking; LAN starts when associated)");
        return;
    }
    lanLog("Waiting for WiFi connection...");
    const int maxRetries = 3;
    const unsigned long timeoutPerRetry = 10000;
    bool connected = false;
    for (int retry = 0; retry < maxRetries && !connected; retry++) {
        unsigned long startAttempt = millis();
        bool abortCurrentRetry = false;
        while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt < timeoutPerRetry)) {
            delay(500);
            wl_status_t status = WiFi.status();
            lanLog("WiFi status: " + String(status));
            if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
                lanLog("Connection failed immediately (Status: " + String(status) + ")");
                abortCurrentRetry = true;
                break;
            }
        }
        if (WiFi.status() == WL_CONNECTED) {
            connected = true;
            break;
        }
        if (!abortCurrentRetry) {
            lanLog("WiFi attempt " + String(retry + 1) + " timed out");
        }
        if (retry < maxRetries - 1) {
            delay(2000);
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        lanLog("=== WiFi connected ===");
        lanLog("IP: " + WiFi.localIP().toString());
        startLanServer();
    } else {
        wifiConnected = false;
        lanLog("=== WiFi connection failed ===");
    }
}

void disconnectWiFiServer() {
    tlsCloseSession();
    if (wifiClient.connected()) {
        lanLog("Closing LAN client");
        clearEncryptionSession();
        wifiClient.stop();
    }
    wifiServerConnected = false;
    tcpReceiveBufferPos = 0;
    // F4: abort any in-flight direct-write / pipe / partial transfer + tear down a
    // mid-transfer panel session, DEFERRED to loop() (serviceBleDisconnectCleanup)
    // so cleanup never races an in-progress EPD refresh. Reuses the BLE path's flag.
    bleDisconnectCleanupPending = true;
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
    int space = (int)sizeof(tcpReceiveBuffer) - (int)tcpReceiveBufferPos;
    if (space <= 0) {
        lanLog("LAN RX buffer full, dropping connection");
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

void handleWiFiServer() {
    // Execute a queued roam (RSSI dropped below OD_LAN_ROAM_RSSI_THRESHOLD) before any
    // other work; it self-gates on idle, so this is a no-op mid-transfer.
    serviceLanRoam();

    if (wifiInitialized && WiFi.status() == WL_CONNECTED && !wifiConnected) {
        wifiConnected = true;
        lanLog("=== WiFi connected ===");
        lanLog("IP: " + WiFi.localIP().toString() +
                    ", RSSI " + String(WiFi.RSSI()) + " dBm, ch " + String(WiFi.channel()) +
                    ", BSSID " + WiFi.BSSIDstr());
        startLanServer();
    }
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        if (wifiServerConnected || wifiClient.connected()) {
            lanLog("WiFi lost, closing LAN session");
            disconnectWiFiServer();
        }
        return;
    }

    WiFiClient incoming = wifiServer.accept();
    if (incoming) {
        if (wifiClient.connected()) {
            lanLog("LAN: new client, replacing previous");
            tlsCloseSession();
            clearEncryptionSession();
            wifiClient.stop();
        }
        wifiClient = incoming;
        // TCP_NODELAY: every LAN write is a complete, self-delimited frame, so there
        // is never a following write for Nagle to coalesce it with -- it can only
        // hold a small frame until the peer's delayed ACK fires (40-200 ms). With
        // per-chunk direct-write ACKs that lands on every frame of a transfer.
        wifiClient.setNoDelay(true);
        wifiClient.setTimeout(30000);
        tcpReceiveBufferPos = 0;
        wifiServerConnected = true;
        lastLanActivityMs = millis();
        lanLog("LAN client connected from " + wifiClient.remoteIP().toString());
        if (tlsMode) {
            if (!tlsBeginSession()) {
                lanLog("LAN: TLS session start failed, dropping");
                disconnectWiFiServer();
                return;
            }
        }
    }

    if (!wifiServerConnected || !wifiClient.connected()) {
        if (wifiServerConnected) {
            lanLog("LAN client disconnected");
            disconnectWiFiServer();
        }
        return;
    }

    // Drive the TLS handshake incrementally; return until it completes.
    if (tlsMode && tlsSessionActive && !tlsHandshakeDone) {
        int hs = mbedtls_ssl_handshake(&tlsSsl);
        if (hs == 0) {
            tlsHandshakeDone = true;
            lastLanActivityMs = millis();
            lanLog("LAN: TLS handshake complete");
        } else if (hs == MBEDTLS_ERR_SSL_WANT_READ || hs == MBEDTLS_ERR_SSL_WANT_WRITE) {
            // still handshaking; but honor the idle timeout below
        } else {
            // The handshake struct is another internal-DRAM allocation, so annotate the
            // heap here too -- an OOM mid-handshake looks like a protocol error otherwise.
            lanLog("LAN: TLS handshake failed" + tlsFailNote(hs) + ", dropping");
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
            lanLog("LAN: client closed the connection");
            disconnectWiFiServer();
            return;
        }
        if (got < 0) {
            // Real fault -- name the mbedTLS code, otherwise this is undiagnosable.
            lanLog("LAN: channel read error" + tlsFailNote(s_lastLanReadErr) + ", dropping");
            disconnectWiFiServer();
            return;
        }
        if (got > 0) {
            lastLanActivityMs = millis();
            drainedBytes += (uint32_t)got;
        } else if (drainedBytes == 0) {
            // No traffic at all this tick: drop only after OD_LAN_READ_TIMEOUT_S
            // of silence (persistent client is otherwise kept). Any valid frame
            // below resets the timer.
            if ((millis() - lastLanActivityMs) > (uint32_t)OD_LAN_READ_TIMEOUT_S * 1000UL) {
                lanLog("LAN: idle timeout, dropping client");
                disconnectWiFiServer();
            }
            return;
        }

        while (tcpReceiveBufferPos >= 2) {
            uint16_t flen = (uint16_t)(tcpReceiveBuffer[0] | (tcpReceiveBuffer[1] << 8));
            if (flen == 0 || flen > OD_LAN_MAX_PAYLOAD) {
                lanLog("LAN: invalid frame length, closing");
                disconnectWiFiServer();
                return;
            }
            if (tcpReceiveBufferPos < (uint32_t)(2 + flen)) {
                break;
            }
            // F4: tag the frame's origin so the dispatcher bypasses app-layer CCM on
            // TLS (already-secure) and routes the response back over LAN only.
            g_commandOrigin = tlsMode ? ORIGIN_LAN_TLS : ORIGIN_LAN_PLAIN;
            lastLanActivityMs = millis();
            imageDataWritten(NULL, NULL, tcpReceiveBuffer + 2, flen);
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
    } while (got > 0 && drainedBytes < sizeof(tcpReceiveBuffer));
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
    lanLog("LAN/WiFi torn down before restart", true);
}

#endif  // OPENDISPLAY_HAS_WIFI
