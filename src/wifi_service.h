#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#include <stdint.h>

// OPENDISPLAY_HAS_WIFI gates the entire WiFi/LAN transport surface (mDNS, TCP
// server, TLS-PSK listener, RX reassembly buffer, LAN response framing). It is
// defined only on ESP32 targets built with -DOPENDISPLAY_ENABLE_WIFI, which is
// applied to every S3 platformio env (esp32-s3-E1004 sets no flag of its own but
// inherits it from esp32-s3-N32R8-extuart).
//
// SET THAT FLAG ONLY ON ENVS THAT ALSO SET -DBOARD_HAS_PSRAM. It is the single
// control point for two DRAM-expensive subsystems, because it is also what gates
// OPENDISPLAY_USE_TINFL (src/od_inflate_tinfl.h) -- together they cost roughly
// 50 KB of internal DRAM: 34,816 B of mbedTLS record slots (od_tls_reserve_records
// below), tinfl's 15,120 B of tables, and a 16 KB RX buffer that PSRAM builds
// relocate off the internal heap (odLanReserveRxBuffer below). A part without
// PSRAM has nowhere to put any of it -- see the DRAM-exhaustion panics that
// motivated this. Nothing in code enforces the pairing; this comment and the
// per-env notes in platformio.ini are the guard.
//
// FIVE envs therefore lack it and compile no WiFi surface, reclaiming the RX
// buffer + WiFiServer/WiFiClient RAM and falling back to uzlib for inflate:
// esp32-N4, esp32-wrover-e-N4R8 (classic ESP32) and esp32-c3-N4, esp32-c6-N4,
// esp32-c3-N16 (no PSRAM). Note esp32-wrover-e-N4R8 is NOT in platformio.ini's
// default_envs, so a bare `pio run` skips it: it ships via
// .github/firmware-targets.json, and it is the target most likely to catch a
// broken #ifndef OPENDISPLAY_HAS_WIFI path. Build it explicitly. Call sites in
// main.cpp / communication.cpp / display_service.cpp / device_control.cpp /
// config_parser.cpp are #ifdef-guarded on this macro.
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_ENABLE_WIFI)
#define OPENDISPLAY_HAS_WIFI 1
#endif

#ifdef OPENDISPLAY_HAS_WIFI

// Reserve mbedTLS's two ~16.7 KB record buffers while the heap is still contiguous, and
// route mbedTLS allocations through them. MUST be called early in setup() -- after
// full_config_init() (it needs securityConfig to know whether TLS is used) and before
// BleTransport::begin()/initWiFi() take their ~100 KB. No-op when encryption is disabled, and
// idempotent. Without it, ssl_setup() intermittently fails with -0x7f00 even with ~50 KB
// free, because the two buffers need contiguous internal DRAM.
void od_tls_reserve_records(void);

// LAN RX reassembly buffer. 16 KB = four max wire frames (OD_LAN_MAX_FRAME 4096):
// headroom for a streaming client to keep whole frames queued ahead of the parser.
//
// Declared HERE rather than in main.h so that the definition (main.h, which includes
// this header) is type-checked against it. It used to be a 16 KB array defined in
// main.h and re-declared `extern uint8_t[16384]` in wifi_service.cpp, with the size
// literal duplicated: because C++ does not mangle namespace-scope variable names,
// converting one side to a pointer and not the other would have LINKED CLEANLY and
// then read the first bytes of the buffer as a pointer.
#define OD_LAN_RX_BUFFER_SIZE 16384u
extern uint8_t* tcpReceiveBuffer;
extern uint32_t tcpReceiveBufferPos;

// Reserve the RX buffer above, preferring PSRAM so it costs no internal DRAM -- 16 KB
// that is idle during every BLE transfer anyway, since the link-owner rule makes LAN
// and BLE mutually exclusive. Falls back to internal DRAM (i.e. exactly the old static
// cost) if PSRAM is absent or dead, which CONFIG_SPIRAM_IGNORE_NOTFOUND=1 makes silent.
// Call from setup() AFTER od_tls_reserve_records(): on the fallback path this takes
// internal DRAM and must not fragment the contiguous blocks mbedTLS needs. Idempotent;
// never freed. On failure startLanServer() refuses to listen rather than accepting a
// socket the parser cannot serve.
void odLanReserveRxBuffer(void);

void initWiFi(bool waitForConnection = true);
void disconnectWiFiServer();
/**
 * Close the owned LAN socket and its TLS context, without the crypto/transfer
 * teardown that disconnectWiFiServer() also does.
 *
 * The LAN arm of abortToKnownState()'s drop step: the abort owns those other steps,
 * so this must not repeat them. Synchronous -- a TCP close needs no wait bound,
 * unlike a BLE disconnect.
 */
void wifiLanDropOwnedSocket(void);
/**
 * Tear down a LAN session whose peer has already closed the socket.
 *
 * Called early in loop(), ahead of the deferred disconnect cleanup, so the token is
 * released before the accept later in the same pass -- otherwise an ordinary
 * reconnect is refused against the departed session's token (7d step 1 before
 * step 2). No-op when there is no session or the peer is still connected.
 */
void wifiLanReapClosedSession(void);
void handleWiFiServer();

// Re-associate to the strongest AP for the configured SSID after the link degrades
// past OD_LAN_ROAM_RSSI_THRESHOLD (-75 dBm default). Self-gating: does nothing unless a
// roam is queued AND no LAN client / direct-write / pipe transfer is in flight, since
// re-association drops the TCP session and the full-channel scan steals the shared radio
// from BLE under coex. Called from handleWiFiServer() each tick.
void serviceLanRoam(void);
void restartWiFiLanAfterReconnect();
/// Publish MSD (bytes after company ID) as mDNS TXT key ``msd`` (28 hex chars). No-op if Wi-Fi down.
void opendisplay_mdns_update_msd_txt(void);
/// Tear down the LAN session, TLS context, TCP server, and WiFi before a reboot.
void opendisplay_lan_teardown(void);
/// Frame [len:2 LE][payload] and write it over the ACTIVE LAN channel (TLS or
/// plaintext). Used by communication.cpp to route LAN-origin responses.
void opendisplay_lan_send_frame(const uint8_t* payload, uint16_t len);
/// True while a LAN client is connected (plaintext or TLS).
bool wifiLanClientConnected(void);
/// Port the LAN listener binds: WifiConfig.server_port (or OD_LAN_TCP_PORT when 0),
/// +1 when the TLS-PSK channel is active. Derived, never configured directly.
uint16_t lanActivePort(void);
/// True when the LAN channel is TLS-PSK rather than plaintext (= isEncryptionEnabled()).
bool lanTlsEnabled(void);
// NOTE: this firmware deliberately never calls esp_wifi_set_ps(). WiFi and BLE share
// one radio with software coex compiled in, and the coex arbiter needs WiFi's
// modem-sleep windows to time-share the antenna with the always-on BLE advertiser.
// Forcing WIFI_PS_NONE for a transfer was measured on hardware (2026-07-23) to
// collapse throughput in both directions; the DTIM ack-ladder stall it was chasing
// was negligible. The driver default is left untouched.

#endif

#endif
