#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#include <stdint.h>

// OPENDISPLAY_HAS_WIFI gates the entire WiFi/LAN transport surface (mDNS, TCP
// server, TLS-PSK listener, RX reassembly buffer, LAN response framing). It is
// defined only on ESP32 targets built with -DOPENDISPLAY_ENABLE_WIFI, which is
// applied to the S3, C6, and C3 platformio envs. The classic esp32-N4 builds
// without it, so it does not compile the WiFi surface and reclaims the 8 KB RX
// buffer + WiFiServer/WiFiClient RAM. Call sites in main.cpp /
// communication.cpp / display_service.cpp are #ifdef-guarded on this macro.
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_ENABLE_WIFI)
#define OPENDISPLAY_HAS_WIFI 1
#endif

#ifdef OPENDISPLAY_HAS_WIFI

// Reserve mbedTLS's two ~16.7 KB record buffers while the heap is still contiguous, and
// route mbedTLS allocations through them. MUST be called early in setup() -- after
// full_config_init() (it needs securityConfig to know whether TLS is used) and before
// ble_init()/initWiFi() take their ~100 KB. No-op when encryption is disabled, and
// idempotent. Without it, ssl_setup() intermittently fails with -0x7f00 even with ~50 KB
// free, because the two buffers need contiguous internal DRAM.
void od_tls_reserve_records(void);

void initWiFi(bool waitForConnection = true);
void disconnectWiFiServer();
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
/// Drop to WIFI_PS_NONE for the duration of a LAN-origin streaming transfer.
/// Modem sleep only guarantees the radio listens at each DTIM beacon, so the
/// per-chunk ack ladder of a direct/partial write can stall up to DTIM x 102.4 ms
/// on every inbound frame. Idempotent; no-op when WiFi is down or already suspended.
void lanPowerSaveSuspend(void);
/// Restore WIFI_PS_MIN_MODEM. Idempotent; no-op when not suspended. Called from the
/// transfer teardown funnels, so it must tolerate being invoked when no transfer ran.
void lanPowerSaveRestore(void);
/// True while power save is suspended for a transfer (for the loop() safety net).
bool lanPowerSaveSuspended(void);

#endif

#endif
