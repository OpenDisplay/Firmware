#ifdef TARGET_ESP32

#include "wifi_service.h"
#include "communication.h"
#include "encryption.h"
#include "structs.h"
#include "od_log.h"
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <string.h>

#ifndef COMM_MODE_WIFI
#define COMM_MODE_WIFI (1 << 2)
#endif

#ifndef WIFI_LAN_MAX_PAYLOAD
#define WIFI_LAN_MAX_PAYLOAD 4096U
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
extern uint8_t tcpReceiveBuffer[8192];
extern uint32_t tcpReceiveBufferPos;
extern uint8_t msd_payload[16];

String getChipIdHex();

typedef void* BLEConnHandle;
typedef void* BLECharPtr;
void imageDataWritten(BLEConnHandle conn_hdl, BLECharPtr chr, uint8_t* data, uint16_t len);

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

static void restartLanService(void) {
    String deviceName = "OD" + getChipIdHex();
    if (!MDNS.begin(deviceName.c_str())) {
        od_log_error("ERROR: mDNS responder failed");
        return;
    }
    od_log_info("mDNS: %s.local", deviceName.c_str());
    MDNS.addService("opendisplay", "tcp", wifiServerPort);
    od_log_info("mDNS: advertised _opendisplay._tcp port %u", wifiServerPort);
    opendisplay_mdns_update_msd_txt();
}

void initWiFi(bool waitForConnection) {
    od_log_info("=== Initializing WiFi ===");

    if (!(globalConfig.system_config.communication_modes & COMM_MODE_WIFI)) {
        od_log_info("WiFi not enabled in communication_modes, skipping");
        wifiInitialized = false;
        return;
    }
    if (!wifiConfigured) {
        od_log_warn("WiFi: system_config has WiFi mode on, but wifi_config TLV (0x26) is not in saved "
                    "configuration (or failed to parse). Enable Wi-Fi in config, set SSID, and write full "
                    "config to the device.");
        wifiInitialized = false;
        return;
    }
    if (wifiSsid[0] == '\0' || strlen(wifiSsid) == 0) {
        od_log_warn("WiFi: wifi_config packet present but SSID field is empty.");
        wifiInitialized = false;
        return;
    }
    od_log_info("SSID: \"%s\"", wifiSsid);
    WiFi.setAutoReconnect(true);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    wifiSsid[32] = '\0';
    wifiPassword[32] = '\0';
    od_log_info("Encryption type: 0x%02X", wifiEncryptionType);
    wifiConnected = false;
    wifiInitialized = true;
    WiFi.begin(wifiSsid, wifiPassword);
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
            od_log_debug("WiFi status: %d", status);
            if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
                od_log_warn("Connection failed immediately (Status: %d)", status);
                abortCurrentRetry = true;
                break;
            }
        }
        if (WiFi.status() == WL_CONNECTED) {
            connected = true;
            break;
        }
        if (!abortCurrentRetry) {
            od_log_warn("WiFi attempt %d timed out", retry + 1);
        }
        if (retry < maxRetries - 1) {
            delay(2000);
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        od_log_info("=== WiFi connected ===");
        String ip = WiFi.localIP().toString();
        od_log_info("IP: %s", ip.c_str());
        wifiServer.begin(wifiServerPort);
        od_log_info("TCP server listening on port %u", wifiServerPort);
        restartLanService();
    } else {
        wifiConnected = false;
        od_log_warn("=== WiFi connection failed ===");
    }
}

void disconnectWiFiServer() {
    if (wifiClient.connected()) {
        od_log_info("Closing LAN client");
        clearEncryptionSession();
        wifiClient.stop();
    }
    wifiServerConnected = false;
    tcpReceiveBufferPos = 0;
}

void handleWiFiServer() {
    if (wifiInitialized && WiFi.status() == WL_CONNECTED && !wifiConnected) {
        wifiConnected = true;
        od_log_info("=== WiFi connected ===");
        String ip = WiFi.localIP().toString();
        od_log_info("IP: %s", ip.c_str());
        wifiServer.begin(wifiServerPort);
        od_log_info("TCP server listening on port %u", wifiServerPort);
        restartLanService();
    }
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        if (wifiServerConnected || wifiClient.connected()) {
            od_log_warn("WiFi lost, closing LAN session");
            disconnectWiFiServer();
        }
        return;
    }

    WiFiClient incoming = wifiServer.accept();
    if (incoming) {
        if (wifiClient.connected()) {
            od_log_info("LAN: new client, replacing previous");
            clearEncryptionSession();
            wifiClient.stop();
        }
        wifiClient = incoming;
        wifiClient.setTimeout(30000);
        tcpReceiveBufferPos = 0;
        wifiServerConnected = true;
        String remoteIp = wifiClient.remoteIP().toString();
        od_log_info("LAN client connected from %s", remoteIp.c_str());
    }

    if (!wifiServerConnected || !wifiClient.connected()) {
        if (wifiServerConnected) {
            od_log_info("LAN client disconnected");
            clearEncryptionSession();
            wifiServerConnected = false;
            tcpReceiveBufferPos = 0;
        }
        return;
    }

    int available = wifiClient.available();
    if (available <= 0) {
        return;
    }
    int bytesToRead = available;
    if (tcpReceiveBufferPos + bytesToRead > (int)sizeof(tcpReceiveBuffer)) {
        bytesToRead = (int)sizeof(tcpReceiveBuffer) - (int)tcpReceiveBufferPos;
    }
    if (bytesToRead <= 0) {
        od_log_warn("LAN RX buffer full, dropping connection");
        disconnectWiFiServer();
        return;
    }
    int bytesRead = wifiClient.read(&tcpReceiveBuffer[tcpReceiveBufferPos], bytesToRead);
    if (bytesRead > 0) {
        tcpReceiveBufferPos += (uint32_t)bytesRead;
    }

    while (tcpReceiveBufferPos >= 2) {
        uint16_t flen = (uint16_t)(tcpReceiveBuffer[0] | (tcpReceiveBuffer[1] << 8));
        if (flen == 0 || flen > WIFI_LAN_MAX_PAYLOAD) {
            od_log_warn("LAN: invalid frame length, closing");
            disconnectWiFiServer();
            return;
        }
        if (tcpReceiveBufferPos < (uint32_t)(2 + flen)) {
            break;
        }
        imageDataWritten(NULL, NULL, tcpReceiveBuffer + 2, flen);
        uint32_t consumed = 2u + (uint32_t)flen;
        uint32_t rem = tcpReceiveBufferPos - consumed;
        if (rem > 0) {
            memmove(tcpReceiveBuffer, tcpReceiveBuffer + consumed, rem);
        }
        tcpReceiveBufferPos = rem;
    }
}

void restartWiFiLanAfterReconnect() {
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        return;
    }
    disconnectWiFiServer();
    wifiServer.begin(wifiServerPort);
    od_log_info("TCP server restarted on port %u", wifiServerPort);
    restartLanService();
}

#endif
