#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#ifdef TARGET_ESP32

#define WIFI_SERVER_RECONNECT_DELAY_MS 30000U

void initWiFi();
void discoverAndConnectWiFiServer();
void disconnectWiFiServer();
void handleWiFiServer();

#endif

#endif
