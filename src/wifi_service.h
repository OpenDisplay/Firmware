#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#ifdef TARGET_ESP32

void initWiFi(bool waitForConnection = true);
void disconnectWiFiServer();
void handleWiFiServer();
void restartWiFiLanAfterReconnect();
/// Publish MSD (bytes after company ID) as mDNS TXT key ``msd`` (28 hex chars). No-op if Wi-Fi down.
void opendisplay_mdns_update_msd_txt(void);

#endif

#endif
