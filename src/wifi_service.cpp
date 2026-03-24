#ifdef TARGET_ESP32

#include "wifi_service.h"
#include "communication.h"
#include "display_service.h"
#include "structs.h"
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <string.h>

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
extern WiFiClient wifiClient;
extern bool wifiServerConnected;
extern uint32_t wifiServerLastConnectAttempt;
extern uint8_t tcpReceiveBuffer[8192];
extern uint32_t tcpReceiveBufferPos;
extern uint32_t wifiNextImageRequestTime;
extern uint32_t wifiPollInterval;
extern bool wifiImageRequestPending;
extern bool woke_from_deep_sleep;

void writeSerial(String message, bool newLine = true);
String getChipIdHex();

typedef void* BLEConnHandle;
typedef void* BLECharPtr;
void imageDataWritten(BLEConnHandle conn_hdl, BLECharPtr chr, uint8_t* data, uint16_t len);

void initWiFi() {
    writeSerial("=== Initializing WiFi ===");

    if (!(globalConfig.system_config.communication_modes & COMM_MODE_WIFI)) {
        writeSerial("WiFi not enabled in communication_modes, skipping");
        wifiInitialized = false;
        return;
    }
    if (!wifiConfigured || wifiSsid[0] == '\0' || strlen(wifiSsid) == 0) {
        writeSerial("WiFi configuration not available or SSID empty, skipping");
        wifiInitialized = false;
        return;
    }
    writeSerial("SSID: \"" + String(wifiSsid) + "\"");
    String deviceName = "OD" + getChipIdHex();
    WiFi.setAutoReconnect(true);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    wifiSsid[32] = '\0';
    wifiPassword[32] = '\0';
    writeSerial("Encryption type: 0x" + String(wifiEncryptionType, HEX));
    wifiConnected = false;
    wifiInitialized = true;
    WiFi.begin(wifiSsid, wifiPassword);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    writeSerial("Waiting for WiFi connection...");
    const int maxRetries = 3;
    const unsigned long timeoutPerRetry = 10000;
    bool connected = false;
    for (int retry = 0; retry < maxRetries && !connected; retry++) {
        unsigned long startAttempt = millis();
        bool abortCurrentRetry = false;
        while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt < timeoutPerRetry)) {
            delay(500);
            wl_status_t status = WiFi.status();
            writeSerial("WiFi status: " + String(status));
            if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
                writeSerial("Connection failed immediately (Status: " + String(status) + ")");
                abortCurrentRetry = true;
                break;
            }
        }
        if (WiFi.status() == WL_CONNECTED) {
            connected = true;
            break;
        } else {
            if (!abortCurrentRetry) {
                writeSerial("Connection attempt " + String(retry + 1) + " timed out");
            }
            if (retry < maxRetries - 1) {
                delay(2000);
            }
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        writeSerial("=== WiFi Connected Successfully ===");
        writeSerial("SSID: " + String(wifiSsid));
        writeSerial("IP Address: " + WiFi.localIP().toString());
        writeSerial("RSSI: " + String(WiFi.RSSI()) + " dBm");
        writeSerial("=== Initializing mDNS ===");
        if (MDNS.begin(deviceName.c_str())) {
            writeSerial("mDNS responder started: " + deviceName + ".local");
        } else {
            writeSerial("ERROR: Failed to start mDNS responder");
        }

        discoverAndConnectWiFiServer();

        if (woke_from_deep_sleep) {
            wifiNextImageRequestTime = 0;
            wifiPollInterval = 60;
            writeSerial("Deep sleep wake detected - will send Image Request immediately when connected");
        }
    } else {
        wifiConnected = false;
        writeSerial("=== WiFi Connection Failed ===");
        writeSerial("Final Status: " + String(WiFi.status()));
    }
}

void discoverAndConnectWiFiServer() {
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        writeSerial("WiFi not connected, cannot discover server");
        return;
    }
    if (wifiClient.connected()) {
        writeSerial("Already connected to server");
        return;
    }
    writeSerial("=== Discovering OpenDisplay Server via mDNS ===");
    int n = MDNS.queryService("opendisplay", "tcp");

    if (n == 0) {
        writeSerial("No OpenDisplay server found via mDNS");
        wifiServerLastConnectAttempt = millis();
        return;
    }
    writeSerial("Found " + String(n) + " OpenDisplay server(s)");
    String serverName;
    int serverPort = 0;
    bool hostnameValid = false;
    for (int retry = 0; retry < 5 && !hostnameValid; retry++) {
        delay(200 * (retry + 1));
        String tempHostname = MDNS.hostname(0);
        serverPort = MDNS.port(0);
        if (tempHostname.length() > 0) {
            const char* hostnameCStr = tempHostname.c_str();
            if (hostnameCStr != nullptr && hostnameCStr[0] != '\0') {
                serverName = String(hostnameCStr);
                hostnameValid = true;
            }
        }

        if (!hostnameValid && retry < 4) {
            writeSerial("Hostname not available yet, retrying... (" + String(retry + 1) + "/5)");
        }
    }
    if (serverPort == 0) {
        serverPort = 2446;
        writeSerial("Port not found in mDNS, using default: " + String(serverPort));
    }
    IPAddress serverIP;
    bool gotIPFromTxt = false;
    if (MDNS.hasTxt(0, "ip")) {
        String ipFromTxt = MDNS.txt(0, "ip");
        if (ipFromTxt.length() > 0) {
            if (serverIP.fromString(ipFromTxt)) {
                gotIPFromTxt = true;
                writeSerial("Got IP from mDNS TXT record: " + serverIP.toString());
            }
        }
    }
    if (!gotIPFromTxt && hostnameValid && serverName.length() > 0) {
        writeSerial("Server discovered:");
        writeSerial("  Name: ");
        writeSerial(serverName);
        writeSerial("  Port: " + String(serverPort));
        const char* serverNameCStr = serverName.c_str();
        if (serverNameCStr != nullptr && WiFi.hostByName(serverNameCStr, serverIP)) {
            gotIPFromTxt = true;
            writeSerial("  IP from hostname resolution: " + serverIP.toString());
        } else {
            String hostnameWithLocal = serverName + ".local";
            const char* hostnameLocalCStr = hostnameWithLocal.c_str();
            if (hostnameLocalCStr != nullptr && WiFi.hostByName(hostnameLocalCStr, serverIP)) {
                gotIPFromTxt = true;
                writeSerial("  IP from hostname resolution (.local): " + serverIP.toString());
            }
        }
    }
    if (!gotIPFromTxt) {
        writeSerial("ERROR: Could not get IP address from mDNS (TXT, hostname, or direct IP)");
        wifiServerLastConnectAttempt = millis();
        return;
    }
    writeSerial("Server discovered:");
    writeSerial("  Port: " + String(serverPort));
    writeSerial("  IP: " + serverIP.toString());
    writeSerial("=== Connecting to TCP Server ===");
    writeSerial("Server: " + serverIP.toString() + ":" + String(serverPort));
    wifiClient.setTimeout(10000);
    bool connected = wifiClient.connect(serverIP, serverPort);
    if (connected) {
        wifiServerConnected = true;
        wifiServerLastConnectAttempt = millis();
        writeSerial("=== TCP Server Connected Successfully ===");
        writeSerial("Remote IP: " + wifiClient.remoteIP().toString());
        writeSerial("Remote Port: " + String(wifiClient.remotePort()));
        sendConnectionNotification(0x01);
        delay(100);
        wifiNextImageRequestTime = 0;
        wifiPollInterval = 60;
        sendImageRequest();
    } else {
        wifiServerConnected = false;
        wifiServerLastConnectAttempt = millis();
        writeSerial("=== TCP Server Connection Failed ===");
        writeSerial("Error: " + String(wifiClient.getWriteError()));
        writeSerial("Will retry in " + String(WIFI_SERVER_RECONNECT_DELAY_MS / 1000) + " seconds");
    }
}

void disconnectWiFiServer() {
    if (wifiClient.connected()) {
        writeSerial("=== Disconnecting from TCP Server ===");
        sendConnectionNotification(0x00);
        delay(100);
        wifiClient.stop();
        wifiServerConnected = false;
        writeSerial("TCP connection closed");
    }
}

void handleWiFiServer() {
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        if (wifiServerConnected) {
            writeSerial("WiFi disconnected, closing TCP connection");
            disconnectWiFiServer();
        }
        return;
    }
    if (!wifiServerConnected) {
        uint32_t now = millis();
        if (now - wifiServerLastConnectAttempt >= WIFI_SERVER_RECONNECT_DELAY_MS) {
            discoverAndConnectWiFiServer();
        }
        return;
    }
    if (!wifiClient.connected()) {
        if (wifiServerConnected) {
            writeSerial("TCP connection lost");
            wifiServerConnected = false;
            wifiServerLastConnectAttempt = millis();
        }
        return;
    }
    int available = wifiClient.available();
    if (available > 0) {
        int bytesToRead = available;
        if (tcpReceiveBufferPos + bytesToRead > sizeof(tcpReceiveBuffer)) {
            bytesToRead = sizeof(tcpReceiveBuffer) - tcpReceiveBufferPos;
            writeSerial("WARNING: Receive buffer full, truncating data");
        }
        int bytesRead = wifiClient.read(&tcpReceiveBuffer[tcpReceiveBufferPos], bytesToRead);
        if (bytesRead > 0) {
            tcpReceiveBufferPos += bytesRead;
            uint32_t parsePos = 0;
            while (parsePos + 5 <= tcpReceiveBufferPos) {
                uint16_t packetLength = tcpReceiveBuffer[parsePos] | (tcpReceiveBuffer[parsePos + 1] << 8);
                if (packetLength < 5 || packetLength > sizeof(tcpReceiveBuffer)) {
                    writeSerial("ERROR: Invalid packet length: " + String(packetLength));
                    parsePos++;
                    continue;
                }
                if (parsePos + packetLength > tcpReceiveBufferPos) {
                    break;
                }
                writeSerial("Received TCP packet: " + String(packetLength) + " bytes");
                uint32_t packetOffset = parsePos + 2;
                if (packetLength < 5) {
                    writeSerial("ERROR: Packet too short");
                    parsePos++;
                    continue;
                }
                uint8_t version = tcpReceiveBuffer[packetOffset++];
                if (version != 0x01) {
                    writeSerial("ERROR: Unsupported protocol version: " + String(version));
                    parsePos += packetLength;
                    continue;
                }
                uint32_t dataEnd = parsePos + packetLength - 2;
                uint32_t currentOffset = packetOffset;
                while (currentOffset + 2 <= dataEnd) {
                    (void)tcpReceiveBuffer[currentOffset++];
                    uint8_t packetId = tcpReceiveBuffer[currentOffset++];
                    uint16_t payloadLen = dataEnd - currentOffset;
                    uint8_t* payload = &tcpReceiveBuffer[currentOffset];
                    if (packetId == 0x81) {
                        if (payloadLen >= 4) {
                            uint32_t pollInterval = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
                            writeSerial("Server response: No new image, poll again in " + String(pollInterval) + " seconds");
                            wifiPollInterval = pollInterval;
                            wifiNextImageRequestTime = millis() + (pollInterval * 1000);
                            wifiImageRequestPending = false;
                            writeSerial("Next Image Request scheduled in " + String(pollInterval) + " seconds");
                        } else {
                            writeSerial("ERROR: Packet 0x81 payload too short: " + String(payloadLen));
                        }
                    } else if (packetId == 0x82) {
                        if (payloadLen >= 7) {
                            uint16_t imageLength = payload[0] | (payload[1] << 8);
                            uint32_t pollInterval = payload[2] | (payload[3] << 8) | (payload[4] << 16) | (payload[5] << 24);
                            uint8_t refreshType = payload[6];

                            writeSerial("Server response: New image (" + String(imageLength) + " bytes), poll again in " + String(pollInterval) + " seconds, refresh_type=" + String(refreshType));
                            if (payloadLen >= 7 + imageLength) {
                                uint8_t* imageData = payload + 7;
                                uint8_t emptyData[4] = {0, 0, 0, 0};
                                handleDirectWriteStart(emptyData, 0);
                                uint16_t remaining = imageLength;
                                uint16_t offset = 0;
                                while (remaining > 0) {
                                    uint16_t chunkSize = (remaining > 512) ? 512 : remaining;
                                    handleDirectWriteData(imageData + offset, chunkSize);
                                    offset += chunkSize;
                                    remaining -= chunkSize;
                                }
                                uint8_t refreshData = refreshType;
                                handleDirectWriteEnd(&refreshData, 1);
                                wifiPollInterval = pollInterval;
                                wifiNextImageRequestTime = millis() + (pollInterval * 1000);
                                wifiImageRequestPending = false;
                                writeSerial("Next Image Request scheduled in " + String(pollInterval) + " seconds");
                            } else {
                                writeSerial("ERROR: Incomplete image data (have " + String(payloadLen - 7) + ", need " + String(imageLength) + ")");
                            }
                        } else {
                            writeSerial("ERROR: Packet 0x82 payload too short: " + String(payloadLen));
                        }
                    } else if (packetId == 0x83) {
                        writeSerial("Server requests configuration, sending Display Announcement");
                        sendDisplayAnnouncement();
                    } else {
                        if (payloadLen >= 3) {
                            imageDataWritten(NULL, NULL, payload + 1, payloadLen - 1);
                        } else {
                            writeSerial("ERROR: Unknown packet ID 0x" + String(packetId, HEX) + ", payload too short: " + String(payloadLen));
                        }
                    }
                    break;
                }
                parsePos += packetLength;
            }
            if (parsePos > 0) {
                uint32_t remaining = tcpReceiveBufferPos - parsePos;
                if (remaining > 0) {
                    memmove(tcpReceiveBuffer, &tcpReceiveBuffer[parsePos], remaining);
                }
                tcpReceiveBufferPos = remaining;
            }
        }
    }
}

#endif
