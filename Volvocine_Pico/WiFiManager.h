#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>

// WiFi接続を初期化する関数
void connectToWiFi(const char* ssid, const char* password);

#endif // WIFI_MANAGER_H