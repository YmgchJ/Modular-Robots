#include "WiFiManager.h"
#include <Arduino.h>

void connectToWiFi(const char* ssid, const char* password) {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  delay(500);  // 初期接続待機
  // WiFi接続待機ループ
  while (WiFi.status() != WL_CONNECTED) {
    delay(3000);
    Serial.print(".");
    //WiFi.disconnect();
    WiFi.begin(ssid, password);
  }

  Serial.println("\nWiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}