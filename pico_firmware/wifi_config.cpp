#include <pico_ota.h>
#include <WiFi.h>
#include <Arduino.h>

struct WiFiCredential { const char* ssid; const char* password; };
const WiFiCredential wifiList[] = {
  { "Kura-Station-Ex", "Daruk355" },
  { "Buffalo-G-4510",  "33354682" },
  // 追加はここ
};
const int wifiCount = sizeof(wifiList) / sizeof(wifiList[0]);

bool connectToAnyWiFi() {
  for (int i = 0; i < wifiCount; i++) {
    Serial.printf("[WiFi] Trying: %s\n", wifiList[i].ssid);
    WiFi.begin(wifiList[i].ssid, wifiList[i].password);
    unsigned long start = millis();
    while (millis() - start < 8000) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] Connected to: %s  IP: %s\n",
          wifiList[i].ssid, WiFi.localIP().toString().c_str());
        return true;
      }
      delay(500);
    }
    WiFi.disconnect();
    delay(200);
  }
  return false;
}

// Core1専用：OTA処理（READMEのデュアルコア方式に準拠）
void setup1() {
  // Core0のWiFi接続完了を待つ
  while (WiFi.status() != WL_CONNECTED) delay(100);

  // PICO_OTAライブラリのAPI使用（ssid/passwordは不要、すでに接続済み）
  otaSetup(wifiList[0].ssid, wifiList[0].password, "pico-ota", "");
  Serial.println("[OTA] Ready");
}

void loop1() {
  otaLoop();
}