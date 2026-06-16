#include <WiFi.h>
#include <Arduino.h>
#include <pico_ota.h>

// ===== WiFiリスト：新しい場所に行ったらここに追加するだけ =====
struct WiFiCredential { const char* ssid; const char* password; };
const WiFiCredential wifiList[] = {
  { "Kura-Station-Ex", "Daruk355"  },
  //{ "Buffalo-G-4510",  "33354682"  },
  // { "新しいSSID",   "パスワード" },  ← 追加はここ
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

// ===== OTA：Core1専用 =====
void setup1() {
  // WiFi接続完了を待ってからOTA開始
  while (WiFi.status() != WL_CONNECTED) delay(100);
  otaSetup(WiFi.SSID().c_str(), "", "pico-ota", "admin");
}

void loop1() {
  otaLoop();
}