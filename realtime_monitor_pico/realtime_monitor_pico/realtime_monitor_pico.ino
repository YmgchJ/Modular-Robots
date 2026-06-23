// realtime_monitor.ino
//
// 【仕様】
//   - トライアルの開始・停止は「物理ボタンのみ」で行う。
//   - 記録中（トライアル中）は、WiFi自体は繋いだままだが、OTAチェック・PCからの
//     コマンド受信処理を一切行わない（通信処理ゼロ）。
//     ※以前はWiFi.disconnect()で物理的に切断していたが、再接続後にUDPソケットが
//       不安定になり、PCからのコマンドを受信できなくなる問題があったため、
//       「繋いだままにして、処理だけスキップする」方式に変更した。
//   - ボタンでトライアルを停止すると、OTA・コマンド受信処理を再開する。
//   - PCから "REQUEST_DATA" コマンドを受信したら、直近のトライアルの全データを
//     ACK付きで確実にPCへ転送する。
//
// ★ Core1は使わない。wifi_config.h から setup1()/loop1() を削除済みであることが前提。
//
// 必要ライブラリ: Servo, Wire, WiFi, WiFiUdp, pico_ota
// wifi_config.h は本ファイルと同じフォルダに置いてください（ssid/password/hostname を定義）

#include <pico_ota.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifi_config.h"   // ssid, password, hostname を定義

// ---------------------------------------------------
// PC側の受信スクリプトのIP・ポート、およびコマンド受信用のポートをここで指定
// ---------------------------------------------------
IPAddress pcIP(192, 168, 0, 145);   // ← 受信側PCのIPアドレス
const unsigned int pcPort = 6000;   // ← Python側スクリプトの「データ受信用」ポートと合わせる
const unsigned int commandPort = 6001;  // ← Python側スクリプトの「コマンド送信先」ポートと合わせる

WiFiUDP udp;

Servo myServo;

// ピン設定
const int servoPin = 1;
const int switchPin = 4;
const int analogPin1 = 27;
const int analogPin2 = 28;   // 曲げセンサ

// 動作パラメータ
const int centerAngle = 90;
const int amplitude = 60;
const float frequencyHz = 1.25f;

// INA226設定
uint8_t ina226Addr = 0x40;
const float SHUNT_OHMS = 0.056f;
bool inaReady = false;

bool motionMode = false;       // true=トライアル中（記録中・WiFi切断状態）
bool lastSwitchState = true;   // INPUT_PULLUPなので未押下時はHIGH
unsigned long lastToggleTime = 0;
const unsigned long debounceMs = 200;
float phase = 0.0f;
unsigned long lastPhaseUpdateMs = 0;

unsigned long lastSampleMs = 0;
const unsigned long sampleIntervalMs = 20;  // 50Hzでバッファに記録

// OTA処理の呼び出し間隔（待機中のみ呼ぶ）
unsigned long lastOtaLoopMs = 0;
const unsigned long otaLoopIntervalMs = 5;

// ---------------------------------------------------
// トライアル記録用バッファ（START〜STOPの間、全サンプルを保持。WiFiは使わない）
// ---------------------------------------------------
#pragma pack(push, 1)
struct LogEntry {
  uint32_t tMs;
  uint8_t  angle;
  uint16_t flexRaw;
  int16_t  currentCenti;  // 電流[mA]を10倍した値
};
#pragma pack(pop)

#define LOG_BUFFER_SIZE 15000  // 50Hzで約5分。RAM使用量 約135KB
LogEntry logBuffer[LOG_BUFFER_SIZE];
int logIndex = 0;
bool bufferOverflowed = false;
unsigned long trialStartMs = 0;

// ---------------------------------------------------
// INA226 ヘルパー
// ---------------------------------------------------
bool inaWriteReg16(uint8_t reg, uint16_t value) {
  Wire1.beginTransmission(ina226Addr);
  Wire1.write(reg);
  Wire1.write((uint8_t)(value >> 8));
  Wire1.write((uint8_t)(value & 0xFF));
  return Wire1.endTransmission() == 0;
}

bool inaReadReg16(uint8_t reg, uint16_t &value) {
  Wire1.beginTransmission(ina226Addr);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((int)ina226Addr, 2) != 2) return false;
  value = ((uint16_t)Wire1.read() << 8) | (uint16_t)Wire1.read();
  return true;
}

void initIna226() {
  inaWriteReg16(0x00, 0x0007);  // AVG=1, VBUSCT=140us, VSHCT=140us, MODE=111(連続)
}

bool detectIna226Address() {
  for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      ina226Addr = addr;
      return true;
    }
  }
  return false;
}

bool readIna226Measurement(float &currentmA, float &busVoltV) {
  uint16_t shuntRawU16 = 0, busRawU16 = 0;
  if (!inaReadReg16(0x01, shuntRawU16) || !inaReadReg16(0x02, busRawU16)) {
    return false;
  }
  int16_t shuntRaw = (int16_t)shuntRawU16;
  float shuntVoltV = (float)shuntRaw * 2.5e-6f;
  currentmA = (shuntVoltV / SHUNT_OHMS) * 1000.0f;
  busVoltV = (float)busRawU16 * 1.25e-3f;
  return true;
}

// ---------------------------------------------------
// 待機中(WiFi接続済み)に呼ぶ: バッファ内の直近トライアルデータをACK付きで確実に転送する
// ---------------------------------------------------
void sendLogBufferReliable() {
  if (logIndex == 0) {
    if (Serial) Serial.println("[DUMP] No trial data available. Skipping transfer.");
    return;
  }

  if (Serial) Serial.printf("[DUMP] Starting reliable transfer of %d samples...\n", logIndex);

  while (WiFi.status() != WL_CONNECTED) {
    if (Serial) Serial.println("[DUMP] Waiting for WiFi to (re)connect before transferring data...");
    unsigned long waitStart = millis();
    while (millis() - waitStart < 2000) {
      otaLoop();
      delay(50);
    }
  }

  udp.beginPacket(pcIP, pcPort);
  udp.write((const uint8_t *)"DUMPSTART\n", 10);
  udp.endPacket();
  delay(50);

  const int maxPacketBytes = 480;
  char packet[520];
  const int maxRetries = 50;
  int i = 0;
  uint32_t batchId = 0;

  while (i < logIndex) {
    batchId++;
    int retry = 0;
    bool acked = false;

    while (retry < maxRetries && !acked) {
      while (WiFi.status() != WL_CONNECTED) {
        if (Serial) Serial.println("[DUMP] WiFi dropped during transfer. Waiting to reconnect...");
        unsigned long waitStart = millis();
        while (millis() - waitStart < 2000) {
          otaLoop();
          delay(50);
        }
      }

      int startIndex = i;
      int offset = snprintf(packet, sizeof(packet), "BATCH:%lu\n", (unsigned long)batchId);
      int count = 0;

      while (i < logIndex) {
        char line[40];
        int lineLen = snprintf(line, sizeof(line), "%lu,%u,%u,%d\n",
                                (unsigned long)logBuffer[i].tMs,
                                (unsigned int)logBuffer[i].angle,
                                (unsigned int)logBuffer[i].flexRaw,
                                (int)logBuffer[i].currentCenti);
        if (offset + lineLen >= maxPacketBytes) break;
        memcpy(packet + offset, line, lineLen);
        offset += lineLen;
        i++;
        count++;
      }

      udp.beginPacket(pcIP, pcPort);
      udp.write((const uint8_t *)packet, offset);
      udp.endPacket();

      char expectedAck[32];
      snprintf(expectedAck, sizeof(expectedAck), "ACK:%lu", (unsigned long)batchId);

      unsigned long ackWaitStart = millis();
      while (millis() - ackWaitStart < 1000) {
        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
          char buf[32] = {0};
          int len = udp.read(buf, sizeof(buf) - 1);
          if (len > 0) {
            buf[len] = '\0';
            if (strcmp(buf, expectedAck) == 0) {
              acked = true;
              break;
            }
          }
        }
        delay(5);
      }

      if (!acked) {
        i = startIndex;
        retry++;
        if (Serial) Serial.printf("[DUMP] Batch %lu not acked (retry %d/%d)\n",
                       (unsigned long)batchId, retry, maxRetries);
        delay(50);
      } else {
        if (Serial) Serial.printf("[DUMP] Batch %lu acked (%d samples, %d/%d total)\n",
                       (unsigned long)batchId, count, i, logIndex);
      }
    }

    if (!acked) {
      if (Serial) Serial.println("[DUMP] Max retries reached for a batch. Aborting transfer.");
      break;
    }
  }

  udp.beginPacket(pcIP, pcPort);
  udp.write((const uint8_t *)"DUMPEND\n", 8);
  udp.endPacket();

  if (Serial) Serial.println("[DUMP] Transfer finished.");
}

// ---------------------------------------------------
// setup
// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();
  Wire1.setClock(400000);
  inaReady = detectIna226Address();
  if (inaReady) initIna226();

  pinMode(switchPin, INPUT_PULLUP);  // ボタン未押下=HIGH、押下=LOW（フローティング防止）
  analogReadResolution(12);
  myServo.attach(servoPin);
  myServo.write(centerAngle);
  lastPhaseUpdateMs = millis();

  // 起動時にWiFi接続（OTA書き込み・最初の動作確認用）。
  // トライアル開始（ボタン押下）と同時に切断するので、これは「待機状態」のための接続。
  if (Serial) Serial.println("[Setup] Connecting WiFi via otaSetupWithTimeout...");
  bool ok = otaSetupWithTimeout(ssid, password, 20000, hostname, "", true);
  if (ok) {
    if (Serial) Serial.print("[Setup] WiFi connected. IP: ");
    if (Serial) Serial.println(WiFi.localIP());
  } else {
    if (Serial) Serial.println("[Setup] WiFi connect failed (timeout). Button control still works; "
                    "data request will retry connecting when needed.");
  }

  udp.begin(commandPort);

  lastSwitchState = digitalRead(switchPin);
  if (Serial) Serial.println("[Setup] Ready. Press the button to start a trial (no PC connection needed).");
}

// ---------------------------------------------------
// loop
// ---------------------------------------------------
void loop() {
  // --- 待機中（トライアル中でない）の時だけ、OTA・PCからのコマンドを処理する ---
  if (!motionMode) {
    unsigned long nowMsForOta = millis();
    if (WiFi.status() == WL_CONNECTED && nowMsForOta - lastOtaLoopMs >= otaLoopIntervalMs) {
      lastOtaLoopMs = nowMsForOta;
      otaLoop();
    }

    int cmdPacketSize = udp.parsePacket();
    if (cmdPacketSize > 0) {
      char cmdBuf[32] = {0};
      int cmdLen = udp.read(cmdBuf, sizeof(cmdBuf) - 1);
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        if (strcmp(cmdBuf, "REQUEST_DATA") == 0) {
          if (Serial) Serial.println("[CMD] REQUEST_DATA received. Sending latest trial data...");
          sendLogBufferReliable();
        }
      }
    }
  }

  // --- ボタンでトライアルの開始・停止をトグル ---
  bool currentSwitchState = digitalRead(switchPin);  // INPUT_PULLUP: 押下時LOW
  if (currentSwitchState == LOW && lastSwitchState == HIGH) {
    unsigned long now = millis();
    if (now - lastToggleTime >= debounceMs) {
      lastToggleTime = now;

      if (!motionMode) {
        // --- トライアル開始: WiFiは繋いだままにする。
        //     通信処理（OTAチェック・コマンド受信）自体は、上の `if (!motionMode)` ブロックで
        //     スキップされるので、実質的に「記録中は通信を行わない」状態になる。
        //     ※以前はWiFi.disconnect()で物理的に切断していたが、再接続後にUDPソケットが
        //       不安定になる問題があったため、切断はせず繋ぎっぱなしにする方式に変更した。
        if (Serial) Serial.println("[SWITCH] Starting trial. (WiFi stays connected, but no "
                                    "communication processing happens until the trial stops.)");

        motionMode = true;
        phase = 0.0f;
        lastPhaseUpdateMs = now;
        trialStartMs = now;
        logIndex = 0;
        bufferOverflowed = false;
      } else {
        // --- トライアル停止: 通信処理（OTA・コマンド受信）を再開する ---
        // WiFiはずっと繋がったままなので、再接続処理は不要。
        motionMode = false;
        if (Serial) Serial.printf("[SWITCH] Trial stopped. %d samples recorded. "
                       "Waiting for PC to send REQUEST_DATA.\n", logIndex);
      }
    }
  }
  lastSwitchState = currentSwitchState;

  // --- サーボ角度の更新 ---
  int angle = centerAngle;
  if (motionMode) {
    unsigned long now = millis();
    float dt = (now - lastPhaseUpdateMs) / 1000.0f;
    lastPhaseUpdateMs = now;

    phase += 2.0f * PI * frequencyHz * dt;
    if (phase >= 2.0f * PI) phase = fmodf(phase, 2.0f * PI);

    float wave = sinf(phase);
    angle = (int)(centerAngle + amplitude * wave);
    myServo.write(angle);
  }

  // --- 曲げセンサ・電流の読み取りとバッファ記録（トライアル中のみ。WiFiとは無関係） ---
  if (motionMode) {
    unsigned long nowMs = millis();
    if (nowMs - lastSampleMs >= sampleIntervalMs) {
      lastSampleMs = nowMs;

      int rawFlex = analogRead(analogPin2);
      float currentmA = 0.0f;
      float busVoltV = 0.0f;
      bool inaOk = inaReady ? readIna226Measurement(currentmA, busVoltV) : false;

      if (logIndex < LOG_BUFFER_SIZE) {
        LogEntry &entry = logBuffer[logIndex++];
        entry.tMs = nowMs - trialStartMs;
        entry.angle = (uint8_t)constrain(angle, 0, 255);
        entry.flexRaw = (uint16_t)rawFlex;
        entry.currentCenti = (int16_t)((inaOk ? currentmA : 0.0f) * 10.0f);
      } else if (!bufferOverflowed) {
        bufferOverflowed = true;
        if (Serial) Serial.println("[WARN] Log buffer full! Further samples in this trial will be lost.");
      }
    }
  }
}