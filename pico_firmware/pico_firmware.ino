#include <pico_ota.h>
#include "pico/unique_id.h"
#include "agent_id_mapper.h"
#include <Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <math.h> 
#include <vector>
#include <algorithm>
#include <tuple> // std::tupleを使用するために必要
#include <cstdlib>   // rand(), srand()
#include "agent_config.h"
#include "ServerUtils.h"
#include "WiFiManager.h"
#include "wifi_config.h"
#include "calculateTrimmedMean.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// agent_id: 不変なのでRAMで持つだけでOK (送信時にのみ使用)
int agent_id;

// UDP設定
IPAddress serverIP(192, 168, 13, 98);
unsigned int serverPort = 5000; // 最初はメインポートに接続
unsigned int agentPort; // エージェント専用ポート
unsigned long lastStatusSendTime = 0;
const unsigned long STATUS_SEND_INTERVAL_MS = 100; // 100msごとに送信
WiFiUDP udp;

// ピン設定
const int digitalInputPin = 4;  // ボタン
const int analogPin1 = 27; 
const int analogPin2 = 28;

Servo myServo;

// 1レコード6バイトの圧縮構造体 (RAM保持用)
#pragma pack(push, 1)
struct CompressedLogData {
  uint32_t micros24 : 24;  // 3バイト: (micros >> 8)
  uint8_t  analog0;        // 1バイト
  uint8_t  analog1;        // 1バイト
  uint8_t  analog2;        // 1バイト
};
#pragma pack(pop)

#define CONTROL_PERIOD_US 2000 // 制御周期 (μs)
#define LOG_BUFFER_SIZE   18000
CompressedLogData logBuffer[LOG_BUFFER_SIZE];
int logIndex = 0;
bool paused = false;
bool lastButtonState = false;

// START受信時のログ開始時刻
unsigned long startLoggingMillis = 0;
unsigned long startLoggingMicros = 0;
float t_delay;

unsigned long prevLoopEndTime = 0;
unsigned long prevLoopEndTime2 = 0;
float phi = 0;
float omega = 3.0f * 3.14f;
float kappa = 1.0f;  // フィードバックゲイン
float kappa_init = 0.0f;
float kappa_now = 0.0f;

// ★変更箇所1：フィルタを軽くして反応を見やすくする (1.0f -> 0.1f)
float feedbackTauSec = 0.1f;       // 一次遅れフィルタの時定数 [s]

float feedbackFiltered = 0.0f;     // 一次遅れ後のフィードバック項
const int PRC_MAX_HARMONICS = 10;
int prcHarmonics = 1;
float prcCosCoeffs[PRC_MAX_HARMONICS + 1] = {0.0f};
float prcSinCoeffs[PRC_MAX_HARMONICS + 1] = {0.0f};
bool bufferOverflowed = false;
float wait_max = 2.0f * M_PI / omega;
float previousFlex = 0.0f; // 前回のフレックスセンサ値

// サーボ制御用パラメータ (サーバーから受信)
float servoCenter = 110.0f;    // サーボ中心角度のデフォルト値
float servoAmplitude = 60.0f; // サーボ振幅のデフォルト値

// 停止制御用パラメータ (サーバーから受信)
int stopAgentId = 0; // 停止対象のエージェントID (0は特殊な意味を持つ場合など)
int stopDelaySeconds = 0; // 停止までの秒数

// データ保存間隔を設定 (例: 5ループごとに保存)
const int saveInterval = 5;
int loopCounter = 0;

unsigned long lastRequestTime = 0;  // 最後にリクエストを送信した時刻

// 窓サイズを定義
const int windowSize = 1000; // 必要なサイズに変更
std::vector<int> raw2Window(windowSize, 0); // 固定サイズのリングバッファ
int raw2Index = 0; // 現在のインデックスを管理

// 正規化する関数
float normalize(float value, float lower, float upper) {
  float normalized = (value - lower) / (upper - lower) - 0.5f;
  if (normalized > 0.5f) {
    normalized = 0.5f;
  } else if (normalized < -0.5f) {
    normalized = -0.5f;
  }
  return normalized;
}

float evaluatePRC(float psi) {
  float z = prcCosCoeffs[0];
  int nMax = prcHarmonics;
  if (nMax > PRC_MAX_HARMONICS) {
    nMax = PRC_MAX_HARMONICS;
  }
  for (int n = 1; n <= nMax; n++) {
    float npsi = (float)n * psi;
    z += prcCosCoeffs[n] * cosf(npsi) + prcSinCoeffs[n] * sinf(npsi);
  }
  return z;
}

// ---------------------------------------------------
// 送信バッファをまとめてUDP送信
// ---------------------------------------------------
void sendLogBuffer() {
  const int maxPacketBytes = 512;
  uint8_t packet[maxPacketBytes];
  const int maxRetries = 100;
  int sentCount = 0;
  int i = 0;

  while (i < logIndex) {
    int retry = 0;
    bool ackReceived = false;
    while (retry < maxRetries && !ackReceived) {
      size_t offset = 0;
      int startIndex = i;

      while (WiFi.status() != WL_CONNECTED) {
        Serial.println("[ERROR] WiFi disconnected. Reconnecting...");
        while (!connectToAnyWiFi()) {
          Serial.println("[WiFi] All failed. Retrying...");
          delay(3000);
        }
      }

      packet[offset++] = (uint8_t)agent_id;
      uint32_t sendMicros = micros();
      memcpy(&packet[offset], &sendMicros, sizeof(sendMicros));
      offset += sizeof(sendMicros);

      int perPacketCount = 0;
      uint32_t lastMicros24 = 0;

      while (i < logIndex) {
        if (offset + sizeof(CompressedLogData) > maxPacketBytes) break;
        uint32_t micros24Value = logBuffer[i].micros24;
        memcpy(&packet[offset], &micros24Value, 3);
        offset += 3;
        memcpy(&packet[offset], &logBuffer[i].analog0, sizeof(CompressedLogData) - 3);
        offset += sizeof(CompressedLogData) - 3;
        lastMicros24 = micros24Value;
        i++;
        perPacketCount++;
      }

      udp.beginPacket(serverIP, serverPort);
      udp.write(packet, offset);
      udp.endPacket();

      Serial.printf("[INFO] Packet sent (%d records). Waiting for ACK...\n", perPacketCount);
      ackReceived = waitForAck(udp, agent_id, lastMicros24, 1000);
      if (!ackReceived) {
        retry++;
        Serial.printf("[WARN] ACK not received (retry %d/%d). Resending...\n", retry, maxRetries);
        i = startIndex;
        delay(100);
      } else {
        sentCount += perPacketCount;
      }
    }
    if (!ackReceived) {
      Serial.println("[ERROR] Failed after max retries. Skipping packet.");
      i++; 
    }
  }

  Serial.printf("[INFO] Sent %d records from RAM\n", sentCount);
  if (bufferOverflowed) {
    Serial.println("[WARN] Some data lost due to buffer overflow.");
    bufferOverflowed = false;
  }
}

// ---------------------------------------------------
// センサ読み取り＋サーボ制御処理
// ---------------------------------------------------
void logSensorData() {
  unsigned long now = micros();
  unsigned long dt = now - prevLoopEndTime;
  unsigned long elapsed = now - startLoggingMicros;
  prevLoopEndTime = now;

  int raw1 = analogRead(analogPin1);  
  int raw2 = analogRead(analogPin2);

  raw2Window[raw2Index] = raw2;
  raw2Index = (raw2Index + 1) % windowSize; 

  auto [lowerValue, upperValue, trimmedMean] = calculateTrimmedMean(raw2Window, windowSize);

  float flex = normalize((float)raw2 / 4095.0f, lowerValue, upperValue);
  float dflex = (flex - previousFlex)/dt; 
  previousFlex = flex; 

  float psi = (float)elapsed / 1e6f * omega + phi;
  float zPrc = evaluatePRC(psi);
  float dtSec = (float)dt * 1e-6f;
  
  float feedbackRaw = kappa_now * zPrc * flex;
  if (feedbackTauSec > 0.0f) {
    float alpha = 1.0f - expf(-dtSec / feedbackTauSec);
    feedbackFiltered += alpha * (feedbackRaw - feedbackFiltered);
  } else {
    feedbackFiltered = feedbackRaw;
  }
  
  phi += feedbackFiltered * dtSec;
  float currentCos = cosf(psi);
  myServo.write(servoCenter + servoAmplitude * currentCos); 
  
  // デバッグ用出力を追加（シリアルプロッタで確認可能）
  //Serial.printf("flex:%.3f, raw_fb:%.3f, flt_fb:%.3f, phi:%.3f\n", 
  //            flex, feedbackRaw, feedbackFiltered, phi);

  if (loopCounter % saveInterval == 0) {
    CompressedLogData entry;
    entry.micros24 = now >> 8;  

    float phiMod = fmodf((float)elapsed / 1e6f * omega + phi, 2.0f * (float)M_PI);
    if (phiMod < 0) phiMod += 2.0f * (float)M_PI;
    entry.analog0 = (uint8_t)(phiMod * (255.0f / (2.0f * (float)M_PI)));
    entry.analog1 = (uint8_t)(raw1 >> 4);

    int extended2 = raw2 << 2;  
    if (extended2 > 4095) extended2 = 4095;
    entry.analog2 = (uint8_t)(extended2 >> 4);

    if (logIndex < LOG_BUFFER_SIZE) {
      logBuffer[logIndex++] = entry;
    } else {
      if (!bufferOverflowed) {
        Serial.println("[WARN] log buffer overflow!");
        bufferOverflowed = true;
      }
    }

    if (millis() - lastStatusSendTime >= STATUS_SEND_INTERVAL_MS) {
      int raw3 = analogRead(26); 
      float servoAngle = servoCenter + servoAmplitude * cosf(psi);
      char statusBuf[128];
      snprintf(statusBuf, sizeof(statusBuf),
        "STATUS,id:%d,angle:%.1f,flex:%.4f,light:%d,current:%d",
        agent_id, servoAngle, flex, raw3, raw1
      );
      udp.beginPacket(serverIP, serverPort);
      udp.write(statusBuf);
      udp.endPacket();
      lastStatusSendTime = millis();
    }
  }

  unsigned long now2 = micros();
  unsigned long dt2 = now2 - prevLoopEndTime2;
  if (dt2 < CONTROL_PERIOD_US) {
    delayMicroseconds(CONTROL_PERIOD_US - dt2);
    prevLoopEndTime2 = micros();
  } else{
    prevLoopEndTime2 = micros();
  }

  loopCounter++;
}

void setup() {
  pinMode(digitalInputPin, INPUT_PULLUP);
  Serial.begin(115200);
  analogReadResolution(12);
  myServo.attach(1);
  myServo.write(servoCenter);

  pico_unique_board_id_t id;
  pico_get_unique_board_id(&id);
  agent_id = getAgentId();
  if (agent_id < 0) {
    Serial.println("[ERROR] Unknown board! Add this UID to agent_id_mapper.cpp");
    while (true) delay(1000);  
  }
  Serial.print("agent_id = ");
  Serial.println(agent_id);
  agentPort = 5000 + agent_id;

  while (!connectToAnyWiFi()) {
    Serial.println("[WiFi] All failed. Retrying...");
    delay(3000);
  }

  udp.begin(agentPort);
  Serial.printf("Loaded agent_id: %d\n", agent_id);
  Serial.println("[INFO] Using local parameters.");

  myServo.write(servoCenter);
  Serial.println("[INFO] Servo moved to center position");

  prevLoopEndTime = micros();
  prevLoopEndTime2 = prevLoopEndTime;

  paused = true;
  logIndex = 0;
  kappa_now = kappa_init;
  feedbackFiltered = 0.0f;
  srand(micros());
  Serial.println("[INFO] System is paused. Press the button to start.");
  lastButtonState = digitalRead(digitalInputPin);
}

void loop() {
  // --- 1. UDPコマンド受信処理 ---
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char buf[32] = {0};
    udp.read(buf, sizeof(buf) - 1);
    int id; char cmd[16];
    if (sscanf(buf, "%d:%s", &id, cmd) == 2 && id == agent_id) {
      if (strcmp(cmd, "ON") == 0) {
        paused = false; startLoggingMillis = millis(); startLoggingMicros = micros();
        t_delay = (rand() / (float)RAND_MAX) * wait_max; startLoggingMicros += (unsigned long)(t_delay * 1e6f);
        prevLoopEndTime = micros(); prevLoopEndTime2 = prevLoopEndTime; feedbackFiltered = 0.0f;
        
        // UDPからのON受信時にも念のためセット
        kappa_now = kappa;
        prcHarmonics = 1;
        prcCosCoeffs[1] = 1.0f;
      } else if (strcmp(cmd, "OFF") == 0) {
        paused = true; logIndex = 0; kappa_now = kappa_init; feedbackFiltered = 0.0f;
      }
    } else if (strcmp(buf, "START") == 0 && paused) {
      paused = false; startLoggingMillis = millis(); startLoggingMicros = micros();
      t_delay = (rand() / (float)RAND_MAX) * wait_max; startLoggingMicros += (unsigned long)(t_delay * 1e6f);
      prevLoopEndTime = micros(); prevLoopEndTime2 = prevLoopEndTime; feedbackFiltered = 0.0f;
      Serial.println("[INFO] Received START");
      
      // UDPからのSTART受信時にも念のためセット
      kappa_now = kappa;
      prcHarmonics = 1;
      prcCosCoeffs[1] = 1.0f;
    } else if (strcmp(buf, "STOP") == 0 && !paused) {
      paused = true; Serial.println("[INFO] Received STOP");
      logIndex = 0; kappa_now = kappa_init; feedbackFiltered = 0.0f;
    }
  }

  // --- 2. ボタン処理 ---
  bool currentButtonState = digitalRead(digitalInputPin);

  // ボタンが「押された瞬間（HIGHからLOWになった時）」だけを検知
  if (currentButtonState == LOW && lastButtonState == HIGH) {
      delay(20); // チャタリング防止
      if (digitalRead(digitalInputPin) == LOW) { 
          
          paused = !paused; // 状態を反転（トグル）

          if (paused) {
              // ストップ処理
              Serial.println("[INFO] Button -> STOP (OFF)");
              logIndex = 0;
              kappa_now = kappa_init;
              feedbackFiltered = 0.0f;
          } else {
              // スタート処理
              Serial.println("[INFO] Button -> START (ON)");
              startLoggingMillis = millis();
              startLoggingMicros = micros();

              t_delay = (rand() / (float)RAND_MAX) * wait_max;
              startLoggingMicros += (unsigned long)(t_delay * 1e6f);

              prevLoopEndTime = micros();
              prevLoopEndTime2 = prevLoopEndTime;
              feedbackFiltered = 0.0f;

              // ★変更箇所2：サーバー通信なしでフィードバックを有効化するためのパラメータ強制設定
              kappa_now = kappa;       // ゲインをゼロから有効な値(1.0)にする
              prcHarmonics = 1;        // PRC（位相応答曲線）の計算を有効化
              prcCosCoeffs[1] = 1.0f;  // Cos波の係数を1.0にしてフィードバックを生かす
          }
      }
  }
  lastButtonState = currentButtonState;

  // --- 3. メイン処理の実行 ---
  if (!paused) {
      logSensorData(); 
  }
}