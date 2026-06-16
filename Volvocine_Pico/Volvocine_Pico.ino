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
#include "calculateTrimmedMean.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// agent_id: 不変なのでRAMで持つだけでOK (送信時にのみ使用)
int agent_id;

// WiFi設定
const char* ssid = "Kura-Station-Ex";
const char* password = "Daruk355";
//const char* ssid = "Buffalo-G-4510";
//const char* password = "33354682";

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
#define LOG_BUFFER_SIZE   28000
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
float feedbackTauSec = 1.0f;       // 一次遅れフィルタの時定数 [s]
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
  // 正規化
  float normalized = (value - lower) / (upper - lower) - 0.5f;

  // ±0.5にクリップ
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
//   (各パケット先頭に agent_id の1バイトと送信時の時刻4バイトを付加して送る)
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

      // isServerReady()の代わりにWiFi確認だけ
      while (WiFi.status() != WL_CONNECTED) {
        Serial.println("[ERROR] WiFi disconnected. Reconnecting...");
        connectToWiFi(ssid, password);
      }

      // 1) agent_id (1バイト)
      packet[offset++] = (uint8_t)agent_id;

      // 2) 送信時刻 (4バイト)
      uint32_t sendMicros = micros();
      memcpy(&packet[offset], &sendMicros, sizeof(sendMicros));
      offset += sizeof(sendMicros);

      // 3) データパック詰め
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

      // 4) UDP送信
      udp.beginPacket(serverIP, serverPort);
      udp.write(packet, offset);
      udp.endPacket();

      Serial.printf("[INFO] Packet sent (%d records). Waiting for ACK...\n", perPacketCount);

      // 5) ACK待機
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
      i++; // 無限ループを防ぐためスキップ
    }
  }

  Serial.printf("[INFO] Sent %d records from RAM\n", sentCount);

  if (bufferOverflowed) {
    Serial.println("[WARN] Some data lost due to buffer overflow.");
    bufferOverflowed = false;
  }
}
// ---------------------------------------------------
// センサ読み取り＋RAMバッファ保存（dtはサーボ用のみ）
// ---------------------------------------------------
void logSensorData() {
  unsigned long now = micros();
  unsigned long dt = now - prevLoopEndTime;
  unsigned long elapsed = now - startLoggingMicros;
  prevLoopEndTime = now;

  int raw1 = analogRead(analogPin1);  // 0..4095
  int raw2 = analogRead(analogPin2);

  // リングバッファにデータを追加
  raw2Window[raw2Index] = raw2;
  raw2Index = (raw2Index + 1) % windowSize; // インデックスを循環させる

  // 下位10%と上位10%の値、およびその平均を計算
  auto [lowerValue, upperValue, trimmedMean] = calculateTrimmedMean(raw2Window, windowSize);

  // 正規化
  float flex = normalize((float)raw2 / 4095.0f, lowerValue, upperValue);
  float dflex = (flex - previousFlex)/dt; // 前回との差分
  previousFlex = flex; // 前回の値を更新

  // サーボ制御
  float psi = (float)elapsed / 1e6f * omega + phi;
  float zPrc = evaluatePRC(psi);
  float dtSec = (float)dt * 1e-6f;
  // 生のフィードバック項
  float feedbackRaw = kappa_now * zPrc * flex;
  // 一次遅れフィルタ: tau * df/dt = feedbackRaw - f
  if (feedbackTauSec > 0.0f) {
    float alpha = 1.0f - expf(-dtSec / feedbackTauSec);
    feedbackFiltered += alpha * (feedbackRaw - feedbackFiltered);
  } else {
    feedbackFiltered = feedbackRaw;
  }
  // フィルタ後のフィードバック項をphiに積分
  phi += feedbackFiltered * dtSec;
  float currentCos = cosf(psi);
  myServo.write(servoCenter + servoAmplitude * currentCos); // 変更点: 変数を使用

  // データ保存は指定された間隔でのみ実行
  if (loopCounter % saveInterval == 0) {
    // ログ用構造体
    CompressedLogData entry;
    entry.micros24 = now >> 8;  // 24ビットに圧縮

    // analog0: phiを [0..2π) → 0..255 に圧縮
    float phiMod = fmodf((float)elapsed / 1e6f * omega + phi, 2.0f * (float)M_PI);
    if (phiMod < 0) phiMod += 2.0f * (float)M_PI;
    entry.analog0 = (uint8_t)(phiMod * (255.0f / (2.0f * (float)M_PI)));

    entry.analog1 = (uint8_t)(raw1 >> 4);

    int extended2 = raw2 << 2;  // ×4
    if (extended2 > 4095) extended2 = 4095;
    entry.analog2 = (uint8_t)(extended2 >> 4);

    // バッファに書き込み
    if (logIndex < LOG_BUFFER_SIZE) {
      logBuffer[logIndex++] = entry;
    } else {
      // 1度だけWarnを出す
      if (!bufferOverflowed) {
        Serial.println("[WARN] log buffer overflow!");
        bufferOverflowed = true;
      }
    }

    // バッファ使用率 (10件毎に表示)
    if (logIndex % 10 == 0) {
      float usage = (float)logIndex / LOG_BUFFER_SIZE * 100.0f;
      //Serial.printf("[STATUS] buffer: %d/%d (%.1f%%)\n", logIndex, LOG_BUFFER_SIZE, usage);
    }
    // ステータスをサーバーに送信
    if (millis() - lastStatusSendTime >= STATUS_SEND_INTERVAL_MS) {
      int raw3 = analogRead(26); // 光センサ（ピン番号は適宜変更）
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
  // 周期制御
  if (dt2 < CONTROL_PERIOD_US) {
    delayMicroseconds(CONTROL_PERIOD_US - dt2);
    prevLoopEndTime2 = micros();
    //Serial.printf("[INFO] Loop took %lu us (expected %d us)\n", dt2, CONTROL_PERIOD_US);
  } else{
    prevLoopEndTime2 = micros();
    //Serial.printf("[WARN] Loop took too long: %lu us (expected %d us)\n", dt2, CONTROL_PERIOD_US);
  }

  // ループカウンタをインクリメント
  loopCounter++;
}

void setup() {
  pinMode(digitalInputPin, INPUT);
  Serial.begin(115200);
  analogReadResolution(12);
  myServo.attach(1);
  myServo.write(servoCenter);

  // agent_id を取得
  pico_unique_board_id_t id;
  pico_get_unique_board_id(&id);
  agent_id = getAgentId();
  if (agent_id < 0) {
    Serial.println("[ERROR] Unknown board! Add this UID to agent_id_mapper.cpp");
    while (true) delay(1000);  // 未登録ボードは停止
  }
  Serial.print("agent_id = ");
  Serial.println(agent_id);
  agentPort = 5000 + agent_id;

  // WiFi接続（IPはここでシリアルに表示される）
  connectToWiFi(ssid, password);

  // UDPをagent専用ポートで開く
  udp.begin(agentPort);

  // サーバー候補
  IPAddress serverIP1(192, 168, 0, 142);
  IPAddress serverIP2(192, 168, 13, 99);

  // ウォームアップ
  warmUpUDP(udp, serverIP1, 5000);
  warmUpUDP(udp, serverIP2, 5000);

  // どちらかのサーバーに接続できるまで待つ
  while (true) {
    if (isServerReady(udp, serverIP1, 5000, agent_id)) {
      serverIP = serverIP1;
      Serial.println("[INFO] Connected to server at 192.168.0.142");
      break;
    } else if (isServerReady(udp, serverIP2, 5000, agent_id)) {
      serverIP = serverIP2;
      Serial.println("[INFO] Connected to server at 192.168.13.99");
      break;
    } else {
      Serial.println("[WARN] No servers are ready. Retrying in 1 second...");
      delay(1000);
    }
  }

  Serial.printf("Loaded agent_id: %d\n", agent_id);

  // メインポートでパラメータ取得
  requestParametersFromServer(udp, serverIP, 5000, agent_id, omega, kappa,
    servoCenter, servoAmplitude, stopAgentId, stopDelaySeconds,
    feedbackTauSec, prcHarmonics, prcCosCoeffs, prcSinCoeffs, PRC_MAX_HARMONICS);

  // 以降はagent専用ポートを使う
  serverPort = agentPort;
  Serial.printf("[INFO] Switched to agent-specific port: %d\n", serverPort);

  myServo.write(servoCenter);
  Serial.println("[INFO] Servo moved to center position");

  prevLoopEndTime = micros();
  prevLoopEndTime2 = prevLoopEndTime;

  paused = true;
  logIndex = 0;
  sendLogBuffer();
  kappa_now = kappa_init;
  feedbackFiltered = 0.0f;
  srand(micros());
  Serial.println("[INFO] System is paused. Press the button to start.");
}

// UDPコマンド受信処理
void checkControlCommand() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char buf[16] = {0};
    udp.read(buf, sizeof(buf) - 1);
    if (strcmp(buf, "START") == 0 && paused == true) {
      paused = false;
      startLoggingMillis = millis(); // ログ開始時刻を記録
      startLoggingMicros = micros(); // ログ開始時刻を記録
      Serial.println("[INFO] Received START command from server.");
      t_delay = (rand() / (float)RAND_MAX) * wait_max;
      startLoggingMicros += (unsigned long)(t_delay * 1e6f);
      unsigned long nowReset = micros();
      prevLoopEndTime = nowReset;
      prevLoopEndTime2 = nowReset;
      feedbackFiltered = 0.0f;
    } else if (strcmp(buf, "STOP") == 0 && paused == false) {
      paused = true;
      Serial.println("[INFO] Received STOP command from server.");
      sendLogBuffer();
      logIndex = 0;
      kappa_now = kappa_init;
      feedbackFiltered = 0.0f;
    }
  }
}

void checkPythonCommand() {
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  char buf[32];
  udp.read(buf, sizeof(buf) - 1);

  int id;
  char cmd[8];

  sscanf(buf, "%d:%s", &id, cmd);

  if (id != agent_id) return;

  if (strcmp(cmd, "ON") == 0) paused = false;
  if (strcmp(cmd, "OFF") == 0) paused = true;
}

void loop() {
  // パケットを一度だけ読み、両コマンドを振り分ける
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char buf[32] = {0};
    udp.read(buf, sizeof(buf) - 1);

    int id;
    char cmd[16];

    if (sscanf(buf, "%d:%s", &id, cmd) == 2 && id == agent_id) {
      // Python形式: "2:ON" / "2:OFF"
      if (strcmp(cmd, "ON") == 0) {
        paused = false;
        startLoggingMillis = millis();
        startLoggingMicros = micros();
        t_delay = (rand() / (float)RAND_MAX) * wait_max;
        startLoggingMicros += (unsigned long)(t_delay * 1e6f);
        prevLoopEndTime = micros();
        prevLoopEndTime2 = prevLoopEndTime;
        feedbackFiltered = 0.0f;
      } else if (strcmp(cmd, "OFF") == 0) {
        paused = true;
        sendLogBuffer();
        logIndex = 0;
        kappa_now = kappa_init;
        feedbackFiltered = 0.0f;
      }
    } else if (strcmp(buf, "START") == 0 && paused) {
      // サーバー形式: "START"
      paused = false;
      startLoggingMillis = millis();
      startLoggingMicros = micros();
      t_delay = (rand() / (float)RAND_MAX) * wait_max;
      startLoggingMicros += (unsigned long)(t_delay * 1e6f);
      prevLoopEndTime = micros();
      prevLoopEndTime2 = prevLoopEndTime;
      feedbackFiltered = 0.0f;
      Serial.println("[INFO] Received START");
    } else if (strcmp(buf, "STOP") == 0 && !paused) {
      // サーバー形式: "STOP"
      paused = true;
      Serial.println("[INFO] Received STOP");
      sendLogBuffer();
      logIndex = 0;
      kappa_now = kappa_init;
      feedbackFiltered = 0.0f;
    }
  }

  // ポーズ中に一定間隔でパラメータをリクエスト
  if (paused && millis() - lastRequestTime >= 10000) {
    while (WiFi.status() != WL_CONNECTED) {
      connectToWiFi(ssid, password);
    }
    Serial.println("[INFO] WiFi connected.");
    
    // パラメータリクエスト時は一時的にメインポートを使用
    unsigned int tempPort = serverPort;
    serverPort = 5000; // メインポートに一時切り替え
    requestParametersFromServer(udp, serverIP, serverPort, agent_id, omega, kappa, servoCenter, servoAmplitude, stopAgentId, stopDelaySeconds, feedbackTauSec, prcHarmonics, prcCosCoeffs, prcSinCoeffs, PRC_MAX_HARMONICS);
    serverPort = tempPort; // 専用ポートに戻す
    lastRequestTime = millis();  // リクエスト送信時刻を更新
  }

  // 記録中
  if (!paused) {
    logSensorData();
  }
}