#include "agent_config.h"
#include <LittleFS.h>
#include <string.h>

static void printCurrentPrcSeries(int prcHarmonics, const float *prcCosCoeffs, const float *prcSinCoeffs, int prcMaxHarmonics) {
  int nMax = prcHarmonics;
  if (nMax < 0) {
    nMax = 0;
  }
  if (nMax > prcMaxHarmonics) {
    nMax = prcMaxHarmonics;
  }

  Serial.printf("[INFO] Current PRC Fourier series (n=0..%d):\n", nMax);
  for (int n = 0; n <= nMax; n++) {
    Serial.printf("[INFO]   a%d=%.6f, b%d=%.6f\n", n, prcCosCoeffs[n], n, prcSinCoeffs[n]);
  }
}

int readAgentIdFromFile() {

    if(!LittleFS.begin()){
        Serial.println("LittleFS Mount Failed!");
        return 4;
    }

    Serial.println("LittleFS mounted");

    File f = LittleFS.open("/config.txt", "r");

    if(!f){
        Serial.println("config.txt NOT FOUND");
        return 4;
    }

    String line = f.readStringUntil('\n');

    Serial.print("config.txt contents = ");
    Serial.println(line);

    f.close();

    return line.toInt();
}

void requestParametersFromServer(WiFiUDP &udp, IPAddress serverIP, unsigned int serverPort, int agent_id, float &omega, float &kappa, float &servoCenter, float &servoAmplitude, int &stopAgentId, int &stopDelaySeconds, float &feedbackTauSec, int &prcHarmonics, float *prcCosCoeffs, float *prcSinCoeffs, int prcMaxHarmonics) {
  // デバッグ情報を含むリクエスト文字列を作成
  int analogValue26 = analogRead(26);  // 26ピンのアナログ入力値を取得
  char requestBuffer[128]; // バッファサイズを拡張して新しいパラメータに対応
  snprintf(requestBuffer, sizeof(requestBuffer), "REQUEST_PARAMS,id:%d,analog26:%d", agent_id, analogValue26);

  // リクエスト送信
  udp.beginPacket(serverIP, serverPort);
  udp.write(requestBuffer);  // デバッグ情報付きリクエスト
  udp.endPacket();

  // 応答待機
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {  // 最大2秒待機
    int packetSize = udp.parsePacket();
    if (packetSize) {
      char buffer[1024]; // フーリエ係数を含む応答を受けるため拡張
      int len = udp.read(buffer, sizeof(buffer) - 1);
      if (len > 0) {
        buffer[len] = '\0';  // 文字列終端を追加
        char originalBuffer[1024];
        strncpy(originalBuffer, buffer, sizeof(originalBuffer) - 1);
        originalBuffer[sizeof(originalBuffer) - 1] = '\0';

        int parsedBaseFields = 0;
        int parsedPrcFields = 0;
        bool gotPrcN = false;
        int receivedPrcN = 0;
        float tempCos[16] = {0.0f};
        float tempSin[16] = {0.0f};

        char *token = strtok(buffer, ",");
        while (token != nullptr) {
          float fval = 0.0f;
          int ival = 0;
          int idx = 0;

          if (sscanf(token, "omega:%f", &fval) == 1) {
            omega = fval;
            parsedBaseFields++;
          } else if (sscanf(token, "kappa:%f", &fval) == 1) {
            kappa = fval;
            parsedBaseFields++;
          } else if (sscanf(token, "center:%f", &fval) == 1) {
            servoCenter = fval;
            parsedBaseFields++;
          } else if (sscanf(token, "amplitude:%f", &fval) == 1) {
            servoAmplitude = fval;
            parsedBaseFields++;
          } else if (sscanf(token, "stop_id:%d", &ival) == 1) {
            stopAgentId = ival;
            parsedBaseFields++;
          } else if (sscanf(token, "stop_delay:%d", &ival) == 1) {
            stopDelaySeconds = ival;
            parsedBaseFields++;
          } else if (sscanf(token, "feedback_tau:%f", &fval) == 1) {
            feedbackTauSec = fval;
            parsedBaseFields++;
          } else if (sscanf(token, "prc_n:%d", &ival) == 1) {
            receivedPrcN = ival;
            gotPrcN = true;
            parsedPrcFields++;
          } else if (sscanf(token, "prc_a%d:%f", &idx, &fval) == 2) {
            if (idx >= 0 && idx <= prcMaxHarmonics && idx < (int)(sizeof(tempCos) / sizeof(tempCos[0]))) {
              tempCos[idx] = fval;
              parsedPrcFields++;
            }
          } else if (sscanf(token, "prc_b%d:%f", &idx, &fval) == 2) {
            if (idx >= 0 && idx <= prcMaxHarmonics && idx < (int)(sizeof(tempSin) / sizeof(tempSin[0]))) {
              tempSin[idx] = fval;
              parsedPrcFields++;
            }
          }

          token = strtok(nullptr, ",");
        }

        // PRCが来ない場合は既定の1次フーリエ係数へフォールバック。
        for (int n = 0; n <= prcMaxHarmonics; n++) {
          prcCosCoeffs[n] = 0.0f;
          prcSinCoeffs[n] = 0.0f;
        }

        if (gotPrcN && receivedPrcN >= 0 && parsedPrcFields > 1) {
          if (receivedPrcN > prcMaxHarmonics) {
            receivedPrcN = prcMaxHarmonics;
          }
          prcHarmonics = receivedPrcN;
          prcCosCoeffs[0] = tempCos[0];
          prcSinCoeffs[0] = tempSin[0];
          for (int n = 1; n <= prcHarmonics; n++) {
            prcCosCoeffs[n] = tempCos[n];
            prcSinCoeffs[n] = tempSin[n];
          }
        } else {
          prcHarmonics = 1;
          prcCosCoeffs[1] = 1.0f;
          prcSinCoeffs[1] = 0.0f;
        }

        if (parsedBaseFields >= 6) {
            Serial.printf("[INFO] Received parameters: omega=%.2f, kappa=%.2f, center=%.1f, amplitude=%.1f, stop_id=%d, stop_delay=%d, feedback_tau=%.3f, prc_n=%d (parsed_prc=%d)\n", omega, kappa, servoCenter, servoAmplitude, stopAgentId, stopDelaySeconds, feedbackTauSec, prcHarmonics, parsedPrcFields);
            printCurrentPrcSeries(prcHarmonics, prcCosCoeffs, prcSinCoeffs, prcMaxHarmonics);
        } else {
          Serial.printf("[WARN] Failed to parse all base parameters. Received: %s (parsed_base=%d, parsed_prc=%d)\n", originalBuffer, parsedBaseFields, parsedPrcFields);
          printCurrentPrcSeries(prcHarmonics, prcCosCoeffs, prcSinCoeffs, prcMaxHarmonics);
        }
        return;
      }
    }
    delay(100);  // 少し待機
  }

  Serial.println("[WARN] Parameter request timed out. Keeping last received PRC.");
  printCurrentPrcSeries(prcHarmonics, prcCosCoeffs, prcSinCoeffs, prcMaxHarmonics);
}
