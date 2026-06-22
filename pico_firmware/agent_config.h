#ifndef AGENT_CONFIG_H
#define AGENT_CONFIG_H

#include <Arduino.h> // WiFi.hやWiFiUdp.hより前にインクルードすることが推奨される場合がある
#include <WiFi.h>
#include <WiFiUdp.h>
// #pragma once // 通常、インクルードガードがあれば不要、またはファイルの先頭に置く

// サーバから omega, kappa, servoCenter, servoAmplitude, stopAgentId, stopDelaySeconds, feedbackTauSec と PRC係数を取得する関数
void requestParametersFromServer(WiFiUDP &udp, IPAddress serverIP, unsigned int serverPort, int agent_id, float &omega, float &kappa, float &servoCenter, float &servoAmplitude, int &stopAgentId, int &stopDelaySeconds, float &feedbackTauSec, int &prcHarmonics, float *prcCosCoeffs, float *prcSinCoeffs, int prcMaxHarmonics);

// agent_id をファイルから読み取る関数
int readAgentIdFromFile();

#endif  // AGENT_CONFIG_H