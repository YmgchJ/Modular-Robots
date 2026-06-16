#include "ServerUtils.h"
#include <Arduino.h>

bool isServerReady(WiFiUDP& udp, IPAddress serverIP, unsigned int serverPort, int agent_id) {
  char handshakeMessage[16];
  snprintf(handshakeMessage, sizeof(handshakeMessage), "HELLO:%d", agent_id);
  const int timeoutMs = 1000;
  char response[10];

  udp.beginPacket(serverIP, serverPort);
  udp.write((const uint8_t*)handshakeMessage, strlen(handshakeMessage));
  udp.endPacket();

  unsigned long startTime = millis();
  while (millis() - startTime < timeoutMs) {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      int len = udp.read(response, sizeof(response) - 1);
      if (len > 0) {
        response[len] = '\0';
        if (strcmp(response, "READY") == 0) {
          Serial.println("[INFO] Server is ready.");
          return true;
        }
      }
    }
  }
  Serial.println("[WARN] No response from server.");
  return false;
}

void warmUpUDP(WiFiUDP& udp, IPAddress serverIP, unsigned int serverPort) {
  udp.beginPacket(serverIP, serverPort);
  udp.write((uint8_t)0);
  udp.endPacket();
  delay(50);
}

bool waitForAck(WiFiUDP& udp, int agent_id, uint32_t expected_micros24, unsigned long timeout_ms) {
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    int len = udp.parsePacket();
    if (len >= 4) {
      uint8_t buffer[6] = {0};
      udp.read(buffer, len);
      if ((uint8_t)buffer[0] != agent_id) continue;
      uint32_t receivedMicros24 = buffer[1] | (buffer[2] << 8) | (buffer[3] << 16);
      if (receivedMicros24 == expected_micros24) {
        Serial.println("[INFO] ACK received.");
        return true;
      } else {
        Serial.printf("[WARN] ACK mismatch: expected %lu, got %lu\n", expected_micros24, receivedMicros24);
      }
    }
    delay(10);
  }
  return false;
}