#pragma once
#include <WiFiUdp.h>
#include <IPAddress.h>

bool isServerReady(WiFiUDP& udp, IPAddress serverIP, unsigned int serverPort, int agent_id);
void warmUpUDP(WiFiUDP& udp, IPAddress serverIP, unsigned int serverPort);
bool waitForAck(WiFiUDP& udp, int agent_id, uint32_t expected_micros24, unsigned long timeout_ms);