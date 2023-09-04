#include "AsyncUDP.h"
#include <WiFiManager.h>

#define IPBROADCASTPORT 50375
#define IPBROADCASTPERIOD 10000

WiFiManager wifiManager;
AsyncUDP udp;
unsigned long lastIPBroadcastTime;

void setupWifi() {

  wifiManager.setConnectTimeout(10);
  wifiManager.autoConnect();
  lastIPBroadcastTime = 0;
}

void broadcastIP() {
  long now = millis();
  if ((now - lastIPBroadcastTime) > IPBROADCASTPERIOD) {
    lastIPBroadcastTime=now;
    // Check if the device is connected to the WiFi
    if (WiFi.status() != WL_CONNECTED) {
      return;
    }
    if (udp.connect(
            IPAddress(255, 255, 255, 255),
            IPBROADCASTPORT)) { // Choose any available port, e.g., 12345
      udp.print("EQIP=" + WiFi.localIP().toString());
    }
  }
}
