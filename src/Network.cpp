#include "AsyncUDP.h"
#include <WiFiManager.h>

#define IPBROADCASTPORT 50375
#define IPBROADCASTPERIOD 10000
#include "Logging.h"
WiFiManager wifiManager;
AsyncUDP udp;
unsigned long lastIPBroadcastTime;

void setupWifi() {

  wifiManager.setConnectTimeout(10);
  wifiManager.autoConnect();
  lastIPBroadcastTime = 0;
}

// every x second, send whether platform is tracking,
// and how many seconds it will take to reach center.
//(can be negative is center passed)
void broadcastStatus(double secondsToCenter, double secondsToEnd, bool platformTracking) {
  long now = millis();
  if ((now - lastIPBroadcastTime) > IPBROADCASTPERIOD) {
    log("Preparing to send packet");
    lastIPBroadcastTime = now;
    // Check if the device is connected to the WiFi
    if (WiFi.status() != WL_CONNECTED) {
      return;
    }
    if (udp.connect(
            IPAddress(255, 255, 255, 255),
            IPBROADCASTPORT)) { // Choose any available port, e.g., 12345
      char response[400];

      snprintf(response, sizeof(response),
               "EQ:{ "
               "\"timeToCenter\": %.2lf, "
               "\"timeToEnd\": %.2lf, "
               "\"isTracking\" : %s "
               " }",
               secondsToCenter, secondsToEnd,platformTracking ? "true" : "false");
      udp.print(response);
      log("Status Packet sent");
    }
  }
}
