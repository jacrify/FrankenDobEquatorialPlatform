#include "AsyncUDP.h"
#include <WiFiManager.h>

#include "Network.h"
#define IPBROADCASTPORT 50375
#define IPBROADCASTPERIOD 10000
#include "Logging.h"
WiFiManager wifiManager;
AsyncUDP udp;
unsigned long lastIPBroadcastTime;

void loopNetwork(Preferences &prefs) {
  if (digitalRead(0) == LOW) {
    prefs.putBool("homeWifi", true);
    prefs.end();

    delay(300);
    esp_restart();
  }
}

#define HOMEWIFISSID "HOMEWIFISSID"
#define HOMEWIFIPASS "HOMEWIFIPASS"

void setupWifi(Preferences &prefs) {
  log("Scanning for networks...");


  int n = WiFi.scanNetworks();

  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == "dontlookup") {
      log("Connecting to 'dontlookup'...");
      WiFi.begin("dontlookup", "dontlookdown");
      return;
    }
  }
  if (prefs.isKey(HOMEWIFISSID) && prefs.isKey(HOMEWIFIPASS)) {
    log("Connnecting to home wifi...");
    WiFi.begin(prefs.getString(HOMEWIFISSID), prefs.getString(HOMEWIFIPASS));

  } else {
    log("No wifi details in prefs");
  }
}

// every x second, send whether platform is tracking,
// and how many seconds it will take to reach center.
//(can be negative is center passed)
//AxixMoveRate is max speed in degrees per second
void broadcastStatus(double secondsToCenter, double secondsToEnd,
                     bool platformTracking,double axisMoveRate) {
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
               "\"axisMoveRate\": %.2lf, "
               " }",
               secondsToCenter, secondsToEnd,
               platformTracking ? "true" : "false",
               axisMoveRate);
      udp.print(response);
      log("Status Packet sent");
    }
  }
}
