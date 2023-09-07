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

    delay(300);
    esp_restart();
  }
}

void setupWifi(Preferences &prefs) {
  pinMode(0, INPUT); // boot button
  // set up as hotspot by default.
  // If boot button pressed, reboot and connect to home wifi
  bool homeWifi = prefs.getBool("homeWifi");
  log("Home wifi flag value: %d", homeWifi);
  if (homeWifi) {
    log("Connecting to home wifi");
    prefs.putBool("homeWifi", false);

    wifiManager.setConnectTimeout(10);
    wifiManager.autoConnect();

  } else {
    log("Connecting to access point");
    wifiManager.setConnectTimeout(10);
    
    wifiManager.autoConnect("dontlookup", "dontlookup");
  }
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
