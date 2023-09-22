
#include <WiFiManager.h>

#include "Network.h"


#include "Logging.h"

WiFiManager wifiManager;


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

