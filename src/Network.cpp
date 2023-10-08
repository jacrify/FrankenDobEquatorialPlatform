#include "Network.h"
#include "Logging.h"
#include <WiFiManager.h>

WiFiManager wifiManager;

#define HOMEWIFISSID "HOMEWIFISSID"
#define HOMEWIFIPASS "HOMEWIFIPASS"

void logIP() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    log("Waiting for connection...");
  }

  // Once connected, log the IP address
  IPAddress ip = WiFi.localIP();
  log("Connected! IP address: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
}
void setupWifi(Preferences &prefs) {
  log("Scanning for networks...");

  int n = WiFi.scanNetworks();
  // prefs.putString(HOMEWIFISSID, "");
  // prefs.putString(HOMEWIFIPASS, "");

  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == "dontlookup") {
      log("Connecting to 'dontlookup'...");
      WiFi.begin("dontlookup", "dontlookdown");
      logIP();
      return;
    }
  }
  if (prefs.isKey(HOMEWIFISSID) && prefs.isKey(HOMEWIFIPASS)) {
    log("Connnecting to home wifi...");
    WiFi.begin(prefs.getString(HOMEWIFISSID).c_str(), prefs.getString(HOMEWIFIPASS).c_str());
    logIP();
    return;
  } else {
    log("No wifi details in prefs");
  }
}
