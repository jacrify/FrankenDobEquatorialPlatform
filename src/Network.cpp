#include <WiFiManager.h>

void setupWifi() {
  WiFiManager wifiManager;
  wifiManager.autoConnect("dontlookdown", "hahaha123");
}
