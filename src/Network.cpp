#include <WiFiManager.h>

void setupWifi() {
  WiFiManager wifiManager;
  wifiManager.setConnectTimeout(10);
  wifiManager.autoConnect("dontlookdown", "hahaha123");
}
