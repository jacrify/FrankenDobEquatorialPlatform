#include <WiFiManager.h>

WiFiManager wifiManager;

void setupWifi() {
  
  wifiManager.setConnectTimeout(10);
  wifiManager.autoConnect("dontlookdown", "hahaha123");
}
