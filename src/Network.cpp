#include "Network.h"
#include "Logging.h"
#include <WiFiManager.h>

#define HOMEWIFISSID "HOMEWIFISSID"
#define HOMEWIFIPASS "HOMEWIFIPASS"

#define PHONEWIFISSID "PHONEWIFISSID"
#define PHONEWIFIPASS "PHONEWIFIPASS"

#define ESPWIFISSID "ESPWIFISSID"
#define ESPWIFIPASS "ESPWIFIPASS"

WiFiManager wifiManager;

/*
 * Initiative network, saying which device (focuser, eq, dsc)
 * we are. This allows setting of static IPs for the various
 * networks we are going to connect to.
 * Note this code is copied to all devices
 */
Network::Network(Preferences &p, int whoarewe)
    : preferences(p), who_are_we(whoarewe) {

  // IP Setup for phone as hotspot
  phoneNetwork = {PHONEWIFISSID,
                  PHONEWIFIPASS,
                  IPAddress(172, 20, 10, 5),
                  IPAddress(172, 20, 10, 1),
                  IPAddress(255, 255, 255, 240),
                  IPAddress(172, 20, 10, 1)};

  // IP Setup for home wifi
  homeNetwork = {HOMEWIFISSID,
                 HOMEWIFIPASS,
                 IPAddress(192, 168, 20, 150),
                 IPAddress(192, 168, 20, 1),
                 IPAddress(255, 255, 255, 0),
                 IPAddress(192, 168, 20, 1)};

  // IP Setup for focuser as hotspot
  espNetwork = {ESPWIFISSID,
                ESPWIFIPASS,
                IPAddress(192, 168, 10, 5),
                IPAddress(192, 168, 20, 1),
                IPAddress(255, 255, 255, 0),
                IPAddress(192, 168, 20, 1)};
  switch (whoarewe) {
  case WE_ARE_DSC:
    homeNetwork.local_IP = IPAddress(192, 168, 20, 151);
    phoneNetwork.local_IP = IPAddress(172, 20, 10, 6);
    espNetwork.local_IP = IPAddress(192, 168, 10, 6);
    break;
  case WE_ARE_EQ:
    homeNetwork.local_IP = IPAddress(192, 168, 20, 150);
    phoneNetwork.local_IP = IPAddress(172, 20, 10, 5);
    espNetwork.local_IP = IPAddress(192, 168, 10, 5);
    break;
  case WE_ARE_FOCUSER:
    homeNetwork.local_IP = IPAddress(192, 168, 20, 152);
    phoneNetwork.local_IP = IPAddress(172, 20, 10, 7);
    espNetwork.local_IP = IPAddress(192, 168, 10, 6);
    break;
  default:
    log("Invalid who we are passed");
  }
}

void Network::storeWifiCreds(String ssid, String pw, const char *ssidKey,
                             const char *pwKey) {
  preferences.putString(ssidKey, ssid);
  preferences.putString(pwKey, pw);
}

void Network::storeHomeWifiCreds(String ssid, String pw) {
  storeWifiCreds(ssid, pw, HOMEWIFISSID, HOMEWIFIPASS);
}

void Network::storePhoneWifiCreds(String ssid, String pw) {
  storeWifiCreds(ssid, pw, PHONEWIFISSID, PHONEWIFIPASS);
}

void Network::storeESP32WifiCreds(String ssid, String pw) {
  storeWifiCreds(ssid, pw, ESPWIFISSID, ESPWIFIPASS);
}

void Network::connectToWiFi(WiFiNetwork &network) {
  String ssid = preferences.getString(network.ssidKey);
  String password = preferences.getString(network.passwordKey);

  if (ssid.length() == 0 || password.length() == 0) {
    log("SSID or password not stored on ESP32, please set");
    return;
  }

  if (!WiFi.config(network.local_IP, network.gateway, network.subnet,
                   network.primaryDNS)) {
    log("WIFI Failed to configure");
    return;
  }

  WiFi.begin(ssid.c_str(), password.c_str());

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    log("Waiting for connection to %s...", ssid.c_str());
  }

  // Once connected, log the IP address
  IPAddress ip = WiFi.localIP();
  log("Connected to %s! IP address: %d.%d.%d.%d", ssid.c_str(), ip[0], ip[1],
      ip[2], ip[3]);
}

/*
 * Scan for networks, and connect in priority order.
 * Assumes that credentials have been written to device first using
 * store*creds()
 */
void Network::setupWifi() {
  log("Scanning for networks...");

  int n = WiFi.scanNetworks();
  bool espFound = false;
  bool phoneFound = false;
  bool homeFound = false;

  for (int i = 0; i < n; i++) {
    String foundSSID = WiFi.SSID(i);
    log("Found network %s", foundSSID.c_str());

    if (foundSSID == preferences.getString(espNetwork.ssidKey)) {
      espFound = true;
    } else if (foundSSID == preferences.getString(phoneNetwork.ssidKey)) {
      phoneFound = true;
    } else if (foundSSID == preferences.getString(homeNetwork.ssidKey)) {
      homeFound = true;
    }
  }

  if (espFound) {
    log("Connecting to ESP32 hotspot...");
    connectToWiFi(espNetwork);
  } else if (phoneFound) {
    log("Connecting to Phone hotspot...");
    connectToWiFi(phoneNetwork);
  } else if (homeFound) {
    log("Connecting to Home WiFi...");
    connectToWiFi(homeNetwork);
  } else {
    log("No known networks found.");
  }
}

/*
 * Special method to set up access point on one device if required.
 * Currently only used on the focuser as a cheat code.
 */

void Network::setUpAccessPoint() {
  WiFi.softAPConfig(espNetwork.gateway, espNetwork.gateway, espNetwork.subnet);
  WiFi.softAP(preferences.getString(espNetwork.ssidKey).c_str(),
              preferences.getString(espNetwork.passwordKey).c_str());
  log("Access Point set up with IP address: %d.%d.%d.%d", espNetwork.gateway[0],
      espNetwork.gateway[1], espNetwork.gateway[2], espNetwork.gateway[3]);
}