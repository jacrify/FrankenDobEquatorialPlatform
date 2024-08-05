#ifndef NETWORK_H
#define NETWORK_H

#include <Preferences.h>

#define WE_ARE_EQ 1
#define WE_ARE_FOCUSER 2
#define WE_ARE_DSC 3

struct WiFiNetwork {
  const char *ssidKey;
  const char *passwordKey;
  IPAddress local_IP;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress primaryDNS;
};

class Network {
public:
  Network(Preferences &p, int whoarewe);
  void setupWifi();
  void setUpAccessPoint();
  void storeHomeWifiCreds(String ssid, String pw);
  void storePhoneWifiCreds(String ssid, String pw);
  void storeESP32WifiCreds(String ssid, String pw);

private:
  void storeWifiCreds(String ssid, String pw, const char *ssidKey,
                      const char *pwKey);
  void connectToWiFi(WiFiNetwork &network);
  void logIP();

  Preferences &preferences;
  int who_are_we;
  WiFiNetwork phoneNetwork;
  WiFiNetwork homeNetwork;
  WiFiNetwork espNetwork;
};

#endif // NETWORK_H