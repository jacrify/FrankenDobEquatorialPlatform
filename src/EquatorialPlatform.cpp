#include "DigitalCaliper.h"
#include "Logging.h"
#include "Network.h"
#include "OTA.h"
#include "EQWebServer.h"

void setup() {
  setupWifi();
  setupOTA();
  DigitalCaliper caliper;
  setupWebServer(caliper);
  // WebSerial.begin(&server);

  Serial.begin(115200);
  Serial.println("Booting");
}

void loop() { loopOTA(); }
