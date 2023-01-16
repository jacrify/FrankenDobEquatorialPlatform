#include "DigitalCaliper.h"
#include "EQWebServer.h"
#include "Logging.h"
#include "Network.h"
#include "OTA.h"

// Radius of circle from platform rotaion axis to caliper strip is 230.5mm
// Therefore circumference of that circle is 1448.3mm
// Therefore target speed along the strip is 1mm per minute (1.006!) : 1488.3 /
// 24/60

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
