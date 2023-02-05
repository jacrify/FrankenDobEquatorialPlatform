//#include "DigitalCaliper.h"
#include "EQWebServer.h"
#include "FS.h"
#include "Logging.h"
#include "Network.h"
#include "OTA.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

DigitalCaliper caliper;
// DigitalCaliper caliper;

// Radius of circle from platform rotaion axis to caliper strip is 230.5mm
// Therefore circumference of that circle is 1448.3mm
// Therefore target speed along the strip is 1mm per minute (1.006!) : 1488.3 /
// 24/60

// void sampleLoop(void *parameter) {
//   // DigitalCaliper *caliper = (DigitalCaliper *)parameter;

//     // log("In loop");
//     caliper.takeSample();
//     // caliper.sleepBetweenSamples();

// }
void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  LittleFS.begin();
  setupWifi();
  caliper.setUp();
  setupWebServer(caliper); // don't use log() before this point
  setupOTA();

}

void loop() {
  loopOTA();
  delay(1000);
  caliper.takeSample();
  logWrite();
}
