// #include "DigitalCaliper.h"
#include "EQWebServer.h"
#include "FS.h"
#include "Logging.h"
#include "MotorUnit.h"
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
// In micros per second this is:
// 1488.3/24/60/60*100 = 1.72256944

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
  setupMotor();
  caliper.setUp();
  setupWebServer(caliper); // don't use log() before this point
  setupOTA();
}

int sampleStopper = 0;

void loop() {
  // loopOTA();
  delay(100);
  sampleStopper++;
   if (sampleStopper >= 50) {
    caliper.takeSample();
    sampleStopper = 0;
  }
  onLoop();
  logWrite();
}
