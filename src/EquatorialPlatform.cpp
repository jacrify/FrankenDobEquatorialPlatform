// #include "DigitalCaliper.h"
#include "EQWebServer.h"
#include "FS.h"
#include "Logging.h"
#include "MotorUnit.h"
#include "Network.h"
#include "OTA.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

MotorUnit motorUnit;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  LittleFS.begin();
  setupWifi();
  motorUnit.setupMotor();
  setupWebServer(motorUnit); // don't use log() before this point
  setupOTA();
}

void loop() {
  // loopOTA();
  delay(100);
  motorUnit.onLoop();
  logWrite();
}
