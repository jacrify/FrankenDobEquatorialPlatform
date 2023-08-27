// #include "DigitalCaliper.h"
#include "EQWebServer.h"
#include "FS.h"
#include "Logging.h"
#include "MotorUnit.h"
#include "Network.h"
#include "OTA.h"
#include "PlatformModel.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Preferences.h>

MotorUnit motorUnit;
PlatformModel model;
Preferences prefs;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  LittleFS.begin();
  setupWifi();
  prefs.begin("Platform", false);
 
  

  model.setupModel();
  motorUnit.setupMotor(model, prefs);
  setupWebServer(motorUnit, model,prefs); // don't use log() before this point
  // setupOTA();
}

void loop() {
  // loopOTA();
  delay(100);
  motorUnit.onLoop();
  logWrite();
}
