// #include "DigitalCaliper.h"
#include "EQWebServer.h"
#include "FS.h"
#include "Logging.h"
#include "MotorUnit.h"
#include "Network.h"
#include "OTA.h"
#include "PlatformModel.h"
#include "UDPListener.h"
#include "UDPSender.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Preferences.h>


PlatformModel model;
Preferences prefs;

MotorUnit motorUnit(model, prefs);

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  LittleFS.begin();

  prefs.begin("Platform", false);
  setupWifi(prefs);

  
  motorUnit.setupMotor();
  //TODO #3 Fix: needs to happen after motor setup as guide rate gets set here
  model.setupModel();
  delay(500);
  setupWebServer(motorUnit, model, prefs);
  setupUDPListener(motorUnit);
  // don't use log() before this point
  // setupOTA();
}

void loop() {
  // loopOTA();
  delay(100);
  broadcastStatus(motorUnit, model);
  motorUnit.onLoop();
}
