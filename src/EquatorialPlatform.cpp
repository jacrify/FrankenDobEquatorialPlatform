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

PlatformModel model;
Preferences prefs;

MotorUnit motorUnit(model,prefs);

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  LittleFS.begin();
  setupWifi();
  
  prefs.begin("Platform", false); 

  model.setupModel();
  motorUnit.setupMotor();
  delay(500);
  setupWebServer(motorUnit, model,prefs); // don't use log() before this point
  // setupOTA();
}

void loop() {
  // loopOTA();
  delay(100);
  broadcastStatus(motorUnit.getTimeToCenterInSeconds(),
                  // motorUnit.getTimeToCenterInSeconds(),
                      motorUnit.getTimeToEndOfRunInSeconds(),
                  // 0,
                  motorUnit.getTrackingStatus());
  motorUnit.onLoop();
  logWrite();
}
