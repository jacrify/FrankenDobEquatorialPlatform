// #include "DigitalCaliper.h"
#include "EQWebServer.h"
#include "FS.h"
#include "Logging.h"
#include "MotorUnit.h"
#include "Network.h"
#include "OTA.h"
#include "PlatformModel.h"
#include "UDPSender.h"
#include "UDPListener.h"
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
  
  
  prefs.begin("Platform", false);
  setupWifi(prefs);

  model.setupModel();
  motorUnit.setupMotor();
  delay(500);
  setupWebServer(motorUnit, model,prefs); 
  setUpUDPListener(motorUnit);
  // don't use log() before this point
  // setupOTA();
}

void loop() {
  // loopOTA();
  delay(100);
  loopNetwork(prefs);
  broadcastStatus(motorUnit,model);
  motorUnit.onLoop();
}
