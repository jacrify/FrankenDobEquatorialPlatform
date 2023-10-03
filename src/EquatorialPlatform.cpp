// #include "DigitalCaliper.h"
#include "ConcreteStepperWrapper.h"
#include "EQWebServer.h"
#include "FS.h"
#include "Logging.h"
#include "MotorUnit.h"
#include "Network.h"
#include "OTA.h"
#include "PlatformControl.h"
#include "PlatformModel.h"
#include "UDPListener.h"
#include "UDPSender.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Preferences.h>

PlatformModel model;
Preferences prefs;

PlatformControl control(model);

MotorUnit motorUnit(model, control, prefs);

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  LittleFS.begin();

  prefs.begin("Platform", false);
  setupWifi(prefs);

  model.setupModel();
  
  delay(500);
  setupWebServer(motorUnit, model, control,prefs);

  motorUnit.setupMotor();

  setupUDPListener(motorUnit,control);
  // don't use log() before this point
  // setupOTA();
}

void loop() {
  // loopOTA();
  delay(100);
  broadcastStatus(motorUnit, model,control);
  motorUnit.onLoop();

}
