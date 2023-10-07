#include "ConcreteStepperWrapper.h"
#include "EQWebServer.h"
#include "FS.h"
#include "Logging.h"
#include "MotorUnit.h"
#include "Network.h"
// #include "OTA.h"
#include "PlatformDynamic.h"
#include "PlatformStatic.h"
#include "UDPListener.h"
#include "UDPSender.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Preferences.h>

PlatformStatic model;
Preferences prefs;

PlatformDynamic control(model);
MotorUnit motorUnit(model, control, prefs);

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  LittleFS.begin();

  prefs.begin("Platform", false);
  setupWifi(prefs);

  model.setupModel();

  delay(500);
  // order of setup matters here. Web server loads prefs
  setupWebServer(motorUnit, model, control, prefs);

  motorUnit.setupMotor();

  setupUDPListener(motorUnit, control);
}

void loop() {
  delay(100);
  // send status to dsc via udp
  broadcastStatus(motorUnit, model, control);
  // motor control loop
  motorUnit.onLoop();
}
