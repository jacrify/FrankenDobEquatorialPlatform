#include "ConcreteStepperWrapper.h"
#include "EQWebServer.h"
#include "FS.h"
#include "Logging.h"
#include "MotorUnit.h"
#include "Network.h"
#include <SPI.h> //needed to make tcmstepper compile!
// #include "OTA.h"
#include "PlatformDynamic.h"
#include "PlatformStatic.h"
#include "UDPListener.h"
#include "UDPSender.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Preferences.h>

// How long we delay the main loop.
// Half of this time is the average pulsetime end error
#define MAINLOOPTIME 25 // ms

PlatformStatic model;
Preferences prefs;

RADynamic control(model);
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
  delay(MAINLOOPTIME);
  // send status to dsc via udp (contains a timer to stop spamming each loop)
  broadcastStatus(motorUnit, model, control);
  // motor control loop
  motorUnit.onLoop();
}
