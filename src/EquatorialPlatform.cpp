#include "ConcreteStepperWrapper.h"
#include "EQWebServer.h"
#include "FS.h"
#include "Logging.h"
#include "MotorUnit.h"
#include "Network.h"
#include <SPI.h> //needed to make tcmstepper compile!
// #include "OTA.h"
#include "DecDynamic.h"
#include "DecStatic.h"
#include "RADynamic.h"
#include "RAStatic.h"
#include "UDPListener.h"
#include "UDPSender.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Preferences.h>

// How long we delay the main loop.
// Half of this time is the average pulsetime end error
#define MAINLOOPTIME 25 // ms

RAStatic raStatic;
DecStatic decStatic;
Preferences prefs;

RADynamic raDynamic(raStatic);
DecDynamic decDynamic(decStatic);
MotorUnit motorUnit(raStatic, raDynamic, decStatic, decDynamic, prefs);
Network network(prefs, WE_ARE_EQ);

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  LittleFS.begin();

  prefs.begin("Platform", false);
  // Fresh ESP32s need their wifi creds initialised (once off) as follows. Do not commit.
  // network.storeESP32WifiCreds("","");
  // network.storeHomeWifiCreds("", "");
  // network.storePhoneWifiCreds("", "");
  network.setupWifi();

  // raStatic.setupModel();

  delay(500);
  // order of setup matters here. Web server loads prefs
  setupWebServer(motorUnit, raStatic, raDynamic, decStatic, decDynamic, prefs);

  motorUnit.setupMotors();

  setupUDPListener(motorUnit, raDynamic, decDynamic);
}

void loop() {
  try {
    delay(MAINLOOPTIME);
    // long now=millis();
    // send status to dsc via udp (contains a timer to stop spamming each loop)
    broadcastStatus(motorUnit, raStatic, raDynamic);
    // long broadcast=millis();
    // motor raDynamic loop
    motorUnit.onLoop();
    // long loop=millis();
    // log("Main loop processing: broadcast time %ld, loop time %ld",broadcast-now,loop-broadcast);
  }

  catch (const std::exception &ex) {
    log(ex.what());
  } catch (const std::string &ex) {
    log("Unhandled error %s",ex);
  }
}
