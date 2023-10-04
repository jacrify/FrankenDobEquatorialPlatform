#include "EQWebServer.h"

#include "Logging.h"
#include "MotorUnit.h"
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

AsyncWebServer server(80);

#define IPBROADCASTPORT 50375

//TODO #8 add pulseguide speed here
void setLimitToMiddleDistance(AsyncWebServerRequest *request,
                              PlatformModel &model, Preferences &preferences) {
  log("/setLimitToMiddle");
  if (request->hasArg("value")) {
    String distance = request->arg("value");

    int distanceValue = distance.toInt();
    if (distanceValue == 0 && distance != "0") {
      log("Could not parse limit to middle");
      return;
    }
    model.setLimitSwitchToMiddleDistance(distanceValue);
    preferences.putUInt(PREF_MIDDLE_KEY, distanceValue);
    return;
  }
  log("No distance arg found");
}

void setRewindFastFowardSpeedInHz(AsyncWebServerRequest *request,
                                  PlatformModel &model,
                                  Preferences &preferences) {
  log("/setrunbackSpeed");
  if (request->hasArg("value")) {
    String speed = request->arg("value");

    int speedValue = speed.toInt();
    if (speedValue == 0 && speed != "0") {
      log("Could not parse speed");
      return;
    }
    model.setRewindFastFowardSpeedInHz(speedValue);
    preferences.putUInt(PREF_SPEED_KEY, speedValue);
    return;
  }
  log("No speed arg found");
}

void setGreatCircleRadius(AsyncWebServerRequest *request, PlatformModel &model,
                          Preferences &preferences) {

  log("/setGreatCircleRadius");
  if (request->hasArg("value")) {
    String radius = request->arg("value");

    double radValue = radius.toDouble();
    if (radValue == 0 && radius != "0.0") {
      log("Could not parse radius");
      return;
    }
    model.setGreatCircleRadius(radValue);
    preferences.putUInt(PREF_CIRCLE_KEY, radValue);
    return;
  }
  log("No speed arg found");
}

void getStatus(AsyncWebServerRequest *request, MotorUnit &motor,
               PlatformModel &model) {
  // log("/getStatus");

  char buffer[300];
  sprintf(buffer,
          R"({
        "runbackSpeed" : %ld,
        "greatCircleRadius": %f,
        "limitToMiddleDistance":%ld,
        "raGuideRate" : %f,
        "position" : %f,
        "velocity" : %f
      })",
          model.getRewindFastFowardSpeed(), model.getGreatCircleRadius(),
          model.getLimitSwitchToMiddleDistance(),
          model.getRaGuideRateMultiplier(), motor.getPositionInMM(),
          motor.getVelocityInMMPerMinute());

  String json = buffer;

  // String json = "{\"timestamp\":" + String(times.get()) + ",\"position\":"
  // + String(positions.get()) + ",\"velocity\":" + String(velocities.get()) +
  // "}";
  request->send(200, "application/json", json);
}

void setupWebServer(MotorUnit &motor, PlatformModel &model,PlatformControl &control,
                    Preferences &preferences) {

  int rewindFastFowardSpeed =
      preferences.getUInt(PREF_SPEED_KEY, DEFAULT_SPEED);
  int limitSwitchToMiddleDistance =
      preferences.getUInt(PREF_MIDDLE_KEY, DEFAULT_MIDDLE_DISTANCE);
  int greatCircleRadius =
      preferences.getUInt(PREF_CIRCLE_KEY, DEFAULT_CIRCLE_RADIUS);
  double raGuideSpeedMultiplier =
      preferences.getDouble(RA_GUIDE_KEY, DEFAULT_RA_GUIDE);
  log("Preferences loaded for model rewindspeed: %d limitToMiddle %d radius "
      "%d RA Guide multiplier %f",
      rewindFastFowardSpeed, limitSwitchToMiddleDistance, greatCircleRadius,
      raGuideSpeedMultiplier);
  //order matters here: rewind fast forward speed uses previous sets for calcs
  model.setRaGuideRateMultiplier(raGuideSpeedMultiplier);
  model.setLimitSwitchToMiddleDistance(limitSwitchToMiddleDistance);
  model.setGreatCircleRadius(greatCircleRadius);
  model.setRewindFastFowardSpeedInHz(rewindFastFowardSpeed);
  
  server.on("/getStatus", HTTP_GET,
            [&motor, &model](AsyncWebServerRequest *request) {
              getStatus(request, motor, model);
            });

  server.on("/runbackSpeed", HTTP_POST,
            [&model, &preferences](AsyncWebServerRequest *request) {
              setRewindFastFowardSpeedInHz(request, model, preferences);
            });

  server.on("/greatCircleRadius", HTTP_POST,
            [&model, &preferences](AsyncWebServerRequest *request) {
              setGreatCircleRadius(request, model, preferences);
            });
  server.on("/limitToMiddleDistance", HTTP_POST,
            [&model, &preferences](AsyncWebServerRequest *request) {
              setLimitToMiddleDistance(request, model, preferences);
            });

  server.on("/home", HTTP_POST, [&control](AsyncWebServerRequest *request) {
    control.gotoStart();
  });

  server.on("/park", HTTP_POST, [&control](AsyncWebServerRequest *request) {
    control.gotoEndish();
  });

  server.on("/center", HTTP_POST, [&control](AsyncWebServerRequest *request) {
    control.gotoMiddle();
  });
  // TODO #2 implement tracking on off
  //  server.on("/trackingOn", HTTP_POST,
  //            [&motor](AsyncWebServerRequest *request) {
  //            motor.setTracking(true); });

  // server.on("/trackingOff", HTTP_POST,
  //           [&motor](AsyncWebServerRequest *request) {
  //           motor.setTracking(false); });

  // server.serveStatic("/www/", LittleFS, "/fs/");
  server.serveStatic("/", LittleFS, "/fs/");

  // WebSerial is accessible at "<IP Address>/webserial" in browser

  server.begin();
  log("Server started");
  return;
}
