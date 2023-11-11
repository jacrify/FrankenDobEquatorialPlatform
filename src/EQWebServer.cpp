#include "EQWebServer.h"

#include "Logging.h"
#include "MotorUnit.h"
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

AsyncWebServer server(80);

#define IPBROADCASTPORT 50375

// TODO #8 add pulseguide speed here
void setLimitToMiddleDistance(AsyncWebServerRequest *request,
                              RAStatic &raStatic, Preferences &preferences) {
  log("/setLimitToMiddle");
  if (request->hasArg("value")) {
    String distance = request->arg("value");

    int distanceValue = distance.toInt();
    if (distanceValue == 0 && distance != "0") {
      log("Could not parse limit to middle");
      return;
    }
    raStatic.setLimitSwitchToMiddleDistance(distanceValue);
    preferences.putUInt(PREF_MIDDLE_KEY, distanceValue);
    return;
  }
  log("No distance arg found");
}

void setNunChukMultiplier(AsyncWebServerRequest *request, RAStatic &raStatic,
                          Preferences &preferences) {
  log("/setNunChukMultipler");
  if (request->hasArg("value")) {
    String nunChuk = request->arg("value");

    int nunChukValue = nunChuk.toInt();
    if (nunChukValue == 0 && nunChuk != "0") {
      log("Could not parse nunchuk multiplier");
      return;
    }
    raStatic.setNunChukMultiplier(nunChukValue);
    preferences.putInt(NUNCHUK_MULIPLIER_KEY, nunChukValue);
    return;
  }
  log("No Nunchuk multiplier");
}

void setRewindFastFowardSpeedInHz(AsyncWebServerRequest *request,
                                  RAStatic &raStatic,
                                  Preferences &preferences) {
  log("/setrunbackSpeed");
  if (request->hasArg("value")) {
    String speed = request->arg("value");
    long speedValue = std::stoul(speed.c_str());

    if (speedValue == 0 && speed != "0") {
      log("Could not parse speed");
      return;
    }
    raStatic.setRewindFastFowardSpeedInHz(speedValue);
    preferences.putUInt(PREF_SPEED_KEY, speedValue);
    return;
  }
  log("No speed arg found");
}

void setAcceleration(AsyncWebServerRequest *request, Preferences &preferences,
                     MotorUnit &motor) {
  log("/setAcceleration");
  if (request->hasArg("value")) {
    String accel = request->arg("value");
    try {
      unsigned long accelValue = std::stoul(accel.c_str());
      motor.setAcceleration(accelValue);
      preferences.putULong(ACCEL_KEY, accelValue);
      log("Acceleration value set and saved");
      return;
    } catch (const std::invalid_argument &ia) {
      log("Invalid acceleration value: not a number");
    } catch (const std::out_of_range &oor) {
      log("Invalid acceleration value: out of range");
    }
  }
  log("No acceleration arg found");
}

void setRAGuideRate(AsyncWebServerRequest *request, RAStatic &raStatic,
                    Preferences &preferences) {

  log("/setRAGuideRate");
  if (request->hasArg("value")) {
    String rarate = request->arg("value");

    double rarateval = rarate.toDouble();
    if (rarateval == 0 && rarate != "0.0") {
      log("Could not parse rarateval");
      return;
    }
    raStatic.setGuideRateMultiplier(rarateval);
    preferences.putDouble(RA_GUIDE_KEY, rarateval);
    log("Saved new RA guide rate");
    return;
  }
  log("No speed arg found");
}

void setConeRadiusAtAttachmentPoint(AsyncWebServerRequest *request,
                                    RAStatic &raStatic,
                                    Preferences &preferences) {

  log("/setConeRadiusAtAttachmentPoint");
  if (request->hasArg("value")) {
    String radius = request->arg("value");

    double radValue = radius.toDouble();
    if (radValue == 0 && radius != "0.0") {
      log("Could not parse radius");
      return;
    }
    raStatic.setScrewToPivotInMM(radValue);
    preferences.putDouble(PREF_CIRCLE_KEY, radValue);
    return;
  }
  log("No speed arg found");
}

void getStatus(AsyncWebServerRequest *request, MotorUnit &motor,
               RAStatic &raStatic) {
  // log("/getStatus");

  char buffer[400];
  sprintf(buffer,
          R"({
        "runbackSpeed" : %ld,
        "coneRadius": %f,
        "limitToMiddleDistance":%ld,
        "raGuideRate" : %f,
        "position" : %f,
        "velocity" : %f,
        "acceleration" : %ld,
        "nunChukMultiplier" : %d
      })",
          raStatic.getRewindFastFowardSpeed(), raStatic.getScrewToPivotInMM(),
          raStatic.getLimitSwitchToMiddleDistance(),
          raStatic.getGuideRateMultiplier(), motor.getPositionInMM(),
          motor.getVelocityInMMPerMinute(), motor.getAcceleration(),
          raStatic.getNunChukMultiplier());

  String json = buffer;

  // String json = "{\"timestamp\":" + String(times.get()) + ",\"position\":"
  // + String(positions.get()) + ",\"velocity\":" + String(velocities.get()) +
  // "}";
  request->send(200, "application/json", json);
}

void setupWebServer(MotorUnit &motor, RAStatic &raStatic, RADynamic &raDynamic,
                    Preferences &preferences) {
  int rewindFastFowardSpeed =
      preferences.getUInt(PREF_SPEED_KEY, DEFAULT_SPEED);
  int limitSwitchToMiddleDistance =
      preferences.getUInt(PREF_MIDDLE_KEY, DEFAULT_MIDDLE_DISTANCE);
  double coneRadius =
      preferences.getDouble(PREF_CIRCLE_KEY, DEFAULT_CIRCLE_RADIUS);
  int nunChukMultiplier =
      preferences.getInt(NUNCHUK_MULIPLIER_KEY, DEFAULT_NUNCHUK_MULIPLIER);
  double raGuideSpeedMultiplier =
      preferences.getDouble(RA_GUIDE_KEY, DEFAULT_RA_GUIDE);
  unsigned long acceleration = preferences.getULong(ACCEL_KEY, DEFAULT_ACCEL);

  log("Preferences loaded for raStatic rewindspeed: %d limitToMiddle %d "
      "radius "
      "%f NunChuk multiplier %f RA Guide multiplier %f Accel: %d",
      rewindFastFowardSpeed, limitSwitchToMiddleDistance, coneRadius,
      nunChukMultiplier, raGuideSpeedMultiplier, acceleration);
  // order matters here: rewind fast forward speed uses previous sets for
  // calcs
  raStatic.setNunChukMultiplier(nunChukMultiplier);
  raStatic.setGuideRateMultiplier(raGuideSpeedMultiplier);
  raStatic.setLimitSwitchToMiddleDistance(limitSwitchToMiddleDistance);
  raStatic.setScrewToPivotInMM(coneRadius);
  raStatic.setRewindFastFowardSpeedInHz(rewindFastFowardSpeed);
  motor.setAcceleration(acceleration);

  server.on("/getStatus", HTTP_GET,
            [&motor, &raStatic](AsyncWebServerRequest *request) {
              getStatus(request, motor, raStatic);
            });

  server.on("/runbackSpeed", HTTP_POST,
            [&raStatic, &preferences](AsyncWebServerRequest *request) {
              setRewindFastFowardSpeedInHz(request, raStatic, preferences);
            });

  server.on("/coneRadius", HTTP_POST,
            [&raStatic, &preferences](AsyncWebServerRequest *request) {
              setConeRadiusAtAttachmentPoint(request, raStatic, preferences);
            });
  server.on("/limitToMiddleDistance", HTTP_POST,
            [&raStatic, &preferences](AsyncWebServerRequest *request) {
              setLimitToMiddleDistance(request, raStatic, preferences);
            });

  server.on("/nunChukMultiplier", HTTP_POST,
            [&raStatic, &preferences](AsyncWebServerRequest *request) {
              setNunChukMultiplier(request, raStatic, preferences);
            });
  server.on("/acceleration", HTTP_POST,
            [&motor, &preferences](AsyncWebServerRequest *request) {
              setAcceleration(request, preferences, motor);
            });

  server.on("/raGuideRate", HTTP_POST,
            [&raStatic, &preferences](AsyncWebServerRequest *request) {
              setRAGuideRate(request, raStatic, preferences);
            });

  server.on("/home", HTTP_POST, [&raDynamic](AsyncWebServerRequest *request) {
    raDynamic.gotoStart();
  });

  server.on("/park", HTTP_POST, [&raDynamic](AsyncWebServerRequest *request) {
    raDynamic.gotoEndish();
  });

  server.on("/center", HTTP_POST, [&raDynamic](AsyncWebServerRequest *request) {
    raDynamic.gotoMiddle();
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
