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
                              PlatformStatic &model, Preferences &preferences) {
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

void setNunChukMultiplier(AsyncWebServerRequest *request, PlatformStatic &model,
                          Preferences &preferences) {
  log("/setNunChukMultipler");
  if (request->hasArg("value")) {
    String nunChuk = request->arg("value");

    int nunChukValue = nunChuk.toInt();
    if (nunChukValue == 0 && nunChuk != "0") {
      log("Could not parse nunchuk multiplier");
      return;
    }
    model.setNunChukMultiplier(nunChukValue);
    preferences.putInt(NUNCHUK_MULIPLIER_KEY, nunChukValue);
    return;
  }
  log("No Nunchuk multiplier");
}

void setRewindFastFowardSpeedInHz(AsyncWebServerRequest *request,
                                  PlatformStatic &model,
                                  Preferences &preferences) {
  log("/setrunbackSpeed");
  if (request->hasArg("value")) {
    String speed = request->arg("value");
    long speedValue = std::stoul(speed.c_str());

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

void setRAGuideRate(AsyncWebServerRequest *request, PlatformStatic &model,
                    Preferences &preferences) {

  log("/setRAGuideRate");
  if (request->hasArg("value")) {
    String rarate = request->arg("value");

    double rarateval = rarate.toDouble();
    if (rarateval == 0 && rarate != "0.0") {
      log("Could not parse rarateval");
      return;
    }
    model.setRaGuideRateMultiplier(rarateval);
    preferences.putDouble(RA_GUIDE_KEY, rarateval);
    log("Saved new RA guide rate");
    return;
  }
  log("No speed arg found");
}

void setConeRadiusAtAttachmentPoint(AsyncWebServerRequest *request,
                                    PlatformStatic &model,
                                    Preferences &preferences) {

  log("/setConeRadiusAtAttachmentPoint");
  if (request->hasArg("value")) {
    String radius = request->arg("value");

    double radValue = radius.toDouble();
    if (radValue == 0 && radius != "0.0") {
      log("Could not parse radius");
      return;
    }
    model.setConeRadiusAtAttachmentPoint(radValue);
    preferences.putDouble(PREF_CIRCLE_KEY, radValue);
    return;
  }
  log("No speed arg found");
}

void getStatus(AsyncWebServerRequest *request, MotorUnit &motor,
               PlatformStatic &model) {
  // log("/getStatus");

  char buffer[400];
  sprintf(
      buffer,
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
      model.getRewindFastFowardSpeed(), model.getConeRadiusAtAttachmentPoint(),
      model.getLimitSwitchToMiddleDistance(), model.getRaGuideRateMultiplier(),
      motor.getPositionInMM(), motor.getVelocityInMMPerMinute(),
      motor.getAcceleration(), model.getNunChukMultiplier());

  String json = buffer;

  // String json = "{\"timestamp\":" + String(times.get()) + ",\"position\":"
  // + String(positions.get()) + ",\"velocity\":" + String(velocities.get()) +
  // "}";
  request->send(200, "application/json", json);
}

void setupWebServer(MotorUnit &motor, PlatformStatic &model, RADynamic &control,
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

  log("Preferences loaded for model rewindspeed: %d limitToMiddle %d "
      "radius "
      "%f NunChuk multiplier %f RA Guide multiplier %f Accel: %d",
      rewindFastFowardSpeed, limitSwitchToMiddleDistance, coneRadius,
      nunChukMultiplier, raGuideSpeedMultiplier, acceleration);
  // order matters here: rewind fast forward speed uses previous sets for
  // calcs
  model.setNunChukMultiplier(nunChukMultiplier);
  model.setRaGuideRateMultiplier(raGuideSpeedMultiplier);
  model.setLimitSwitchToMiddleDistance(limitSwitchToMiddleDistance);
  model.setConeRadiusAtAttachmentPoint(coneRadius);
  model.setRewindFastFowardSpeedInHz(rewindFastFowardSpeed);
  motor.setAcceleration(acceleration);

  server.on("/getStatus", HTTP_GET,
            [&motor, &model](AsyncWebServerRequest *request) {
              getStatus(request, motor, model);
            });

  server.on("/runbackSpeed", HTTP_POST,
            [&model, &preferences](AsyncWebServerRequest *request) {
              setRewindFastFowardSpeedInHz(request, model, preferences);
            });

  server.on("/coneRadius", HTTP_POST,
            [&model, &preferences](AsyncWebServerRequest *request) {
              setConeRadiusAtAttachmentPoint(request, model, preferences);
            });
  server.on("/limitToMiddleDistance", HTTP_POST,
            [&model, &preferences](AsyncWebServerRequest *request) {
              setLimitToMiddleDistance(request, model, preferences);
            });

  server.on("/nunChukMultiplier", HTTP_POST,
            [&model, &preferences](AsyncWebServerRequest *request) {
              setNunChukMultiplier(request, model, preferences);
            });
  server.on("/acceleration", HTTP_POST,
            [&motor, &preferences](AsyncWebServerRequest *request) {
              setAcceleration(request, preferences, motor);
            });

  server.on("/raGuideRate", HTTP_POST,
            [&model, &preferences](AsyncWebServerRequest *request) {
              setRAGuideRate(request, model, preferences);
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
