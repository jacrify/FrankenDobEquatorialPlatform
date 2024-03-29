#include "EQWebServer.h"

#include "Logging.h"
#include "MotorUnit.h"
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

AsyncWebServer server(80);

#define IPBROADCASTPORT 50375

// TODO #8 add pulseguide speed here
void setRaLimitToMiddleDistance(AsyncWebServerRequest *request,
                                RAStatic &raStatic, Preferences &preferences) {
  log("/setRaLimitToMiddle");
  if (request->hasArg("value")) {
    String distance = request->arg("value");

    int distanceValue = distance.toInt();
    if (distanceValue == 0 && distance != "0") {
      log("Could not parse limit to middle");
      return;
    }
    raStatic.setLimitSwitchToMiddleDistance(distanceValue);
    preferences.putUInt(RA_PREF_MIDDLE_KEY, distanceValue);
    return;
  }
  log("No distance arg found");
}

void setDecLimitToMiddleDistance(AsyncWebServerRequest *request,
                                 DecStatic &decStatic,
                                 Preferences &preferences) {
  log("/setDecLimitToMiddle");
  if (request->hasArg("value")) {
    String distance = request->arg("value");

    int distanceValue = distance.toInt();
    if (distanceValue == 0 && distance != "0") {
      log("Could not parse limit to middle");
      return;
    }
    decStatic.setLimitSwitchToMiddleDistance(distanceValue);
    preferences.putUInt(DEC_PREF_MIDDLE_KEY, distanceValue);
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

void setRARewindFastFowardSpeedInHz(AsyncWebServerRequest *request,
                                  RAStatic &raStatic,
                                  Preferences &preferences) {
  log("/setrarunbackSpeed");
  if (request->hasArg("value")) {
    String speed = request->arg("value");
    long speedValue = std::stoul(speed.c_str());

    if (speedValue == 0 && speed != "0") {
      log("Could not parse speed");
      return;
    }
    raStatic.setRewindFastFowardSpeedInHz(speedValue);
    preferences.putUInt(RA_PREF_SPEED_KEY, speedValue);
    return;
  }
  log("No speed arg found");
}

void setDecRewindFastFowardSpeedInHz(AsyncWebServerRequest *request,
                                    DecStatic &decStatic,
                                    Preferences &preferences) {
  log("/setdecrunbackSpeed");
  if (request->hasArg("value")) {
    String speed = request->arg("value");
    long speedValue = std::stoul(speed.c_str());

    if (speedValue == 0 && speed != "0") {
      log("Could not parse speed");
      return;
    }
    decStatic.setRewindFastFowardSpeedInHz(speedValue);
    preferences.putUInt(DEC_PREF_SPEED_KEY, speedValue);
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
  log("No guide rate found");
}

void setRaLeadToPivotDistance(AsyncWebServerRequest *request,
                              RAStatic &raStatic, Preferences &preferences) {

  log("/setRaLeadToPivotDistance");
  if (request->hasArg("value")) {
    String radius = request->arg("value");

    double radValue = radius.toDouble();
    if (radValue == 0 && radius != "0.0") {
      log("Could not parse radius");
      return;
    }
    raStatic.setScrewToPivotInMM(radValue);
    preferences.putDouble(RA_LEAD_TO_PIVOT_KEY, radValue);
    return;
  }
  log("No pivot distance found");
}

void setDecLeadToPivotDistance(AsyncWebServerRequest *request,
                               DecStatic &decStatic, Preferences &preferences) {

  log("/setDecLeadToPivotDistance");
  if (request->hasArg("value")) {
    String radius = request->arg("value");

    double radValue = radius.toDouble();
    if (radValue == 0 && radius != "0.0") {
      log("Could not parse radius");
      return;
    }
    decStatic.setScrewToPivotInMM(radValue);
    preferences.putDouble(DEC_LEAD_TO_PIVOT_KEY, radValue);
    return;
  }
  log("No pivot distance found");
}

void getStatus(AsyncWebServerRequest *request, MotorUnit &motor,
               RAStatic &raStatic, DecStatic &decStatic) {
  // log("/getStatus");

  const size_t capacity = JSON_OBJECT_SIZE(15);

  DynamicJsonDocument doc(capacity);
  // Populate the JSON object
  doc["raRunbackSpeed"] = raStatic.getRewindFastFowardSpeed();
  doc["decRunbackSpeed"] = decStatic.getRewindFastFowardSpeed();
  doc["raLeadToPivotDistance"] = raStatic.getScrewToPivotInMM();
  doc["raLimitToMiddleDistance"] = raStatic.getLimitSwitchToMiddleDistance();

  doc["decLeadToPivotDistance"] = decStatic.getScrewToPivotInMM();
  doc["decLimitToMiddleDistance"] = decStatic.getLimitSwitchToMiddleDistance();

  doc["raGuideRate"] = raStatic.getGuideRateMultiplier();
  doc["raPosition"] = motor.getRaPositionInMM();
  doc["decPosition"] = motor.getDecPositionInMM();
  doc["velocity"] = motor.getVelocityInMMPerMinute();
  doc["acceleration"] = motor.getAcceleration();
  doc["nunChukMultiplier"] = raStatic.getNunChukMultiplier();

  doc["raStepsMM"] = raStatic.getStepsPerMM();
  doc["decStepsMM"] = decStatic.getStepsPerMM();

  String json;
  serializeJson(doc, json);

  request->send(200, "application/json", json);
}

void setupWebServer(MotorUnit &motor, RAStatic &raStatic, RADynamic &raDynamic,
                    DecStatic &decStatic, DecDynamic &decDynamic,
                    Preferences &preferences) {

  int raRewindFastFowardSpeed =
      preferences.getUInt(RA_PREF_SPEED_KEY, DEFAULT_SPEED);

  int decRewindFastFowardSpeed =
      preferences.getUInt(DEC_PREF_SPEED_KEY, DEFAULT_SPEED);

  
  int raLimitSwitchToMiddleDistance =
      preferences.getUInt(RA_PREF_MIDDLE_KEY, DEFAULT_RA_MIDDLE_DISTANCE);

  int decLimitSwitchToMiddleDistance =
      preferences.getUInt(DEC_PREF_MIDDLE_KEY, DEFAULT_DEC_MIDDLE_DISTANCE);

  double raLeadScrewToPivotMM = preferences.getDouble(
      RA_LEAD_TO_PIVOT_KEY, DEFAULT_RA_LEAD_SCREW_TO_PIVOT);

  double decLeadScrewToPivotMM = preferences.getDouble(
      DEC_LEAD_TO_PIVOT_KEY, DEFAULT_DEC_LEAD_SCREW_TO_PIVOT);

  int nunChukMultiplier =
      preferences.getInt(NUNCHUK_MULIPLIER_KEY, DEFAULT_NUNCHUK_MULIPLIER);
  double raGuideSpeedMultiplier =
      preferences.getDouble(RA_GUIDE_KEY, DEFAULT_RA_GUIDE);
  unsigned long acceleration = preferences.getULong(ACCEL_KEY, DEFAULT_ACCEL);

  log("Preferences loaded : rarewindspeed: %d decrewindspeed: %d limitToMiddle "
      "%d "
      "radius "
      "%f NunChuk multiplier %f RA Guide multiplier %f Accel: %d",
      raRewindFastFowardSpeed,
      decRewindFastFowardSpeed, raLimitSwitchToMiddleDistance,
      raLeadScrewToPivotMM, nunChukMultiplier, raGuideSpeedMultiplier,
      acceleration);
  // order matters here: rewind fast forward speed uses previous sets
  // for calcs
  raStatic.setNunChukMultiplier(nunChukMultiplier);
  raStatic.setGuideRateMultiplier(raGuideSpeedMultiplier);
  raStatic.setLimitSwitchToMiddleDistance(raLimitSwitchToMiddleDistance);
  raStatic.setScrewToPivotInMM(raLeadScrewToPivotMM);
  raStatic.setRewindFastFowardSpeedInHz(raRewindFastFowardSpeed);

  decStatic.setNunChukMultiplier(nunChukMultiplier);
  decStatic.setGuideRateMultiplier(raGuideSpeedMultiplier);
  decStatic.setLimitSwitchToMiddleDistance(decLimitSwitchToMiddleDistance);
  decStatic.setScrewToPivotInMM(decLeadScrewToPivotMM);
  decStatic.setRewindFastFowardSpeedInHz(decRewindFastFowardSpeed);

  motor.setAcceleration(acceleration);

  server.on("/getStatus", HTTP_GET,
            [&motor, &raStatic,&decStatic](AsyncWebServerRequest *request) {
              getStatus(request, motor, raStatic,decStatic);
            });

  server.on("/rarunbackSpeed", HTTP_POST,
            [&raStatic, &preferences](AsyncWebServerRequest *request) {
              setRARewindFastFowardSpeedInHz(request, raStatic, preferences);
            });

  server.on("/decrunbackSpeed", HTTP_POST,
            [&decStatic, &preferences](AsyncWebServerRequest *request) {
              setDecRewindFastFowardSpeedInHz(request, decStatic, preferences);
            });

  server.on("/raLeadToPivotDistance", HTTP_POST,
            [&raStatic, &preferences](AsyncWebServerRequest *request) {
              setRaLeadToPivotDistance(request, raStatic, preferences);
            });
  server.on("/raLimitToMiddleDistance", HTTP_POST,
            [&raStatic, &preferences](AsyncWebServerRequest *request) {
              setRaLimitToMiddleDistance(request, raStatic, preferences);
            });
  server.on("/decLeadToPivotDistance", HTTP_POST,
            [&decStatic, &preferences](AsyncWebServerRequest *request) {
              setDecLeadToPivotDistance(request, decStatic, preferences);
            });
  server.on("/decLimitToMiddleDistance", HTTP_POST,
            [&decStatic, &preferences](AsyncWebServerRequest *request) {
              setDecLimitToMiddleDistance(request, decStatic, preferences);
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

  server.on("/homera", HTTP_POST, [&raDynamic](AsyncWebServerRequest *request) {
    raDynamic.gotoStart();
  });

  server.on("/parkra", HTTP_POST, [&raDynamic](AsyncWebServerRequest *request) {
    raDynamic.gotoEndish();
  });

  server.on(
      "/centerra", HTTP_POST,
      [&raDynamic](AsyncWebServerRequest *request) { raDynamic.gotoMiddle(); });

  server.on("/homedec", HTTP_POST,
            [&decDynamic](AsyncWebServerRequest *request) {
              decDynamic.gotoStart();
            });

  server.on("/parkdec", HTTP_POST,
            [&decDynamic](AsyncWebServerRequest *request) {
              decDynamic.gotoEndish();
            });

  server.on("/centerdec", HTTP_POST,
            [&decDynamic](AsyncWebServerRequest *request) {
              decDynamic.gotoMiddle();
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
