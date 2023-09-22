#include "EQWebServer.h"
#include "AsyncUDP.h"
#include "Logging.h"
#include "MotorUnit.h"
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WebSerial.h>

AsyncWebServer server(80);
AsyncUDP dscUDP;
#define IPBROADCASTPORT 50375

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
        "position" : %f,
        "velocity" : %f
      })",
          model.getRewindFastFowardSpeed(), model.getGreatCircleRadius(),
          model.getLimitSwitchToMiddleDistance(), motor.getPositionInMM(),
          motor.getVelocityInMMPerMinute());

  String json = buffer;

  // String json = "{\"timestamp\":" + String(times.get()) + ",\"position\":"
  // + String(positions.get()) + ",\"velocity\":" + String(velocities.get()) +
  // "}";
  request->send(200, "application/json", json);
}

void setupWebServer(MotorUnit &motor, PlatformModel &model,
                    Preferences &preferences) {

  int rewindFastFowardSpeed =
      preferences.getUInt(PREF_SPEED_KEY, DEFAULT_SPEED);
  int limitSwitchToMiddleDistance =
      preferences.getUInt(PREF_MIDDLE_KEY, DEFAULT_MIDDLE_DISTANCE);
  int greatCircleRadius =
      preferences.getUInt(PREF_CIRCLE_KEY, DEFAULT_CIRCLE_RADIUS);
  log("Preferences loaded for model rewindspeed: %d limitToMiddle %d radius "
      "%d ",
      rewindFastFowardSpeed, limitSwitchToMiddleDistance, greatCircleRadius);
  model.setRewindFastFowardSpeedInHz(rewindFastFowardSpeed);
  model.setLimitSwitchToMiddleDistance(limitSwitchToMiddleDistance);
  model.setGreatCircleRadius(greatCircleRadius);

  if (dscUDP.listen(IPBROADCASTPORT)) {
    log("Listening for dsc platform broadcasts");
    dscUDP.onPacket([&motor](AsyncUDPPacket packet) {
      unsigned long now = millis();
      String msg = packet.readString();
      log("UDP Broadcast received: %s", msg.c_str());

      // Check if the broadcast is from EQ Platform
      if (msg.startsWith("DSC:")) {
        msg = msg.substring(4);
        log("Got payload from dsc");

        // Create a JSON document to hold the payload
        const size_t capacity = JSON_OBJECT_SIZE(5) +
                                40; // Reserve some memory for the JSON document
        StaticJsonDocument<capacity> doc;

        // Deserialize the JSON payload
        DeserializationError error = deserializeJson(doc, msg);
        if (error) {
          log("Failed to parse payload %s with error %s", msg.c_str(),
              error.c_str());
          return;
        }

        if (doc.containsKey("command") && doc.containsKey("parameter")) {
          String command = doc["command"];
          double parameter = doc["parameter"];

          if (command == "home") {
            motor.slewToStart();
          } else if (command = "park") {
            motor.slewToEnd();
          } else if (command == "track") {
            motor.setTracking(parameter > 0 ? true : false);
          } else if (command="moveaxis") {
            motor.moveAxis(parameter);
          }
          // IPAddress remoteIp = packet.remoteIP();

          // // Convert the IP address to a string
          // eqPlatformIP = remoteIp.toString();

        } else {
          log("Payload missing required fields.");
          return;
        }
      } else {
        log("Message has bad starting chars");
      }
    });
  }

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
  ;

  server.on("/home", HTTP_POST,
            [&motor](AsyncWebServerRequest *request) { motor.slewToStart(); });
  ;
  server.on("/park", HTTP_POST,
            [&motor](AsyncWebServerRequest *request) { motor.slewToEnd(); });
  ;
  server.on("/center", HTTP_POST,
            [&motor](AsyncWebServerRequest *request) { motor.slewToMiddle(); });
  ;

  // server.serveStatic("/www/", LittleFS, "/fs/");
  server.serveStatic("/", LittleFS, "/fs/");

  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);

  server.begin();
  log("Server started");
  return;
}

// TODO
//  void loop() {

//   ws.cleanupClients();
// }