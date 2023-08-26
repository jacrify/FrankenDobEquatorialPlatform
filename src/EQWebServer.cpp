#include "Logging.h"
#include "MotorUnit.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WebSerial.h>

AsyncWebServer server(80);

void setRewindFastFowardSpeed(AsyncWebServerRequest *request,
                              PlatformModel &model) {
  log("/setrunbackSpeed");
  if (request->hasArg("value")) {
    String speed = request->arg("value");

    int speedValue = speed.toInt();
    if (speedValue == 0 && speed != "0") {
      log("Could not parse speed");
      return;
    }
    model.setRewindFastFowardSpeed(speedValue);
    return;
  }
  log("No speed arg found");
}

void setGreatCircleRadius(AsyncWebServerRequest *request,
                          PlatformModel &model) {

  log("/setGreatCircleRadius");
  if (request->hasArg("value")) {
    String radius = request->arg("value");

    double radValue = radius.toDouble();
    if (radValue == 0 && radius != "0.0") {
      log("Could not parse radius");
      return;
    }
    model.setGreatCircleRadius(radValue);
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
        "runbackSpeed" : %d,
        "greatCircleRadius": %f,
        "position" : %f,
        "velocity" : %d
      })",
          model.getRewindFastFowardSpeed(),
          model.getGreatCircleRadius(), motor.getPositionInMM(),
          motor.getVelocityInMMPerMinute());

  String json = buffer;

  // String json = "{\"timestamp\":" + String(times.get()) + ",\"position\":"
  // + String(positions.get()) + ",\"velocity\":" + String(velocities.get()) +
  // "}";
  request->send(200, "application/json", json);
}

void setupWebServer(MotorUnit &motor, PlatformModel &model) {

  server.on("/getStatus", HTTP_GET,
            [&motor, &model](AsyncWebServerRequest *request) {
              getStatus(request, motor, model);
            });

  server.on("/runbackSpeed", HTTP_POST,
            [&model](AsyncWebServerRequest *request) {
              setRewindFastFowardSpeed(request, model);
            });

  server.on("/greatCircleRadius", HTTP_POST,
            [&model](AsyncWebServerRequest *request) {
              setGreatCircleRadius(request, model);
            });

  ;

  server.serveStatic("/www/", LittleFS, "/fs/");
  server.serveStatic("/", LittleFS, "/fs/index.htm");

  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);

  server.begin();
  log("Server started");
  return;
}
