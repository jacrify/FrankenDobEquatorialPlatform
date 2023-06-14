#include "Logging.h"
#include "MotorUnit.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WebSerial.h>

AsyncWebServer server(80);

void moveTo(AsyncWebServerRequest *request, MotorUnit &motor) {
  log("/moveTo");
  if (request->hasArg("position")) {
    String position = request->arg("value");

    int positionValue = position.toInt();
    if (positionValue == 0 && position != "0") {
      log("Could not parse position");
      return;
    }

    motor.moveTo(positionValue);
    return;
  }

  log("No position arg found");
}

void setCalibrationSpeed(AsyncWebServerRequest *request, MotorUnit &motor) {

  log("/setCalibrationSpeed");
  if (request->hasArg("value")) {
    String speed = request->arg("value");

    int speedValue = speed.toInt();
    if (speedValue == 0 && speed != "0") {
      log("Could not parse speed");
      return;
    }
    motor.setCalibrationSpeed(speedValue);
    return;
  }
  log("No speed arg found");
}
void setRunbackSpeed(AsyncWebServerRequest *request, MotorUnit &motor) {
  log("/setrunbackSpeed");
  if (request->hasArg("value")) {
    String speed = request->arg("value");

    int speedValue = speed.toInt();
    if (speedValue == 0 && speed != "0") {
      log("Could not parse speed");
      return;
    }
    motor.setrunbackSpeed(speedValue);
    return;
  }
  log("No speed arg found");
}

void setGreatCircleRadius(AsyncWebServerRequest *request, MotorUnit &motor) {

  log("/setGreatCircleRadius");
  if (request->hasArg("value")) {
    String radius = request->arg("value");

    double radValue = radius.toDouble();
    if (radValue == 0 && radius != "0.0") {
      log("Could not parse radius");
      return;
    }
    motor.setGreatCircleRadius(radValue);
    return;
  }
  log("No speed arg found");
}

void getStatus(AsyncWebServerRequest *request, MotorUnit &motor) {
  // log("/getStatus");

  char buffer[300];
  sprintf(buffer,
          R"({
        "runbackSpeed" : %d,
        "calibrationSpeed" : %d,
        "greatCircleRadius": %f,
        "position" : %f,
        "velocity" : %d
      })",
          motor.getRunBackSpeed(), motor.getCalibrationSpeed(),
          motor.getGreatCircleRadius(), motor.getPositionInMM(),
          motor.getVelocityInMMPerMinute());

  String json = buffer;

  // String json = "{\"timestamp\":" + String(times.get()) + ",\"position\":"
  // + String(positions.get()) + ",\"velocity\":" + String(velocities.get()) +
  // "}";
  request->send(200, "application/json", json);
}

void setupWebServer(MotorUnit &motor) {

  server.on("/getStatus", HTTP_GET, [&motor](AsyncWebServerRequest *request) {
    getStatus(request, motor);
  });

  server.on("/calibrationSpeed", HTTP_POST,
            [&motor](AsyncWebServerRequest *request) {
              setCalibrationSpeed(request, motor);
            });

  server.on("/runbackSpeed", HTTP_POST,
            [&motor](AsyncWebServerRequest *request) {
              setRunbackSpeed(request, motor);
            });

  server.on("/greatCircleRadius", HTTP_POST,
            [&motor](AsyncWebServerRequest *request) {
              setGreatCircleRadius(request, motor);
            });

  server.on("/moveTo", HTTP_POST, [&motor](AsyncWebServerRequest *request) {
    moveTo(request, motor);
  });

  server.serveStatic("/www/", LittleFS, "/fs/");
  server.serveStatic("/", LittleFS, "/fs/index.htm");

  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);

  server.begin();
  log("Server started");
  return;
}
