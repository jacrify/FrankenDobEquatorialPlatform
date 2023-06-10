#include "Logging.h"
#include "MotorUnit.h"
// #include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WebSerial.h>

AsyncWebServer server(80);

void moveTo(AsyncWebServerRequest *request, MotorUnit &motor) {
  log("/moveTo");
  if (request->hasArg("position")) {
    String position = request->arg("position");

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
  if (request->hasArg("speed")) {
    String speed = request->arg("speed");

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
void setrunbackSpeed(AsyncWebServerRequest *request, MotorUnit &motor) {
  log("/setrunbackSpeed");
  if (request->hasArg("speed")) {
    String speed = request->arg("speed");

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

void getStatus(AsyncWebServerRequest *request, MotorUnit &motor) {
  log("/getStatus");
}

  void setupWebServer(MotorUnit & motor) {

    server.on("/getStatus", HTTP_GET,
              [&motor](AsyncWebServerRequest *request) {getStatus(request,motor);  });

    server.on("/setCalibrationSpeed", HTTP_GET,
              [&motor](AsyncWebServerRequest *request) {
                setCalibrationSpeed(request, motor);
              });

    server.on("/setrunbackSpeed", HTTP_GET,
              [&motor](AsyncWebServerRequest *request) {
                setrunbackSpeed(request, motor);
              });

    server.on("/moveTo", HTTP_GET, [&motor](AsyncWebServerRequest *request) {
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
