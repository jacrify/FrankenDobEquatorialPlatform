#include "EQWebServer.h"
#include "Logging.h"
#include "MotorUnit.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WebSerial.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// When polled via web socket, send a response.
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len,
                            AsyncWebSocketClient *client, MotorUnit &motor) {
  log("Handling web socket request...");
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len &&
      info->opcode == WS_TEXT) {
    data[len] = 0; // Null-terminate
    // Now compare the message
    if (strcmp((char *)data, "request_data") == 0) {
      double timeToCenter = motor.getTimeToCenterInSeconds();
      bool isTracking = motor.getTrackingStatus();
      log("Responding to telescope client with time to center %lf and status "
          "%d",
          timeToCenter, isTracking);
      char
          response[256]; // Assuming 256 characters is enough. Adjust if needed.
      snprintf(response, sizeof(response),
               "{"
               "\"timeToCenter\": %lf,"
               "\"isTracking\": %s"
               "}",
               timeToCenter, isTracking ? "true" : "false");

      client->text(response);
    }
  }
}

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

void setRewindFastFowardSpeed(AsyncWebServerRequest *request,
                              PlatformModel &model, Preferences &preferences) {
  log("/setrunbackSpeed");
  if (request->hasArg("value")) {
    String speed = request->arg("value");

    int speedValue = speed.toInt();
    if (speedValue == 0 && speed != "0") {
      log("Could not parse speed");
      return;
    }
    model.setRewindFastFowardSpeed(speedValue);
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

  // DSCs regularly poll via web socket asking for position. Set up the handler
  // here
  ws.onEvent([&motor](AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
    log("Web socket event");

    switch (type) {
    case WS_EVT_CONNECT:
      log("WebSocket client #%u connected from %s\n", client->id(),
          client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      log("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      log("Web socket message received");

      // TODO add tracking status
      handleWebSocketMessage(arg, data, len, client, motor);
      break;
      // case WS_EVT_PONG:
      // case WS_EVT_ERROR:
      break;
    }
  });
  server.addHandler(&ws);

  int rewindFastFowardSpeed =
      preferences.getUInt(PREF_SPEED_KEY, DEFAULT_SPEED);
  int limitSwitchToMiddleDistance =
      preferences.getUInt(PREF_MIDDLE_KEY, DEFAULT_MIDDLE_DISTANCE);
  int greatCircleRadius =
      preferences.getUInt(PREF_CIRCLE_KEY, DEFAULT_CIRCLE_RADIUS);
  log("Preferences loaded for model rewindspeed: %d limitToMiddle %d radius "
      "%d ",
      rewindFastFowardSpeed, limitSwitchToMiddleDistance, greatCircleRadius);
  model.setRewindFastFowardSpeed(rewindFastFowardSpeed);
  model.setLimitSwitchToMiddleDistance(limitSwitchToMiddleDistance);
  model.setGreatCircleRadius(greatCircleRadius);

  server.on("/getStatus", HTTP_GET,
            [&motor, &model](AsyncWebServerRequest *request) {
              getStatus(request, motor, model);
            });

  server.on("/runbackSpeed", HTTP_POST,
            [&model, &preferences](AsyncWebServerRequest *request) {
              setRewindFastFowardSpeed(request, model, preferences);
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