#include "DigitalCaliper.h"
#include "Logging.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WebSerial.h>

AsyncWebServer server(80);

void setupWebServer(DigitalCaliper &caliper) {

  // server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  //   request->send(200, "text/plain", "Hi! I am ESP32.");
  // });

  server.on("/position", HTTP_GET, [&caliper](AsyncWebServerRequest *request) {
    log("/position");
    char buffer[40];
    sprintf(buffer, "%d", caliper.getPosition());
    String outbuffer = buffer;
    request->send(200, "text/plain", outbuffer);
  });

  server.serveStatic("/", LittleFS, "/fs/");

  server.on("/velocity", HTTP_GET, [&caliper](AsyncWebServerRequest *request) {
    // Create a JSON object with the data

    char buffer[80];
    sprintf(buffer, "{\"timestamp\":%d,\"position\":%d,\"velocity\":%5.2f}",
            caliper.getTime(), caliper.getPosition(), caliper.getVelocity());
    String json = buffer;

    // String json = "{\"timestamp\":" + String(times.get()) + ",\"position\":"
    // + String(positions.get()) + ",\"velocity\":" + String(velocities.get()) +
    // "}";
    request->send(200, "application/json", json);
  });

  server.on("/reset", HTTP_GET, [&caliper](AsyncWebServerRequest *request) {
    caliper.reset();
    request->send(200, "text/plain", "Position Reset");
  });

  // AsyncElegantOTA.begin(&server); // Start ElegantOTA

  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);

  server.begin();
  log("Server started");
  // webserver.begin();
  return ;
}
