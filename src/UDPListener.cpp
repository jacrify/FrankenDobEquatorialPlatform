#include "UDPListener.h"
#include "AsyncUDP.h"
#include "Logging.h"
#include <ArduinoJson.h>

AsyncUDP dscUDP;
#define IPBROADCASTPORT 50375
/**
 * Listen for UDP broadcasts from Digital Setting Circles.
 * This is used for alpaca commands passed from DSC.
*/
void setupUDPListener(MotorUnit &motor) {
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
          } else if (command == "park") {
            motor.slewToEnd();
          } else if (command == "track") {
            motor.setTracking(parameter > 0 ? true : false);
          } else if (command = "moveaxis") {
            motor.moveAxis(parameter);
          } else {
            log("Unknown command %s",command.c_str());
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
}