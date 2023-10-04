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
void setupUDPListener(MotorUnit &motor,PlatformControl &control) {
  if (dscUDP.listen(IPBROADCASTPORT)) {
    log("Listening for dsc platform broadcasts");
    dscUDP.onPacket([&motor,&control](AsyncUDPPacket packet) {
      unsigned long now = millis();
      String start = packet.readStringUntil(':');
      // log("UDP Broadcast received: %s", msg.c_str());

      // Check if the broadcast is from EQ Platform
      if (start=="EQ") {
        // msg = msg.substring(4);
        log("Got payload from dsc");

        // Create a JSON document to hold the payload
        const size_t capacity = JSON_OBJECT_SIZE(5) +
                                40; // Reserve some memory for the JSON document
        StaticJsonDocument<capacity> doc;

        // Deserialize the JSON payload
        DeserializationError error = deserializeJson(doc, packet);
        if (error) {
          log("Failed to parse payload with error %s",
              error.c_str());
          return;
        }

        if (doc.containsKey("command") && doc.containsKey("parameter1") &&
            doc.containsKey("parameter2")) {
          
          String command = doc["command"];
          double parameter1 = doc["parameter1"];
          double parameter2 = doc["parameter2"];

          if (command == "home") {
            control.gotoStart();
            return;
          }
          if (command == "park") {
            control.gotoEndish();
            return;
          }
          if (command == "track") {
            control.setTrackingOnOff(parameter1 > 0 ? true : false);
            return;
          }
          if (command == "moveaxis") {
            control.moveAxis(parameter1);
            return;
          }
          if (command == "pulseguide") {
            control
                .pulseGuide(parameter1, parameter2);
            return;
          }

          log("Unknown command %s", command.c_str());
          return;

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