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
void setupUDPListener(MotorUnit &motor, RADynamic &raDynamic,
                      DecDynamic &decDynamic) {
  if (dscUDP.listen(IPBROADCASTPORT)) {
    log("Listening for dsc platform broadcasts");
    dscUDP.onPacket([&motor, &raDynamic, &decDynamic](AsyncUDPPacket packet) {
      unsigned long now = millis();
      String start = packet.readStringUntil(':');
      // log("UDP Broadcast received: %s", msg.c_str());

      // Check if the broadcast is from EQ Platform
      if (start == "EQ") {
        // msg = msg.substring(4);
        log("Got payload from dsc");

        // Create a JSON document to hold the payload
        const size_t capacity = JSON_OBJECT_SIZE(5) +
                                40; // Reserve some memory for the JSON document
        StaticJsonDocument<capacity> doc;

        // Deserialize the JSON payload
        DeserializationError error = deserializeJson(doc, packet);
        if (error) {
          log("Failed to parse payload with error %s", error.c_str());
          return;
        }

        if (doc.containsKey("command") && doc.containsKey("parameter1") &&
            doc.containsKey("parameter2")) {

          String command = doc["command"];
          double parameter1 = doc["parameter1"];
          double parameter2 = doc["parameter2"];

          if (command == "home") {
            raDynamic.gotoStart();
            decDynamic.gotoMiddle();
            return;
          }
          if (command == "park") {
            raDynamic.gotoEndish();
            decDynamic.gotoMiddle();
            return;
          }
          if (command == "track") {
            raDynamic.setTrackingOnOff(parameter1 > 0 ? true : false);
            return;
          }
          if (command == "moveaxis") {
            int axis = parameter1;
            double degreesPerSecond = parameter2;
            if (axis == 0)
              raDynamic.moveAxis(degreesPerSecond);
            if (axis == 1)
              decDynamic.moveAxis(degreesPerSecond);
            return;
          }

          if (command == "slewbydegrees") {
            int axis = parameter1;
            double degreesToSlew = parameter2;
            if (axis == 0)
              raDynamic.slewByDegrees(degreesToSlew);
            if (axis == 1)
              decDynamic.slewByDegrees(degreesToSlew);
            return;
          }

          if (command == "moveaxispercentage") {
            int axis = parameter1;
            double percentageOfSpeed = parameter2;
            log("Move axis percentage received %i %f", axis,percentageOfSpeed);
            if (axis == 0)
              raDynamic.moveAxisPercentage(percentageOfSpeed);
            if (axis == 1)
              decDynamic.moveAxisPercentage(percentageOfSpeed);
            return;
          }
          if (command == "pulseguide") {
            int direction = parameter1;
            long duration = parameter2;

            if (direction == 2 || direction == 3) {
              raDynamic.pulseGuide(direction, duration);
              return;
            } else if (direction == 0 || direction == 1) {
              decDynamic.pulseGuide(direction, duration);
              return;
            }
            log("Unknown pulseguide direction %d", direction);
            return;
          }

          log("Unknown command %s", command.c_str());
          return;

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