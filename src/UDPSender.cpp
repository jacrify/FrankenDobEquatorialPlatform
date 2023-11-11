#include "UDPSender.h"
#include "AsyncUDP.h"
#include "Logging.h"
#include "WiFi.h"
#include <ArduinoJson.h>
#define IPBROADCASTPERIOD 1000
#define IPBROADCASTPORT 50375
AsyncUDP udp;
unsigned long lastIPBroadcastTime;

// every x second, send whether platform is tracking,
// and how many seconds it will take to reach center.
//(can be negative is center passed)
// AxixMoveRate is max speed in degrees per second
void broadcastStatus(MotorUnit &motorUnit, RAStatic &raStatic,
                     RADynamic &raDynamic) {

  long now = millis();
  if ((now - lastIPBroadcastTime) > IPBROADCASTPERIOD) {
    lastIPBroadcastTime = now;
    // Check if the device is connected to the WiFi
    if (WiFi.status() != WL_CONNECTED) {
      return;
    }
    if (udp.connect(
            IPAddress(255, 255, 255, 255),
            IPBROADCASTPORT)) { // Choose any available port, e.g., 12345

      // Estimate JSON capacity
      const size_t capacity = JSON_OBJECT_SIZE(15);

      DynamicJsonDocument doc(capacity);
      // Populate the JSON object
      doc["timeToCenter"] = raDynamic.getTimeToEndOfRunInSeconds();
      doc["timeToEnd"] = raDynamic.getTimeToEndOfRunInSeconds();
      doc["isTracking"] = raDynamic.isTrackingOn();
      doc["slewing"] = raDynamic.isSlewing();
      doc["guideMoveRate"] = raStatic.getGuideRateDegreesSec();
      doc["trackingRate"] = raStatic.getTrackingRateArcsSecondsSec();
      doc["axisMoveRateMax"] = raStatic.getMaxAxisMoveRateDegreesSec();
      doc["axisMoveRateMin"] = raStatic.getMinAxisMoveRateDegreesSec();

      String json;
      serializeJson(doc, json);
      json = "DSC:" + json;
      udp.print(json.c_str());

      // log("Status Packet sent\r\n %s", response);
    }
  }
}