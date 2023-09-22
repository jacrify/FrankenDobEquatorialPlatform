#include "UDPSender.h"
#include "AsyncUDP.h"
#include "WiFi.h"
#include "Logging.h"

#define IPBROADCASTPERIOD 10000
#define IPBROADCASTPORT 50375
AsyncUDP udp;
unsigned long lastIPBroadcastTime;

// every x second, send whether platform is tracking,
// and how many seconds it will take to reach center.
//(can be negative is center passed)
// AxixMoveRate is max speed in degrees per second
void broadcastStatus(MotorUnit &motorUnit, PlatformModel &model) {
  double secondsToCenter = motorUnit.getTimeToCenterInSeconds();
  double secondsToEnd = motorUnit.getTimeToEndOfRunInSeconds();

  bool platformTracking = motorUnit.getTrackingStatus();
  double axisMoveRate = model.getAxisMoveRate();

  long now = millis();
  if ((now - lastIPBroadcastTime) > IPBROADCASTPERIOD) {
    log("Preparing to send packet");
    lastIPBroadcastTime = now;
    // Check if the device is connected to the WiFi
    if (WiFi.status() != WL_CONNECTED) {
      return;
    }
    if (udp.connect(
            IPAddress(255, 255, 255, 255),
            IPBROADCASTPORT)) { // Choose any available port, e.g., 12345
      char response[400];

      snprintf(response, sizeof(response),
               "EQ:{ "
               "\"timeToCenter\": %.2lf, "
               "\"timeToEnd\": %.2lf, "
               "\"isTracking\" : %s "
               "\"axisMoveRate\": %.2lf, "
               " }",
               secondsToCenter, secondsToEnd,
               platformTracking ? "true" : "false", axisMoveRate);
      udp.print(response);
      log("Status Packet sent");
    }
  }
}