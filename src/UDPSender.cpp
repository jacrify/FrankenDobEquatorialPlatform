#include "UDPSender.h"
#include "AsyncUDP.h"
#include "Logging.h"
#include "WiFi.h"

#define IPBROADCASTPERIOD 1000
#define IPBROADCASTPORT 50375
AsyncUDP udp;
unsigned long lastIPBroadcastTime;

// every x second, send whether platform is tracking,
// and how many seconds it will take to reach center.
//(can be negative is center passed)
// AxixMoveRate is max speed in degrees per second
void broadcastStatus(MotorUnit &motorUnit, PlatformModel &model) {

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
      char response[400];
      double secondsToCenter = motorUnit.getTimeToCenterInSeconds();
      double secondsToEnd = motorUnit.getTimeToEndOfRunInSeconds();
      double platformResetOffsetSeconds =
          motorUnit.getPlatformResetOffsetSeconds();

      bool platformTracking = motorUnit.getTrackingStatus();
      double axisMoveRate = model.getAxisMoveRate();
      double guideMoveRate = model.getRAGuideRateDegreesSec();
      double trackingRate = model.getTrackingRateArcsSecondsSec();
         

              snprintf(response, sizeof(response),
                       "EQ:{ "
                       "\"timeToCenter\": %.2lf, "
                       "\"timeToEnd\": %.2lf, "
                       "\"platformResetOffset\": %.2lf, "
                       "\"isTracking\" : %s, "
                       "\"guideMoveRate\": %.2lf, "
                       "\"trackingRate\": %.2lf, "
                       "\"axisMoveRate\": %.2lf "
                       " }\n",
                       secondsToCenter, secondsToEnd,
                       platformResetOffsetSeconds,
                       platformTracking ? "true" : "false", guideMoveRate,
                       trackingRate,axisMoveRate);
      udp.print(response);
      log("Status Packet sent\r\n %s", response);
    }
  }
}