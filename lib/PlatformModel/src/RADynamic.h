#ifndef ___RADynamic_H__
#define ___RADynamic_H__
#include "MotorDynamic.h"
#include "RAStatic.h"
#include "StepperWrapper.h"
#include <cstdint>

/** Responsible for the dynamic state of the platform.
 * Handles the following:
 * - what to do when external commands (webui/network) received
 * - remembers whether tracking is on or off
 * - keeps running clock offset for rewind/fast forward moves.
 *
 * Called in two ways: directly from webui/network, or periodically from
 * loop.
 *
 * Delegates to RAStatic for calculations.
 *
 * Uses StepperWrapper to send commands to motor.
 *
 */
class RADynamic : public MotorDynamic {
public:
  RADynamic(RAStatic &m);

  void moveAxisPercentage(int percentage);
  void moveAxis(double degreesPerSecond);
  void pulseGuide(int direction, long pulseDurationInMilliseconds);
  void stopOrTrack(int32_t pos);

  void setTrackingOnOff(bool tracking);
  bool isTrackingOn();

  // Get run time until platform is centered
  double getTimeToCenterInSeconds();
  // Get time left to run
  double getTimeToEndOfRunInSeconds();

private:
  bool trackingOn;
  RAStatic &model;
};

#endif // __PlatformStatic_H__