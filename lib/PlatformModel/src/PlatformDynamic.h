#ifndef __PlatformStatic_H__
#define __PlatformStatic_H__
#include "PlatformStatic.h"
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
 * Delegates to PlatformStatic for calculations.
 *
 * Uses StepperWrapper to send commands to motor.
 *
 */
class PlatformDynamic {
public:
  PlatformDynamic(PlatformStatic &m);
  // Input
  // Is Limit switch pushed
  void setLimitSwitchState(bool state);

  void setTrackingOnOff(bool tracking);

  /**
   * When limit switch position is not saved in preferences,
   * we want to run slower when running towards limit switch,
   * so as not to stress the physical fixing.
   */
  void setSafetyMode(bool b);

  bool isTrackingOn();

  // Called from loop
  // Returns either 0, or a number of milliseconds.
  // If non zero, called should pause for this
  // number of millis then call again.
  // This allows for exact implementation of pulseguides.
  // IE a queued pulseguide will set the pulseguide speed,
  // then client will delay, then speed will be set back
  // to tracking speed.
  // Note there will be a delay in pulseguide start of
  // on average half the main loop delay
  // However pulseguide duration should be accurate.
  long calculateOutput();

  // Extneral commands

  /**
   * Queue a pulseguide.
   * Direction is either  2 = guideEast, 3 = guideWest
   *    */
  void pulseGuide(int direction, long pulseDurationInMilliseconds);

  /**
   * Slew forward or back on ra axis by a number of degrees
   */
  void slewByDegrees(double degreesToSlew);
  /**
   * Queue a moveaxis.  degreesPerSecond can be positive or negative.
   */
  void moveAxis(double degreesPerSecond);

  /**
   * Queue a movement based on a percentage (-100 to 100)
   * This is a percentage of nunChukMultiplier * sidereal tracking rate
   */
  void moveAxisPercentage(int percentage);

  void stop();
  void gotoMiddle();

  // Goto a position just short of end
  void gotoEndish();

  // Find limit switch
  void gotoStart();

  // Get run time until platform is centered
  double getTimeToCenterInSeconds();

  // Get time left to run
  double getTimeToEndOfRunInSeconds();

  void setStepperWrapper(StepperWrapper *wrapper);

  int32_t getTargetPosition();
  uint32_t getTargetSpeedInMilliHz();

  // Output

private:
  bool limitSwitchState;
  int32_t currentPosition;

  bool trackingOn;

  int32_t targetPosition;
  uint32_t targetSpeedInMilliHz;

  StepperWrapper *stepperWrapper;
  PlatformStatic &model;

  bool isExecutingMove;
  bool isMoveQueued;
  bool stopMove;
  bool safetyMode;

  long pulseGuideDurationMillis;

  double startMoveTimeOffset;
};

#endif // __PlatformStatic_H__