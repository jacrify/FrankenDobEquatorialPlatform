#ifndef __MOTORDYNAMIC_H__
#define __MOTORDYNAMIC_H__

#include "MotorStatic.h"
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
class MotorDynamic {
public:
  MotorDynamic(MotorStatic &m);

  /**
   * Queue a pulseguide.
   * Direction is either  2 = guideEast, 3 = guideWest
   *    */
  virtual void pulseGuide(int direction, long pulseDurationInMilliseconds) = 0;

  /**
   * Queue a moveaxis.  degreesPerSecond can be positive or negative.
   */
  virtual void moveAxis(double degreesPerSecond) = 0;

  /**
   * Queue a movement based on a percentage (-100 to 100)
   * This is a percentage of nunChukMultiplier * sidereal tracking rate
   */
  virtual void moveAxisPercentage(int percentage) = 0;

  /**
   * Called from main loop. Should either stop or resume tracking.
   */
  virtual void stopOrTrack(int32_t pos) = 0;

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

  bool isSlewing();

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
   * Resume tracking. Called from isr so needs to be fast
   */
  void stopPulse();

  /**
   * Slew forward or back on ra axis by a number of degrees
   */
  void slewByDegrees(double degreesToSlew);

  void stop();
  void gotoMiddle();

  // Goto a position just short of end
  void gotoEndish();

  // Find limit switch
  void gotoStart();

  void setStepperWrapper(StepperWrapper *wrapper);

  int32_t getTargetPosition();
  uint32_t getTargetSpeedInMilliHz();

  // Output

protected:
  bool limitSwitchState;
  int32_t currentPosition;

  int32_t targetPosition;
  uint32_t targetSpeedInMilliHz;

  StepperWrapper *stepperWrapper;
  MotorStatic &model;

  bool isExecutingMove;
  bool isPulseGuiding;
  bool isMoveQueued;
  bool stopMove;
  bool safetyMode;

  long pulseGuideDurationMillis;

  uint32_t speedBeforePulseMHz;
};

#endif // __MOTORDYNAMIC_H__