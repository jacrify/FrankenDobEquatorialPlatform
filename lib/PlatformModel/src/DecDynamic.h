#ifndef __DECDYNAMIC_H__
#define __DECDYNAMIC_H__

#include "DecStatic.h"
#include "StepperWrapper.h"
#include <cstdint>

/** Responsible for the dynamic state of the dexc axis.
 * Handles the following:
 * - what to do when external commands (webui/network) received
 * - keeps running clock offset for rewind/fast forward moves.
 *
 * Called in two ways: directly from webui/network, or periodically from
 * loop.
 *
 * Delegates to DecStatic for calculations.
 *
 * Uses StepperWrapper to send commands to motor.
 *
 */
class DecDynamic {
public:
  DecDynamic(DecStatic &m);
  // Input
  // Is Limit switch pushed
  void setLimitSwitchState(bool state);

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
  // then client will delay,
  // Note there will be a delay in pulseguide start of
  // on average half the main loop delay
  // However pulseguide duration should be accurate.
  long calculateOutput();

  // Extneral commands

  /**
   * Queue a pulseguide.
   * Direction is either  TODO
   *    */
  void pulseGuide(int direction, long pulseDurationInMilliseconds);

  /**
   * Resume tracking. Called from isr so needs to be fast
   */
  void stopPulse();

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

  // Find limit switch
  void gotoStart();

  void setStepperWrapper(StepperWrapper *wrapper);

  int32_t getTargetPosition();

  // Output

private:
  bool limitSwitchState;
  int32_t currentPosition;

  int32_t targetPosition;
  uint32_t targetSpeedInMilliHz;

  StepperWrapper *stepperWrapper;
  DecStatic &model;

  bool isExecutingMove;
  bool isPulseGuiding;
  bool isMoveQueued;
  bool stopMove;
  bool safetyMode;

  long pulseGuideDurationMillis;

  uint32_t speedBeforePulseMHz;
};

#endif // __DECDYNAMIC_H__