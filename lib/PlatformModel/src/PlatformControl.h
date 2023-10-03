#ifndef __PLATFORMCONTROL_H__
#define __PLATFORMCONTROL_H__
#include "PlatformModel.h"
#include "StepperWrapper.h"
#include <cstdint>

/** Responsible for the dynamic state of the platform.
 * Handles the following:
 * - what to do when buttons pushed/released
 * - what to do when external commands (webui/network) received
 * - remembers whether tracking is on or off
 * - keeps running clock offset for rewind/fast forward moves.
 *
 * Called in two ways: directly from webui/network, or periodically from
 * loop.
 *
 * Delegates to PlatformModel for calculations.
 *
 * Uses StepperWrapper to send commands to motor.
 *
 */
class PlatformControl {
public:
  PlatformControl(PlatformModel &m);
  // Input
  void setLimitSwitchState(bool state);
  void setPlayButtonState(bool state);
  void setRewindButtonState(bool state);
  void setFastForwardButtonState(bool state);
  void setTrackingOnOff(bool tracking);

  bool isTrackingOn();

  // called from loop
  // returns either 0, or a number of milliseconds.
  // If non zero, called should pause for this
  // number of millis then call again.
  // This allows for exact implementation of pulseguides.
  // Note there will be a delay in pulseguide start of
  // on average half the main loop delay
  // However pulseguide duration should be accurate.
  long calculateOutput(unsigned long nowInMillis);

  // Extneral commands

  void pulseGuide(int direction, long pulseDurationInMilliseconds);
  void moveAxis(double degreesPerSecond);
  void stop();
  void gotoMiddle();
  void gotoEndish();
  void gotoStart();
  int32_t getTargetPosition();
  uint32_t getTargetSpeedInMilliHz();
  double getPlatformResetOffset();
  double getTimeToCenterInSeconds();
  double getTimeToEndOfRunInSeconds();
  void setStepperWrapper(StepperWrapper *wrapper);

  // Output

private:
  bool limitSwitchState;
  bool playButtonState;
  bool rewindButtonState;
  bool fastForwardButtonState;
  int32_t currentPosition;

  bool trackingOn;
  
  int32_t targetPosition;
  uint32_t targetSpeedInMilliHz;
  
  StepperWrapper *stepperWrapper;
  PlatformModel &model;

  bool isExecutingMove;
  bool isMoveQueued;
  bool stopMove;

  // unsigned long pulseGuideEndTime;
  long pulseGuideDurationMillis;

  double startMoveTimeOffset;
  double platformResetOffset;
};

#endif // __PLATFORMCONTROL_H__