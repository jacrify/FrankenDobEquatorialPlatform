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
  void calculateOutput(unsigned long nowInMillis);

  // Extneral commands

  void pulseGuide(int direction, long pulseDurationInMilliseconds,
                  unsigned long nowInMillis);
  void moveAxis(double degreesPerSecond);
  void stop();
  void gotoMiddle();
  void gotoEndish();
  void gotoStart();
  int32_t getTargetPosition();
  uint32_t getTargetSpeedInMilliHz();
  double getPlatformResetOffset();
  void setStepperWrapper(StepperWrapper *wrapper);


  // Output

private:
  bool limitSwitchState;
  bool playButtonState;
  bool rewindButtonState;
  bool fastForwardButtonState;
  int32_t currentPosition;

  bool trackingOn;
  bool positonResetRequired;
  int32_t platformResetPosition;

  int32_t targetPosition;
  uint32_t targetSpeedInMilliHz;
  bool targetSet;
  StepperWrapper *stepperWrapper;
  PlatformModel &model;

  bool isExecutingMove;
  bool isMoveQueued;

  unsigned long pulseGuideEndTime;

  double startMoveTimeOffset;
  double platformResetOffset;
};

#endif // __PLATFORMCONTROL_H__