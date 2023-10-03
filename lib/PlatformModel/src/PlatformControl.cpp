#include "PlatformControl.h"
#include "Logging.h"
#include <cmath>
void PlatformControl::setLimitSwitchState(bool state) {
  limitSwitchState = state;
}

void PlatformControl::setPlayButtonState(bool state) {
  playButtonState = state;
}

void PlatformControl::setRewindButtonState(bool state) {
  rewindButtonState = state;
}

void PlatformControl::setFastForwardButtonState(bool state) {
  fastForwardButtonState = state;
}

void PlatformControl::setTrackingOnOff(bool t) { trackingOn = t; }

bool PlatformControl::isTrackingOn() { return trackingOn; }

long PlatformControl::calculateOutput(unsigned long nowInMillis) {

  if (pulseGuideDurationMillis > 0) {
    stepperWrapper->setStepperSpeed(targetSpeedInMilliHz);
    long delay = pulseGuideDurationMillis;
    pulseGuideDurationMillis = 0;
    return delay; // caller will call back right after delay.
  }

  // used to flag the end of a move
  // handle limit switch
  if (limitSwitchState) {
    int32_t limitPos = model.getLimitPosition();
    stepperWrapper->resetPosition(limitPos);
    if (targetPosition > limitPos) {
      if (isExecutingMove) {
        // move target is past limit switch. Stop unless tracking on
        isExecutingMove = false;
        isMoveQueued = false;
        stopMove = true;
      }
    }
  }

  int32_t pos = stepperWrapper->getPosition();

  if (isMoveQueued) {
    if (pos != targetPosition) {
      log("Moving");
      stepperWrapper->moveTo(targetPosition, targetSpeedInMilliHz);
      isMoveQueued = false;
      // store how many seconds from center we are at start of move
      startMoveTimeOffset = model.calculateTimeToCenterInSeconds(pos);
    }
  }
  // check for move end
  if (isExecutingMove) {
    // are we there yet?
    if (pos == targetPosition) {
      isExecutingMove = false;
      isMoveQueued = false;
      stopMove = true;
      // stepperWrapper->stop();
    } else {
      return 0;
    }
  }

  if (stopMove) {
    // we've finished a move. Work out sidereal clock offset to apply
    double endMoveTimeOffset = model.calculateTimeToCenterInSeconds(pos);
    // if move started at startMoveTimeOffset=10, and finishes at
    // startMoveTimeOffset=20 then we've rewound by 10s. We should accumalate
    // -10s in platformResetOffset
    double moveClockDelta = startMoveTimeOffset - endMoveTimeOffset;
    platformResetOffset += moveClockDelta;
    stopMove = false;
  }
  if (trackingOn) {
    if (pos > 0) {
      targetPosition = 0;
      // TODO calc this every time? Careful of pulseguide if we move it
      log("Tracking");
      targetSpeedInMilliHz = model.calculateFowardSpeedInMilliHz(pos);
      stepperWrapper->moveTo(targetPosition, targetSpeedInMilliHz);
    } else {
      stepperWrapper->stop();
      trackingOn = false; // turn off at end of run.
    }
  } else {
    stepperWrapper->stop();
  }
  return 0;
}

void PlatformControl::gotoMiddle() {
  targetPosition = model.getMiddlePosition();
  targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz();
  isExecutingMove = true;
  isMoveQueued = true;
}

void PlatformControl::gotoEndish() {
  // TODO set back from end
  // slewToPosition(model.getStepsPerMM() * 10);
  targetPosition = 0;
  targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz();
  isExecutingMove = true;
  isMoveQueued = true;
}

void PlatformControl::gotoStart() {
  // should run until limit switch hit
  targetPosition = INT32_MAX;
  targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz();
  isExecutingMove = true;
  isMoveQueued = true;
}

int32_t PlatformControl::getTargetPosition() { return targetPosition; }

uint32_t PlatformControl::getTargetSpeedInMilliHz() {
  return targetSpeedInMilliHz;
}

double PlatformControl::getPlatformResetOffset() { return platformResetOffset; }

void PlatformControl::setStepperWrapper(StepperWrapper *wrapper) {
  stepperWrapper = wrapper;
}

PlatformControl::PlatformControl(PlatformModel &m) : model(m) {
  isMoveQueued = false;
  isExecutingMove = false;
  trackingOn = false;
  stopMove = false;
  pulseGuideDurationMillis = 0;
}

void PlatformControl::pulseGuide(int direction,
                                 long pulseDurationInMilliseconds) {
  // if not tracking, do nothing
  if (trackingOn) {
    // Direction is either  2 = guideEast, 3 = guideWest.
    // If 2 then returned value will be higher than stepperCurrentPosition
    // If 3 then return value will be lower.
    double targetSpeedInArcSecsSec = model.getTrackingRateArcsSecondsSec();
    // NOTE: this assumes target speed will be positive
    if (direction == 3) // west: go faster
      targetSpeedInArcSecsSec += model.getRAGuideRateArcSecondsSecond();

    if (direction == 2) // east: go slower
      targetSpeedInArcSecsSec -= model.getRAGuideRateArcSecondsSecond();

    targetSpeedInMilliHz =
        model.calculateFowardSpeedInMilliHz(targetSpeedInArcSecsSec);
    pulseGuideDurationMillis = pulseDurationInMilliseconds;

    // // divide by 1000 to get hz, then 1000 to get seconds.
    // // Ie if speed in millihz is 1000 (ie 1 hz, or one step per second)
    // // and we move for 1000 millis (is one second)
    // // then we move 1,000,000 / 1,000,000 = 1 step
    // int32_t stepsToMove =
    //     (stepperCurrentPosition * pulseDurationInMilliseconds) / 1000000;

    // if (direction == 2) // east.Positive step change ie towards limit switch
    //   targetPosition += stepsToMove;

    // if (direction == 3) // west. negative step change
    //   targetPosition -= stepsToMove;

    // // make sure we don't run off end
    // targetPosition = (targetPosition < 0) ? 0 : targetPosition;

    // log("Pulse guiding %d for %ld ms to position %ld at speed (millihz) %lf",
    //     direction, pulseDurationInMilliseconds, targetPosition);

    // isExecutingMove = true;
    // stepperWrapper->moveTo(targetPosition, targetSpeedInMilliHz);
  }
}

void PlatformControl::stop() {
  isExecutingMove = false;
  stopMove = true;
}

void PlatformControl::moveAxis(double degreesPerSecond) {

  log("Incoming movexis command speed %lf", degreesPerSecond);
  if (degreesPerSecond == 0) {
    isExecutingMove = false; // loop should perform stop / resume track
    return;
  }
  // If we are currently tracking, add the tracking speed.
  // If we don't do this, if rate from client is 1x sidereal and they move
  // 1x sidereal forward, stars stay stationary.

  // positive is west (arbitary?)
  if (trackingOn) {
    degreesPerSecond -= model.getTrackingRateDegreesSec();
  }
  targetSpeedInMilliHz = model.calculateFowardSpeedInMilliHz(
      stepperWrapper->getPosition(), 3600.0 * fabs(degreesPerSecond));

  isExecutingMove = true;
  isMoveQueued = true;
  // forward
  if (degreesPerSecond > 0) {
    targetPosition = 0;
  } else {
    targetPosition = INT32_MAX;
  }
}

double PlatformControl::getTimeToCenterInSeconds() {
  return model.calculateTimeToCenterInSeconds(stepperWrapper->getPosition());
}

double PlatformControl::getTimeToEndOfRunInSeconds() {
  return model.calculateTimeToEndOfRunInSeconds(stepperWrapper->getPosition());
}