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

void PlatformControl::setSafetyMode(bool s) { safetyMode = s; }

void PlatformControl::setTrackingOnOff(bool t) { trackingOn = t; }

bool PlatformControl::isTrackingOn() { return trackingOn; }

long PlatformControl::calculateOutput() {

  // Pulseguide command has been queued. Apply new motor
  // speed, and ask client to call back after pulseguide milliseconds.
  // Second call should fall through and resume tracking speed.
  if (pulseGuideDurationMillis > 0) {
    stepperWrapper->setStepperSpeed(targetSpeedInMilliHz);
    long delay = pulseGuideDurationMillis;
    pulseGuideDurationMillis = 0;
    return delay; // caller will call back right after delay.
  }

  // handle limit switch
  if (limitSwitchState) {
    int32_t limitPos = model.getLimitPosition();
    stepperWrapper->resetPosition(limitPos);
    safetyMode = false;
    if (targetPosition > limitPos ||
        targetPosition == model.getLimitSwitchSafetyStandoffPosition()) {
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
    // are we there yet? If not just return
    if (pos != targetPosition)
      return 0;
    // if we have arrived at the limit switch safety standoff position,
    // assume the move is a move towards the safety.
    // Set new position and much lower speed
    if (pos == model.getLimitSwitchSafetyStandoffPosition()) {
      targetPosition = INT32_MAX;
      targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz() / 5;
      stepperWrapper->moveTo(targetPosition, targetSpeedInMilliHz);
      return 0;
    }
    // we've arrived. Move gets stopped below.
    isExecutingMove = false;
    isMoveQueued = false;
    stopMove = true;
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
  targetPosition = model.getLimitSwitchSafetyStandoffPosition();
  int32_t limitPos = model.getLimitPosition();
  // when limit not known, find it slowly
  if (safetyMode)
    targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz() / 10;
  else
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
    log("Target guide rate arc seconds %lf ", targetSpeedInArcSecsSec);
    log("Model RA guide rate %lf ", model.getRAGuideRateArcSecondsSecond());
    // NOTE: this assumes target speed will be positive
    if (direction == 3) { // west: go faster {}
      targetSpeedInArcSecsSec += model.getRAGuideRateArcSecondsSecond();
      log("Adjusted W guide rate arc seconds %lf ", targetSpeedInArcSecsSec);
    }
    if (direction == 2) { // east: go slower
      targetSpeedInArcSecsSec -= model.getRAGuideRateArcSecondsSecond();
      log("Adjusted E guide rate arc seconds %lf ", targetSpeedInArcSecsSec);
    }
    targetSpeedInMilliHz = model.calculateFowardSpeedInMilliHz(
        stepperWrapper->getPosition(), targetSpeedInArcSecsSec);
        
    pulseGuideDurationMillis = pulseDurationInMilliseconds;
    log("Pulseguiding %s for %ld ms at speed %lu",
        direction == 3 ? "West" : "East", pulseDurationInMilliseconds,
        targetSpeedInMilliHz);

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
    log("Move axis target is 0 ");
    isExecutingMove = false; // loop should perform stop / resume track
    return;
  }
  // If we are currently tracking, add the tracking speed.
  // If we don't do this, if rate from client is 1x sidereal and they move
  // 1x sidereal forward, stars stay stationary.

  // positive is east, away from tracking direction (arbitary?)
  if (trackingOn) {
    degreesPerSecond -= model.getTrackingRateDegreesSec();
  }
  targetSpeedInMilliHz = model.calculateFowardSpeedInMilliHz(
      stepperWrapper->getPosition(), 3600.0 * fabs(degreesPerSecond));
  log("Move axis target speed millihz %lu", targetSpeedInMilliHz);

  isExecutingMove = true;
  isMoveQueued = true;
  // forward
  if (degreesPerSecond < 0) {
    targetPosition = 0;
  } else {
    targetPosition = INT32_MAX;
  }
}

/**
 * When passed a percentage (-100 to +100) turn this into a degrees
 * per second value and do moveaxis. Currently considers percentage
 * as a % of tracking rate.
*/
void PlatformControl::moveAxisPercentage(int percentage) {
  log("Received moveaxispercentage with value %d",percentage);
   if (percentage == 0) {
    moveAxis(0);
    return;
  }
  double degreesPerSecond=model.getNunChukMultiplier()* model.getTrackingRateDegreesSec() * (double)percentage/100.0;
  log("Moving axis with %lf degrees sec", degreesPerSecond);

  moveAxis(degreesPerSecond);
};


double PlatformControl::getTimeToCenterInSeconds() {
  return model.calculateTimeToCenterInSeconds(stepperWrapper->getPosition());
}

double PlatformControl::getTimeToEndOfRunInSeconds() {
  return model.calculateTimeToEndOfRunInSeconds(stepperWrapper->getPosition());
}