#include "DecDynamic.h"
#include "Logging.h"
#include <cmath>
void DecDynamic::setLimitSwitchState(bool state) { limitSwitchState = state; }

void DecDynamic::setSafetyMode(bool s) { safetyMode = s; }

long DecDynamic::calculateOutput() {

  // If pulse guide is in progress, then exit.
  if (isPulseGuiding) {
    return 0;
  }

  // Pulseguide command has been queued. Apply new motor
  // speed, and ask client to call back after pulseguide milliseconds.
  // Second call should fall through and resume tracking speed.
  if (pulseGuideDurationMillis > 0) {
    stepperWrapper->setStepperSpeed(targetSpeedInMilliHz);
    long delay = pulseGuideDurationMillis;
    pulseGuideDurationMillis = 0;
    isPulseGuiding = true;
    return delay; // caller will call back right after delay.
  }

  // handle limit switch
  if (limitSwitchState) {
    int32_t limitPos = model.getLimitPosition();
    stepperWrapper->resetPosition(limitPos);
    safetyMode = false;
    // TODO add new limit swtich to model
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
    // TODO limit switch
    if (pos == model.getLimitSwitchSafetyStandoffPosition()) {
      targetPosition = INT32_MAX;
      // TODO dec speed
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
    stopMove = false;
  }
  stepperWrapper->stop();

  return 0;
}

void DecDynamic::gotoMiddle() {
  // TODO model middle position
  targetPosition = model.getMiddlePosition();
  targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz();
  isExecutingMove = true;
  isMoveQueued = true;
}

void DecDynamic::gotoStart() {
  // should run until limit switch hit
  // TODO model limits
  targetPosition = model.getLimitSwitchSafetyStandoffPosition();
  int32_t limitPos = model.getLimitPosition();
  // when limit not known, find it slowly
  if (safetyMode)
    targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz() / 3;
  else
    targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz();
  isExecutingMove = true;
  isMoveQueued = true;
}

int32_t DecDynamic::getTargetPosition() { return targetPosition; }

void DecDynamic::setStepperWrapper(StepperWrapper *wrapper) {
  stepperWrapper = wrapper;
}

DecDynamic::DecDynamic(DecStatic &m) : model(m) {
  isMoveQueued = false;
  isExecutingMove = false;

  stopMove = false;
  pulseGuideDurationMillis = 0;
  isPulseGuiding = false;
}

void DecDynamic::stopPulse() {
  stepperWrapper->setStepperSpeed(speedBeforePulseMHz);
  isPulseGuiding = false;
}
void DecDynamic::pulseGuide(int direction, long pulseDurationInMilliseconds) {
  // TODO rewrite.

  // Direction is either  0 = guideNorth, 1 = guideSouth.
  // As wedge angle increases, platform tilts north
  // Motor position 0 is at top?, so
  // If 0 then returned value will be higher than stepperCurrentPosition
  // If 1 then return value will be lower.

  double targetSpeedInArcSecsSec;
  
  log("Target guide rate arc seconds %lf ;", targetSpeedInArcSecsSec);
  
  // NOTE: this assumes target speed will be positive
  if (direction == 0) { // north: go up {}
    targetSpeedInArcSecsSec = model.getGuideRateArcSecondsSecond();
    log("N guide rate arc seconds %lf ", targetSpeedInArcSecsSec);
  }
  if (direction == 2) { // east: go slower
    targetSpeedInArcSecsSec = -model.getGuideRateArcSecondsSecond();
    log("S guide rate arc seconds %lf ", targetSpeedInArcSecsSec);
  }
  targetSpeedInMilliHz = model.calculateFowardSpeedInMilliHz(
      stepperWrapper->getPosition(), targetSpeedInArcSecsSec);

  pulseGuideDurationMillis = pulseDurationInMilliseconds;
  speedBeforePulseMHz = stepperWrapper->getStepperSpeed();
  log("Pulseguiding %s for %ld ms at speed %lu",
      direction == 3 ? "West" : "East", pulseDurationInMilliseconds,
      targetSpeedInMilliHz);

  // // divide by 1000 to get hz, then 1000 to get seconds.
  // // Ie if speed in millihz is 1000 (ie 1 hz, or one step per second)
  // // and we move for 1000 millis (is one second)
  // // then we move 1,000,000 / 1,000,000 = 1 step
  // int32_t stepsToMove =
  //     (stepperCurrentPosition * pulseDurationInMilliseconds) / 1000000;

  // if (direction == 2) // east.Positive step change ie towards limit
  // switch
  //   targetPosition += stepsToMove;

  // if (direction == 3) // west. negative step change
  //   targetPosition -= stepsToMove;

  // // make sure we don't run off end
  // targetPosition = (targetPosition < 0) ? 0 : targetPosition;

  // log("Pulse guiding %d for %ld ms to position %ld at speed (millihz)
  // %lf",
  //     direction, pulseDurationInMilliseconds, targetPosition);

  // isExecutingMove = true;
  // stepperWrapper->moveTo(targetPosition, targetSpeedInMilliHz);
}

void DecDynamic::stop() {
  isExecutingMove = false;
  stopMove = true;
}
/**
 * Slew by degrees.
 * Caclulates target position
 * */
void DecDynamic::slewByDegrees(double degreesToSlew) {
  log("Incoming slewByDegrees command, degrees to slew is  %lf", degreesToSlew);

  // TODO mdeol

  targetPosition = model.calculatePositionByDegreeShift(
      degreesToSlew, stepperWrapper->getPosition());
  isExecutingMove = true;
  isMoveQueued = true;
  targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz();
}

void DecDynamic::moveAxis(double degreesPerSecond) {

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
  // TODO
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
void DecDynamic::moveAxisPercentage(int percentage) {
  log("Received moveaxispercentage with value %d", percentage);
  if (percentage == 0) {
    moveAxis(0);
    return;
  }
  //TODO fix hardcoded degrees sec here
  double degreesPerSecond = model.getNunChukMultiplier() *
                            15 *
                            (double)percentage / 100.0;
  log("Moving axis with %lf degrees sec", degreesPerSecond);

  moveAxis(degreesPerSecond);
};

bool DecDynamic::isSlewing() { return isExecutingMove; }