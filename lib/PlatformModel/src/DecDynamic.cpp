#include "DecDynamic.h"
#include "Logging.h"
#include <cmath>

DecDynamic::DecDynamic(DecStatic &m) : MotorDynamic(m), model(m) {
  isMoveQueued = false;
  isExecutingMove = false;

  stopMove = false;
  pulseGuideDurationMillis = 0;
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
  // TODO fix hardcoded degrees sec here
  double degreesPerSecond =
      model.getNunChukMultiplier() * 15 * (double)percentage / 100.0;
  log("Moving axis with %lf degrees sec", degreesPerSecond);

  moveAxis(degreesPerSecond);
};
