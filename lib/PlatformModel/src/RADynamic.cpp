#include "RADynamic.h"
#include "Logging.h"
#include <cmath>

void RADynamic::setTrackingOnOff(bool t) { trackingOn = t; }

bool RADynamic::isTrackingOn() { return trackingOn; }

void RADynamic::stopOrTrack(int32_t pos) {
  if (trackingOn) {
    if (pos > 0) {
      targetPosition = 0;
      targetSpeedInMilliHz = model.calculateTrackingSpeedInMilliHz(pos);
      stepperWrapper->moveTo(targetPosition, targetSpeedInMilliHz);
    } else {
      stepperWrapper->stop();
      trackingOn = false; // turn off at end of run.
    }
  } else {
    stepperWrapper->stop();
  }
}

RADynamic::RADynamic(RAStatic &m) : MotorDynamic(m), model(m) {
  trackingOn = false;
}

void RADynamic::pulseGuide(int direction, long pulseDurationInMilliseconds) {
  // if not tracking, do nothing
  if (trackingOn) {
    // Direction is either  2 = guideEast, 3 = guideWest.
    // If 2 then returned value will be higher than stepperCurrentPosition
    // If 3 then return value will be lower.
    double targetSpeedInArcSecsSec = model.getTrackingRateArcsSecondsSec();
    log("Target guide rate arc seconds %lf ", targetSpeedInArcSecsSec);
    log("Model RA guide rate %lf ", model.getGuideRateArcSecondsSecond());
    // NOTE: this assumes target speed will be positive
    if (direction == 3) { // west: go faster {}
      targetSpeedInArcSecsSec += model.getGuideRateArcSecondsSecond();
      log("Adjusted W guide rate arc seconds %lf ", targetSpeedInArcSecsSec);
    }
    if (direction == 2) { // east: go slower
      targetSpeedInArcSecsSec -= model.getGuideRateArcSecondsSecond();
      log("Adjusted E guide rate arc seconds %lf ", targetSpeedInArcSecsSec);
    }
    targetSpeedInMilliHz = model.calculateSpeedInMilliHz(
        stepperWrapper->getPosition(), targetSpeedInArcSecsSec);

    pulseGuideDurationMillis = pulseDurationInMilliseconds;
    speedBeforePulseMHz = stepperWrapper->getStepperSpeed();
    log("Pulseguiding %s for %ld ms at speed %lu",
        direction == 3 ? "West" : "East", pulseDurationInMilliseconds,
        targetSpeedInMilliHz);
  }
}

void RADynamic::moveAxis(double degreesPerSecond) {

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
  targetSpeedInMilliHz = model.calculateSpeedInMilliHz(
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
void RADynamic::moveAxisPercentage(int percentage) {
  log("Received moveaxispercentage with value %d", percentage);
  if (percentage == 0) {
    moveAxis(0);
    return;
  }
  double degreesPerSecond = model.getNunChukMultiplier() *
                            model.getTrackingRateDegreesSec() *
                            (double)percentage / 100.0;
  log("Moving axis with %lf degrees sec", degreesPerSecond);

  moveAxis(degreesPerSecond);
};

double RADynamic::getTimeToCenterInSeconds() {
  return (dynamic_cast<RAStatic &>(model))
      .calculateTimeToCenterInSeconds(stepperWrapper->getPosition());
}

double RADynamic::getTimeToEndOfRunInSeconds() {
  return model.calculateTimeToEndOfRunInSeconds(stepperWrapper->getPosition());
}
