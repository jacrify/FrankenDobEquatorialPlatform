#include "MotorDynamic.h"
#include "Logging.h"
#include <cmath>
void MotorDynamic::setLimitSwitchState(bool state) { limitSwitchState = state; }

void MotorDynamic::setSafetyMode(bool s) { safetyMode = s; }
bool MotorDynamic::isSafetyModeOn() { return safetyMode; }

long MotorDynamic::onLoop() {

  // If pulse guide is in progress, then exit.
  if (isPulseGuiding) {
    return 0;
  }

  // Pulseguide command has been queued. Apply new motor
  // speed, and ask client to call back after pulseguide milliseconds.
  // Second call should fall through and resume tracking speed.
  if (pulseGuideDurationMillis > 0) {
    stepperWrapper->setStepperSpeed(targetSpeedInMilliHz);
    stepperWrapper->moveTo(targetPosition, targetSpeedInMilliHz);
    long delay = pulseGuideDurationMillis;
    pulseGuideDurationMillis = 0;
    isPulseGuiding = true;
    log("Returning after pulse");
    return delay; // caller will call back right after delay.
  }

  // handle limit switch
  if (limitSwitchState) {
    log("Limit is hit");
    int32_t limitPos = model.getLimitPosition();
    stepperWrapper->resetPosition(limitPos);
    safetyMode = false;
    // If we were moving towards limit switch, we should stop.
    // If we are not moving or moving away from limit, no further action.
    // SO to tell if we are moving towards limit: one of two things is true:
    // the target was max_int, ie > than limit position, or
    // the target was the standoff position and we've hit the limit early
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
    log("In loop, and move is queued. Pos is %ld and target is %ld", pos,
        targetPosition);
    if (pos != targetPosition) {
      log("Pushing queued move to motor");
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
    if (pos == model.getLimitSwitchSafetyStandoffPosition()) {
      log("Standoff position (%ld) reached. Slowing down to find limit", pos);
      targetPosition = INT32_MAX;
      targetSpeedInMilliHz =
          model.getRewindFastFowardSpeedInMilliHz() / SAFETY_RATIO;
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
  // log("Stop or track fallthrough");
  // either stop, or resume tracking (delegeated to subclass)
  stopOrTrack(pos);

  return 0;
}

void MotorDynamic::gotoMiddle() {

  targetPosition = model.getMiddlePosition();
  log("goto middle: target %ld ", targetPosition);
  targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz();
  isExecutingMove = true;
  isMoveQueued = true;
}

void MotorDynamic::gotoEndish() {

  targetPosition = model.getGotoEndPosition();
  log("goto end: target %ld ", targetPosition);
  targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz();
  isExecutingMove = true;
  isMoveQueued = true;
}

void MotorDynamic::gotoStart() {
  // should run until limit switch hit
  targetPosition = model.getLimitSwitchSafetyStandoffPosition();
  log("goto start: target %ld ", targetPosition);
  // int32_t limitPos = model.getLimitPosition();
  // when limit not known, find it slowly
  if (safetyMode)
    targetSpeedInMilliHz =
        model.getRewindFastFowardSpeedInMilliHz() / SAFETY_RATIO;
  else
    targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz();
  isExecutingMove = true;
  isMoveQueued = true;
}

int32_t MotorDynamic::getTargetPosition() { return targetPosition; }

uint32_t MotorDynamic::getTargetSpeedInMilliHz() {
  return targetSpeedInMilliHz;
}

void MotorDynamic::setStepperWrapper(StepperWrapper *wrapper) {
  stepperWrapper = wrapper;
}

MotorDynamic::MotorDynamic(MotorStatic &m) : model(m) {
  isMoveQueued = false;
  isExecutingMove = false;
  stopMove = false;
  pulseGuideDurationMillis = 0;
  isPulseGuiding = false;
}

void MotorDynamic::stopPulse() {
  log("Setting speed to %ld", speedBeforePulseMHz);
  stepperWrapper->setStepperSpeed(speedBeforePulseMHz);
  isPulseGuiding = false;
}

void MotorDynamic::stop() {
  isExecutingMove = false;
  stopMove = true;
}
/**
 * Slew by degrees.
 * Caclulates target position
 * */
void MotorDynamic::slewByDegrees(double degreesToSlew) {
  log("Incoming slewByDegrees command, degrees to slew is  %lf", degreesToSlew);

  targetPosition = model.calculatePositionByDegreeShift(
      degreesToSlew, stepperWrapper->getPosition());
  isExecutingMove = true;
  isMoveQueued = true;
  targetSpeedInMilliHz = model.getRewindFastFowardSpeedInMilliHz();
}

bool MotorDynamic::isSlewing() { return isExecutingMove; }