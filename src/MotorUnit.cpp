#include "MotorUnit.h"
#include "FastAccelStepper.h"
#include "PlatformModel.h"
#include "Logging.h"
#include <Arduino.h>


#define dirPinStepper 19
#define stepPinStepper 18

#define forwardSwitchPin 23
#define backwardSwitchPin 22

#define limitSwitchPin 21



// See
// https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md


int calibrationSpeed = 30000; // this could be faster as platform unloaded
int runBackspeed = 30000;


int forwardSpeed = 0;
int backwardSpeed = 0;

bool runningForward = false;
bool runningBackward = false;

bool calibrationMode = true;
bool limitSwitchFound = false;

int moveToLocation = 0;
bool moveToMode = false;


long lastCheckTime = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void MotorUnit::setupMotor(PlatformModel m) {
  pinMode(forwardSwitchPin, INPUT_PULLUP);
  pinMode(backwardSwitchPin, INPUT_PULLUP);
  pinMode(limitSwitchPin, INPUT_PULLUP);

  model=m;

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setAutoEnable(true);

    // stepper->setSpeedInHz(5000);
    stepper->setAcceleration(1000000); // 100 steps/s²
  }

  // calculateFowardSpeedInMilliHz(-62.0);
  // calculateFowardSpeedInMilliHz(-0);
  // calculateFowardSpeedInMilliHz(60);
}

bool isSwitchForward() { return digitalRead(forwardSwitchPin) == LOW; }
bool isSwitchBackward() { return digitalRead(backwardSwitchPin) == LOW; }
bool isLimitSwitchHit() { return digitalRead(limitSwitchPin) == LOW; }

void MotorUnit::calibrationModeSwitchCheck() {

  // In calibration mode, platform runs forward and backwards at speed.
  // Once limit switch is hit, position is set to known value,
  // platform moves to middle to allow for top to go on,
  // and platform moves out of calibration mode (reboot required to reset)
  if (isSwitchForward()) { // running forward is for convenience only
    if (!runningForward) {
      log("Calibration Forward");
      stepper->setSpeedInHz(calibrationSpeed);
      // lol wiring is other way around and there is no way I am resoldering
      stepper->runBackward();
      runningForward = true;
      runningBackward = false;
      return;
    }
    return;
  }

  if (isSwitchBackward()) {
    // once limit switch found, just move to middle
    if (limitSwitchFound) {
      // Are we at the middle?
      if (stepper->getCurrentPosition() <= model.getMiddlePosition()) {
        log("Middle hit, stopping calibration");
        calibrationMode = false;
        runningForward = false;
        runningBackward = true; // switch is still up. This makes it stop.
        stepper->stopMove();
        return;
      }
      // has switch been turned off and on?
      if (!runningBackward) {
        // even though we are moving forward, we count as back.
        log("Restarting calibration move");
        stepper->moveTo(model.getMiddlePosition());
        // stepper->moveTo(0); // for testing end point
        runningForward = false;
        runningBackward = true;
        return;
      }
      return;
    }

    if (isLimitSwitchHit()) {
      log("Start Position Hit. Setting position to %d and moving to %d",
          model.getLimitPosition(), model.getMiddlePosition());
      stepper->stopMove();
      limitSwitchFound = true;
      runningForward = false;
      runningBackward = true;
      stepper->setCurrentPosition(model.getLimitPosition());
      // stepper->moveTo(middlePosition);
      // stepper->moveTo(0);
      return;
    }
    // run till limit hit
    if (!runningBackward) {
      log("Calibration Backward");
      stepper->setSpeedInHz(calibrationSpeed);
      stepper->runForward();
      runningForward = false;
      runningBackward = true;
      return;
    }
    return;
  }
  // switch is off, stop.
  if (runningForward || runningBackward) {
    log("Calibration : both switches off, stopping");
    runningForward = false;
    runningBackward = false;
    stepper->stopMove();
    return;
  }
}

/*
This function checks the status of two limit switches and controls the movement
of a stepper motor accordingly.

If the forward limit switch is active, the motor moves forward slowly, and the
speed is updated once per second. When moving forward, it checks if the motor is
not already running forward, which prevents duplicate log messages and speed
updates. The motor's target position is set to 0, and the runningForward flag is
set to true. If the backward limit switch is active and the motor is not already
running backward, the motor moves backward at the runBackspeed and goes all the
way back to the limitPosition. The runningBackward flag is set to true in this
case.

If neither switch is active, it checks whether the motor is currently running
either forward or backward. If so, it stops the motor using the stopMove()
function and sets both running flags to false.
*/

void MotorUnit::runModeSwitchCheck() {
  // In run mode, platform moves forward slowly, but back at
  // reset speed.
  if (isSwitchForward()) {
    // update speed periodically
    long now = millis();
    if (now - lastCheckTime > 1000) {

      lastCheckTime = now;
      // if (!runningForward) {
      // log("Run Mode Forward");
      stepper->setSpeedInMilliHz(model.calculateFowardSpeedInMilliHz(
          ((double)(model.getMiddlePosition() - stepper->getCurrentPosition())) /
          model.getStepsPerMM()));
      // stepper->setSpeedInHz(calibrationSpeed); //for finding end fast
      stepper->moveTo(0);

      runningForward = true;
      runningBackward = false;
      return;
    }
    return;
  }

  if (isSwitchBackward()) {
    if (!runningBackward) {
      log("Run Mode Backward");
      stepper->setSpeedInHz(runBackspeed);
      stepper->moveTo(model.getLimitPosition());

      runningForward = false;
      runningBackward = true;
      return;
    }
    return;
  }

  // switch is off. Check for moveto move (set via web ui), and stop otherwise.
  if (runningForward || runningBackward) {
    if (moveToMode) {
      if (stepper->getCurrentPosition() != moveToLocation) {
        stepper->setSpeedInHz(runBackspeed);
        stepper->moveTo(moveToLocation);
        return;
      } else {
        moveToMode = false;
        stepper->stopMove();
        return;
      }
    }
    runningForward = false;
    runningBackward = false;
    stepper->stopMove();
    return;
  }
}
void MotorUnit::onLoop() {

  if (calibrationMode) {
    // log("Current Position %d", stepper->getCurrentPosition());
    calibrationModeSwitchCheck();
  } else {
    // log("Current Position %d", stepper->getCurrentPosition());
    runModeSwitchCheck();
  }
}



int MotorUnit::getCalibrationSpeed() { return calibrationSpeed; }
void MotorUnit::setCalibrationSpeed(int speed) { calibrationSpeed = speed; }

int MotorUnit::getRunBackSpeed() { return runBackspeed; }
void MotorUnit::setrunbackSpeed(int speed) { runBackspeed = speed; }



double MotorUnit::getVelocityInMMPerMinute() {
  double speedInHz = stepper->getCurrentSpeedInMilliHz() / 1000.0;
  double speedInMMPerSecond = speedInHz / model.getStepsPerMM();
  double speedInMMPerMinute = speedInMMPerSecond * 60.0;
  return speedInMMPerMinute;
}
double MotorUnit::getPositionInMM() {
  return ((double)stepper->getCurrentPosition()) /
         (double)model.getStepsPerMM();
}

// set via web ui:
void MotorUnit::moveTo(int location) {
  moveToLocation = location * model.getStepsPerMM();
  moveToMode = true;
  ;
}
