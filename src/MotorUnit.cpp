#include "MotorUnit.h"
#include "FastAccelStepper.h"
#include "Logging.h"
#include <Arduino.h>
#include <Math.h>

#define dirPinStepper 19
#define stepPinStepper 18

#define forwardSwitchPin 23
#define backwardSwitchPin 22

#define limitSwitchPin 21

#define teethOnStepperPulley 16
#define teethOnRodPulley 36
#define stepperStepsPerRevolution 200
#define microsteps 16
#define threadedRodPitch 2 // mm

#define limitSwitchToMiddleDistance 62 // mm
#define limitSwitchToEndDistance 135   // mm

// See
// https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md

double greatCircleRadiansPerMinute = M_PI * 2 / 24.0 / 60.0;
int calibrationSpeed = 30000; // this could be faster as platform unloaded
int runBackspeed = 10000;

double rodStepperRatio =
    (double)teethOnRodPulley / (double)teethOnStepperPulley;

double greatCircleRadius =
    482.5; // this is millimeters from pivot to center rod

int forwardSpeed = 0;
int backwardSpeed = 0;

bool runningForward = false;
bool runningBackward = false;

bool calibrationMode = true;
bool limitSwitchFound = false;

long stepsPerMM = (stepperStepsPerRevolution * microsteps * teethOnRodPulley) /
                  (teethOnStepperPulley * threadedRodPitch);

// Limit position is highest. Then it counts down to zero at end.
// this  is expressed in microsteps
int32_t limitPosition = limitSwitchToEndDistance * stepsPerMM;
int32_t middlePosition =
    limitPosition - (limitSwitchToMiddleDistance * stepsPerMM);

// Number of steps per output rotation
const int stepsPerRevolution = 200;

long lastCheckTime = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

int calculateFowardSpeedInMilliHz(double distanceFromCenterInMM) {

  // log("distanceFromCenter %f", distanceFromCenterInMM);
  // log("greatCircleRadiansPerMinute %f", greatCircleRadiansPerMinute);

  double absoluteAngleMovedAtThisPoint =
      atan(distanceFromCenterInMM / greatCircleRadius);
  // log("Current angle (deg) %f", absoluteAngleMovedAtThisPoint * 180.0 / PI);

  // TODO handle -
  double absoluteAngleAfterOneMoreMinute =
      absoluteAngleMovedAtThisPoint + greatCircleRadiansPerMinute;

  // log("New angle (deg) %f", absoluteAngleAfterOneMoreMinute * 180.0 / PI);

  double distanceAlongRodAfterOneMoreMinute =
      greatCircleRadius * tan(absoluteAngleAfterOneMoreMinute);

  // log("distanceAlongRodAfterOneMoreMinute %f",
  // distanceAlongRodAfterOneMoreMinute);

  double threadDistancePerMinute =
      distanceAlongRodAfterOneMoreMinute - distanceFromCenterInMM;

  // log("threadDistancePerMinute %f", threadDistancePerMinute);
  // // ignore distance for now
  // double distanceToMoveAlongRodPerMinuteAtZero =
  //     tan(greatCircleRadiansPerMinute) * greatCircleRadius; // mm

  // log("distanceToMoveAlongRodPerMinuteAtZero %f",
  //     distanceToMoveAlongRodPerMinuteAtZero);

  double numberOfTurnsPerMinuteOfRod =
      threadDistancePerMinute / threadedRodPitch;

  // log("numberOfTurnsPerMinuteOfRod %f", numberOfTurnsPerMinuteOfRod);
  double numberOfTurnsPerMinuteOfStepper =
      numberOfTurnsPerMinuteOfRod * rodStepperRatio;
  // log("numberOfTurnsPerMinuteOfStepper %f", numberOfTurnsPerMinuteOfStepper);

  double numberOfStepsPerMinuteInMiddle =
      numberOfTurnsPerMinuteOfStepper * stepperStepsPerRevolution * microsteps;

  // log("numberOfStepsPerMinute %f", numberOfStepsPerMinuteInMiddle);

  double stepperSpeedInHertz = numberOfStepsPerMinuteInMiddle / 60.0;
  // log("stepperSpeedInHertz %f", stepperSpeedInHertz);
  int stepperSpeedInMilliHertz = stepperSpeedInHertz * 1000;
  // log("stepperSpeedInMilliHertz %d", stepperSpeedInMilliHertz);
  return stepperSpeedInMilliHertz;
}

void MotorUnit::setupMotor() {
  pinMode(forwardSwitchPin, INPUT_PULLUP);
  pinMode(backwardSwitchPin, INPUT_PULLUP);
  pinMode(limitSwitchPin, INPUT_PULLUP);

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

void calibrationModeSwitchCheck() {

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
      if (stepper->getCurrentPosition() <= middlePosition) {
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
        stepper->moveTo(middlePosition);
        // stepper->moveTo(0); // for testing end point
        runningForward = false;
        runningBackward = true;
        return;
      }
      return;
    }

    if (isLimitSwitchHit()) {
      log("Start Position Hit. Setting position to %d and moving to %d",
          limitPosition, middlePosition);
      stepper->stopMove();
      limitSwitchFound = true;
      runningForward = false;
      runningBackward = true;
      stepper->setCurrentPosition(limitPosition);
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

void runModeSwitchCheck() {
  // In run mode, platform moves forward slowly, but back at
  // reset speed.
  if (isSwitchForward()) {
    // update speed periodically
    long now = millis();
    if (now - lastCheckTime > 1000) {

      lastCheckTime = now;
      // if (!runningForward) {
      // log("Run Mode Forward");
      stepper->setSpeedInMilliHz(calculateFowardSpeedInMilliHz(
          ((double)(middlePosition - stepper->getCurrentPosition())) /
          stepsPerMM));
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
      stepper->moveTo(limitPosition);

      runningForward = false;
      runningBackward = true;
      return;
    }
    return;
  }

  // switch is off, stop.
  if (runningForward || runningBackward) {
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

double MotorUnit::getGreatCircleRadius() { return greatCircleRadius; }
void MotorUnit::setGreatCircleRadius(double radius) {
  greatCircleRadius = radius;
}

int MotorUnit::getCalibrationSpeed() { return calibrationSpeed; }
void MotorUnit::setCalibrationSpeed(int speed) { calibrationSpeed = speed; }

int MotorUnit::getRunBackSpeed() { return runBackspeed; }
void MotorUnit::setrunbackSpeed(int speed) { runBackspeed = speed; }

int MotorUnit::getLimitSwitchToEndDistance() {
  return limitSwitchToEndDistance;
}

int MotorUnit::getVelocity() { return stepper->getCurrentSpeedInMilliHz(); }

int MotorUnit::getPosition() { return stepper->getCurrentPosition(); }

void MotorUnit::moveTo(int location) { stepper->moveTo(location); }
