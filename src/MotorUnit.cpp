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
#define microsteps 8
#define threadedRodPitch 2 // mm

#define limitSwitchToMiddleDistance 68 // mm
#define limitSwitchToEndDistance 135   // mm

// See
// https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md

double greatCircleRadiansPerMinute = M_PI * 2 / 24.0 / 60.0;
double rodStepperRatio =
    (double)teethOnRodPulley / (double)teethOnStepperPulley;

double greatCircleRadius =
    355.0; // this is millimeters from pivot to center touch

int calculateFowardSpeedInMilliHz(double distanceFromCenter) {

  log("greatCircleRadiansPerMinute %f", greatCircleRadiansPerMinute);

  // ignore distance for now
  double distanceToMoveAlongRodPerMinuteAtZero =
      tan(greatCircleRadiansPerMinute) * greatCircleRadius; // mm
  log("distanceToMoveAlongRodPerMinuteAtZero %f",
      distanceToMoveAlongRodPerMinuteAtZero);

  double numberOfTurnsPerMinuteOfRod =
      distanceToMoveAlongRodPerMinuteAtZero / threadedRodPitch;
  log("numberOfTurnsPerMinuteOfRod %f", numberOfTurnsPerMinuteOfRod);
  double numberOfTurnsPerMinuteOfStepper =
      numberOfTurnsPerMinuteOfRod * rodStepperRatio;
  log("numberOfTurnsPerMinuteOfStepper %f", numberOfTurnsPerMinuteOfStepper);

  double numberOfStepsPerMinute =
      numberOfTurnsPerMinuteOfStepper * stepperStepsPerRevolution * microsteps;
  log("numberOfStepsPerMinute %f", numberOfStepsPerMinute);

  double stepperSpeedInHertz = numberOfStepsPerMinute / 60.0;
  log("stepperSpeedInHertz %f", stepperSpeedInHertz);
  int stepperSpeedInMilliHertz = stepperSpeedInHertz * 1000;
  log("stepperSpeedInMilliHertz %d", stepperSpeedInMilliHertz);
  return stepperSpeedInMilliHertz;
}
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
int32_t middlePosition = limitPosition-(limitSwitchToMiddleDistance * stepsPerMM);

int calibrationSpeed = 15000; // this could be faster as platform unloaded
int runBackspeed = 5000;

// Number of steps per output rotation
const int stepsPerRevolution = 200;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setupMotor() {
  pinMode(forwardSwitchPin, INPUT_PULLUP);
  pinMode(backwardSwitchPin, INPUT_PULLUP);
  pinMode(limitSwitchPin, INPUT_PULLUP);

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setAutoEnable(true);

    stepper->setSpeedInHz(5000);
    stepper->setAcceleration(1000000); // 100 steps/s²
  }

  calculateFowardSpeedInMilliHz(0);
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
        runningBackward = true;  //switch is still up. This makes it stop. 
        stepper->stopMove();
        return;
      }
      // has switch been turned off and on?
      if (!runningBackward) {
        // even though we are moving forward, we count as back.
        log("Restarting calibration move");
        stepper->moveTo(middlePosition);
        runningForward = false;
        runningBackward = true;
        return;
      }
      return;
    }

    if (isLimitSwitchHit()) {
      log("Start Position Hit. Setting position to %d and moving to %d", limitPosition,middlePosition);
      stepper->stopMove();
      limitSwitchFound = true;
      runningForward = false;
      runningBackward = true;
      stepper->setCurrentPosition(limitPosition);
      stepper->moveTo(middlePosition);
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

void runModeSwitchCheck() {
  // In run mode, platform moves forward slowly, but back at
  // reset speed.
  if (isSwitchForward()) {
    if (!runningForward) {
      log("Run Mode Forward");
      stepper->setSpeedInMilliHz(calculateFowardSpeedInMilliHz(0));
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
void onLoop() {

  if (calibrationMode) {
    log("Current Position %d", stepper->getCurrentPosition());
    calibrationModeSwitchCheck();
  } else {
    log("Current Position %d", stepper->getCurrentPosition());
    runModeSwitchCheck();
  }
}
