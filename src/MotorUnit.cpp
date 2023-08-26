#include "MotorUnit.h"
#include "FastAccelStepper.h"
#include "Logging.h"
#include "PlatformModel.h"
#include <Arduino.h>

#define dirPinStepper 19
#define stepPinStepper 18

#define fastForwardSwitchPin 23
#define rewindSwitchPin 22
#define playSwitchPin 9999 // TODO

#define limitSwitchPin 21

// See
// https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md

long lastCheckTime = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void MotorUnit::setupMotor(PlatformModel m) {
  pinMode(fastForwardSwitchPin, INPUT_PULLUP);
  pinMode(rewindSwitchPin, INPUT_PULLUP);
  pinMode(playSwitchPin, INPUT_PULLUP);

  pinMode(limitSwitchPin, INPUT_PULLUP);

  model = m;

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setAutoEnable(true);

    // stepper->setSpeedInHz(5000);
    stepper->setAcceleration(1000000); // 100 steps/s²
  }
}

bool isFastForward() { return digitalRead(fastForwardSwitchPin) == LOW; }
bool isRewind() { return digitalRead(rewindSwitchPin) == LOW; }
bool isPlay() { return digitalRead(playSwitchPin) == LOW; }

bool isLimitSwitchHit() { return digitalRead(limitSwitchPin) == LOW; }

void MotorUnit::onLoop() {
  if (isLimitSwitchHit()) {
    stepper->setCurrentPosition(model.getLimitPosition());
  }

  if (isRewind() && !isLimitSwitchHit()) {
    stepper->setSpeedInHz(model.getRewindFastFowardSpeed());
    stepper->runForward();
    return;
  }

  if (isFastForward()) {
    stepper->setSpeedInHz(model.getRewindFastFowardSpeed());
    stepper->moveTo(0);
    return;
  }

  if (isPlay()) {
    // update speed periodically. Don't need to do every cycle
    long now = millis();
    if (now - lastCheckTime > 1000) {

      lastCheckTime = now;
      stepper->setSpeedInMilliHz(model.calculateFowardSpeedInMilliHz(
          ((double)(model.getMiddlePosition() -
                    stepper->getCurrentPosition())) /
          model.getStepsPerMM())); // TODO tidy this up to be readable (mobe to
                                   // model)
      stepper->moveTo(0);
    }
    return;
  }
  // no button hit, stop
  stepper->stopMove();
}

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
