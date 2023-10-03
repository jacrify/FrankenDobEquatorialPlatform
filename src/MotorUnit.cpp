#include "MotorUnit.h"
#include "ConcreteStepperWrapper.h"
#include "FastAccelStepper.h"
#include "Logging.h"
#include "PlatformControl.h"
#include "PlatformModel.h"
#include <Arduino.h>
#include <Bounce2.h>
#include <Preferences.h>

#define dirPinStepper 19
#define stepPinStepper 18

#define fastForwardSwitchPin 22
#define rewindSwitchPin 23
#define playSwitchPin 27

#define limitSwitchPin 21

#define PREF_SAVED_POS_KEY "SavedPosition"

// See
// https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md

long lastCheckTime = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
Preferences preferences;
Bounce bounceFastForward = Bounce();
Bounce bounceRewind = Bounce();
Bounce bouncePlay = Bounce();
Bounce bounceLimit = Bounce();



MotorUnit::MotorUnit(PlatformModel &m, PlatformControl &c, Preferences &p)
    : model(m), control(c), preferences(p) {

  // control = PlatformControl(ConcreteStepperWrapper(stepper), model);
}

void MotorUnit::setupMotor() {
  bounceFastForward.attach(fastForwardSwitchPin, INPUT_PULLUP);
  bounceRewind.attach(rewindSwitchPin, INPUT_PULLUP);
  bouncePlay.attach(playSwitchPin, INPUT_PULLUP);

  bounceLimit.attach(limitSwitchPin, INPUT_PULLUP);

  // DEBOUNCE INTERVAL IN MILLISECONDS
  bounceFastForward.interval(100); // interval in ms
  bounceRewind.interval(100);      // interval in ms
  bouncePlay.interval(100);        // interval in ms

  bounceLimit.interval(10); // interval in ms

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setAutoEnable(true);

    // stepper->setSpeedInHz(5000);
    stepper->setAcceleration(1000000); // 100 steps/s²

    // preferences = p;
    uint32_t savedPosition = preferences.getUInt(PREF_SAVED_POS_KEY, 0);
    log("Loaded saved position %d", savedPosition);
    stepper->setCurrentPosition(savedPosition);
    stepper->moveTo(savedPosition+10000,model.getRewindFastFowardSpeed());
  }
  ConcreteStepperWrapper *wrapper = new ConcreteStepperWrapper();
  wrapper->setStepper(stepper);
  control.setStepperWrapper(wrapper);
}

bool isFastForwardJustPushed() {
  return bounceFastForward.changed() && bounceFastForward.read() == LOW;
}

// this returns false if rewind has been pushed, as that takes precedence
bool isFastForwardJustReleased() {
  return bounceFastForward.changed() && bounceFastForward.read() != LOW &&
         !isFastForwardJustPushed();
}

bool isRewindJustPushed() {
  return bounceRewind.changed() && bounceRewind.read() == LOW;
}

//this returns false if fast forward has been pushed, as that takes precedence
bool isRewindJustReleased() {
  return bounceRewind.changed() && bounceRewind.read() != LOW &&
         !isFastForwardJustPushed();
}
bool isPlayJustPushed() {
  return bouncePlay.changed() && bouncePlay.read() == LOW;
}

bool isPlayJustReleased() {
  return bouncePlay.changed() && bouncePlay.read() != LOW;
}

bool isLimitSwitchHit() { return bounceLimit.read() == LOW; }

// double degreesPerSecondToArcSecondsPerSecond(double degreesPerSecond) {
//   return degreesPerSecond * 3600.0;
// }

void MotorUnit::onLoop() {
  bounceFastForward.update();
  bounceRewind.update();
  bouncePlay.update();
  bounceLimit.update();

  control.setLimitSwitchState(isLimitSwitchHit());

  if (isFastForwardJustPushed()) {
    control.gotoEndish();
  }
  if (isFastForwardJustReleased()) {
    control.stop();
  }
  if (isRewindJustPushed()) {
    control.gotoStart();
  }
  if (isRewindJustReleased()) {
    control.stop();
  }
  if (isPlayJustPushed()) {
    control.setTrackingOnOff(true);
  }
  if (isPlayJustReleased()) {
    control.setTrackingOnOff(false);
  }

  long delay = control.calculateOutput(millis());
  // handle pulseguide delay
  if (delay > 0)
    control.calculateOutput(millis());
}

double MotorUnit::getVelocityInMMPerMinute() {
  double speedInMHz = (double)stepper->getCurrentSpeedInMilliHz();
  double speedInHz = speedInMHz / 1000.0;
  double speedInMMPerSecond = speedInHz / model.getStepsPerMM();
  double speedInMMPerMinute = speedInMMPerSecond * 60.0;
  return speedInMMPerMinute;
}
double MotorUnit::getPositionInMM() {
  return ((double)stepper->getCurrentPosition()) / model.getStepsPerMM();
}


