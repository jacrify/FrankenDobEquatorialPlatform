#include "MotorUnit.h"
#include "FastAccelStepper.h"
#include "Logging.h"
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

bool positionSavedOnStop = false;

void MotorUnit::setupMotor(PlatformModel &m, Preferences &p) {
  bounceFastForward.attach(fastForwardSwitchPin, INPUT_PULLUP);
  // pinMode(fastForwardSwitchPin, INPUT_PULLUP);
  bounceRewind.attach(rewindSwitchPin, INPUT_PULLUP);
  bouncePlay.attach(playSwitchPin, INPUT_PULLUP);

  bounceLimit.attach(limitSwitchPin, INPUT_PULLUP);

  // DEBOUNCE INTERVAL IN MILLISECONDS
  bounceFastForward.interval(100); // interval in ms
  bounceRewind.interval(100);      // interval in ms
  bouncePlay.interval(100);        // interval in ms

  bounceLimit.interval(10); // interval in ms

  model = m;

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setAutoEnable(true);

    // stepper->setSpeedInHz(5000);
    stepper->setAcceleration(1000000); // 100 steps/s²

    preferences = p;
    uint32_t savedPosition = preferences.getUInt(PREF_SAVED_POS_KEY, 0);
    log("Loaded saved position %d", savedPosition);
    stepper->setCurrentPosition(savedPosition);
  }
}

bool isFastForward() { return bounceFastForward.read() == LOW; }
bool isRewind() { return bounceRewind.read() == LOW; }
bool isPlay() { return bouncePlay.read() == LOW; }

bool isLimitSwitchHit() { return bounceLimit.read() == LOW; }

void MotorUnit::onLoop() {
  bounceFastForward.update();
  bounceRewind.update();
  bouncePlay.update();
  bounceLimit.update();

  if (isLimitSwitchHit()) {
    int32_t pos = model.getLimitPosition();
    stepper->setCurrentPosition(pos);
    log("Limit hit, setting position to %ld", pos);
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
    if ((now - lastCheckTime) > 1000) {

      lastCheckTime = now;
      int32_t currentPosition = stepper->getCurrentPosition();
      log("Setting speed of stepper to %lu millnz at position %ld",
          model.calculateFowardSpeedInMilliHz(currentPosition),
          currentPosition);
      stepper->setSpeedInMilliHz(
          model.calculateFowardSpeedInMilliHz(currentPosition));
      stepper->moveTo(0);
    }
    return;
  }
  // no button hit, stop
  if (stepper->isRunning()) {
    stepper->stopMove();
    uint32_t pos = stepper->getCurrentPosition();
    preferences.putUInt(PREF_SAVED_POS_KEY, pos);
    log("Stopped, saving position %ld", pos);
  }
}

double MotorUnit::getVelocityInMMPerMinute() {
  double speedInMHz = (double)stepper->getCurrentSpeedInMilliHz();
  double speedInHz = speedInMHz / 1000.0;
  double speedInMMPerSecond = speedInHz / model.getStepsPerMM();
  double speedInMMPerMinute = speedInMMPerSecond * 60.0;
  // log("Speed in mhz %f", speedInMHz);
  // log("Speed in hz %f", speedInHz);
  // log("Speed in mm s %f", speedInMMPerSecond);
  log("Speed in mm m %f", speedInMMPerMinute);
  return speedInMMPerMinute;
}
double MotorUnit::getPositionInMM() {
  return ((double)stepper->getCurrentPosition()) / model.getStepsPerMM();
}
