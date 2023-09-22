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

double MotorUnit::getTimeToCenterInSeconds() {
  return model.calculateTimeToCenterInSeconds(stepper->getCurrentPosition());
}

double MotorUnit::getTimeToEndOfRunInSeconds() {
  return model.calculateTimeToEndOfRunInSeconds(stepper->getCurrentPosition());
}

bool MotorUnit::getTrackingStatus() {
  // todo fix this so only
  if (!stepper->isRunning())
    return false;
  // if running we'll be running at forward/rewind spped. This is in hz, so
  // convert to compare add fudge factor.
  return (stepper->getSpeedInMilliHz() <=
          model.getRewindFastFowardSpeed() * 500);
}

MotorUnit::MotorUnit(PlatformModel &m, Preferences &p)
    : model(m), preferences(p) {
  slewing = false;
  tracking = false;
}

void MotorUnit::setupMotor() {
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
  }
}

bool isFastForward() { return bounceFastForward.read() == LOW; }
bool isRewind() { return bounceRewind.read() == LOW; }
bool isPlay() { return bouncePlay.read() == LOW; }

bool isLimitSwitchHit() { return bounceLimit.read() == LOW; }

void MotorUnit::moveAxis(double degreesPerSecond) {
  // TODO ignores slew speed. does it matter>
  // TODO slew is not really the same as move axis
  if (degreesPerSecond == 0) {
    slewing = false; // loop should perform stop or track
    return;
  }
  if (degreesPerSecond > 0) // forward?
  {
    slewToEnd();
    return;
  }
  slewToStart();
}
void MotorUnit::onLoop() {
  bounceFastForward.update();
  bounceRewind.update();
  bouncePlay.update();
  bounceLimit.update();

  if (isLimitSwitchHit()) {
    int32_t pos = model.getLimitPosition();
    stepper->setCurrentPosition(pos);
    
    if (slewingToStart) {
      log("Limit hit, setting position to %ld", pos);
      slewing = false;
      slewingToStart = false;
    }
  }

  // if slewing, stop when position reached.
  if (slewing) {
    if (stepper->getCurrentPosition() == slew_target_pos) {
      slewing = false;
      stepper->stopMove();
    } else {
      return; // ignore buttons
    }
  }

  // cheat code to park scope
  if (isFastForward() && (isRewind())) {
    slewToMiddle();
    return;
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

  // if clients tell us to track, we can't stop locally.
  // so whenever the tracking flag is set externally,
  // we clear it when play is pressed.
  // So if we want to run from external, start with play not pushed.
  // Then if we want to stop but cannot stop externally,
  // press play then stop.
  if (isPlay() && tracking) {
    tracking = false;
  }

  if (isPlay() || tracking) {

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

void MotorUnit::slewToStart() {
  log("Slewing to start");
  if (!isLimitSwitchHit()) {
    slew_target_pos = model.getLimitPosition();

    stepper->setSpeedInHz(model.getRewindFastFowardSpeed());
    stepper->runForward();
    slewing = true;
    slewingToStart = true;
    return;
  }
  slewing = false;
}

void MotorUnit::slewToMiddle() { slewToPosition(model.getMiddlePosition()); }
void MotorUnit::slewToEnd() {
  // TODO make constant
  slewToPosition(model.getStepsPerMM() * 10);
  // 10 mm from end
  // 2 mm per minute=five minutes to end?
}
bool MotorUnit::isSlewing() { return slewing; }
void MotorUnit::slewToPosition(int32_t position) {
  
    slew_target_pos = position;
    slewing = true;
    stepper->setSpeedInHz(model.getRewindFastFowardSpeed());
    stepper->moveTo(position);
    return;
}
// void MotorUnit::slewToPosition(long position);

void MotorUnit::setTracking(bool b) { tracking = b; }