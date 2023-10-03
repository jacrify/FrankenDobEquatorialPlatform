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

bool positionSavedOnStop = false;

double MotorUnit::getTimeToCenterInSeconds() {
  return model.calculateTimeToCenterInSeconds(stepper->getCurrentPosition());
}

double MotorUnit::getTimeToEndOfRunInSeconds() {
  return model.calculateTimeToEndOfRunInSeconds(stepper->getCurrentPosition());
}
/**
 * Accumulates ff/rewinds moves as an offset
 */
double MotorUnit::getPlatformResetOffsetSeconds() {
  return control.getPlatformResetOffset();
}

bool MotorUnit::getTrackingStatus() {

  if (!stepper->isRunning())
    return false;
  // if running we'll be running at forward/rewind spped. This is in hz, so
  // convert to compare add fudge factor.
  return (abs(stepper->getCurrentSpeedInMilliHz()) <=
          model.getRewindFastFowardSpeed() * 500);
}

MotorUnit::MotorUnit(PlatformModel &m, PlatformControl &c,Preferences &p)
    : model(m),control(c), preferences(p) {
 
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
  }
  ConcreteStepperWrapper *wrapper = new ConcreteStepperWrapper();
  wrapper->setStepper(stepper);
  control.setStepperWrapper(wrapper);

}

bool isFastForward() { return bounceFastForward.read() == LOW; }
bool isRewind() { return bounceRewind.read() == LOW; }
bool isPlay() { return bouncePlay.read() == LOW; }

bool isLimitSwitchHit() { return bounceLimit.read() == LOW; }

double degreesPerSecondToArcSecondsPerSecond(double degreesPerSecond) {
  return degreesPerSecond * 3600.0;
}
void MotorUnit::moveAxis(double degreesPerSecond) {
  log("Incoming movexis command speed %lf", degreesPerSecond);
  control.moveAxis(degreesPerSecond);
}

void MotorUnit::onLoop() {
  bounceFastForward.update();
  bounceRewind.update();
  bouncePlay.update();
  bounceLimit.update();
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

void MotorUnit::slewToStart() {
  log("Slewing to start");
  control.gotoStart();
}

void MotorUnit::slewToMiddle() { control.gotoMiddle(); }

void MotorUnit::slewToEnd() {
  // TODO make constant
  control.gotoEndish();
  // slewToPosition(model.getStepsPerMM() * 10);
  // 10 mm from end
  // 2 mm per minute=five minutes to end?
}
// bool MotorUnit::isSlewing() { return slewing; }

// void MotorUnit::slewToPosition(int32_t position) {
//   slew_target_pos = position;
//   slewing = true;
//   stepper->setSpeedInHz(model.getRewindFastFowardSpeed());
//   stepper->moveTo(position);
//   return;
// }
// void MotorUnit::slewToPosition(long position);

void MotorUnit::setTracking(bool b) { control.setTrackingOnOff(b); }

// TODO this is wrong: it should simply set up some fraction of sidereal
// speed that is subtracted from tracking for some number of millis.
void MotorUnit::pulseGuide(int direction, long pulseDurationInMilliseconds) {
  unsigned long now = millis();
  control.pulseGuide(direction, pulseDurationInMilliseconds, now);
}