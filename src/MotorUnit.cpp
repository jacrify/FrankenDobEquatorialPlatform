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
/**
 * Accumulates ff/rewinds moves as an offset
 */
double MotorUnit::getPlatformResetOffsetSeconds() {
  return platformResetOffsetSeconds;
}

bool MotorUnit::getTrackingStatus() {

  if (!stepper->isRunning())
    return false;
  // if running we'll be running at forward/rewind spped. This is in hz, so
  // convert to compare add fudge factor.
  return (abs(stepper->getCurrentSpeedInMilliHz()) <=
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

  platformResetOffsetSeconds = 0;
  firstMoveCycleForCalc = true;

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

double degreesPerSecondToArcSecondsPerSecond(double degreesPerSecond) {
  return degreesPerSecond * 3600.0;
}
void MotorUnit::moveAxis(double degreesPerSecond) {
  log("Incoming movexis command speed %lf", degreesPerSecond);
  // TODO slew is not really the same as move axis conceptually
  if (degreesPerSecond == 0) {
    slewing = false; // loop should perform stop / resume track
    stepper->stopMove();
    lastCheckTime =0; // forcce check on next loop
    return;
  }
  // If we are currently tracking, add the tracking speed.
  // If we don't do this, if rate from client is 1x sidereal and they move
  // 1x sidereal forward, stars stay stationary.

  if (tracking) {
    degreesPerSecond -= model.getTrackingRateDegreesSec();
  }
  uint32_t speedMilliHz = model.calculateFowardSpeedInMilliHz(
      stepper->getCurrentPosition(),
      degreesPerSecondToArcSecondsPerSecond(abs(degreesPerSecond)));

  stepper->setSpeedInMilliHz(speedMilliHz);

  // forward
  if (degreesPerSecond > 0) {
    slew_target_pos = 0;
    slewing = true;
    stepper->moveTo(0);
    return;
  }
  // backward
  if (!isLimitSwitchHit()) {
    slew_target_pos = model.getLimitPosition();
    stepper->runForward();
    slewing = true;
    slewingToStart = true;
    return;
  }
}

void MotorUnit::onLoop() {
  bounceFastForward.update();
  bounceRewind.update();
  bouncePlay.update();
  bounceLimit.update();

  // TODO #4 bug here. When we turn on tracking remotely, then move back, it
  // takes 10 second to restart tracking.


//TODO #5 Bug slewing to start does not get reset in some scenarios 

  /**
   * Each loop, check for current fast move (rewind/ff)
   * by looking at motor.
   * If we are in ff/rewind:
   * If this is the first loops where we've detected that, just stored the
   * current time to center
   * Otherwise, take the difference between where we were last cycle
   * and this cycle, and accumation in platformResetOffsetSeconds.
   * This is used to offset the model on dsc by howver far we've rewound/ffed.
   *
   * Eg say we are rewinding.
   * Cycle 1 currentTimeToCenter=30s
   * Cycle 2 currentTimeToCenter=25s
   * Delta is 5s
   * platformResetOffsetSeconds -=5s
   * This is added to model calc time, so stars ra are 5s less.
   *
   */
  if (stepper->isRunning()) {
    int speedInMilliHz = abs(stepper->getCurrentSpeedInMilliHz());
    // check for tracking, only offset when not tracking but moving
    //TODO #6 fix platformOffset when moveaxis at slow speed
    if (speedInMilliHz > (model.getRewindFastFowardSpeed() * 1000 / 4)) {
      double currentTimeToCenter =
          model.calculateTimeToCenterInSeconds(stepper->getCurrentPosition());
      if (firstMoveCycleForCalc) {
        firstMoveCycleForCalc = false;
      } else {
        platformResetOffsetSeconds -=
            lastTimeToCenterSeconds - currentTimeToCenter;
      }
      lastTimeToCenterSeconds = currentTimeToCenter;
    }
  } else {
    firstMoveCycleForCalc = true;
  }

  if (isLimitSwitchHit()) {
    //TODO #7 fix bug where setCurrentPosition breaks later slew_target_pos checks
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
      uint32_t targetSpeed =
          model.calculateFowardSpeedInMilliHz(currentPosition);
         int32_t currentSpeed= stepper->getCurrentSpeedInMilliHz();
      log("Setting speed of stepper to %lu millhz at position %ld", targetSpeed,
          currentPosition);
          log("Current speed %ld",currentSpeed);
      stepper->setSpeedInMilliHz(targetSpeed);
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
  // log("Speed in mm m %f", speedInMMPerMinute);
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

//TODO this is wrong: it should simply set up some fraction of sidereal
//speed that is subtracted from tracking for some number of millis.
void MotorUnit::pulseGuide(int direction, long pulseDurationInMilliseconds) {
  // Direction is either  2 = guideEast, 3 = guideWest.
  // If 2 then returned value will be higher than stepperCurrentPosition
  // If 3 then return value will be lower.

  int32_t stepperCurrentPosition = stepper->getCurrentPosition();
  int32_t targetPosition = stepperCurrentPosition;

  // speed is steps per second * 1000
  uint32_t speedInMilliHz = model.calculateFowardSpeedInMilliHz(
      stepperCurrentPosition, model.getRAGuideRateArcSecondsSecond());

  // divide by 1000 to get hz, then 1000 to get seconds.
  // Ie if speed in millihz is 1000 (ie 1 hz, or one step per second)
  // and we move for 1000 millis (is one second)
  // then we move 1,000,000 / 1,000,000 = 1 step
  int32_t stepsToMove =
      (speedInMilliHz * pulseDurationInMilliseconds) / 1000000;

  if (direction == 2) // east.Positive step change ie towards limit switch
    targetPosition += stepsToMove;

  if (direction == 3) // west. negative step change
    targetPosition -= stepsToMove;

  // make sure we don't run off end
  targetPosition = (targetPosition < 0) ? 0 : targetPosition;

  log("Pulse guiding %d for %ld ms to position %ld at speed (millihz) %lf",
      direction, pulseDurationInMilliseconds, targetPosition, speedInMilliHz);
  slew_target_pos = targetPosition;
  slewing = true;
  stepper->setSpeedInMilliHz(speedInMilliHz);
  stepper->moveTo(targetPosition);
}