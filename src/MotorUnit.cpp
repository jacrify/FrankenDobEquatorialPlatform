#include "MotorUnit.h"

#include "Logging.h"
#include "RADynamic.h"
#include "RAStatic.h"
#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <Preferences.h>


#define raDirPinStepper 19
#define raStepPinStepper 18

// TODO define dec pins
#define decDirPinStepper 99
#define decStepPinStepper 99

#define fastForwardSwitchPin 22
#define rewindSwitchPin 23
#define playSwitchPin 27

#define raLimitSwitchPin 21

// How often we run the button check and calculation.
// Half of this timen is the average delay to starrt a pulseguide
#define BUTTONANDRECALCPERIOD 250

#define RA_PREF_SAVED_POS_KEY "RASavedPosition"
#define DEC_PREF_SAVED_POS_KEY "DecSavedPosition"

unsigned long lastButtonAndSpeedCalc;

unsigned long pulseGuideUntil; // absolute time in millis to pulseguide until

// See
// https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md

HardwareSerial &serial_stream = Serial1;
// const long SERIAL_BAUD_RATE = 115200;
const long SERIAL_BAUD_RATE = 19200;

const int RX_PIN = 16;
const int TX_PIN = 17;

// TODO define driver addresses
const uint8_t RA_DRIVER_ADDRESS = 0; // Assuming address 0. Adjust if necessary.
const uint8_t DEC_DRIVER_ADDRESS = 0;

const float R_SENSE = 0.11; // Check your board's documentation. Typically it's
                            // 0.11 or 0.22 for TMC2209 modules.
const float HOLD_MULTIPLIER =
    0.5; // Specifies the hold current as a fraction of the run current
const uint16_t RA_MICROSTEPS = 16; // 1/16th microstepping
const uint16_t DEC_MICROSTEPS = 16;

// Initialize the driver instance
TMC2209Stepper ra_stepper_driver =
    TMC2209Stepper(&serial_stream, R_SENSE, RA_DRIVER_ADDRESS);
TMC2209Stepper dec_stepper_driver =
    TMC2209Stepper(&serial_stream, R_SENSE, DEC_DRIVER_ADDRESS);

long lastCheckTime = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *rastepper = NULL;
FastAccelStepper *decstepper = NULL;
Preferences preferences;
Bounce bounceFastForward = Bounce();
Bounce bounceRewind = Bounce();
Bounce bouncePlay = Bounce();
Bounce bounceLimit = Bounce();

MotorUnit::MotorUnit(RAStatic &rs, RADynamic &rd, DecStatic &ds, DecDynamic &dd,
                     Preferences &p)
    : raStatic(rs), raDynamic(rd), decStatic(ds), decDynamic(dd),
      preferences(p) {
  // raDynamic = PlatformStatic(ConcreteStepperWrapper(stepper), raStatic);
}

void MotorUnit::setupButtons() {
  lastButtonAndSpeedCalc = 0;
  bounceFastForward.attach(fastForwardSwitchPin, INPUT_PULLUP);
  bounceRewind.attach(rewindSwitchPin, INPUT_PULLUP);
  bouncePlay.attach(playSwitchPin, INPUT_PULLUP);

  bounceLimit.attach(raLimitSwitchPin, INPUT_PULLUP);

  // DEBOUNCE INTERVAL IN MILLISECONDS
  bounceFastForward.interval(100); // interval in ms
  bounceRewind.interval(100);      // interval in ms
  bouncePlay.interval(100);        // interval in ms

  bounceLimit.interval(10); // interval in ms
}

void MotorUnit::setUpTMCDriver(TMC2209Stepper driver,int microsteps) {
  driver.begin();
  // Set motor current
  driver.microsteps(microsteps);

  // StealthChop configuration
  driver.toff(5);
  driver.intpol(true);
  driver.rms_current(1700, 0.1);
  driver.en_spreadCycle(false); // This enables StealthChop
  driver.pwm_autograd(1);       // This enables automatic gradient adaptation
  driver.pwm_autoscale(1); // This enables automatic current scaling
}
ConcreteStepperWrapper *MotorUnit::setUpFastAccelStepper(int32_t savedPosition,
                                                         int stepPin,
                                                         int dirPin) {
  FastAccelStepper *stepper = engine.stepperConnectToPin(stepPin);
  if (stepper) {
    stepper->setDirectionPin(dirPin);
    stepper->setAutoEnable(true);
    stepper->setAcceleration(acceleration); // 100 steps/s²
    stepper->setCurrentPosition(savedPosition);
    ConcreteStepperWrapper *wrapper = new ConcreteStepperWrapper(preferences);
    wrapper->setStepper(stepper);
    return wrapper;

  } else {
    log("Error: stepper not initalised (step pin: %d dir pin: %d)",stepPin,dirPin);
    return nullptr;
  }
}
void MotorUnit::setupMotors() {
  pulseGuideUntil = 0;

  setupButtons();

  serial_stream.begin(SERIAL_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  setUpTMCDriver(ra_stepper_driver,RA_MICROSTEPS);
  setUpTMCDriver(dec_stepper_driver, DEC_MICROSTEPS);

  engine.init();

  int32_t raSavedPosition =
      preferences.getInt(RA_PREF_SAVED_POS_KEY, INT32_MAX);
  log("Loaded saved ra position %d", raSavedPosition);
  if (raSavedPosition == INT32_MAX) {
    raDynamic.setSafetyMode(true);
  }
  ConcreteStepperWrapper *rawrapper = setUpFastAccelStepper(
      raSavedPosition, raStepPinStepper, raDirPinStepper);
  raDynamic.setStepperWrapper(rawrapper);

  int32_t decSavedPosition =
      preferences.getInt(DEC_PREF_SAVED_POS_KEY, INT32_MAX);
  log("Loaded saved dec position %d", decSavedPosition);
  if (decSavedPosition == INT32_MAX) {
    decDynamic.setSafetyMode(true);
  }
  ConcreteStepperWrapper *decwrapper = setUpFastAccelStepper(
      decSavedPosition, decStepPinStepper, decDirPinStepper);
  decDynamic.setStepperWrapper(decwrapper);
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

// this returns false if fast forward has been pushed, as that takes precedence
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
  unsigned long now = millis();
  if (pulseGuideUntil != 0) {
    if (now > pulseGuideUntil) {
      // stops the pulse and resets back to original speed
      // we do this here to minise time overrun
      raDynamic.stopPulse();
      pulseGuideUntil = 0;
      log("Pulse guide ended. Delta in milliseconds from requested duration "
          "was %ld",
          now - pulseGuideUntil);
      lastButtonAndSpeedCalc = 0; // force recalc below
    } else {
      return;
    }
  }

  if ((now - lastButtonAndSpeedCalc) > BUTTONANDRECALCPERIOD) {
    lastButtonAndSpeedCalc = now;

    bounceFastForward.update();
    bounceRewind.update();
    bouncePlay.update();
    bounceLimit.update();

    raDynamic.setLimitSwitchState(isLimitSwitchHit());

    int32_t pos = rastepper->getCurrentPosition();

    if (isFastForwardJustPushed()) {
      if (pos <= raStatic.getMiddlePosition())
        raDynamic.gotoEndish();
      else
        raDynamic.gotoMiddle();
    }
    if (isFastForwardJustReleased()) {
      raDynamic.stop();
    }
    if (isRewindJustPushed()) {
      if (pos >= raStatic.getMiddlePosition())
        raDynamic.gotoStart();
      else
        raDynamic.gotoMiddle();
    }
    if (isRewindJustReleased()) {
      raDynamic.stop();
    }
    if (isPlayJustPushed()) {
      raDynamic.setTrackingOnOff(true);
    }
    if (isPlayJustReleased()) {
      raDynamic.setTrackingOnOff(false);
    }

    long d = raDynamic.onLoop();

    // handle pulseguide delay
    if (d > 0) {
      pulseGuideUntil = millis() + d;
    }
  }
}

double MotorUnit::getVelocityInMMPerMinute() {
  double speedInMHz = (double)rastepper->getCurrentSpeedInMilliHz();
  double speedInHz = speedInMHz / 1000.0;
  double speedInMMPerSecond = speedInHz / raStatic.getStepsPerMM();
  double speedInMMPerMinute = speedInMMPerSecond * 60.0;
  return speedInMMPerMinute;
}

unsigned long MotorUnit::getAcceleration() { return acceleration; }

void MotorUnit::setAcceleration(unsigned long a) {
  acceleration = a;
  if (rastepper != NULL) {
    rastepper->setAcceleration(a);
  }
}
double MotorUnit::getPositionInMM() {
  return ((double)rastepper->getCurrentPosition()) / raStatic.getStepsPerMM();
}
