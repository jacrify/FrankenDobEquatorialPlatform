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
#define decDirPinStepper 33
#define decStepPinStepper 32

// #define raDirPinStepper 33
// #define raStepPinStepper 32
// #define decDirPinStepper 19
// #define decStepPinStepper 18

#define fastForwardSwitchPin 22
#define rewindSwitchPin 23
#define playSwitchPin 27

#define raLimitSwitchPin 21
// TODO change
#define decLimitSwitchPin 13

// How often we run the button check and calculation.
// Half of this timen is the average delay to starrt a pulseguide
#define BUTTONANDRECALCPERIOD 250

#define RA_PREF_SAVED_POS_KEY (char *)"RASavedPosition"

#define DEC_PREF_SAVED_POS_KEY (char *)"DCSavedPosition"

unsigned long lastButtonAndSpeedCalc;

unsigned long raPulseGuideUntil;  // absolute time in millis to pulseguide until
unsigned long decPulseGuideUntil; // absolute time in millis to pulseguide until

// See
// https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md

HardwareSerial &ra_serial_stream = Serial1;
HardwareSerial &dec_serial_stream = Serial2;
// const long SERIAL_BAUD_RATE = 115200;
const long SERIAL_BAUD_RATE = 19200;

const int RA_RX_PIN = 16; // not actually used
const int RA_TX_PIN = 17;

const int DEC_RX_PIN = 39; // not actually used
const int DEC_TX_PIN = 4;

// const int RA_RX_PIN = 39; // not actually used
// const int RA_TX_PIN = 4;

// const int DEC_RX_PIN =  16;// not actually used
// const int DEC_TX_PIN = 17;

const uint8_t RA_DRIVER_ADDRESS = 0;
const uint8_t DEC_DRIVER_ADDRESS = 0;

const float R_SENSE = 0.11; // Check your board's documentation. Typically it's
                            // 0.11 or 0.22 for TMC2209 modules.
const float HOLD_MULTIPLIER =
    0.5; // Specifies the hold current as a fraction of the run current
const int RA_MICROSTEPS = 16; // 1/16th microstepping
const int DEC_MICROSTEPS = 16;

// Initialize the driver instance
TMC2209Stepper ra_stepper_driver =
    TMC2209Stepper(&ra_serial_stream, R_SENSE, RA_DRIVER_ADDRESS);
TMC2209Stepper dec_stepper_driver =
    TMC2209Stepper(&dec_serial_stream, R_SENSE, DEC_DRIVER_ADDRESS);

long lastCheckTime = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
ConcreteStepperWrapper *rawrapper;
ConcreteStepperWrapper *decwrapper;
// FastAccelStepper *rastepper = NULL;
// FastAccelStepper *decstepper = NULL;
Preferences preferences;
Bounce bounceFastForward = Bounce();
Bounce bounceRewind = Bounce();
Bounce bouncePlay = Bounce();
Bounce bounceLimitRa = Bounce();
Bounce bounceLimitDec = Bounce();

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

  bounceLimitRa.attach(raLimitSwitchPin, INPUT_PULLUP);
  bounceLimitDec.attach(decLimitSwitchPin, INPUT_PULLUP);

  // DEBOUNCE INTERVAL IN MILLISECONDS
  bounceFastForward.interval(100); // interval in ms
  bounceRewind.interval(100);      // interval in ms
  bouncePlay.interval(100);        // interval in ms

  bounceLimitRa.interval(10);   // interval in ms
  bounceLimitDec.interval(100); // interval in ms. Longer as we had ghost pushes
}

void MotorUnit::setUpTMCDriver(TMC2209Stepper &driver, int microsteps) {
  driver.begin();
  // Set motor current
  driver.microsteps(microsteps);

  // StealthChop configuration
  driver.toff(5);
  driver.intpol(true);
  driver.rms_current(1700, 0.1);
  driver.en_spreadCycle(false); // This enables StealthChop
  driver.pwm_autograd(1);       // This enables automatic gradient adaptation
  driver.pwm_autoscale(1);      // This enables automatic current scaling
}
ConcreteStepperWrapper *MotorUnit::setUpFastAccelStepper(int32_t savedPosition,
                                                         int stepPin,
                                                         int dirPin,
                                                         char *prefsKey) {
  FastAccelStepper *stepper = engine.stepperConnectToPin(stepPin);
  if (stepper) {
    stepper->setDirectionPin(dirPin);
    stepper->setAutoEnable(true);
    stepper->setAcceleration(acceleration); // 100 steps/s²
    stepper->setCurrentPosition(savedPosition);
    ConcreteStepperWrapper *wrapper =
        new ConcreteStepperWrapper(preferences, prefsKey);
    wrapper->setStepper(stepper);
    return wrapper;

  } else {
    log("Error: stepper not initalised (step pin: %d dir pin: %d)", stepPin,
        dirPin);
    return nullptr;
  }
}
void MotorUnit::setupMotors() {
  raPulseGuideUntil = 0;
  decPulseGuideUntil = 0;

  setupButtons();

  ra_serial_stream.begin(SERIAL_BAUD_RATE, SERIAL_8N1, RA_RX_PIN, RA_TX_PIN);
  dec_serial_stream.begin(SERIAL_BAUD_RATE, SERIAL_8N1, DEC_RX_PIN, DEC_TX_PIN);
  setUpTMCDriver(ra_stepper_driver, RA_MICROSTEPS);
  setUpTMCDriver(dec_stepper_driver, DEC_MICROSTEPS);

  engine.init();

  int32_t raSavedPosition =
      preferences.getInt(RA_PREF_SAVED_POS_KEY, INT32_MAX);
  log("Loaded saved ra position %d", raSavedPosition);
  if (raSavedPosition > raStatic.getLimitPosition()) {
    raDynamic.setSafetyMode(true);
    raSavedPosition = 0;
  }
  rawrapper = setUpFastAccelStepper(raSavedPosition, raStepPinStepper,
                                    raDirPinStepper, RA_PREF_SAVED_POS_KEY);
  raDynamic.setStepperWrapper(rawrapper);

  int32_t decSavedPosition =
      preferences.getInt(DEC_PREF_SAVED_POS_KEY, INT32_MAX);
  log("Loaded saved dec position %d", decSavedPosition);
  if (decSavedPosition > decStatic.getLimitPosition()) {
    decDynamic.setSafetyMode(true);
    decSavedPosition = 0;
  }
  decwrapper = setUpFastAccelStepper(decSavedPosition, decStepPinStepper,
                                     decDirPinStepper, DEC_PREF_SAVED_POS_KEY);
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

bool isRALimitJustPushed() {
  return bounceLimitRa.changed() && bounceLimitRa.read() == LOW;
}

bool isRALimitJustReleased() {
  return bounceLimitRa.changed() && bounceLimitRa.read() != LOW;
}

// Note dec switch is wired the other way (high=on) as it was givng false
// positives.
bool isDecLimitJustReleased() {
  return bounceLimitDec.changed() && bounceLimitDec.read() != HIGH;
}

//
bool isDecLimitJustPushed() {
  return bounceLimitDec.changed() && bounceLimitDec.read() == HIGH;
}

// double degreesPerSecondToArcSecondsPerSecond(double degreesPerSecond) {
//   return degreesPerSecond * 3600.0;
// }

void MotorUnit::onLoop() {

  unsigned long now = millis();
  if (raPulseGuideUntil != 0) {
    if (now > raPulseGuideUntil) {
      // stops the pulse and resets back to original speed
      // we do this here to minise time overrun
      raDynamic.stopPulse();
      raPulseGuideUntil = 0;
      log("RA Pulse guide ended. Delta in milliseconds from requested duration "
          "was %ld",
          now - raPulseGuideUntil);
      lastButtonAndSpeedCalc = 0; // force recalc below
    } else {
      // need to check to stop dec pulse if both are running
      if (decPulseGuideUntil == 0)
        return;
    }
  }

  if (decPulseGuideUntil != 0) {
    if (now > decPulseGuideUntil) {
      // stops the pulse and resets back to original speed
      // we do this here to minise time overrun
      decDynamic.stopPulse();
      decPulseGuideUntil = 0;
      log("Dec Pulse guide ended. Delta in milliseconds from requested "
          "duration "
          "was %ld",
          now - decPulseGuideUntil);
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
    bounceLimitRa.update();
    bounceLimitDec.update();

    if (isDecLimitJustPushed()) {
      decDynamic.setLimitJustHit();
    } else if (isDecLimitJustReleased()) {
      decDynamic.setLimitJustReleased();
    }

    if (isRALimitJustPushed()) {
      raDynamic.setLimitJustHit();
    } else if (isRALimitJustReleased()) {
      raDynamic.setLimitJustReleased();
    }

    // when ff pushed, goto middle if we are less than halfway. Otherwise go to
    // end
    int32_t pos = rawrapper->getPosition();

    if (isFastForwardJustPushed()) {
      if (pos <= raStatic.getMiddlePosition()) {
        raDynamic.gotoEndish();
        decDynamic.gotoMiddle();
      } else {
        raDynamic.gotoMiddle();
        decDynamic.gotoMiddle();
      }
    }
    if (isFastForwardJustReleased()) {
      raDynamic.stop();
      decDynamic.stop();
    }

    if (isRewindJustPushed()) {

      if (raDynamic.isSafetyModeOn() || pos >= raStatic.getMiddlePosition()) {
        raDynamic.gotoStart();
        decDynamic.gotoMiddle();
      } else {
        raDynamic.gotoMiddle();
        decDynamic.gotoMiddle();
      }
    }
    //TODO bug here? If FF pushed in same cycle, it doesn't do anything?
    if (isRewindJustReleased()) {
      raDynamic.stop();
      decDynamic.stop();
    }
    if (isPlayJustPushed()) {
      raDynamic.setTrackingOnOff(true);
    }
    if (isPlayJustReleased()) {
      raDynamic.setTrackingOnOff(false);
    }


    long rd = raDynamic.onLoop();
    // handle pulseguide delay
    if (rd > 0) {
      raPulseGuideUntil = millis() + rd;
    }

    long dd = decDynamic.onLoop();
    // handle pulseguide delay
    if (dd > 0) {
      decPulseGuideUntil = millis() + dd;
    }
  }
}

double MotorUnit::getVelocityInMMPerMinute() {
  double speedInMHz =
      (double)rawrapper
          ->getStepperSpeed(); //  rastepper->getCurrentSpeedInMilliHz();
  double speedInHz = speedInMHz / 1000.0;
  double speedInMMPerSecond = speedInHz / raStatic.getStepsPerMM();
  double speedInMMPerMinute = speedInMMPerSecond * 60.0;
  return speedInMMPerMinute;
}

unsigned long MotorUnit::getAcceleration() { return acceleration; }

void MotorUnit::setAcceleration(unsigned long a) {
  acceleration = a;
  if (rawrapper != NULL) {
    rawrapper->setAcceleration(a);
    decwrapper->setAcceleration(a);
  }
}
double MotorUnit::getRaPositionInMM() {
  return ((double)rawrapper->getPosition()) / raStatic.getStepsPerMM();
}

double MotorUnit::getDecPositionInMM() {
  return ((double)decwrapper->getPosition()) / decStatic.getStepsPerMM();
}