#include "MotorUnit.h"
#include "ConcreteStepperWrapper.h"
#include "Logging.h"
#include "PlatformDynamic.h"
#include "PlatformStatic.h"
#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <Preferences.h>
#include <TMCStepper.h>

#define dirPinStepper 19
#define stepPinStepper 18

#define fastForwardSwitchPin 22
#define rewindSwitchPin 23
#define playSwitchPin 27

#define limitSwitchPin 21

// How often we run the button check and calculation.
// Half of this timen is the average delay to starrt a pulseguide
#define BUTTONANDRECALCPERIOD 250

#define PREF_SAVED_POS_KEY "SavedPosition"

unsigned long pulseGuideUntil; // absolute time in millis to pulseguide until

// See
// https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md

HardwareSerial &serial_stream = Serial1;
// const long SERIAL_BAUD_RATE = 115200;
const long SERIAL_BAUD_RATE = 19200;

const int RX_PIN = 16;
const int TX_PIN = 17;
const uint8_t DRIVER_ADDRESS = 0; // Assuming address 0. Adjust if necessary.

const float R_SENSE = 0.11; // Check your board's documentation. Typically it's
                            // 0.11 or 0.22 for TMC2209 modules.
const float HOLD_MULTIPLIER =
    0.5; // Specifies the hold current as a fraction of the run current
const uint16_t MICROSTEPS = 16; // 1/16th microstepping

// Initialize the driver instance
TMC2209Stepper stepper_driver =
    TMC2209Stepper(&serial_stream, R_SENSE, DRIVER_ADDRESS);

long lastCheckTime = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
Preferences preferences;
Bounce bounceFastForward = Bounce();
Bounce bounceRewind = Bounce();
Bounce bouncePlay = Bounce();
Bounce bounceLimit = Bounce();

MotorUnit::MotorUnit(PlatformStatic &m, PlatformDynamic &c, Preferences &p)
    : model(m), control(c), preferences(p) {

  // control = PlatformStatic(ConcreteStepperWrapper(stepper), model);
}

void MotorUnit::setupMotor() {
  pulseGuideUntil = 0;
  lastButtonAndSpeedCalc=0;
   bounceFastForward.attach(fastForwardSwitchPin,
                                                  INPUT_PULLUP);
  bounceRewind.attach(rewindSwitchPin, INPUT_PULLUP);
  bouncePlay.attach(playSwitchPin, INPUT_PULLUP);

  bounceLimit.attach(limitSwitchPin, INPUT_PULLUP);

  // DEBOUNCE INTERVAL IN MILLISECONDS
  bounceFastForward.interval(100); // interval in ms
  bounceRewind.interval(100);      // interval in ms
  bouncePlay.interval(100);        // interval in ms

  bounceLimit.interval(10); // interval in ms

  serial_stream.begin(SERIAL_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  stepper_driver.begin();

  // Set motor current

  stepper_driver.microsteps(MICROSTEPS);

  // StealthChop configuration
  stepper_driver.toff(5);
  stepper_driver.intpol(true);
  stepper_driver.rms_current(1700, 0.1);
  stepper_driver.en_spreadCycle(false); // This enables StealthChop
  stepper_driver.pwm_autograd(1);  // This enables automatic gradient adaptation
  stepper_driver.pwm_autoscale(1); // This enables automatic current scaling

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setAutoEnable(true);

    // stepper->setSpeedInHz(5000);
    stepper->setAcceleration(acceleration); // 100 steps/s²
    // stepper->setAcceleration(100000); // 100 steps/s²

    // preferences = p;
    int32_t savedPosition = preferences.getInt(PREF_SAVED_POS_KEY, INT32_MAX);
    log("Loaded saved position %d", savedPosition);
    // when current position is not know, go slowly.
    if (savedPosition == INT32_MAX) {
      control.setSafetyMode(true);
    }
    stepper->setCurrentPosition(savedPosition);
    // delay(3000);
    // stepper->setSpeedInHz(5000);
    // delay(1000);
    // stepper->moveTo(0);
    // delay(3000);
    // log("Moved");
  }
  ConcreteStepperWrapper *wrapper = new ConcreteStepperWrapper(preferences);
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

unsigned long lastButtonAndSpeedCalc;

void MotorUnit::onLoop() {
  unsigned long now = millis();
  if (pulseGuideUntil != 0) {
    if (now > pulseGuideUntil) {
      //stops the pulse and resets back to original speed
      //we do this here to minise time overrun
      control.stopPulse();
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

    control.setLimitSwitchState(isLimitSwitchHit());

    int32_t pos = stepper->getCurrentPosition();

    if (isFastForwardJustPushed()) {
      if (pos <= model.getMiddlePosition())
        control.gotoEndish();
      else
        control.gotoMiddle();
    }
    if (isFastForwardJustReleased()) {
      control.stop();
    }
    if (isRewindJustPushed()) {
      if (pos >= model.getMiddlePosition())
        control.gotoStart();
      else
        control.gotoMiddle();
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

    long d = control.calculateOutput();

    // handle pulseguide delay
    if (d > 0) {
      pulseGuideUntil = millis() + d;
    }
  }
}

double MotorUnit::getVelocityInMMPerMinute() {
  double speedInMHz = (double)stepper->getCurrentSpeedInMilliHz();
  double speedInHz = speedInMHz / 1000.0;
  double speedInMMPerSecond = speedInHz / model.getStepsPerMM();
  double speedInMMPerMinute = speedInMMPerSecond * 60.0;
  return speedInMMPerMinute;
}

unsigned long MotorUnit::getAcceleration() { return acceleration; }

void MotorUnit::setAcceleration(unsigned long a) {
  acceleration = a;
  if (stepper != NULL) {
    stepper->setAcceleration(a);
  }
}
double MotorUnit::getPositionInMM() {
  return ((double)stepper->getCurrentPosition()) / model.getStepsPerMM();
}
