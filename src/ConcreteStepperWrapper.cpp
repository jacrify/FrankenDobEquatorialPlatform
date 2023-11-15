#include "ConcreteStepperWrapper.h"
#include "Logging.h"
// #define PREF_SAVED_POS_KEY "SavedPosition"
#define STEPPER_MIN_SPEED_HZ 300
ConcreteStepperWrapper::ConcreteStepperWrapper(Preferences &p, char* &pk)
    : prefs(p), prefsKey(pk) {}

void ConcreteStepperWrapper::setStepper(FastAccelStepper *s) { stepper = s; }

void ConcreteStepperWrapper::resetPosition(int32_t position) {
  stepper->setCurrentPosition(position);
}

void ConcreteStepperWrapper::stop() {
  stepper->stopMove();
  // stops flash getting hammered by braking. Assumes stop called every loop.
  if (stepper->getCurrentSpeedInMilliHz() == 0) {
    int32_t currentPos = stepper->getCurrentPosition();
    if (lastSavedPos != currentPos) {
      log("Saving position %ld", currentPos);
      prefs.putInt(prefsKey, currentPos);
      lastSavedPos = currentPos;
    }
  }
}

int32_t ConcreteStepperWrapper::getPosition() {
  return stepper->getCurrentPosition();
}

uint32_t ConcreteStepperWrapper::getStepperSpeed() {
  return stepper->getSpeedInMilliHz();
}

void ConcreteStepperWrapper::setStepperSpeed(uint32_t speedInMillihz) {
  // log("Setting speed");
  stepper->setSpeedInMilliHz(speedInMillihz);

  stepper->applySpeedAcceleration();
}

void ConcreteStepperWrapper::moveTo(int32_t position, uint32_t speedInMillihz) {
  // Stepper does weird stuff at very slow speeds. Treat these as stops
  if (speedInMillihz < STEPPER_MIN_SPEED_HZ) {
    stepper->stopMove();
  } else {
    stepper->setSpeedInMilliHz(speedInMillihz);
    // stepper->setSpeedInHz(5000);

    stepper->moveTo(position);
  }
}

void ConcreteStepperWrapper::setAcceleration(unsigned long a) {
  stepper->setAcceleration(a);
}