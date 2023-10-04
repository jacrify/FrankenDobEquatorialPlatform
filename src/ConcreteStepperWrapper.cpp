#include "ConcreteStepperWrapper.h"
#include "Logging.h"
#define PREF_SAVED_POS_KEY "SavedPosition"
#define STEPPER_MIN_SPEED_HZ 300
ConcreteStepperWrapper::ConcreteStepperWrapper(Preferences &p) : prefs(p) {}

void ConcreteStepperWrapper::setStepper(FastAccelStepper *s) { stepper = s; }

void ConcreteStepperWrapper::resetPosition(int32_t position) {
  stepper->setCurrentPosition(position);
  // log("Resetting position to %ld",position);
}

void ConcreteStepperWrapper::stop() {
  // log("Stopping");
  stepper->stopMove();
  // stops flash getting hammered by braking. Assumes stop called every loop.
  if (stepper->getCurrentSpeedInMilliHz() == 0) {
    int32_t currentPos = stepper->getCurrentPosition();
    if (lastSavedPos != currentPos) {
      log("Saving position %ld", currentPos);
      prefs.putInt(PREF_SAVED_POS_KEY, currentPos);
      lastSavedPos = currentPos;
    }
  }
}

int32_t ConcreteStepperWrapper::getPosition() {
  return stepper->getCurrentPosition();
}

void ConcreteStepperWrapper::setStepperSpeed(uint32_t speedInMillihz) {
  // log("Setting speed");
  stepper->setSpeedInMilliHz(speedInMillihz);

  stepper->applySpeedAcceleration();
}

void ConcreteStepperWrapper::moveTo(int32_t position, uint32_t speedInMillihz) {
  //Stepper does weird stuff at very slow speeds. Treat these as stops
  if (speedInMillihz < STEPPER_MIN_SPEED_HZ) {
    stepper->stopMove();
  } else {
    stepper->setSpeedInMilliHz(speedInMillihz);
    // stepper->setSpeedInHz(5000);

    stepper->moveTo(position);
  }
  // log("Concrete Move called to pos %ld at speed %lu", position,
  //     speedInMillihz);
}
