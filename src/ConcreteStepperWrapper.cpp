#include "ConcreteStepperWrapper.h"
#include "Logging.h"

ConcreteStepperWrapper::ConcreteStepperWrapper() {}

void ConcreteStepperWrapper::setStepper(FastAccelStepper *s) { stepper = s; }

void ConcreteStepperWrapper::resetPosition(int32_t position) {
  stepper->setCurrentPosition(position);
  log("Resetting position to %ld",position);
}

void ConcreteStepperWrapper::stop() {
  // log("Stopping");
  stepper->stopMove();
}

int32_t ConcreteStepperWrapper::getPosition() {
  return stepper->getCurrentPosition();
}

void ConcreteStepperWrapper::setStepperSpeed(uint32_t speedInMillihz) {
  log("Setting speed");
  stepper->setSpeedInMilliHz(speedInMillihz);
 
  stepper->applySpeedAcceleration();
}

void ConcreteStepperWrapper::moveTo(int32_t position, uint32_t speedInMillihz) {
  // stepper->setSpeedInMilliHz(speedInMillihz);
  stepper->setSpeedInHz(5000);
  
  stepper->moveTo(position);
  log("Concrete Move called to pos %ld at speed %lu", position,
      speedInMillihz);
}
