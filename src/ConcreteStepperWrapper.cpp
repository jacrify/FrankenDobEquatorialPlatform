#include "ConcreteStepperWrapper.h"
#include "Logging.h"

ConcreteStepperWrapper::ConcreteStepperWrapper() {}

void ConcreteStepperWrapper::setStepper(FastAccelStepper *s) { stepper = s; }

void ConcreteStepperWrapper::resetPosition(int32_t position) {
  stepper->setCurrentPosition(position);
}

void ConcreteStepperWrapper::stop() {
  stepper->stopMove();
}

int32_t ConcreteStepperWrapper::getPosition() {
  return stepper->getCurrentPosition();
}

void ConcreteStepperWrapper::setStepperSpeed(uint32_t speedInMillihz) {
  stepper->setSpeedInMilliHz(speedInMillihz);
  stepper->applySpeedAcceleration();
}

void ConcreteStepperWrapper::moveTo(int32_t position, uint32_t speedInMillihz) {
  stepper->moveTo(position,speedInMillihz);
  log("Concrete Move called");
}
