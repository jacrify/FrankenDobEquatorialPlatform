#include "ConcreteStepperWrapper.h"

ConcreteStepperWrapper::ConcreteStepperWrapper() {}

void ConcreteStepperWrapper::setStepper(FastAccelStepper *s) { stepper = s; }

void ConcreteStepperWrapper::resetPosition(int32_t position) {}

void ConcreteStepperWrapper::stop() {}

int32_t ConcreteStepperWrapper::getPosition() {}

void ConcreteStepperWrapper::setStepperSpeed(uint32_t speedInMillihz) {
  stepper->setSpeedInMilliHz(speedInMillihz);
  stepper->applySpeedAcceleration();
}

void ConcreteStepperWrapper::moveTo(int32_t position, uint32_t speedInMillihz) {
}