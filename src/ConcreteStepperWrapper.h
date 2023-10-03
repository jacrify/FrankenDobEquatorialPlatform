#ifndef __CONCRETESTEPPERWRAPPER_H__
#define __CONCRETESTEPPERWRAPPER_H__



#include "StepperWrapper.h"
#include "FastAccelStepper.h"

class ConcreteStepperWrapper : public StepperWrapper {

public:
  ConcreteStepperWrapper();
  void setStepper(FastAccelStepper *stepper);
  void moveTo(int32_t position, uint32_t speedInMillihz) override;
  void resetPosition(int32_t position) override;
  void stop() override;
  int32_t getPosition() override;
  void setStepperSpeed(uint32_t speedInMillihz) override;

private:
  FastAccelStepper* stepper;
};

#endif // __CONCRETESTEPPERWRAPPER_H__