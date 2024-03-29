#ifndef __CONCRETESTEPPERWRAPPER_H__
#define __CONCRETESTEPPERWRAPPER_H__

#include "FastAccelStepper.h"
#include "StepperWrapper.h"
#include <Preferences.h>

/**
 * A contrete wrapper class around FastAccelStepper.
 * This allows native unit testing of stepper control via StepperWrapper
*/
class ConcreteStepperWrapper : public StepperWrapper {

public:
  ConcreteStepperWrapper(Preferences &prefs, char* &pk);
  void setStepper(FastAccelStepper *stepper);
  void moveTo(int32_t position, uint32_t speedInMillihz) override;
  void moveAndResetPosition(int32_t positionToMoveTo,
                            int32_t positionToResetTo);
  void resetPosition(int32_t position) override;
  void stop() override;
  int32_t getPosition() override;
  void setStepperSpeed(uint32_t speedInMillihz) override;
  uint32_t getStepperSpeed() override;
  void setAcceleration(unsigned long a);

private:
  FastAccelStepper* stepper;
  Preferences &prefs;
  int32_t lastSavedPos;
  char* prefsKey;
};

#endif // __CONCRETESTEPPERWRAPPER_H__