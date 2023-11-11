#ifndef MOTORUNIT_H
#define MOTORUNIT_H

#include "DecDynamic.h"
#include "DecStatic.h"
#include "RADynamic.h"
#include "RAStatic.h" 
#include "ConcreteStepperWrapper.h"
#include <Preferences.h>
#include <TMCStepper.h>

class MotorUnit {
public:
  MotorUnit(RAStatic &rastatic, RADynamic &radynamic, DecStatic &decstatic,
            DecDynamic &decdynamic, Preferences &p);

  void setupMotors();
  void onLoop();

  double getPositionInMM();
  double getVelocityInMMPerMinute();
  unsigned long getAcceleration();
  void setAcceleration(unsigned long a);

private:
  RAStatic &raStatic;
  RADynamic &raDynamic;
  DecStatic &decStatic;
  DecDynamic &decDynamic;
  Preferences &preferences;
  unsigned long acceleration;

  void setupButtons();

  void setUpTMCDriver(TMC2209Stepper driver, int microsteps);
  ConcreteStepperWrapper *setUpFastAccelStepper(int32_t savedPosition,
                                                int stepPin, int dirPin);
};

#endif