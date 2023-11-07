#ifndef MOTORUNIT_H
#define MOTORUNIT_H

#include "RADynamic.h"
#include "RAStatic.h"
#include <Preferences.h>

class MotorUnit {
public:
  MotorUnit(RAStatic &model, RADynamic &c, Preferences &p);

  void setupMotor();
  void onLoop();

  double getPositionInMM();
  double getVelocityInMMPerMinute();
  unsigned long getAcceleration();
  void setAcceleration(unsigned long a);

private:
  RAStatic &model;
  RADynamic &control;
  Preferences &preferences;
  unsigned long acceleration;
};

#endif