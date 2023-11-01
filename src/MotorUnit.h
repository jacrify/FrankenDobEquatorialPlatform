#ifndef MOTORUNIT_H
#define MOTORUNIT_H

#include "PlatformDynamic.h"
#include "PlatformStatic.h"
#include <Preferences.h>

class MotorUnit {
public:
  MotorUnit(PlatformStatic &model, RADynamic &c, Preferences &p);

  void setupMotor();
  void onLoop();

  double getPositionInMM();
  double getVelocityInMMPerMinute();
  unsigned long getAcceleration();
  void setAcceleration(unsigned long a);

private:
  PlatformStatic &model;
  RADynamic &control;
  Preferences &preferences;
  unsigned long acceleration;
};

#endif