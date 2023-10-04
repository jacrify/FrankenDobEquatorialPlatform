#ifndef MOTORUNIT_H
#define MOTORUNIT_H

#include "PlatformDynamic.h"
#include "PlatformStatic.h"
#include <Preferences.h>

class MotorUnit {
public:
  MotorUnit(PlatformStatic &model, PlatformDynamic &c, Preferences &p);

  void setupMotor();
  void onLoop();

  double getPositionInMM();
  double getVelocityInMMPerMinute();
  unsigned long getAcceleration();
  void setAcceleration(unsigned long a);

private:
  PlatformStatic &model;
  PlatformDynamic &control;
  Preferences &preferences;
  unsigned long acceleration;
};

#endif