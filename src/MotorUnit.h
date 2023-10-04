#ifndef MOTORUNIT_H
#define MOTORUNIT_H

#include "PlatformModel.h"
#include "PlatformControl.h"
#include <Preferences.h>

class MotorUnit {
public:
  MotorUnit(PlatformModel &model, PlatformControl &c,Preferences &p);

  void setupMotor();
  void onLoop();

  double getPositionInMM();
  double getVelocityInMMPerMinute();
  unsigned long getAcceleration();
  void setAcceleration(unsigned long a);

private:
  PlatformModel &model;
  PlatformControl &control;
  Preferences &preferences;
  unsigned long acceleration;
  
};

#endif