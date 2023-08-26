#ifndef MOTORUNIT_H
#define MOTORUNIT_H

#include "PlatformModel.h"
#include <Preferences.h>

class MotorUnit {
public:
  void setupMotor(PlatformModel &model, Preferences &p);
  void onLoop();
  double getGreatCircleRadius();
  void setGreatCircleRadius(double radius); 
  int getLimitSwitchToEndDistance();
  double getPositionInMM();
  double getVelocityInMMPerMinute();

private:
  PlatformModel model;
  void runModeSwitchCheck();
  void calibrationModeSwitchCheck();
  Preferences preferences;
};

#endif