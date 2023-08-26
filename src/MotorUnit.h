#ifndef MOTORUNIT_H
#define MOTORUNIT_H

#include "PlatformModel.h"

class MotorUnit {
public:
  void setupMotor(PlatformModel model);
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
};

#endif