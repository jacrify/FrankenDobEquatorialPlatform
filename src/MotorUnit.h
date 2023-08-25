#ifndef MOTORUNIT_H
#define MOTORUNIT_H

#include "PlatformModel.h"

class MotorUnit {
public:
  PlatformModel model;
  void setupMotor(PlatformModel model);
  void onLoop();

  double getGreatCircleRadius();
  void setGreatCircleRadius(double radius);

  
  int getLimitSwitchToEndDistance();

  double getPositionInMM();
  double getVelocityInMMPerMinute();

  void moveTo(int location);
private:
  void runModeSwitchCheck();
  void calibrationModeSwitchCheck();
};

#endif