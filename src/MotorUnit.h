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

  int getCalibrationSpeed();
  void setCalibrationSpeed(int speed);

  int getRunBackSpeed();
  void setrunbackSpeed(int speed);

  int getLimitSwitchToEndDistance();

  double getPositionInMM();
  double getVelocityInMMPerMinute();

  void moveTo(int location);
private:
  void runModeSwitchCheck();
  void calibrationModeSwitchCheck();
};

#endif