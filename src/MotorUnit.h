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
  // double getGreatCircleRadius();
  // void setGreatCircleRadius(double radius);
  int getLimitSwitchToEndDistance();
  double getPositionInMM();
  double getVelocityInMMPerMinute();


  // double getPlatformResetOffsetSeconds();


  
  bool isSlewing();


private:
  PlatformModel &model;
  PlatformControl &control;
  Preferences &preferences;
  

  // void runModeSwitchCheck();
  // void calibrationModeSwitchCheck();
  // void slewToPosition(int32_t position);


};

#endif