#ifndef MOTORUNIT_H
#define MOTORUNIT_H

#include "PlatformModel.h"
#include <Preferences.h>

class MotorUnit {
public:
  MotorUnit(PlatformModel &model, Preferences &p);

  void setupMotor();
  void onLoop();
  double getGreatCircleRadius();
  void setGreatCircleRadius(double radius);
  int getLimitSwitchToEndDistance();
  double getPositionInMM();
  double getVelocityInMMPerMinute();
  double getTimeToCenterInSeconds();
  double getTimeToEndOfRunInSeconds();
  bool getTrackingStatus();
  void park();
  void home();

private:
  PlatformModel &model;
  Preferences &preferences;
  void runModeSwitchCheck();
  void calibrationModeSwitchCheck();

  boolean homing;
  boolean parking;
};

#endif