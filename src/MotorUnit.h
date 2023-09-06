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
  double getTimeToCenterInSeconds();
  double getTimeToEndOfRunInSeconds();
  bool getTrackingStatus();
  void park();
  void home();

private:
  PlatformModel model;
  void runModeSwitchCheck();
  void calibrationModeSwitchCheck();
  Preferences preferences;
  boolean homing;
  boolean parking;
};

#endif