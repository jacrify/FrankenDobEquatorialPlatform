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
  double getGreatCircleRadius();
  void setGreatCircleRadius(double radius);
  int getLimitSwitchToEndDistance();
  double getPositionInMM();
  double getVelocityInMMPerMinute();
  double getTimeToCenterInSeconds();
  double getTimeToEndOfRunInSeconds();
  bool getTrackingStatus();
  double getPlatformResetOffsetSeconds();
  void pulseGuide(int direction, long pulseDurationInMilliseconds);

  void moveAxis(double degreesPerSecond);
  void slewToStart();
  void slewToMiddle();
  void slewToEnd();
  bool isSlewing();
  void setTracking(bool b);

private:
  PlatformModel &model;
  PlatformControl &control;
  Preferences &preferences;
  

  void runModeSwitchCheck();
  void calibrationModeSwitchCheck();
  void slewToPosition(int32_t position);


};

#endif