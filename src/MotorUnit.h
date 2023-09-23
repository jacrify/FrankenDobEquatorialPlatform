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
  double getPlatformResetOffsetSeconds();

      void
      moveAxis(double degreesPerSecond);
  void slewToStart();
  void slewToMiddle();
  void slewToEnd();
  bool isSlewing();
  void setTracking(bool b);

private:
  PlatformModel &model;
  Preferences &preferences;
  void runModeSwitchCheck();
  void calibrationModeSwitchCheck();
  void slewToPosition(int32_t position);
  int32_t slew_target_pos;
  boolean slewing;
  boolean slewingToStart;
  boolean tracking;
  //accumlates ff/rw offsets
  double platformResetOffsetSeconds;

  double lastTimeToCenterSeconds;
  bool firstMoveCycleForCalc;
};

#endif