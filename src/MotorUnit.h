#ifndef MOTORUNIT_H
#define MOTORUNIT_H

class MotorUnit {
public:
  void setupMotor();
  void onLoop();

  double getGreatCircleRadius();
  void setGreatCircleRadius(double radius);

  int getCalibrationSpeed();
  void setCalibrationSpeed(int speed);

  int getRunBackSpeed();
  void setrunbackSpeed(int speed);

  int getLimitSwitchToEndDistance();

  int getPosition();
  int getVelocity();

      void
      moveTo(int location);
};
#endif