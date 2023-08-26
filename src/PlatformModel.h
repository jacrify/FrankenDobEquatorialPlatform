#ifndef PLATFORM_MODEL_H
#define PLATFORM_MODEL_H



//Represents the static attributes of the platform.
//Use to perform calculations using intrinsic platform attributes
//Exposes methods to change some of those attributes (eg circle radius)
//Does not hold dynamic state of the platform, ie position, as that comes from the motor

class PlatformModel {
public:
  int calculateFowardSpeedInMilliHz(double distanceFromCenterInMM);
  double getGreatCircleRadius();
  void setGreatCircleRadius(double r);
  double getStepsPerMM();
  int getMiddlePosition();
  int getLimitPosition();

  int getRewindFastFowardSpeed();
  void setRewindFastFowardSpeed(int speed);
};

#endif