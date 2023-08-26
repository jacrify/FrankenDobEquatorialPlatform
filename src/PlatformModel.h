#ifndef PLATFORM_MODEL_H
#define PLATFORM_MODEL_H


//Represents the static attributes of the platform.
//Use to perform calculations using intrinsic platform attributes
//Exposes methods to change some of those attributes (eg circle radius)
//Does not hold dynamic state of the platform, ie position, as that comes from the motor

class PlatformModel {
public:
  void setupModel();
  int calculateFowardSpeedInMilliHz(int stepperCurrentPosition);
  double getGreatCircleRadius();
  void setGreatCircleRadius(double r);
  int getStepsPerMM();
  
  //set distance in mm
  void setLimitSwitchToMiddleDistance(int pos);
  //get distance in mm
  int getLimitSwitchToMiddleDistance();

  //get position of limit, in steps
  int getLimitPosition();
  // get position of middle, in steps
  int getMiddlePosition();

  int getRewindFastFowardSpeed();
  void setRewindFastFowardSpeed(int speed);
private:
    
};

#endif