#ifndef PLATFORM_MODEL_H
#define PLATFORM_MODEL_H
#include <cstdint>

//Represents the static attributes of the platform.
//Use to perform calculations using intrinsic platform attributes
//Exposes methods to change some of those attributes (eg circle radius)
//Does not hold dynamic state of the platform, ie position, as that comes from the motor

class PlatformModel {
public:
  void setupModel();
  uint32_t calculateFowardSpeedInMilliHz(int stepperCurrentPosition);
  double calculateTimeToCenterInSeconds(int32_t stepperCurrentPosition);
  double calculateTimeToEndOfRunInSeconds(int32_t stepperCurrentPosition);
  double getGreatCircleRadius();
  void setGreatCircleRadius(double r);
  double getStepsPerMM();
  
  //set distance in mm
  void setLimitSwitchToMiddleDistance(int pos);
  //get distance in mm
  int getLimitSwitchToMiddleDistance();

  //get position of limit, in steps
  int32_t getLimitPosition();
  // get position of middle, in steps
  int32_t getMiddlePosition();

  int getRewindFastFowardSpeed(); //hz
  void setRewindFastFowardSpeed(int speed);
private:
    
};

#endif