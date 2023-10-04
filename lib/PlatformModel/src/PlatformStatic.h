#ifndef PLATFORM_MODEL_H
#define PLATFORM_MODEL_H
#include <cstdint>

// Represents the static attributes of the platform.
// Use to perform calculations using intrinsic platform attributes
// Exposes methods to change some of those attributes (eg circle radius)
// Does not hold dynamic state of the platform, ie position, as that comes from
// the motor

class PlatformStatic {
public:
  void setupModel();

  void setConeRadiusAtAttachmentPoint(double r);
  // void setRAGuideRateArcSecondsPerSecond(double arcSecsPerSecond);
  void setRaGuideRateMultiplier(double d);
  // set distance in mm
  void setLimitSwitchToMiddleDistance(int pos);
  void setRewindFastFowardSpeedInHz(int speed);
  void setNunChukMultiplier(int m);
  int getNunChukMultiplier();
  int32_t getLimitSwitchSafetyStandoffPosition();
  int32_t getEndStandOffPosition();

  uint32_t calculateFowardSpeedInMilliHz(int stepperCurrentPosition);
  uint32_t calculateFowardSpeedInMilliHz(int stepperCurrentPosition,
                                         double desiredArcSecondsPerSecond);
  double calculateTimeToCenterInSeconds(int32_t stepperCurrentPosition);
  double calculateTimeToEndOfRunInSeconds(int32_t stepperCurrentPosition);
  int32_t calculatePulseGuideTargetPosition(int direction,
                                            long pulseDurationInMilliseconds,
                                            int32_t stepperCurrentPosition);

  double getConeRadiusAtAttachmentPoint();
  double getStepsPerMM();
  double getMaxAxisMoveRateDegreesSec();
  double getMinAxisMoveRateDegreesSec();
  double getRAGuideRateDegreesSec();
  double getRAGuideRateArcSecondsSecond();
  double getTrackingRateArcsSecondsSec();
  double getTrackingRateDegreesSec();
  double getRaGuideRateMultiplier();
  // get distance in mm
  int getLimitSwitchToMiddleDistance();
  // get position of limit, in steps
  int32_t getLimitPosition();
  // get position of middle, in steps
  int32_t getMiddlePosition();

  uint32_t getRewindFastFowardSpeedInMilliHz();
  int getRewindFastFowardSpeed(); // hz

private:
};

#endif