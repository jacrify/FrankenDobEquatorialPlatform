#ifndef PLATFORM_MODEL_H
#define PLATFORM_MODEL_H
#include <cstdint>
#include "MotorStatic.h"

// Represents the static attributes of the platform.
// Use to perform calculations using intrinsic platform attributes
// Exposes methods to change some of those attributes (eg circle radius)
// Does not hold dynamic state of the platform, ie position, as that comes from
// the motor

class RAStatic : public MotorStatic {
public:
   RAStatic();

  /**
   * Sets a multiple of guide sidreal rate to be used when
   * pulseguiding (eg 0.5)
   */
  void setGuideRateMultiplier(double d);
  double getGuideRateMultiplier();

   /**
   * When running to end, platform stops some distance from end
   * to allow some runtime (eg for polar alignment).
   * This returns the stepper position of that point.
   */
  int32_t getEndStandOffPosition();

  /** Convenience method to calculate motor sidereal tracking */
  uint32_t calculateFowardSpeedInMilliHz(int stepperCurrentPosition);

  /**
   * Calculate tracking speed for motor,,, correcting for tangent error.
   * Converts a rotation speed of top of  platform, in arc
   * seconds per second, into stepper millihz given a particular position.
   * ie motor should run faster at ends than in middle.
   */

  uint32_t calculateFowardSpeedInMilliHz(int stepperCurrentPosition,
                                         double desiredArcSecondsPerSecond);

  // Calculates runtime to center based on sidreal rate
  double calculateTimeToCenterInSeconds(int32_t stepperCurrentPosition);

  // Calculates runtime to end based on sidreal rate
  double calculateTimeToEndOfRunInSeconds(int32_t stepperCurrentPosition);

  
  double getTrackingRateArcsSecondsSec();
  double getTrackingRateDegreesSec();



private:
  double guideRateMultiplier;
};

#endif