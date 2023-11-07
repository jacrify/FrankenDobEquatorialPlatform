#ifndef __RASTATIC_H__
#define __RASTATIC_H__

#include "MotorStatic.h"
#include <cstdint>

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
  int32_t getGotoEndPosition() override;

  /** Convenience method to calculate motor sidereal tracking */
  uint32_t calculateTrackingSpeedInMilliHz(int stepperCurrentPosition);

  // Calculates runtime to center based on sidreal rate
  double calculateTimeToCenterInSeconds(int32_t stepperCurrentPosition);

  // Calculates runtime to end based on sidreal rate
  double calculateTimeToEndOfRunInSeconds(int32_t stepperCurrentPosition);

  double getTrackingRateArcsSecondsSec();
  double getTrackingRateDegreesSec();

private:
  double guideRateMultiplier;
};

#endif // __RASTATIC_H__