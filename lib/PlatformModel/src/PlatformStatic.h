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

  // When the platform is centered, we can measure from the
  // axis of rotation(which points at the celestial pole)
  // to the center of the lead screw.
  // This is the basis for the calculation of tangent error
  // adjustment, and thus is what is used to control base
  // platform speed. Increase to go faster
  void setConeRadiusAtAttachmentPoint(double r);
  double getConeRadiusAtAttachmentPoint();

  /**
   * Sets a multiple of guide sidreal rate to be used when
   * pulseguiding (eg 0.5)
   */
  void setRaGuideRateMultiplier(double d);
  double getRaGuideRateMultiplier();

  /**
   * Sets distance in mm from limit switch to center.
   * As limit switch is physically fixed, this
   * moves the position of the center.
   * If platform is running at correct speed in center,
   * and too slow at limit, increase this
   *
   */
  void setLimitSwitchToMiddleDistance(int pos);
  // get distance in mm
  int getLimitSwitchToMiddleDistance();

  void setRewindFastFowardSpeedInHz(long speed);

  /** When nunchuk on focuser is moved, a percentage 0-100
   * is sent. This multiplier is muliplied by that perecentage
   * and sidereal rate to work out how fast to move
   */
  void setNunChukMultiplier(int m);
  int getNunChukMultiplier();

  /**
   * When running fast towards limit switch, the platform
   * slows before hitting it to so we don't hit it hard.
   * This gets the stepper position of this point.
   */
  int32_t getLimitSwitchSafetyStandoffPosition();

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

  uint32_t calculatePositionByDegreeShift(double degreesToMove, int32_t stepperCurrentPosition);

  double getStepsPerMM();
  double getMaxAxisMoveRateDegreesSec();
  double getMinAxisMoveRateDegreesSec();
  double getRAGuideRateDegreesSec();
  double getRAGuideRateArcSecondsSecond();
  double getTrackingRateArcsSecondsSec();
  double getTrackingRateDegreesSec();

  // get position of limit, in steps
  int32_t getLimitPosition();

  // get position of middle, in steps
  int32_t getMiddlePosition();

  uint32_t getRewindFastFowardSpeedInMilliHz();
  long getRewindFastFowardSpeed(); // hz

private:
};

#endif