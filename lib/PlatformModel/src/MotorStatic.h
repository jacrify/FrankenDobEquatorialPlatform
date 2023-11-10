#ifndef __MOTORSTATIC_H__
#define __MOTORSTATIC_H__

#include <cstdint>

// Represents the static attributes of the an axis
// Use to perform calculations using intrinsic platform attributes
// Does not hold dynamic state of the platform, ie position, as that comes from
// the motor

class MotorStatic {
public:
  /**
   * Subclasses need to set in constructor:
   * - rodStepperRatio
   * - stepperStepsPerRevolution;
   * - microsteps;
   * - threadedRodPitch; // mm
   * - stepsPerMM
   * - limitSwitchToEndDistance
   */

  MotorStatic();

  virtual int32_t getGotoEndPosition() = 0;

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
   * When seeking limit switch, the platform
   * slows before hitting it to so we don't hit it hard.
   * This gets the stepper position of this point.
   */
  int32_t getLimitSwitchSafetyStandoffPosition();

  uint32_t calculatePositionByDegreeShift(double degreesToMove,
                                          int32_t stepperCurrentPosition);

  double getStepsPerMM();
  double getMaxAxisMoveRateDegreesSec();
  double getMinAxisMoveRateDegreesSec();
  double getGuideRateDegreesSec();
  double getGuideRateArcSecondsSecond();

  void setBaseGuideRateInArcSecondsSecond(double d);

  // get position of limit, in steps
  int32_t getLimitPosition();

  // get position of middle, in steps
  int32_t getMiddlePosition();

  uint32_t getRewindFastFowardSpeedInMilliHz();
  long getRewindFastFowardSpeed(); // hz

  void setScrewToPivotInMM(double d);
  double getScrewToPivotInMM();

  /**
   * Calculate speed for motor,,, correcting for tangent error.
   * Converts a rotation speed of top of  platform, in arc
   * seconds per second, into stepper millihz given a particular position.
   * ie motor should run faster at ends than in middle.
   */
  uint32_t calculateSpeedInMilliHz(int stepperCurrentPosition,
                                   double desiredArcSecondsPerSecond);

protected:
  double screwToPivotInMM;
  double rodStepperRatio;

  double baseGuideRateInArcSecondsSecond;
  int nunchukMultipler;
  int32_t limitSwitchToEndDistance; // from bottom to top
  int32_t limitSwitchToMiddleDistance;

  double guideRateInArcSecondsSecond;

  // speed in hz
  long rewindFastFowardSpeed;

  // used by alpaca.
  double rewindFastForwardSpeedDegreesSec;
  double stepsPerMM;

  // Number of steps per output rotation
  int stepperStepsPerRevolution;
  int microsteps;
  int threadedRodPitch; // mm

  // how far back from limt switch to slow down in mm
  int limitSwitchSafetyStandoffMM;
};

#endif // __MOTORSTATIC_H__