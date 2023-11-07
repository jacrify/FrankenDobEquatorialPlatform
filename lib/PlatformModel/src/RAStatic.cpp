#include "RAStatic.h"
// #include "Logging.h"
#include "Logging.h"
#include <Math.h>

using namespace std;
double coneRadiansPerMinute = M_PI * 2 / 24.0 / 60.0;

// this is millimeters from pivot to center rod. Think of a
// cone lying on the ground, with the axis of the cone pointing at the
// south pole in the sky. If you take a slice through this cone at
// right angles to the axis, so the the slice passes through the contact
// point between the drive unit and the top of the platform,
// this value is the radius of that slice.
// Effectivly it drives the speed of the platform, as it is used
// in the speeed calculations

#define sideRealArcSecondsPerSec 15.041

#define teethOnStepperPulley 16
#define teethOnRodPulley 36

// how far to stop when moving to end.
// 10 mm=approx 5 minutes
#define END_STANDOFF_MM 10

const double fullRotation =
    2 * M_PI; // M_PI is the value for pi from the cmath library
const double fullRotationTimeInSeconds = 24 * 60 * 60;



// double raGuideRateInArcSecondsSecond;
// double raGuideRateMultiplier;
// double raGuideRateDegreesSec;

RAStatic::RAStatic() : MotorStatic() {

  limitSwitchToEndDistance = 130;
  stepsPerMM = (stepperStepsPerRevolution * microsteps * teethOnRodPulley) /
               (teethOnStepperPulley * threadedRodPitch);
  rodStepperRatio = (double)teethOnRodPulley / (double)teethOnStepperPulley;
}

double
RAStatic::calculateTimeToEndOfRunInSeconds(int32_t stepperCurrentPosition) {
  int32_t endToMiddleInMM =
      limitSwitchToEndDistance - limitSwitchToMiddleDistance;
  int32_t endToMiddleInSteps = endToMiddleInMM * stepsPerMM;
  return calculateTimeToCenterInSeconds(stepperCurrentPosition +
                                        endToMiddleInSteps);
}

double
RAStatic::calculateTimeToCenterInSeconds(int32_t stepperCurrentPosition) {
  int middle = getMiddlePosition();
  // note this calculation is the other way around from
  // calculateFowardSpeedInMilliHz:
  double stepsFromMiddle = (double)(stepperCurrentPosition - middle);
  double stepsPerMM = getStepsPerMM();
  double distanceFromCenterInMM = stepsFromMiddle / stepsPerMM;

  double absoluteAngleMovedAtThisPoint =
      atan(distanceFromCenterInMM / screwToPivotInMM);

  double fractionMoved = absoluteAngleMovedAtThisPoint / fullRotation;

  // 24 hours = 86400 seconds
  double fullRotationTimeInSeconds = 24 * 60 * 60;

  double secondsMoved = fractionMoved * fullRotationTimeInSeconds;
  return secondsMoved;
}

uint32_t RAStatic::calculateFowardSpeedInMilliHz(int stepperCurrentPosition) {
  return calculateFowardSpeedInMilliHz(stepperCurrentPosition,
                                       sideRealArcSecondsPerSec);
}

uint32_t
RAStatic::calculateFowardSpeedInMilliHz(int stepperCurrentPosition,
                                        double desiredArcSecondsPerSecond) {

  int middle = getMiddlePosition();
  double stepsFromMiddle = (double)(middle - stepperCurrentPosition);

  double distanceFromCenterInMM = stepsFromMiddle / stepsPerMM;

  double absoluteAngleMovedAtThisPoint =
      atan(distanceFromCenterInMM / screwToPivotInMM);

  double radiansPerSecond =
      desiredArcSecondsPerSecond * (M_PI / 180.0 / 3600.0);

  double absoluteAngleAfterOneMoreSecond =
      absoluteAngleMovedAtThisPoint + radiansPerSecond;

  double distanceAlongRodAfterOneMoreSecond =
      screwToPivotInMM * tan(absoluteAngleAfterOneMoreSecond);

  double threadDistancePerSecond =
      distanceAlongRodAfterOneMoreSecond - distanceFromCenterInMM;

  double numberOfTurnsPerSecondOfRod =
      threadDistancePerSecond / threadedRodPitch;

  double numberOfTurnsPerSecondOfStepper =
      numberOfTurnsPerSecondOfRod * rodStepperRatio;

  double numberOfStepsPerSecondInMiddle =
      numberOfTurnsPerSecondOfStepper * stepperStepsPerRevolution * microsteps;

  double stepperSpeedInHertz = numberOfStepsPerSecondInMiddle;
  uint32_t stepperSpeedInMilliHertz = stepperSpeedInHertz * 1000;
  return stepperSpeedInMilliHertz;
}

void RAStatic::setGuideRateMultiplier(double d) {
  guideRateMultiplier = d;
  guideRateInArcSecondsSecond = sideRealArcSecondsPerSec * d;
  log("guide rate set: %lf arc secs per sec, or %lf x sidereal",
      guideRateInArcSecondsSecond, guideRateMultiplier);
}

int32_t RAStatic::getGotoEndPosition()  { return stepsPerMM * END_STANDOFF_MM; }

double RAStatic::getTrackingRateArcsSecondsSec() {
  return sideRealArcSecondsPerSec;
}

double RAStatic::getTrackingRateDegreesSec() {
  return sideRealArcSecondsPerSec / 3600.0;
}

double RAStatic::getGuideRateMultiplier() { return guideRateMultiplier; }

// double RAStatic::getRAGuideRateDegreesSec() {
//   return raGuideRateInArcSecondsSecond / 3600.0;
// }
// double RAStatic::getRAGuideRateArcSecondsSecond() {
//   return raGuideRateInArcSecondsSecond;
// }
