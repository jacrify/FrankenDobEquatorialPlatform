#include "RAStatic.h"
// #include "Logging.h"
#include "Logging.h"
#include <Math.h>

using namespace std;
// double coneRadiansPerMinute = M_PI * 2 / 24.0 / 60.0;

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
  stepperStepsPerRevolution = 200;
  microsteps = 16;
  threadedRodPitch = 2;
  stepsPerMM = ((double)(stepperStepsPerRevolution * microsteps * teethOnRodPulley)) /
              ((double)(teethOnStepperPulley * threadedRodPitch));
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
  // calculateSpeedInMilliHz:
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

uint32_t RAStatic::calculateTrackingSpeedInMilliHz(int stepperCurrentPosition) {
  return calculateSpeedInMilliHz(stepperCurrentPosition,
                                 sideRealArcSecondsPerSec);
}

void RAStatic::setGuideRateMultiplier(double d) {
  guideRateMultiplier = d;
  guideRateInArcSecondsSecond = sideRealArcSecondsPerSec * d;
  log("guide rate set: %lf arc secs per sec, or %lf x sidereal",
      guideRateInArcSecondsSecond, guideRateMultiplier);
}

int32_t RAStatic::getGotoEndPosition() { return stepsPerMM * END_STANDOFF_MM; }

double RAStatic::getTrackingRateArcsSecondsSec() {
  return sideRealArcSecondsPerSec;
}

double RAStatic::getTrackingRateDegreesSec() {
  return sideRealArcSecondsPerSec / 3600.0;
}

double RAStatic::getGuideRateMultiplier() { return guideRateMultiplier; }
