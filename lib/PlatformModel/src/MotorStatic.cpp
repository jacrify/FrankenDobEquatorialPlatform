#include "MotorStatic.h"
#include "Logging.h"
#include <Math.h>
// #include <iostream>
// #include <sstream>
using namespace std;

MotorStatic::MotorStatic() {
  // how far back from limt switch to slow down in mm
  limitSwitchSafetyStandoffMM = 2;
}

uint32_t
MotorStatic::calculatePositionByDegreeShift(double degreesToMove,
                                            int32_t stepperCurrentPosition) {
  int32_t middle = getMiddlePosition();
  double stepsFromMiddle = (double)(stepperCurrentPosition - middle);
  double stepsPerMM = getStepsPerMM();
  double distanceFromCenterInMM = stepsFromMiddle / stepsPerMM;

  // tan(angle)=o/r
  double angleMovedFromCenterRadians =
      atan(distanceFromCenterInMM / screwToPivotInMM);

  double radiansToMove = degreesToMove * M_PI / 180.0;

  double targetRadians = angleMovedFromCenterRadians + radiansToMove;
  // o=tan(angle)*r

  double targetDistanceFromCenterInMM = tan(targetRadians) * screwToPivotInMM;
  uint32_t targetStepsFromMiddle = targetDistanceFromCenterInMM * stepsPerMM;

  int32_t targetPosition = middle + targetStepsFromMiddle;
  if (targetPosition < 0)
    return 0;
  if (targetPosition > getLimitPosition())
    return getLimitPosition();
  return targetPosition;
}

// returns max drive speed in degrees per second
double MotorStatic::getMaxAxisMoveRateDegreesSec() {
  return rewindFastForwardSpeedDegreesSec;
}

double MotorStatic::getMinAxisMoveRateDegreesSec() {
  // TODO make this something else :)
  return rewindFastForwardSpeedDegreesSec / 100;
}

double MotorStatic::getStepsPerMM() { return stepsPerMM; }

int32_t MotorStatic::getMiddlePosition() {
  return (limitSwitchToEndDistance - limitSwitchToMiddleDistance) * stepsPerMM;
}

void MotorStatic::setLimitSwitchToMiddleDistance(int pos) {
  limitSwitchToMiddleDistance = pos;
}

int MotorStatic::getLimitSwitchToMiddleDistance() {
  return limitSwitchToMiddleDistance;
}

// Limit position is highest. Then it counts down to zero at end.
// this  is expressed in microsteps
int32_t MotorStatic::getLimitPosition() {
  return limitSwitchToEndDistance * stepsPerMM;
}

uint32_t MotorStatic::getRewindFastFowardSpeedInMilliHz() {
  return rewindFastFowardSpeed * 1000;
}
long MotorStatic::getRewindFastFowardSpeed() { return rewindFastFowardSpeed; }

void MotorStatic::setRewindFastFowardSpeedInHz(long speedInHz) {
  // log("updating speed to %d",speed);
  rewindFastFowardSpeed = speedInHz;

  // Calculate speed in degrees/sec for alpaca based on the passed speed

  double rodTurnsPerSec =
      speedInHz / (rodStepperRatio * stepperStepsPerRevolution * microsteps);
  double distancePerSec = rodTurnsPerSec * threadedRodPitch;
  double anglePerSec = atan(distancePerSec / screwToPivotInMM);

  rewindFastForwardSpeedDegreesSec = anglePerSec * (180.0 / M_PI);
}

int MotorStatic::getNunChukMultiplier() { return nunchukMultipler; }
void MotorStatic::setNunChukMultiplier(int m) { nunchukMultipler = m; }

// how far back from limt switch to slow down
int32_t MotorStatic::getLimitSwitchSafetyStandoffPosition() {
  return getLimitPosition() - (limitSwitchSafetyStandoffMM * getStepsPerMM());
}

double MotorStatic::getGuideRateDegreesSec() {
  return guideRateInArcSecondsSecond / 3600.0;
}
double MotorStatic::getGuideRateArcSecondsSecond() {
  return guideRateInArcSecondsSecond;
}

void MotorStatic::setBaseGuideRateInArcSecondsSecond(double d) {
  baseGuideRateInArcSecondsSecond = d;
}

void MotorStatic::setScrewToPivotInMM(double d) { screwToPivotInMM = d; }

double MotorStatic::getScrewToPivotInMM() { return screwToPivotInMM; }

uint32_t
MotorStatic::calculateSpeedInMilliHz(int stepperCurrentPosition,
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

void MotorStatic::setGuideRateMultiplier(double d) {
  guideRateMultiplier = d;
  guideRateInArcSecondsSecond = sideRealArcSecondsPerSec * d;
  log("guide rate set: %lf arc secs per sec, or %lf x sidereal",
      guideRateInArcSecondsSecond, guideRateMultiplier);
}

double MotorStatic::getGuideRateMultiplier() { return guideRateMultiplier; }
