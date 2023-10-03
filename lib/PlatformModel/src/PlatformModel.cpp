#include "PlatformModel.h"
// #include "Logging.h"
#include "Logging.h"
#include <Math.h>
#include <iostream>
#include <sstream>
using namespace std;
double greatCircleRadiansPerMinute = M_PI * 2 / 24.0 / 60.0;

// this is millimeters from pivot to center rod. Think of a
// cone lying on the ground, with the axis of the cone pointing at the
// south pole in the sky. If you take a slice through this cone at
// right angles to the axis, so the the slice passes through the contact
// point between the drive unit and the top of the platform,
// this value is the radius of that slice.
// Effectivly it drives the speed of the platform, as it is used
// in the speeed calculations

double greatCircleRadius;

#define sideRealArcSecondsPerSec 15.041

#define teethOnStepperPulley 16
#define teethOnRodPulley 36
// Number of steps per output rotation
#define stepperStepsPerRevolution 200
#define microsteps 16
#define threadedRodPitch 2 // mm

const double fullRotation =
    2 * M_PI; // M_PI is the value for pi from the cmath library
const double fullRotationTimeInSeconds = 24 * 60 * 60;

int32_t limitSwitchToEndDistance = 130; // length of run in mm
// how far along middle point is in mm. IE distance from limit switch to point
// where platform is flat
int32_t limitSwitchToMiddleDistance;

double raGuideRateInArcSecondsSecond;
// double raGuideRateDegreesSec;

// speed in hz
int rewindFastFowardSpeed;

// used by alpaca.
double rewindFastForwardSpeedDegreesSec;

double rodStepperRatio =
    (double)teethOnRodPulley / (double)teethOnStepperPulley;

double stepsPerMM =
    (stepperStepsPerRevolution * microsteps * teethOnRodPulley) /
    (teethOnStepperPulley * threadedRodPitch);

// const int stepsPerRevolution = 200;
// how far back from limt switch to slow down in mm
#define LIMITSWITCHSAFETYSTANDOFFMM 5


void PlatformModel::setupModel() {}

double PlatformModel::calculateTimeToEndOfRunInSeconds(
    int32_t stepperCurrentPosition) {
  int32_t endToMiddleInMM =
      limitSwitchToEndDistance - limitSwitchToMiddleDistance;
  int32_t endToMiddleInSteps = endToMiddleInMM * stepsPerMM;
  return calculateTimeToCenterInSeconds(stepperCurrentPosition +
                                        endToMiddleInSteps);
}
// work out how long it will be until we are at center. If we've passed center
// will be negative.
double
PlatformModel::calculateTimeToCenterInSeconds(int32_t stepperCurrentPosition) {
  int middle = getMiddlePosition();
  // note this calculation is the other way around from
  // calculateFowardSpeedInMilliHz:
  double stepsFromMiddle = (double)(stepperCurrentPosition - middle);
  double stepsPerMM = getStepsPerMM();
  double distanceFromCenterInMM = stepsFromMiddle / stepsPerMM;

  double absoluteAngleMovedAtThisPoint =
      atan(distanceFromCenterInMM / greatCircleRadius);

  double fractionMoved = absoluteAngleMovedAtThisPoint / fullRotation;

  // 24 hours = 86400 seconds
  double fullRotationTimeInSeconds = 24 * 60 * 60;

  double secondsMoved = fractionMoved * fullRotationTimeInSeconds;
  return secondsMoved;
}

// returns max drive speed in degrees per second
double PlatformModel::getMaxAxisMoveRateDegreesSec() {
  return rewindFastForwardSpeedDegreesSec;
}

double PlatformModel::getMinAxisMoveRateDegreesSec() {
  return sideRealArcSecondsPerSec / 3600.0 / 4.0;
}

uint32_t
PlatformModel::calculateFowardSpeedInMilliHz(int stepperCurrentPosition) {
  return calculateFowardSpeedInMilliHz(stepperCurrentPosition,
                                       sideRealArcSecondsPerSec);
}

uint32_t PlatformModel::calculateFowardSpeedInMilliHz(
    int stepperCurrentPosition, double desiredArcSecondsPerSecond) {

  int middle = getMiddlePosition();
  double stepsFromMiddle = (double)(middle - stepperCurrentPosition);

  double distanceFromCenterInMM = stepsFromMiddle / stepsPerMM;

  double absoluteAngleMovedAtThisPoint =
      atan(distanceFromCenterInMM / greatCircleRadius);

  double radiansPerSecond =
      desiredArcSecondsPerSecond * (M_PI / 180.0 / 3600.0);

  double absoluteAngleAfterOneMoreSecond =
      absoluteAngleMovedAtThisPoint + radiansPerSecond;

  double distanceAlongRodAfterOneMoreSecond =
      greatCircleRadius * tan(absoluteAngleAfterOneMoreSecond);

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

double PlatformModel::getGreatCircleRadius() { return greatCircleRadius; }

void PlatformModel::setGreatCircleRadius(double radius) {
  greatCircleRadius = radius;
}

double PlatformModel::getStepsPerMM() { return stepsPerMM; }

int32_t PlatformModel::getMiddlePosition() {
  return (limitSwitchToEndDistance - limitSwitchToMiddleDistance) * stepsPerMM;
}

void PlatformModel::setLimitSwitchToMiddleDistance(int pos) {
  limitSwitchToMiddleDistance = pos;
}

int PlatformModel::getLimitSwitchToMiddleDistance() {
  return limitSwitchToMiddleDistance;
}

// Limit position is highest. Then it counts down to zero at end.
// this  is expressed in microsteps
int32_t PlatformModel::getLimitPosition() {
  return limitSwitchToEndDistance * stepsPerMM;
}

long PlatformModel::getRewindFastFowardSpeedInMilliHz() {
  return rewindFastFowardSpeed * 1000;
}
int PlatformModel::getRewindFastFowardSpeed() { return rewindFastFowardSpeed; }

void PlatformModel::setRewindFastFowardSpeedInHz(int speedInHz) {
  // log("updating speed to %d",speed);
  rewindFastFowardSpeed = speedInHz;

  // Calculate speed in degrees/sec for alpaca based on the passed speed
  // Calculate speed in degrees/sec for alpaca based on the passed speed

  double rodTurnsPerSec =
      speedInHz / (rodStepperRatio * stepperStepsPerRevolution * microsteps);
  double distancePerSec = rodTurnsPerSec * threadedRodPitch;
  double anglePerSec = atan(distancePerSec / greatCircleRadius);

  rewindFastForwardSpeedDegreesSec = anglePerSec * (180.0 / M_PI);
}

//how far back from limt switch to slow down
uint32_t PlatformModel::getLimitSwitchSafetyStandoffPosition() {
  return getLimitPosition() - (LIMITSWITCHSAFETYSTANDOFFMM * getStepsPerMM());
}

void PlatformModel::setRAGuideRateArcSecondsPerSecond(
    double arcSecondsPerSecond) {
  raGuideRateInArcSecondsSecond = arcSecondsPerSecond;
  // raGuideRateDegreesSec =
  //     degreesPerSecond;
  // double radiansPerSecond = degreesPerSecond * (M_PI / 180.0);
  // double rodTurnsPerSec =
  //     atan(radiansPerSecond * greatCircleRadius) / threadedRodPitch;
  // raGuideRateInMilliHz = 1000 * rodTurnsPerSec * rodStepperRatio *
  //                        stepperStepsPerRevolution * microsteps;
  log("guide rate set: %lf arc secs per sec", raGuideRateInArcSecondsSecond);
}

double PlatformModel::getTrackingRateArcsSecondsSec() {
  return sideRealArcSecondsPerSec;
}

double PlatformModel::getTrackingRateDegreesSec() {
  return sideRealArcSecondsPerSec / 3600.0;
}

double PlatformModel::getRAGuideRateDegreesSec() {
  return raGuideRateInArcSecondsSecond * 3600;
}
double PlatformModel::getRAGuideRateArcSecondsSecond() {
  return raGuideRateInArcSecondsSecond;
}
/**
 * Give current stepper location, a direction, and a duration,
 * this method uses the raGuideRateInHz to calculate what step position
 * to move to that would take the designated number of milliseconds
 * to reach. Acceleration is assumed to be instant.
 */
int32_t PlatformModel::calculatePulseGuideTargetPosition(
    int direction, long pulseDurationInMilliseconds,
    int32_t stepperCurrentPosition) {
  // Direction is either  2 = guideEast, 3 = guideWest.
  // If 2 then returned value will be higher than stepperCurrentPosition
  // If 3 then return value will be lower.

  uint32_t speedInMilliHz = calculateFowardSpeedInMilliHz(
      stepperCurrentPosition, raGuideRateInArcSecondsSecond);
  int32_t stepsToMove = (speedInMilliHz * pulseDurationInMilliseconds) / 1000;

  if (direction == 2) // east.Positive step change ie towards limit switch
    return stepperCurrentPosition + stepsToMove;

  if (direction == 3) // west. negative step change
    return stepperCurrentPosition - stepsToMove;

  log("Error: direction not supported %d ", direction);
  return stepperCurrentPosition;
}