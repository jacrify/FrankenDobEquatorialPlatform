#include "PlatformStatic.h"
// #include "Logging.h"
#include "Logging.h"
#include <Math.h>
#include <iostream>
#include <sstream>
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

double coneRadius;

#define sideRealArcSecondsPerSec 15.041

#define teethOnStepperPulley 16
#define teethOnRodPulley 36
// Number of steps per output rotation
#define stepperStepsPerRevolution 200
#define microsteps 16
#define threadedRodPitch 2 // mm

// const int stepsPerRevolution = 200;
// how far back from limt switch to slow down in mm
#define LIMITSWITCHSAFETYSTANDOFFMM 2

// how far to stop when moving to end.
// 10 mm=approx 5 minutes
#define END_STANDOFF_MM 10

int nunchukMultipler;

const double fullRotation =
    2 * M_PI; // M_PI is the value for pi from the cmath library
const double fullRotationTimeInSeconds = 24 * 60 * 60;

int32_t limitSwitchToEndDistance = 130; // length of run in mm
// how far along middle point is in mm. IE distance from limit switch to point
// where platform is flat
int32_t limitSwitchToMiddleDistance;

double raGuideRateInArcSecondsSecond;
double raGuideRateMultiplier;
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

void PlatformStatic::setupModel() {}

double PlatformStatic::calculateTimeToEndOfRunInSeconds(
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
PlatformStatic::calculateTimeToCenterInSeconds(int32_t stepperCurrentPosition) {
  int middle = getMiddlePosition();
  // note this calculation is the other way around from
  // calculateFowardSpeedInMilliHz:
  double stepsFromMiddle = (double)(stepperCurrentPosition - middle);
  double stepsPerMM = getStepsPerMM();
  double distanceFromCenterInMM = stepsFromMiddle / stepsPerMM;

  double absoluteAngleMovedAtThisPoint =
      atan(distanceFromCenterInMM / coneRadius);

  double fractionMoved = absoluteAngleMovedAtThisPoint / fullRotation;

  // 24 hours = 86400 seconds
  double fullRotationTimeInSeconds = 24 * 60 * 60;

  double secondsMoved = fractionMoved * fullRotationTimeInSeconds;
  return secondsMoved;
}

// returns max drive speed in degrees per second
double PlatformStatic::getMaxAxisMoveRateDegreesSec() {
  return rewindFastForwardSpeedDegreesSec;
}

double PlatformStatic::getMinAxisMoveRateDegreesSec() {
  return sideRealArcSecondsPerSec / 3600.0 / 4.0;
}

uint32_t
PlatformStatic::calculateFowardSpeedInMilliHz(int stepperCurrentPosition) {
  return calculateFowardSpeedInMilliHz(stepperCurrentPosition,
                                       sideRealArcSecondsPerSec);
}

uint32_t PlatformStatic::calculateFowardSpeedInMilliHz(
    int stepperCurrentPosition, double desiredArcSecondsPerSecond) {

  int middle = getMiddlePosition();
  double stepsFromMiddle = (double)(middle - stepperCurrentPosition);

  double distanceFromCenterInMM = stepsFromMiddle / stepsPerMM;

  double absoluteAngleMovedAtThisPoint =
      atan(distanceFromCenterInMM / coneRadius);

  double radiansPerSecond =
      desiredArcSecondsPerSecond * (M_PI / 180.0 / 3600.0);

  double absoluteAngleAfterOneMoreSecond =
      absoluteAngleMovedAtThisPoint + radiansPerSecond;

  double distanceAlongRodAfterOneMoreSecond =
      coneRadius * tan(absoluteAngleAfterOneMoreSecond);

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

double PlatformStatic::getConeRadiusAtAttachmentPoint() { return coneRadius; }

void PlatformStatic::setConeRadiusAtAttachmentPoint(double radius) {
  coneRadius = radius;
}

void PlatformStatic::setRaGuideRateMultiplier(double d) {
  raGuideRateMultiplier = d;
  raGuideRateInArcSecondsSecond = sideRealArcSecondsPerSec * d;
  log("guide rate set: %lf arc secs per sec, or %lf x sidereal",
      raGuideRateInArcSecondsSecond, raGuideRateMultiplier);
}

double PlatformStatic::getStepsPerMM() { return stepsPerMM; }

int32_t PlatformStatic::getMiddlePosition() {
  return (limitSwitchToEndDistance - limitSwitchToMiddleDistance) * stepsPerMM;
}

void PlatformStatic::setLimitSwitchToMiddleDistance(int pos) {
  limitSwitchToMiddleDistance = pos;
}

int PlatformStatic::getLimitSwitchToMiddleDistance() {
  return limitSwitchToMiddleDistance;
}

// Limit position is highest. Then it counts down to zero at end.
// this  is expressed in microsteps
int32_t PlatformStatic::getLimitPosition() {
  return limitSwitchToEndDistance * stepsPerMM;
}

uint32_t PlatformStatic::getRewindFastFowardSpeedInMilliHz() {
  return rewindFastFowardSpeed * 1000;
}
int PlatformStatic::getRewindFastFowardSpeed() { return rewindFastFowardSpeed; }

void PlatformStatic::setRewindFastFowardSpeedInHz(int speedInHz) {
  // log("updating speed to %d",speed);
  rewindFastFowardSpeed = speedInHz;

  // Calculate speed in degrees/sec for alpaca based on the passed speed
  // Calculate speed in degrees/sec for alpaca based on the passed speed

  double rodTurnsPerSec =
      speedInHz / (rodStepperRatio * stepperStepsPerRevolution * microsteps);
  double distancePerSec = rodTurnsPerSec * threadedRodPitch;
  double anglePerSec = atan(distancePerSec / coneRadius);

  rewindFastForwardSpeedDegreesSec = anglePerSec * (180.0 / M_PI);
}

int PlatformStatic::getNunChukMultiplier() { return nunchukMultipler; }
void PlatformStatic::setNunChukMultiplier(int m) { nunchukMultipler = m; }

// how far back from limt switch to slow down
int32_t PlatformStatic::getLimitSwitchSafetyStandoffPosition() {
  return getLimitPosition() - (LIMITSWITCHSAFETYSTANDOFFMM * getStepsPerMM());
}

int32_t PlatformStatic::getEndStandOffPosition() {
  return stepsPerMM * END_STANDOFF_MM;
}

double PlatformStatic::getTrackingRateArcsSecondsSec() {
  return sideRealArcSecondsPerSec;
}

double PlatformStatic::getTrackingRateDegreesSec() {
  return sideRealArcSecondsPerSec / 3600.0;
}

double PlatformStatic::getRaGuideRateMultiplier() {
  return raGuideRateMultiplier;
}

double PlatformStatic::getRAGuideRateDegreesSec() {
  return raGuideRateInArcSecondsSecond / 3600.0;
}
double PlatformStatic::getRAGuideRateArcSecondsSecond() {
  return raGuideRateInArcSecondsSecond;
}
/**
 * Give current stepper location, a direction, and a duration,
 * this method uses the raGuideRateInHz to calculate what step position
 * to move to that would take the designated number of milliseconds
 * to reach. Acceleration is assumed to be instant.
 */
int32_t PlatformStatic::calculatePulseGuideTargetPosition(
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