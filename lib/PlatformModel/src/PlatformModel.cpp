#include "PlatformModel.h"
// #include "Logging.h"
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

#define teethOnStepperPulley 16
#define teethOnRodPulley 36
#define stepperStepsPerRevolution 200
#define microsteps 16
#define threadedRodPitch 2 // mm

int32_t limitSwitchToEndDistance = 135; // length of run in mm
// how far along middle point is in mm. IE distance from limit switch to point
// where platform is flat
int32_t limitSwitchToMiddleDistance;

int rewindFastFowardSpeed;

double rodStepperRatio =
    (double)teethOnRodPulley / (double)teethOnStepperPulley;

double stepsPerMM =
    (stepperStepsPerRevolution * microsteps * teethOnRodPulley) /
    (teethOnStepperPulley * threadedRodPitch);

// Limit position is highest. Then it counts down to zero at end.
// this  is expressed in microsteps

// Number of steps per output rotation
// const int stepsPerRevolution = 200;

void PlatformModel::setupModel() {}

uint32_t
PlatformModel::calculateFowardSpeedInMilliHz(int stepperCurrentPosition) {

  int middle = getMiddlePosition();
  double stepsFromMiddle = (double)(middle - stepperCurrentPosition);
  double stepsPerMM = getStepsPerMM();
  double distanceFromCenterInMM = stepsFromMiddle / stepsPerMM;

  double absoluteAngleMovedAtThisPoint =
      atan(distanceFromCenterInMM / greatCircleRadius);
  
  double absoluteAngleAfterOneMoreMinute =
      absoluteAngleMovedAtThisPoint + greatCircleRadiansPerMinute;

  double distanceAlongRodAfterOneMoreMinute =
      greatCircleRadius * tan(absoluteAngleAfterOneMoreMinute);
  

  double threadDistancePerMinute =
      distanceAlongRodAfterOneMoreMinute - distanceFromCenterInMM;

  double numberOfTurnsPerMinuteOfRod =
      threadDistancePerMinute / threadedRodPitch;

  double numberOfTurnsPerMinuteOfStepper =
      numberOfTurnsPerMinuteOfRod * rodStepperRatio;

  double numberOfStepsPerMinuteInMiddle =
      numberOfTurnsPerMinuteOfStepper * stepperStepsPerRevolution * microsteps;

  double stepperSpeedInHertz = numberOfStepsPerMinuteInMiddle / 60.0;
  
             // log("stepperSpeedInHertz %f", stepperSpeedInHertz);
  uint32_t stepperSpeedInMilliHertz = stepperSpeedInHertz * 1000;
  // log("stepperSpeedInMilliHertz %d", stepperSpeedInMilliHertz);
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

int32_t PlatformModel::getLimitPosition() {
  return limitSwitchToEndDistance * stepsPerMM;
  ;
}

int PlatformModel::getRewindFastFowardSpeed() { return rewindFastFowardSpeed; }

void PlatformModel::setRewindFastFowardSpeed(int speed) {
  // log("updating speed to %d",speed);
  rewindFastFowardSpeed = speed;
}
