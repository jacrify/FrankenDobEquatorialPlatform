#include "PlatformModel.h"
#include <Math.h>

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

#define PREF_CIRCLE_KEY "FFRW_SPEED"
#define PREF_SPEED_KEY "FFRW_SPEED"
#define PREF_MIDDLE_KEY "LIMIT_TO_MIDDLE"

#define DEFAULT_SPEED 30000
#define DEFAULT_MIDDLE_DISTANCE 62
// This value is the tuned value
#define DEFAULT_CIRCLE_RADIUS 448.0
    // 482.5; // And this is the value by design in 3d model

int32_t limitSwitchToEndDistance = 135; // length of run in mm
// how far along middle point is. IE distance from limit switch to point where
// platform is flat
int32_t limitSwitchToMiddleDistance;

int rewindFastFowardSpeed;

double rodStepperRatio =
    (double)teethOnRodPulley / (double)teethOnStepperPulley;

int stepsPerMM =
    (stepperStepsPerRevolution * microsteps * teethOnRodPulley) /
    (teethOnStepperPulley * threadedRodPitch);

// Limit position is highest. Then it counts down to zero at end.
// this  is expressed in microsteps

// Number of steps per output rotation
const int stepsPerRevolution = 200;

PlatformModel::PlatformModel() {
  preferences.begin("Platform", false);
  rewindFastFowardSpeed = preferences.getUInt(PREF_SPEED_KEY, DEFAULT_SPEED);
  limitSwitchToMiddleDistance =
      preferences.getUInt(PREF_MIDDLE_KEY, DEFAULT_MIDDLE_DISTANCE);
  greatCircleRadius =
      preferences.getUInt(PREF_CIRCLE_KEY, DEFAULT_CIRCLE_RADIUS);
}

int PlatformModel::calculateFowardSpeedInMilliHz(int stepperCurrentPosition) {

  double distanceFromCenterInMM =
      ((double)(getMiddlePosition() - stepperCurrentPosition)) /
      (double)getStepsPerMM();
  // log("distanceFromCenter %f", distanceFromCenterInMM);
  // log("greatCircleRadiansPerMinute %f", greatCircleRadiansPerMinute);

  double absoluteAngleMovedAtThisPoint =
      atan(distanceFromCenterInMM / greatCircleRadius);
  // log("Current angle (deg) %f", absoluteAngleMovedAtThisPoint * 180.0 / PI);

  // TODO handle -
  double absoluteAngleAfterOneMoreMinute =
      absoluteAngleMovedAtThisPoint + greatCircleRadiansPerMinute;

  // log("New angle (deg) %f", absoluteAngleAfterOneMoreMinute * 180.0 / PI);

  double distanceAlongRodAfterOneMoreMinute =
      greatCircleRadius * tan(absoluteAngleAfterOneMoreMinute);

  // log("distanceAlongRodAfterOneMoreMinute %f",
  // distanceAlongRodAfterOneMoreMinute);

  double threadDistancePerMinute =
      distanceAlongRodAfterOneMoreMinute - distanceFromCenterInMM;

  // log("threadDistancePerMinute %f", threadDistancePerMinute);
  // // ignore distance for now
  // double distanceToMoveAlongRodPerMinuteAtZero =
  //     tan(greatCircleRadiansPerMinute) * greatCircleRadius; // mm

  // log("distanceToMoveAlongRodPerMinuteAtZero %f",
  //     distanceToMoveAlongRodPerMinuteAtZero);

  double numberOfTurnsPerMinuteOfRod =
      threadDistancePerMinute / threadedRodPitch;

  // log("numberOfTurnsPerMinuteOfRod %f", numberOfTurnsPerMinuteOfRod);
  double numberOfTurnsPerMinuteOfStepper =
      numberOfTurnsPerMinuteOfRod * rodStepperRatio;
  // log("numberOfTurnsPerMinuteOfStepper %f", numberOfTurnsPerMinuteOfStepper);

  double numberOfStepsPerMinuteInMiddle =
      numberOfTurnsPerMinuteOfStepper * stepperStepsPerRevolution * microsteps;

  // log("numberOfStepsPerMinute %f", numberOfStepsPerMinuteInMiddle);

  double stepperSpeedInHertz = numberOfStepsPerMinuteInMiddle / 60.0;
  // log("stepperSpeedInHertz %f", stepperSpeedInHertz);
  int stepperSpeedInMilliHertz = stepperSpeedInHertz * 1000;
  // log("stepperSpeedInMilliHertz %d", stepperSpeedInMilliHertz);
  return stepperSpeedInMilliHertz;
}

double PlatformModel::getGreatCircleRadius() { return greatCircleRadius; }

void PlatformModel::setGreatCircleRadius(double radius) {
  greatCircleRadius = radius;
  preferences.putUInt(PREF_CIRCLE_KEY, radius);
}

int PlatformModel::getStepsPerMM() { return stepsPerMM; }

int PlatformModel::getMiddlePosition() {
  return (limitSwitchToEndDistance - limitSwitchToMiddleDistance) * stepsPerMM;
}

void PlatformModel::setLimitSwitchToMiddleDistance(int pos) {
  limitSwitchToMiddleDistance = pos;
  preferences.putUInt(PREF_MIDDLE_KEY,pos);
}

int PlatformModel::getLimitSwitchToMiddleDistance() {
  return limitSwitchToMiddleDistance;
}

int PlatformModel::getLimitPosition() {
  return limitSwitchToEndDistance * stepsPerMM;
  ;
}

int PlatformModel::getRewindFastFowardSpeed() { return rewindFastFowardSpeed; }

void PlatformModel::setRewindFastFowardSpeed(int speed) {
  rewindFastFowardSpeed = speed;
  preferences.putUInt(PREF_SPEED_KEY,speed);
}
