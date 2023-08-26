#include "PlatformModel.h"
#include "Logging.h"
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



int32_t limitSwitchToEndDistance = 135; // length of run in mm
// how far along middle point is in mm. IE distance from limit switch to point where
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

void PlatformModel::setupModel() {


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
  
}

int PlatformModel::getStepsPerMM() { return stepsPerMM; }

int PlatformModel::getMiddlePosition() {
  return (limitSwitchToEndDistance - limitSwitchToMiddleDistance) * stepsPerMM;
}

void PlatformModel::setLimitSwitchToMiddleDistance(int pos) {
  limitSwitchToMiddleDistance = pos;
  
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
    log("updating speed to %d",speed);
  rewindFastFowardSpeed = speed;
 
}
