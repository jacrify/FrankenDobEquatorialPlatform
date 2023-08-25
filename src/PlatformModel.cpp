#include "PlatformModel.h"
#include <Math.h>

double greatCircleRadiansPerMinute = M_PI * 2 / 24.0 / 60.0;
double greatCircleRadius = 448; // this is millimeters from pivot to center rod.
                                // This value is the tuned value
// 482.5; // And this is the value by design !!

#define teethOnStepperPulley 16
#define teethOnRodPulley 36
#define stepperStepsPerRevolution 200
#define microsteps 16
#define threadedRodPitch 2 // mm

#define limitSwitchToMiddleDistance 62 // mm
#define limitSwitchToEndDistance 135   // mm

double rodStepperRatio =
    (double)teethOnRodPulley / (double)teethOnStepperPulley;

double stepsPerMM =
    (stepperStepsPerRevolution * microsteps * teethOnRodPulley) /
    (teethOnStepperPulley * threadedRodPitch);

// Limit position is highest. Then it counts down to zero at end.
// this  is expressed in microsteps
int32_t limitPosition = limitSwitchToEndDistance * stepsPerMM;
int32_t middlePosition =
    limitPosition - (limitSwitchToMiddleDistance * stepsPerMM);

// Number of steps per output rotation
const int stepsPerRevolution = 200;

int PlatformModel::calculateFowardSpeedInMilliHz(double distanceFromCenterInMM) {

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

double PlatformModel::getStepsPerMM(){
    return stepsPerMM;
}

int PlatformModel::getMiddlePosition() { return middlePosition; }

int PlatformModel::getLimitPosition() { return limitPosition; }