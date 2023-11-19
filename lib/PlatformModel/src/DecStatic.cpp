#include "DecStatic.h"
#include "Logging.h"
#include <Math.h>
// #include <iostream>
// #include <sstream>
using namespace std;

DecStatic::DecStatic() : MotorStatic() {

  limitSwitchToEndDistance = 70;
  stepperStepsPerRevolution = 200;
  microsteps = 16;
  threadedRodPitch = 8;
  rodStepperRatio = 19;//gearbox is 19x
  stepsPerMM = (stepperStepsPerRevolution * microsteps * rodStepperRatio) /
               (threadedRodPitch);
}

int32_t DecStatic::getGotoEndPosition() { return 0; }