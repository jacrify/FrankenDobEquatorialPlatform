#include "DecStatic.h"
#include "Logging.h"
#include <Math.h>
// #include <iostream>
// #include <sstream>
using namespace std;

DecStatic::DecStatic()  : MotorStatic() {

  stepperStepsPerRevolution = 200;
  microsteps = 16;
  threadedRodPitch = 2;
  limitSwitchToEndDistance = 130;
  stepsPerMM = (stepperStepsPerRevolution * microsteps) / (threadedRodPitch);
  rodStepperRatio = 1;
}
