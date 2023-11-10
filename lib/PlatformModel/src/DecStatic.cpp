#include "DecStatic.h"
#include "Logging.h"
#include <Math.h>
// #include <iostream>
// #include <sstream>
using namespace std;

DecStatic::DecStatic() : MotorStatic() {

  limitSwitchToEndDistance = 130;
   stepperStepsPerRevolution = 200;
   microsteps = 16;
   threadedRodPitch = 2;
   stepsPerMM = (double)(stepperStepsPerRevolution * microsteps) /( (double)(threadedRodPitch));

   rodStepperRatio = 1;
}


int32_t DecStatic::getGotoEndPosition() { return 0; }