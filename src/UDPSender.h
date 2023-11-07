#ifndef UDPSENDER
#define UDPSENDER

#include "MotorUnit.h"
#include "RADynamic.h"
#include "RAStatic.h"
void broadcastStatus(MotorUnit &motorUnit, RAStatic &model, RADynamic &control);

#endif
