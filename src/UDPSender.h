#ifndef UDPSENDER
#define UDPSENDER

#include "MotorUnit.h"
#include "PlatformDynamic.h"
#include "PlatformStatic.h"
void broadcastStatus(MotorUnit &motorUnit, PlatformStatic &model,
                     PlatformDynamic &control);

#endif
