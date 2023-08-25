#ifndef MYWEBSERVER_H
#define MYWEBSERVER_H
#include <ESPAsyncWebServer.h>
// #include "DigitalCaliper.h"
#include "MotorUnit.h"
#include "PlatformModel.h"

void setupWebServer(MotorUnit &motor,PlatformModel &model);
#endif
