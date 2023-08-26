#ifndef MYWEBSERVER_H
#define MYWEBSERVER_H
#include <ESPAsyncWebServer.h>
// #include "DigitalCaliper.h"
#include "MotorUnit.h"
#include "PlatformModel.h"
#include <Preferences.h>

#define PREF_CIRCLE_KEY "cr"
#define PREF_SPEED_KEY "ff"
#define PREF_MIDDLE_KEY "mid"

#define DEFAULT_SPEED 30000
#define DEFAULT_MIDDLE_DISTANCE 62
// This value is the tuned value
#define DEFAULT_CIRCLE_RADIUS 448.0
// 482.5; // And this is the value by design in 3d model


void setupWebServer(MotorUnit &motor,PlatformModel &model,Preferences &prefs);
#endif
