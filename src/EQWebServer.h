#ifndef MYWEBSERVER_H
#define MYWEBSERVER_H
#include <ESPAsyncWebServer.h>
// #include "DigitalCaliper.h"
#include "MotorUnit.h"
#include "PlatformModel.h"
#include "PlatformControl.h"
#include <Preferences.h>

#define PREF_CIRCLE_KEY "cr"
#define PREF_SPEED_KEY "ff"
#define PREF_MIDDLE_KEY "mid"
#define RA_GUIDE_KEY "raguide"
#define ACCEL_KEY "accel"

#define NUNCHUK_MULIPLIER_KEY "ncmult"
#define DEFAULT_NUNCHUK_MULIPLIER 2

#define DEFAULT_ACCEL 100000
#define DEFAULT_RA_GUIDE 0.5

#define DEFAULT_SPEED 30000
//this is in hz. In millihz this would be 30,000,000
#define DEFAULT_MIDDLE_DISTANCE 62
// This value is the tuned value
#define DEFAULT_CIRCLE_RADIUS 448.0
// 482.5; // And this is the value by design in 3d model


void setupWebServer(MotorUnit &motor,PlatformModel &model,PlatformControl &control,Preferences &prefs);
#endif
