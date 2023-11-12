#ifndef MYWEBSERVER_H
#define MYWEBSERVER_H
#include <ESPAsyncWebServer.h>
// #include "DigitalCaliper.h"
#include "MotorUnit.h"
#include "RADynamic.h"
#include "RAStatic.h"
#include "DecStatic.h"
#include "DecDynamic.h"
#include <Preferences.h>


//TODO delete this key once new value saved
#define PREF_CIRCLE_KEY "cr"

#define RA_LEAD_TO_PIVOT_KEY "ralpk"
#define DEC_LEAD_TO_PIVOT_KEY "declpk"

#define PREF_SPEED_KEY "ff"


//TODO DELETE MID later once new value saved
#define PREF_MIDDLE_KEY "mid"

#define RA_PREF_MIDDLE_KEY "ramid"
#define DEC_PREF_MIDDLE_KEY "decmid"

#define RA_GUIDE_KEY "raguide"
#define ACCEL_KEY "accel"

#define NUNCHUK_MULIPLIER_KEY "ncmult"
#define DEFAULT_NUNCHUK_MULIPLIER 2

#define DEFAULT_ACCEL 100000
#define DEFAULT_RA_GUIDE 0.5

#define DEFAULT_SPEED 30000
// this is in hz. In millihz this would be 30,000,000
#define DEFAULT_RA_MIDDLE_DISTANCE 62

// This value is the tuned value
#define DEFAULT_RA_LEAD_SCREW_TO_PIVOT 448.0

#define DEFAULT_DEC_MIDDLE_DISTANCE 32
//TODO Measure
#define DEFAULT_DEC_LEAD_SCREW_TO_PIVOT 600 
// 482.5; // And this is the value by design in 3d model

void setupWebServer(MotorUnit &motor, RAStatic &raStatic, RADynamic &raDynamic,
                    DecStatic &decStatic, DecDynamic &decDynamic,
                    Preferences &prefs);
#endif
