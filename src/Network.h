#ifndef NETWORK_H
#define NETWORK_H
#include <Preferences.h>

void setupWifi(Preferences &prefs);
void loopNetwork(Preferences &prefs);
void broadcastStatus(double secondsToCenter, double secondsToEnd,
                     bool platformTracking);
#endif
