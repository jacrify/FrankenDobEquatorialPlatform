#ifndef MYWEBSERVER_H
#define MYWEBSERVER_H
#include <ESPAsyncWebServer.h>
#include "DigitalCaliper.h"

AsyncWebServer setupWebServer(DigitalCaliper &caliper);
#endif
