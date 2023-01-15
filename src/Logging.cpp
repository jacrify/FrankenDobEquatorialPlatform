#include <Arduino.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>

void log(const char *fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, 128, fmt, args);
  va_end(args);
  Serial.println(buf);
}
