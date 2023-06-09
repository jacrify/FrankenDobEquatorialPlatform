#ifndef DIGITAL_CALIPER_H
#define DIGITAL_CALIPER_H
#include "Logging.h"
#include <MovingAveragePlus.h>
#include <driver/adc.h>
#include <functional>


// Packet format: [0 .. 19]=data, 20=sign, [21..22]=unused?, 23=inch

// minimum reading
#define MIN_RANGE -(1 << 20)

#define STARTING_WINDOW_SIZE 40
#define SAMPLEDELAY 1000 // milliseconds? (check)
#define SENSIBLE_SPEED 0.5
#define SENSIBLE_SHIFT 1500
#define MAX_SENSIBLE_POS 7000
#define MIN_SENSIBLE_POS -7000
#define MIN_SAMPLES_FOR_VELOCITY 10

#define DATAPIN  35  // purple
#define CLOCKPIN  34 // grey
//  blue is negative: goes to gnd on pin 2
#define DACPIN 25      // white. Controls power to the caliper
#define DACLEVEL  191    // Need 1.6 volts. Supply=3.5. 255*1.6/3.5
#define ONOFFDELAY  1000 // pulse on off for reset

class DigitalCaliper {
public:
  MovingAveragePlus<long> positions{STARTING_WINDOW_SIZE};
  MovingAveragePlus<unsigned long> times{STARTING_WINDOW_SIZE};
  MovingAveragePlus<float> velocities{STARTING_WINDOW_SIZE};
  int errorCount;

  DigitalCaliper();
  long getPosition();
  unsigned long getTime();
  float getVelocity();
  int getErrorCount();
  void setUp();
  void reset();
  void clear();
  void sleepBetweenSamples();
  void setWindowSize(int size); // resize and clear averages
  void interruptHandler();
  void takeSample();
  static DigitalCaliper *dpInstance;

private:
  
  // convert a packet to signed microns
  long getMicrons(long packet);
  TaskHandle_t TaskHandle;
};


#endif