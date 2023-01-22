#ifndef DIGITAL_CALIPER_H
#define DIGITAL_CALIPER_H
#include "Logging.h"
#include <MovingAveragePlus.h>
#include <driver/adc.h>
#include <functional>

// ADC threshold for 1.5V SPCsignals (at 6dB/11-bit, high comes to around 1570
// in analogRead() )
#define ADC_TRESHOLD 800
// timeout in milliseconds for a bit read ($TODO - change to micros() )
#define BIT_READ_TIMEOUT 100
// timeout for a packet read
#define PACKET_READ_TIMEOUT 250
// Packet format: [0 .. 19]=data, 20=sign, [21..22]=unused?, 23=inch
#define PACKET_BITS 24
// minimum reading
#define MIN_RANGE -(1 << 20)

#define STARTING_WINDOW_SIZE 10
#define SAMPLEDELAY 1000 // milliseconds
#define SENSIBLE_MOVEMENT 1000

class DigitalCaliper {
public:
  MovingAveragePlus<long> positions{STARTING_WINDOW_SIZE};
  MovingAveragePlus<unsigned long> times{STARTING_WINDOW_SIZE};
  MovingAveragePlus<float> velocities{STARTING_WINDOW_SIZE};

  DigitalCaliper();
  long getPosition();
  unsigned long getTime();
  float getVelocity();
  void reset();
  void clear();
  void takeSample();
  void sleepBetweenSamples();
  void setWindowSize(int size); // resize and clear averages


private:
  int dataPin = 35;  // purple
  int clockPin = 34; // grey
  //  blue is negative: goes to gnd on pin 2
  int dacPin = 25;       // white. Controls power to the caliper
  int dacLevel = 116;    // Need 1.6 volts. Supply=3.5. 255*1.6/3.5
  int onOffDelay = 1000; // pulse on off for reset

  // capped read: -1 (timeout), 0, 1
  int getBit();

  // read one full packet
  long getPacket();

  // convert a packet to signed microns
  long getMicrons(long packet);

  TaskHandle_t TaskHandle;
};
void sampleLoop(void *parameter);

#endif