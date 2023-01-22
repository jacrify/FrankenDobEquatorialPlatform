#include "DigitalCaliper.h"
#include "Logging.h"
#include <driver/adc.h>

DigitalCaliper::DigitalCaliper() {
  pinMode(dataPin, INPUT);
  pinMode(clockPin, INPUT);
  dacWrite(dacPin, dacLevel); // power the calipters

  analogReadResolution(11);
  analogSetAttenuation(ADC_6db);
  adc1_config_width(ADC_WIDTH_BIT_10);
}

long DigitalCaliper::getPosition() { return positions.front(); }

unsigned long DigitalCaliper::getTime() { return times.front(); }

float DigitalCaliper::getVelocity() { return velocities.front(); }

void DigitalCaliper::reset() {
  vTaskSuspend(TaskHandle);
  dacWrite(dacPin, 0);
  delay(onOffDelay);
  dacWrite(dacPin, dacLevel);
  clear();
  vTaskResume(TaskHandle);
}
void DigitalCaliper::setWindowSize(int size) {
  positions.resize(size);
  times.resize(size);
  velocities.resize(size);
  clear();
}

void DigitalCaliper::clear() {
  positions.clear();
  times.clear();
  velocities.clear();
}

void DigitalCaliper::takeSample() {
  // unsigned long lastReadTime = millis();

  long packet = getPacket();
  // not sure if I really need this block
  // if (packet < 0) {
  //   // read timeout, display?
  //   if (millis() > lastReadTime + PACKET_READ_TIMEOUT) {
  //     lastReadTime = millis();
  //   }
  // } else {
  if (packet < 0) {
    return;
  }

  unsigned long now = millis();
  long microns = getMicrons(packet);

  // if (times.size() > 0) {
  unsigned long deltaTime = now - times.back(); // oldest time
  long deltaPos = microns - positions.back();
  // TODO filter here
  if (abs(deltaPos) < SENSIBLE_MOVEMENT) {
    log("Dropping sample as out of sensible movement range, actual position "
        "reported is %d",
        microns);
    return;
  }
  times.push(now);
  positions.push(microns);
  float velocity =
      (float)deltaPos / (float)deltaTime *
      100; // microns per second
           // log("Delta time: %d \t delta pos %d \t current pos %d \t
           // Velocity: %f", deltaTime, deltaPos, microns, velocity);
  velocities.push(velocity);
}

void DigitalCaliper::sleepBetweenSamples() { delay(SAMPLEDELAY); }

int DigitalCaliper::getBit() {
  int data;
  int readTimeout = millis() + BIT_READ_TIMEOUT;
  while (analogRead(clockPin) > ADC_TRESHOLD) {
    if (millis() > readTimeout)
      return -1;
  }
  while (analogRead(clockPin) < ADC_TRESHOLD) {
    if (millis() > readTimeout)
      return -1;
  }

  data = (analogRead(dataPin) > ADC_TRESHOLD) ? 1 : 0;
  return data;
}
// read one full packet
long DigitalCaliper::getPacket() {
  long packet = 0;
  int readTimeout = millis() + PACKET_READ_TIMEOUT;

  int bitIndex = 0;
  while (bitIndex < PACKET_BITS) {
    int bit = getBit();
    if (bit < 0) {
      // bit read timeout: reset packet or bail out
      if (millis() > readTimeout) {
        // packet timeout
        return -1;
      }
      bitIndex = 0;
      packet = 0;
      continue;
    }
    packet |= (bit & 1) << bitIndex;
    bitIndex++;
  }
  return packet;
}

// convert a packet to signed microns
long DigitalCaliper::getMicrons(long packet) {
  if (packet < 0)
    return MIN_RANGE;
  long data = (packet & 0xFFFFF) * ((packet & 0x100000) ? -1 : 1);
  if (packet & 0x800000) {
    // inch value (this comes sub-sampled)
    data = data * 254 / 200;
  }
  return data;
}
