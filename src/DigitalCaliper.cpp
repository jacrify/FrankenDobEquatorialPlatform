#include "DigitalCaliper.h"
#include "Logging.h"
#include <driver/adc.h>

DigitalCaliper *DigitalCaliper::dpInstance = nullptr;
long cycleStart = micros();
long lastReadTime = micros();
int bitIndex = 0;
long packet = 0;
long microns = 0;
std::string bitString = "";


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

void IRAM_ATTR clockFall() {
  DigitalCaliper::dpInstance->interruptHandler();
}

void DigitalCaliper::interruptHandler() {
  int data = digitalRead(DATAPIN);
  long now = micros();

  bitString = (data ? "1" : "0") + bitString;
  packet |= (data & 1) << bitIndex;
  bitIndex++;

  long timeInCycle = now - cycleStart;
  long timeSinceLast = now - lastReadTime;

  lastReadTime = now;
  if (timeSinceLast > 600) { // nibble start
    bitString = " " + bitString;
  }
  if (timeInCycle > 90000) {
    cycleStart = micros();
    bitIndex = 0;

    loga(&bitString[0]);
    microns=getMicrons(packet);
    loga("Packet Value: %d which is microns: %d", packet, microns);
    packet = 0;
    bitString = "";
  }
  // loga("%d %d clockfall d1 %d ", timeInCycle, timeSinceLast, data1);
}

DigitalCaliper::DigitalCaliper() { dpInstance = this; }

void DigitalCaliper::setUp() {
  
  pinMode(DATAPIN, INPUT);
  pinMode(CLOCKPIN, INPUT);

  attachInterrupt(CLOCKPIN, clockFall, FALLING);

  dacWrite(DACPIN, DACLEVEL); // power the calipters
}

long DigitalCaliper::getPosition() { return positions.front(); }

unsigned long DigitalCaliper::getTime() { return times.front(); }

float DigitalCaliper::getVelocity() { return velocities.front(); }

void DigitalCaliper::reset() {
  dacWrite(DACPIN, 0);
  delay(ONOFFDELAY);
  dacWrite(DACPIN, DACLEVEL);
  // clear();
  // vTaskResume(TaskHandle);
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
  errorCount = 0;
}

void DigitalCaliper::sleepBetweenSamples() { delay(SAMPLEDELAY); }

void DigitalCaliper::takeSample() {

  if (packet < 0) {
    return;
  }

  unsigned long now = millis();

  if ((microns > MAX_SENSIBLE_POS) || (microns < MIN_SENSIBLE_POS)) {
    log("Dropping sample as out of sensible position range, actual position "
        "reported is %d",
        microns);
    errorCount++;
    return;
  }

  times.push(now);
  positions.push(microns);

  if (times.size() > 0) {
    unsigned long deltaTime = now - times.back(); // oldest time
    long deltaPos = microns - positions.back();

    if (abs(deltaPos) < SENSIBLE_MOVEMENT) {
      log("Dropping sample as out of sensible movement range, actual position "
          "reported is %d",
          microns);
      errorCount++;
      return;
    }

    float velocity =
        (float)deltaPos / (float)deltaTime * 100.0; // microns per second ??
    log("Delta time: %d \t delta pos %d \t current pos %d \t Velocity: %f",
        deltaTime, deltaPos, microns, velocity);
    velocities.push(velocity);
  }
}
