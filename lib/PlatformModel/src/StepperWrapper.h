#ifndef __STEPPERWRAPPER_H__
#define __STEPPERWRAPPER_H__
#include <cstdint>

/**
 * A simple wrapper of stepper based function.
 * Used in unit tests to mock.
 */
class StepperWrapper {
public:
  virtual void moveTo(int32_t position, uint32_t speedInMillihz) = 0;
  // virtual void moveAndResetPosition(int32_t positionToMoveTo, int32_t positionToResetTo);
  virtual void resetPosition(int32_t position) = 0;
  virtual void stop() = 0;
  virtual int32_t getPosition() = 0;
  virtual void setStepperSpeed(uint32_t speedInMillihz) = 0;
  virtual uint32_t getStepperSpeed() = 0;
  virtual void setAcceleration(unsigned long a) = 0;
};
#endif // __STEPPERWRAPPER_H__