#ifndef PLATFORM_MODEL_H
#define PLATFORM_MODEL_H
#include "MotorStatic.h"
#include <cstdint>

// Represents the static attributes of the dec axix.
// Use to perform calculations using intrinsic platform attributes
// Does not hold dynamic state of the platform, ie position, as that comes from
// the motor

class DecStatic : public MotorStatic {
public:
   DecStatic();

private:

};

#endif