#ifndef __DECSTATIC_H__
#define __DECSTATIC_H__

#include "MotorStatic.h"
#include <cstdint>

// Represents the static attributes of the dec axix.
// Use to perform calculations using intrinsic platform attributes
// Does not hold dynamic state of the platform, ie position, as that comes from
// the motor

class DecStatic : public MotorStatic {
public:
  DecStatic();
  int32_t getGotoEndPosition();
};

#endif // __DECSTATIC_H__