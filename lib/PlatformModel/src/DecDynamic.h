#ifndef __DECDYNAMIC_H__
#define __DECDYNAMIC_H__

#include "DecStatic.h"
#include "MotorDynamic.h"
#include "StepperWrapper.h"
#include <cstdint>

/** 
 * Responsible for the dynamic state of the dec axis.
 */
class DecDynamic : public MotorDynamic {
public:
  DecDynamic(DecStatic &m);

  void pulseGuide(int direction, long pulseDurationInMilliseconds) override;
  void moveAxis(double degreesPerSecond) override;
  void moveAxisPercentage(int percentage) override;
  void stopOrTrack(int32_t pos) override;

private:
  DecStatic &model;
};

#endif // __DECDYNAMIC_H__