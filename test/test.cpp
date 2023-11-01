
#include "Logging.h"
#include "PlatformStatic.h"
#include "RADynamic.h"
#include <cstdint>

#include "StepperWrapper.h"
#include "cpp_mock.h"
#include <stdexcept>
#include <unity.h>

void test_timetomiddle_calc(void) {
  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  double time_to_center =
      model.calculateTimeToCenterInSeconds(stepPositionOfMiddle);
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0, time_to_center,
                                  "time to middle should be zero");
  time_to_center = model.calculateTimeToCenterInSeconds(
      stepPositionOfMiddle + 7200); // about a minute from middle
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(61.38793, time_to_center,
                                  "time to middle should be positive");

  time_to_center = model.calculateTimeToCenterInSeconds(
      stepPositionOfMiddle - 7200); // about a minute from middle
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(-61.38793, time_to_center,
                                  "time to middle should be positive");

  double time_to_end =
      model.calculateTimeToEndOfRunInSeconds(stepPositionOfMiddle);
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(2071.392, time_to_end,
                                  "time to end should be 34 minutes");

  time_to_end = model.calculateTimeToEndOfRunInSeconds(0);
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0, time_to_end, "time to end should be 0");
  time_to_end =
      model.calculateTimeToEndOfRunInSeconds(stepPositionOfMiddle + 100);
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(2072.226, time_to_end,
                                  "time to end should be biggers");
}

void test_speed_calc(void) {
  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);
  log("====test_speed_calc====");
  uint32_t speedInMilliHz =
      model.calculateFowardSpeedInMilliHz(stepPositionOfMiddle);

  // we expect to go abut 1 turn per minute
  // one turn is 2mm
  // so 2mm*steps per mm pulses
  // or 7200 pulses per minute
  // 7200/60=120
  // 120 pulses sec?
  // 120 hz
  // 120000 mhz?

  TEST_ASSERT_EQUAL_INT_MESSAGE(117606, speedInMilliHz,
                                "Speed in middle wrong");

  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfLimit);

  TEST_ASSERT_EQUAL_INT_MESSAGE(119857, speedInMilliHz,
                                "Should be faster at limit: approx 1.8%");

  model.setConeRadiusAtAttachmentPoint(447);
  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfMiddle);

  TEST_ASSERT_EQUAL_INT_MESSAGE(117344, speedInMilliHz,
                                "Smaller great circle should be slower speed");

  model.setConeRadiusAtAttachmentPoint(449);
  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfMiddle);

  TEST_ASSERT_EQUAL_INT_MESSAGE(117869, speedInMilliHz,
                                "Larger great circle should be faster speed");

  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(61); // smaller so closer to middle

  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfLimit);

  TEST_ASSERT_EQUAL_INT_MESSAGE(
      119785, speedInMilliHz,
      "If limit switch is closer to middle, speed should be slower at limit "
      "than 62mm value of 119461");

  model.setLimitSwitchToMiddleDistance(63);
  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfLimit);

  TEST_ASSERT_EQUAL_INT_MESSAGE(
      119931, speedInMilliHz,
      "If limit switch is closer to middle, speed should be faster at limit "
      "than  62mm value of 119461");
}
void test_rewind_fast_forward_speed_calc() {
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(62);
  model.setRewindFastFowardSpeedInHz(30000);

  // sidereal we expect to go abut 1 turn per minute
  // one turn is 2mm
  // so 2mm*steps per mm pulses
  // or 7200 pulses per minute
  // 7200/60=120
  // 120 pulses sec?
  // 120 hz
  // @ 120 hz, platform moves across sky @ 360 degrees in 24h
  // = 360/86400 degrees sec
  // = 0.00416667 degrees sec
  // rewind/ff speed is 30000 hz so 250 times faster
  // or 1.0416675 degrees sec

  //

  double rewindFastForwardSpeedDegreesSec =
      model.getMaxAxisMoveRateDegreesSec();
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(
      .1, 1.0416675, rewindFastForwardSpeedDegreesSec,
      "Rewind/fast-forward speed calculation is incorrect");
}

class MockStepper : public StepperWrapper {
public:
  MockMethod(void, resetPosition, (int32_t));
  MockMethod(void, moveTo, (int32_t, uint32_t));
  MockMethod(void, stop, ());
  MockMethod(int32_t, getPosition, ());
  MockMethod(void, setStepperSpeed, (uint32_t));
  MockMethod(uint32_t, getStepperSpeed, ());
};

void testGotoMiddleBasic() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  When(stepper.getPosition).Return(model.getMiddlePosition() - 100);
  // test going to middle
  control.setLimitSwitchState(false);
  control.gotoMiddle();
  control.calculateOutput();

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getMiddlePosition(),
                                  control.getTargetPosition(),
                                  "Target Position should be middle");
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getRewindFastFowardSpeedInMilliHz(),
                                  control.getTargetSpeedInMilliHz(),
                                  "Target speed should be ff rw");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testGotoMiddleLimitSwitch() {

  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test going to middle
  try {
    control.setTrackingOnOff(false);
    control.setLimitSwitchState(true);
    control.gotoMiddle();
    control.calculateOutput();
    Verify(stepper.resetPosition).Times(1);
    Verify(stepper.stop).Times(0);
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getMiddlePosition(),
                                  control.getTargetPosition(),
                                  "Target Position should be middle");
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getRewindFastFowardSpeedInMilliHz(),
                                  control.getTargetSpeedInMilliHz(),
                                  "Target speed should be ff rw");

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testGotoMiddleAtMiddle() {

  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test going to middle, when at middle
  try {
    control.setTrackingOnOff(false);
    control.setLimitSwitchState(false);
    control.gotoMiddle();
    When(stepper.getPosition).Return(model.getMiddlePosition());
    control.calculateOutput();
    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(1);
    Verify(stepper.moveTo).Times(0);
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getMiddlePosition(),
                                  control.getTargetPosition(),
                                  "Target Position should be middle");
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getRewindFastFowardSpeedInMilliHz(),
                                  control.getTargetSpeedInMilliHz(),
                                  "Target speed should be ff rw");

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testGotoMiddleAtMiddleResumeTracking() {

  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test going to middle, when at middle. Tracking should restart
  try {
    control.setTrackingOnOff(true);
    control.setLimitSwitchState(false);
    control.gotoMiddle();
    When(stepper.getPosition).Return(model.getMiddlePosition());
    control.calculateOutput();
    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, control.getTargetPosition(),
                                  "Target Position should be end");
    TEST_ASSERT_TRUE_MESSAGE(control.getTargetSpeedInMilliHz() > 0,
                             "Speed should be greater than 0");

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testGotoStartBasic() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test going to start
  control.setLimitSwitchState(false);
  // safetymode runs at 1/3 speed when limit pos not known
  control.setSafetyMode(false);
  control.gotoStart();
  control.calculateOutput();

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getLimitSwitchSafetyStandoffPosition(),
                                  control.getTargetPosition(),
                                  "Target position should be limit standoff");
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getRewindFastFowardSpeedInMilliHz(),
                                  control.getTargetSpeedInMilliHz(),
                                  "Target speed should be ff rw");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testGotoStartLimit() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test going to start when limit hit
  control.setLimitSwitchState(true);
  When(stepper.getPosition).Return(model.getLimitPosition());
  control.gotoStart();
  control.calculateOutput();

  try {
    Verify(stepper.moveTo).Times(0);
    Verify(stepper.resetPosition).Times(1);
    Verify(stepper.stop).Times(1);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testGotoStartLimitTracking() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test going to start when limit hit
  When(stepper.getPosition).Return(model.getLimitPosition());
  control.setLimitSwitchState(true);
  control.setTrackingOnOff(true);

  control.gotoStart();
  control.calculateOutput();

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, control.getTargetPosition(),
                                  "Target Position should be end");
    TEST_ASSERT_TRUE_MESSAGE(control.getTargetSpeedInMilliHz() > 0,
                             "Speed should be greater than 0");

    Verify(stepper.resetPosition).Times(1);
    Verify(stepper.stop).Times(0);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testGotoEndBasic() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test going to end
  control.setLimitSwitchState(false);
  When(stepper.getPosition).Return(model.getLimitPosition());
  control.gotoEndish();
  control.calculateOutput();

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getEndStandOffPosition(),
                                  control.getTargetPosition(),
                                  "Target Position should be endish");
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getRewindFastFowardSpeedInMilliHz(),
                                  control.getTargetSpeedInMilliHz(),
                                  "Target speed should be ff rw");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testGotoEndLimitHit() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test going to end
  control.setLimitSwitchState(true);
  When(stepper.getPosition).Return(model.getLimitPosition());
  control.gotoEndish();
  control.calculateOutput();

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getEndStandOffPosition(),
                                  control.getTargetPosition(),
                                  "Target Position should be endish");
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getRewindFastFowardSpeedInMilliHz(),
                                  control.getTargetSpeedInMilliHz(),
                                  "Target speed should be ff rw");

    Verify(stepper.resetPosition).Times(1);
    Verify(stepper.stop).Times(0);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testGotoEndAtEndWithTracking() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test going to end
  control.setLimitSwitchState(false);
  When(stepper.getPosition).Return(0);
  control.setTrackingOnOff(true);
  control.gotoEndish();
  control.calculateOutput();

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getEndStandOffPosition(),
                                  control.getTargetPosition(),
                                  "Target Position should be endish");
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getRewindFastFowardSpeedInMilliHz(),
                                  control.getTargetSpeedInMilliHz(),
                                  "Target speed should be ff rw");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testCalculateMoveByDegrees() {
  // MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;

  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  uint32_t target = model.calculatePositionByDegreeShift(0, 0);
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, target, "Target should be end");

  target = model.calculatePositionByDegreeShift(0, stepPositionOfLimit);
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(468000, target, "Target should be limit");

  target = model.calculatePositionByDegreeShift(-1, stepPositionOfLimit);
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(
      439378, target, "Target should be smaller than stepPositionOfLimit");

  target = model.calculatePositionByDegreeShift(-15, stepPositionOfLimit);
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(43323, target,
                                   "Target should be close to zero");

  target = model.calculatePositionByDegreeShift(-20, stepPositionOfLimit);
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, target,
                                   "Target should be  to zero as off end");
  target = model.calculatePositionByDegreeShift(20, 0);
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(stepPositionOfLimit, target,
                                   "Target should be  to limit as off end");
}

void testMoveAxisPositive() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test moveaxis
  control.setLimitSwitchState(false);
  control.setTrackingOnOff(false);
  When(stepper.getPosition).Return(model.getMiddlePosition());
  // positive is east, away from tracking direction
  control.moveAxis(0.004178);
  //  sidereal
  control.calculateOutput();

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(INT32_MAX, control.getTargetPosition(),
                                  "Target Position should be start");
    TEST_ASSERT_EQUAL_INT_MESSAGE(
        117605, control.getTargetSpeedInMilliHz(),
        "Target speed should be some fudged version of sidereal");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

    // now turn tracking on and run again
    control.setTrackingOnOff(true);
    control.moveAxis(0.005178);
    control.calculateOutput();

    Verify(stepper.moveTo).Times(2);
    TEST_ASSERT_EQUAL_INT_MESSAGE(INT32_MAX, control.getTargetPosition(),
                                  "Target Position should be start");
    TEST_ASSERT_EQUAL_INT_MESSAGE(28147, control.getTargetSpeedInMilliHz(),
                                  "Target speed should be almost 0");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

    // now turn tracking off and run backward
    control.setTrackingOnOff(false);
    control.moveAxis(-0.004178);
    control.calculateOutput();

    Verify(stepper.moveTo).Times(3);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, control.getTargetPosition(),
                                  "Target Position should be end");
    // TEST_ASSERT_TRUE_MESSAGE(control.getTargetPosition() >
    //                              model.getLimitPosition(),
    //                          "Target Position should be start");
    TEST_ASSERT_EQUAL_INT_MESSAGE(
        117605, control.getTargetSpeedInMilliHz(),
        "Target speed should be some fudge of sidereal");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

    // now turn tracking on and run backward
    control.setTrackingOnOff(true);
    control.moveAxis(-0.004178);
    control.calculateOutput();

    Verify(stepper.moveTo).Times(4);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, control.getTargetPosition(),
                                  "Target Position should be end");
    // TEST_ASSERT_TRUE_MESSAGE(control.getTargetPosition() >
    //                              model.getLimitPosition(),
    //                          "Target Position should be start");
    TEST_ASSERT_EQUAL_INT_MESSAGE(235211, control.getTargetSpeedInMilliHz(),
                                  "Target speed should be double sidereal");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testGotoStartSafety() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformStatic model;
  model.setConeRadiusAtAttachmentPoint(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  RADynamic control = RADynamic(model);
  control.setStepperWrapper(&stepper);

  // test going to start
  control.setLimitSwitchState(false);
  control.gotoStart();
  control.calculateOutput();

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getLimitSwitchSafetyStandoffPosition(),
                                  control.getTargetPosition(),
                                  "Target position should be limit standoff");
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getRewindFastFowardSpeedInMilliHz(),
                                  control.getTargetSpeedInMilliHz(),
                                  "Target speed should be ff rw");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void setup() {

  UNITY_BEGIN(); // IMPORTANT LINE!
  RUN_TEST(test_speed_calc);
  RUN_TEST(test_rewind_fast_forward_speed_calc);
  RUN_TEST(test_timetomiddle_calc);
  RUN_TEST(testGotoMiddleBasic);
  RUN_TEST(testGotoMiddleLimitSwitch);
  RUN_TEST(testGotoMiddleAtMiddle);
  RUN_TEST(testGotoMiddleAtMiddleResumeTracking);
  RUN_TEST(testGotoStartBasic);
  RUN_TEST(testGotoStartLimit);
  RUN_TEST(testGotoStartLimitTracking);
  RUN_TEST(testGotoEndBasic);
  RUN_TEST(testGotoEndLimitHit);
  RUN_TEST(testGotoEndAtEndWithTracking);
  RUN_TEST(testMoveAxisPositive);
  RUN_TEST(testCalculateMoveByDegrees);
  UNITY_END(); // IMPORTANT LINE!
}

void loop() {
  // Do nothing here.
}

int main() {
  setup();
  return 0;
}
