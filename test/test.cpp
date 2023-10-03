
#include "Logging.h"
#include "PlatformControl.h"
#include "PlatformModel.h"
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
  PlatformModel model;
  model.setGreatCircleRadius(448);
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
  PlatformModel model;
  model.setGreatCircleRadius(448);
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

  model.setGreatCircleRadius(447);
  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfMiddle);

  TEST_ASSERT_EQUAL_INT_MESSAGE(117344, speedInMilliHz,
                                "Smaller great circle should be slower speed");

  model.setGreatCircleRadius(449);
  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfMiddle);

  TEST_ASSERT_EQUAL_INT_MESSAGE(117869, speedInMilliHz,
                                "Larger great circle should be faster speed");

  model.setGreatCircleRadius(448);
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
  PlatformModel model;
  model.setGreatCircleRadius(448);
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
};

void testGotoMiddleBasic() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  When(stepper.getPosition).Return(model.getMiddlePosition()-100);
  // test going to middle
  control.setLimitSwitchState(false);
  control.gotoMiddle();
  control.calculateOutput(0);

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
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test going to middle
  try {
    control.setTrackingOnOff(false);
    control.setLimitSwitchState(true);
    control.gotoMiddle();
    control.calculateOutput(0);
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
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test going to middle, when at middle
  try {
    control.setTrackingOnOff(false);
    control.setLimitSwitchState(false);
    control.gotoMiddle();
    When(stepper.getPosition).Return(model.getMiddlePosition());
    control.calculateOutput(0);
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
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test going to middle, when at middle. Tracking should restart
  try {
    control.setTrackingOnOff(true);
    control.setLimitSwitchState(false);
    control.gotoMiddle();
    When(stepper.getPosition).Return(model.getMiddlePosition());
    control.calculateOutput(0);
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
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test going to start
  control.setLimitSwitchState(false);
  control.gotoStart();
  control.calculateOutput(0);

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_TRUE_MESSAGE(control.getTargetPosition() >
                                 model.getLimitPosition(),
                             "Target position should be past limit");
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
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test going to start when limit hit
  control.setLimitSwitchState(true);
  When(stepper.getPosition).Return(model.getLimitPosition());
  control.gotoStart();
  control.calculateOutput(0);

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
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test going to start when limit hit
  When(stepper.getPosition).Return(model.getLimitPosition());
  control.setLimitSwitchState(true);
  control.setTrackingOnOff(true);

  control.gotoStart();
  control.calculateOutput(0);

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
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test going to end
  control.setLimitSwitchState(false);
  When(stepper.getPosition).Return(model.getLimitPosition());
  control.gotoEndish();
  control.calculateOutput(0);

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, control.getTargetPosition(),
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
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test going to end
  control.setLimitSwitchState(true);
  When(stepper.getPosition).Return(model.getLimitPosition());
  control.gotoEndish();
  control.calculateOutput(0);

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, control.getTargetPosition(),
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
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test going to end
  control.setLimitSwitchState(false);
  When(stepper.getPosition).Return(0);
  control.setTrackingOnOff(true);
  control.gotoEndish();
  control.calculateOutput(0);

  try {
    Verify(stepper.moveTo).Times(0);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, control.getTargetPosition(),
                                  "Target Position should be endish");
    TEST_ASSERT_EQUAL_INT_MESSAGE(model.getRewindFastFowardSpeedInMilliHz(),
                                  control.getTargetSpeedInMilliHz(),
                                  "Target speed should be ff rw");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(1);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testMoveAxisPositive() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test moveaxis
  control.setLimitSwitchState(false);
  control.setTrackingOnOff(false);
  When(stepper.getPosition).Return(model.getMiddlePosition());
  control.moveAxis(0.004178);
  //  sidereal
  control.calculateOutput(0);

  try {
    Verify(stepper.moveTo).Times(1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, control.getTargetPosition(),
                                  "Target Position should be end");
    TEST_ASSERT_EQUAL_INT_MESSAGE(
        117605, control.getTargetSpeedInMilliHz(),
        "Target speed should be some fudged version of sidereal");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

    // now turn tracking on and run again
    control.setTrackingOnOff(true);
    control.moveAxis(0.005178);
    control.calculateOutput(0);

    Verify(stepper.moveTo).Times(2);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, control.getTargetPosition(),
                                  "Target Position should be end");
    TEST_ASSERT_EQUAL_INT_MESSAGE(28147, control.getTargetSpeedInMilliHz(),
                                  "Target speed should be almost 0");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

    // now turn tracking off and run backward
    control.setTrackingOnOff(false);
    control.moveAxis(-0.004178);
    control.calculateOutput(0);

    Verify(stepper.moveTo).Times(3);
    TEST_ASSERT_TRUE_MESSAGE(control.getTargetPosition() >
                                 model.getLimitPosition(),
                             "Target Position should be start");
    TEST_ASSERT_EQUAL_INT_MESSAGE(
        117605, control.getTargetSpeedInMilliHz(),
        "Target speed should be some fudge of sidereal");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

    // now turn tracking on and run backward
    control.setTrackingOnOff(true);
    control.moveAxis(-0.004178);
    control.calculateOutput(0);

    Verify(stepper.moveTo).Times(4);
    TEST_ASSERT_TRUE_MESSAGE(control.getTargetPosition() >
                                 model.getLimitPosition(),
                             "Target Position should be start");
    TEST_ASSERT_EQUAL_INT_MESSAGE(235211, control.getTargetSpeedInMilliHz(),
                                  "Target speed should be double sidereal");

    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(0);

  } catch (std::runtime_error e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void testOffsetAccumulation() {
  // setup
  MockStepper stepper;

  int runTotal = 130;                         // mm
  int limitToMiddle = 62;                     // mm
  int middleToEnd = runTotal - limitToMiddle; // mm

  int stepPositionOfMiddle = middleToEnd * 3600;
  int stepPositionOfLimit = runTotal * 3600;
  PlatformModel model;
  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(limitToMiddle);
  model.setRewindFastFowardSpeedInHz(30000);

  PlatformControl control = PlatformControl(model);
  control.setStepperWrapper(&stepper);

  // test going to middle
  control.setLimitSwitchState(false);
  // position is before middle
  When(stepper.getPosition).Return(model.getMiddlePosition() + 5000);
  control.gotoMiddle();
  control.calculateOutput(0);

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
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.1, 0, control.getPlatformResetOffset(),
                                     "Reset offset should be 0");
    // now mark position as middle, offset should be calculated
    When(stepper.getPosition).Return(model.getMiddlePosition());
    control.calculateOutput(0);
    Verify(stepper.moveTo).Times(1);
    Verify(stepper.resetPosition).Times(0);
    Verify(stepper.stop).Times(1); // should stop
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5, 42.5,
                                     control.getPlatformResetOffset(),
                                     "Reset offset should be postive");

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
  RUN_TEST(testOffsetAccumulation);
  UNITY_END(); // IMPORTANT LINE!
}

void loop() {
  // Do nothing here.
}

int main() {
  setup();
  return 0;
}
