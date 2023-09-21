
#include "PlatformModel.h"
#include <cstdint>
#include <unity.h> // Include the Unity test framework.

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

  TEST_ASSERT_EQUAL_INT_MESSAGE(117286, speedInMilliHz,
                                "Speed in middle wrong");

  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfLimit);

  TEST_ASSERT_EQUAL_INT_MESSAGE(119461, speedInMilliHz,
                                "Should be faster at limit: approx 1.8%");

  model.setGreatCircleRadius(447);
  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfMiddle);

  TEST_ASSERT_EQUAL_INT_MESSAGE(117025, speedInMilliHz,
                                "Smaller great circle should be slower speed");

  model.setGreatCircleRadius(449);
  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfMiddle);

  TEST_ASSERT_EQUAL_INT_MESSAGE(117548, speedInMilliHz,
                                "Larger great circle should be faster speed");

  model.setGreatCircleRadius(448);
  model.setLimitSwitchToMiddleDistance(61); // smaller so closer to middle

  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfLimit);

  TEST_ASSERT_EQUAL_INT_MESSAGE(
      119390, speedInMilliHz,
      "If limit switch is closer to middle, speed should be slower at limit "
      "than 62mm value of 119461");

  model.setLimitSwitchToMiddleDistance(63);
  speedInMilliHz = model.calculateFowardSpeedInMilliHz(stepPositionOfLimit);

  TEST_ASSERT_EQUAL_INT_MESSAGE(
      119532, speedInMilliHz,
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

  double rewindFastForwardSpeedDegreesSec = model.getAxisMoveRate();
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(.1,
      1.0416675, rewindFastForwardSpeedDegreesSec,
      "Rewind/fast-forward speed calculation is incorrect");
}

void setup() {
  UNITY_BEGIN(); // IMPORTANT LINE!
  RUN_TEST(test_speed_calc);
  RUN_TEST(test_rewind_fast_forward_speed_calc);
  RUN_TEST(test_timetomiddle_calc);
  UNITY_END(); // IMPORTANT LINE!
}

void loop() {
  // Do nothing here.
}

int main() {
  setup();
  return 0;
}
