#include "MotorUnit.h"
#include <unity.h> // Include the Unity test framework.



void test_motor_unit_speed(void) {
  
  TEST_ASSERT_EQUAL_FLOAT(2,
                          3); // Replace 'expected_value' with
                                               // the value you're expecting.
}

void setup() {
  UNITY_BEGIN(); // IMPORTANT LINE!
  RUN_TEST(test_motor_unit_speed);
  UNITY_END(); // IMPORTANT LINE!
}

void loop() {
  // Do nothing here.
}

int main() {
  setup();
return 0;
}
