#include "motor.hpp"
#include <iostream>
int main(int argc, char const *argv[]) {
  MotorInit();
  RightMotor(BACK, 512);
  return 0;
}
