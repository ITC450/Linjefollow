#include "motor.hpp"
#include <iostream>
int main(int argc, char const *argv[]) {
  MotorInit();
  LeftMotor(BACK, 512);
  return 0;
}
