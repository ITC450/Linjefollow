#pragma once

#include <wiringPi.h>
#include <iostream>

enum direction {BACK , FORWARD};

void MotorInit();

void LeftMotor(direction dir, int speed);

void RightMotor(direction dir, int speed);
