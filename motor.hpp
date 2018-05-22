#pragma once

#include <wiringPi.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

#define kp 0.0065
#define ki 0.0
#define kd 0.0006

using namespace cv;

enum direction {BACK , FORWARD};

void MotorInit();

void LeftMotor(direction dir, int speed);

void RightMotor(direction dir, int speed);

double pid(int err, std::chrono::time_point<std::chrono::high_resolution_clock> &pid_start);

void MotorFollowLine(int err, int speed, std::chrono::time_point<std::chrono::high_resolution_clock> &start);
