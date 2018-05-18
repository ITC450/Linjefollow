#pragma once

#include <wiringPi.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

#define kp 0.0065
#define ki 0.0003
#define kd 0.0006

using namespace cv;

enum direction {BACK , FORWARD};

void MotorInit();

void LeftMotor(direction dir, int speed, Mat mat, int rows, int cols);

void RightMotor(direction dir, int speed, Mat mat, int rows, int cols);

double pid(int err, std::chrono::time_point<std::chrono::high_resolution_clock> &pid_start);

void MotorFollowLine(int err, Mat mat, int rows, int cols, int speed, std::chrono::time_point<std::chrono::high_resolution_clock> &start);
