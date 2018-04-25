#pragma once

#include <wiringPi.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

#define errDist(err) (0.0004309*err-0.007083)
#define q
#define a

using namespace cv;

enum direction {BACK , FORWARD};

void MotorInit();

void LeftMotor(direction dir, int speed, Mat mat, int rows, int cols);

void RightMotor(direction dir, int speed, Mat mat, int rows, int cols);

void MotorFollowLine(int err, Mat mat, int rows, int cols, int speed);

double radius(double error);

uint16_t pwmSpeed(double v);

double highV(int speed, double radius);
