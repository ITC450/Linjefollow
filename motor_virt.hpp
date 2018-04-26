#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

#define errDist(err) (0.0004309*err-0.007083)
#define Q 0.168
#define AKSEL 0.12

using namespace cv;

enum direction {BACK , FORWARD};

void MotorInit();

void LeftMotor(direction dir, int speed, Mat mat, int rows, int cols);

void RightMotor(direction dir, int speed, Mat mat, int rows, int cols);

void MotorFollowLine(int err, Mat mat, int rows, int cols, double speed);

double radius(double error);

int pwmSpeed(double v);

double highV(double speed, double radius);

double lowV(double speed, double hv);

void speedCheck(double &hv, double &lv, double &radius);
