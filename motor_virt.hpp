#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

#define kp 0.5
#define ki 0.01
#define kd 0.01

using namespace cv;

enum direction {BACK , FORWARD};

void MotorInit();

void LeftMotor(direction dir, int speed, Mat mat, int rows, int cols);

void RightMotor(direction dir, int speed, Mat mat, int rows, int cols);

double pid(int err);

void MotorFollowLine(int err, Mat mat, int rows, int cols, int speed);
