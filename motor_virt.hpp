#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

enum direction {BACK , FORWARD};

void MotorInit();

void LeftMotor(direction dir, int speed, Mat mat, int rows, int cols);

void RightMotor(direction dir, int speed, Mat mat, int rows, int cols);