#include "motor_virt.hpp"

using namespace cv;

void MotorInit()
{
    std::cout << "Not setting up WiringPi" << '\n';
}

void LeftMotor(direction dir, int speed, Mat mat, int rows, int cols)
{
    if (dir == BACK) {
        arrowedLine(mat, Point2f(cols/8,rows/2),     Point2f(cols/8,    (rows/2)+speed), Scalar(0,255-speed,speed), 1, 8, 0);
    }else{
        arrowedLine(mat, Point2f(cols/8,rows/2),     Point2f(cols/8,    (rows/2)-speed), Scalar(0,255-speed,speed), 1, 8, 0);
    }
}

void RightMotor(direction dir, int speed, Mat mat, int rows, int cols)
{
    if (dir == BACK) {
        arrowedLine(mat, Point2f((cols/8)*7,rows/2), Point2f((cols/8)*7,(rows/2)+speed), Scalar(0,255-speed,speed), 1, 8, 0);
    }
    else{
        arrowedLine(mat, Point2f((cols/8)*7,rows/2), Point2f((cols/8)*7,(rows/2)-speed), Scalar(0,255-speed,speed), 1, 8, 0);
    }
}
