#include "motor.hpp"

void MotorInit() {
    std::cout << "Setting up WiringPi" << '\n';
    if (wiringPiSetup() == -1)
    {
        std::cout << "WiringPi setup failed" << '\n';
        exit(1);
    }
    pinMode(1, PWM_OUTPUT);
    pinMode(23, PWM_OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(24, OUTPUT);
    pinMode(25, OUTPUT);
}

void LeftMotor(direction dir, int speed, Mat mat, int rows, int cols) {
    if (dir == BACK)
    {
        digitalWrite(2,HIGH);
        digitalWrite(3,LOW);
        arrowedLine(mat, Point2f(cols/8,rows/2),     Point2f(cols/8,    (rows/2)+speed), Scalar(0,255-speed,speed), 1, 8, 0);
    }else{
        digitalWrite(2,LOW);
        digitalWrite(3,HIGH);
        arrowedLine(mat, Point2f(cols/8,rows/2),     Point2f(cols/8,    (rows/2)-speed), Scalar(0,255-speed,speed), 1, 8, 0);
    }
    pwmWrite(1,speed);
}

void RightMotor(direction dir, int speed, Mat mat, int rows, int cols) {
    if (dir == BACK)
    {
        digitalWrite(25,HIGH);
        digitalWrite(24,LOW);
        arrowedLine(mat, Point2f((cols/8)*7,rows/2), Point2f((cols/8)*7,(rows/2)+speed), Scalar(0,255-speed,speed), 1, 8, 0);
    }else{
        digitalWrite(25,LOW);
        digitalWrite(24,HIGH);
        arrowedLine(mat, Point2f((cols/8)*7,rows/2), Point2f((cols/8)*7,(rows/2)-speed), Scalar(0,255-speed,speed), 1, 8, 0);
    }
    pwmWrite(23,speed);
}
