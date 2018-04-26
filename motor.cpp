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
        //arrowedLine(mat, Point2f(cols/8,rows/2),     Point2f(cols/8,    (rows/2)+speed), Scalar(0,255-speed,speed), 1, 8, 0);
    }else{
        digitalWrite(2,LOW);
        digitalWrite(3,HIGH);
        //arrowedLine(mat, Point2f(cols/8,rows/2),     Point2f(cols/8,    (rows/2)-speed), Scalar(0,255-speed,speed), 1, 8, 0);
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

double radius(double error){
  double r = (pow(Q,2)+pow(error,2))/(2*error);
  return r;
}

int pwmSpeed(double v){
  double ps=-80.64516129 * log((-2189*v+1488.66558)/(5000*v+2189));
  return int(ps);
}

double highV(int speed, double radius){
  double hv =(speed*(AKSEL + 2*radius))/(2*radius);
  return hv;
}

double lowV(int speed, double hv){
  double lv = 2*speed-hv;
  return lv;
}

void speedCheck(double &hv, double &lv, double &radius){
  if (hv > 0.65){
    hv = 0.65;
    lv = (-(AKSEL-2*radius)*hv)/(AKSEL+2*radius);
  }
  if (lv < 0.02){
    lv = 0.02;
    hv = (-lv*(AKSEL+2*radius))/(AKSEL-2*radius);
  }
}
//Follow line function
void MotorFollowLine(int err, Mat mat, int rows, int cols, int speed){
    double error = abs(errDist(err));
    //std::cout << error << "\n";
    double rad = radius(error);
    double hv = highV(speed, rad);
    double lv = lowV(speed, hv);
    speedCheck(hv,lv,rad);
    if(err < 0) {
        LeftMotor(FORWARD, pwmSpeed(lv), mat, rows, cols);
        RightMotor(FORWARD, pwmSpeed(hv), mat, rows, cols);
        return;
    }
    if(err > 0) {
        RightMotor(FORWARD, pwmSpeed(hv), mat, rows, cols);
        LeftMotor(FORWARD, pwmSpeed(lv), mat, rows, cols);
        return;
    }
    if(err == 0) {
        RightMotor(FORWARD, speed, mat, rows, cols);
        LeftMotor(FORWARD, speed, mat, rows, cols);
        return;
    }
}
