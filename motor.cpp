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

double inte = 0;
//std::chrono::time_point start;
//std::chrono::time_point end;
int last_err = 0;

double pid(int err, std::chrono::time_point<std::chrono::high_resolution_clock> &pid_start) {
    err -= 8;
    err = abs(err);
    auto pid_end = std::chrono::system_clock::now();
    std::chrono::duration<double, std::milli> dur = pid_end - pid_start;
    pid_start = std::chrono::system_clock::now();
    double Pout = kp * err; // P delen udregnes
  //  auto end = std::chrono::high_resolution_clock::now();
    //auto result = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    inte +=  err * (dur.count()/1000);  // I delen udregnes
  //  start = std::chrono::high_resolution_clock::now();
    double Iout = ki * inte;
    double derivative = (err - last_err)/dur.count();
    double Dout = kd * derivative;

    double output = Pout + Iout; //+ Dout;
    //if (output > 450)output = 450; // sørger for output holder sig inden for range
    //else if (output < 50)output = 50; // myPID.Compute();

    last_err=err;
    return output;
}

//Follow line function
void MotorFollowLine(int err, Mat mat, int rows, int cols, int speed, std::chrono::time_point<std::chrono::high_resolution_clock> &pid_start){
    double error = pid(err, pid_start);
    std::cout << error << "\n";
    if(err < 0) {
        LeftMotor(FORWARD, speed - int(abs(error)), mat, rows, cols);
        RightMotor(FORWARD, speed + int(abs(error)), mat, rows, cols);
        return;
    }
    if(err > 0) {
        RightMotor(FORWARD, speed - int(abs(error)), mat, rows, cols);
        LeftMotor(FORWARD, speed + int(abs(error)), mat, rows, cols);
        return;
    }
    if(err == 0) {
        RightMotor(FORWARD, speed, mat, rows, cols);
        LeftMotor(FORWARD, speed, mat, rows, cols);
        return;
    }
}
