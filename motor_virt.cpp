#include "motor_virt.hpp"

using namespace cv;

void MotorInit() {
    //std::cout << "Not setting up WiringPi" << '\n';
}

void LeftMotor(direction dir, int speed, Mat mat, int rows, int cols) {
    if (dir == BACK) {
        arrowedLine(mat, Point2f(cols/8,rows/2),     Point2f(cols/8,    (rows/2)+speed), Scalar(0,255-speed,speed), 2, 8, 0);
    }else{
        arrowedLine(mat, Point2f(cols/8,rows/2),     Point2f(cols/8,    (rows/2)-speed), Scalar(0,255-speed,speed), 2, 8, 0);
    }
}

void RightMotor(direction dir, int speed, Mat mat, int rows, int cols) {
    if (dir == BACK) {
        arrowedLine(mat, Point2f((cols/8)*7,rows/2), Point2f((cols/8)*7,(rows/2)+speed), Scalar(0,255-speed,speed), 2, 8, 0);
    }
    else{
        arrowedLine(mat, Point2f((cols/8)*7,rows/2), Point2f((cols/8)*7,(rows/2)-speed), Scalar(0,255-speed,speed), 2, 8, 0);
    }
}

double inte = 0;
//std::chrono::time_point start;
//std::chrono::time_point end;
int last_err = 0;

double pid(int err) {
    err -= 8;


    double Pout = kp * err; // P delen udregnes
    //  auto end = std::chrono::high_resolution_clock::now();
    //auto result = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    inte +=  err;  // I delen udregnes
    //  start = std::chrono::high_resolution_clock::now();
    double Iout = ki * inte;
    double derivative = (err - last_err);
    double Dout = kd * derivative;

    double output = Pout + Dout ;//+ Iout;
    //if (output > 450)output = 450; // sørger for output holder sig inden for range
    //else if (output < 50)output = 50; // myPID.Compute();

    last_err=err;
    return output;
}


//Follow line function
void MotorFollowLine(int err, Mat mat, int rows, int cols, int speed){
    double error = pid(err);
    //std::cout << error << "\n";
    if(err < 0) {
        LeftMotor(FORWARD, speed - int(abs(error)), mat, rows, cols);
        RightMotor(FORWARD, speed + int(abs(error)), mat, rows, cols);
        return;
    }
    if(err > 0) {
        LeftMotor(FORWARD, speed + int(abs(error)), mat, rows, cols);
        RightMotor(FORWARD, speed - int(abs(error)), mat, rows, cols);
        return;
    }
    if(err == 0) {
        RightMotor(FORWARD, speed, mat, rows, cols);
        LeftMotor(FORWARD, speed, mat, rows, cols);
        return;
    }
}
