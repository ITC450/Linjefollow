#include "motor_virt.hpp"

using namespace cv;

void MotorInit() {
    //std::cout << "Not setting up WiringPi" << '\n';
}

void LeftMotor(direction dir, int speed, Mat mat, int rows, int cols) {
    if (dir == BACK) {
        rectangle( mat,Point(0,rows*0.875),Point((cols/2)-1,(rows)-1),Scalar(0,255-speed,speed));
    }else{
        rectangle( mat,Point(0,rows*0.875),Point((cols/2)-1,(rows)-1),Scalar(0,255-speed,speed));
    }
}

void RightMotor(direction dir, int speed, Mat mat, int rows, int cols) {
    if (dir == BACK) {
        rectangle( mat,Point(cols/2,rows*0.875),Point((cols)-1,(rows)-1),Scalar(0,255-speed,speed));
    }
    else{
        rectangle( mat,Point(cols/2,rows*0.875),Point((cols)-1,(rows)-1),Scalar(0,255-speed,speed));
    }
}

double inte = 0;
//std::chrono::time_point start;
//std::chrono::time_point end;
int last_err = 0;

double pid(int err, std::chrono::time_point<std::chrono::high_resolution_clock> &pid_start) {
    err -= 8;
    auto pid_end = std::chrono::system_clock::now();
    std::chrono::duration<double, std::milli> dur = pid_start - pid_end;
    pid_start = std::chrono::system_clock::now();
    double Pout = kp * err; // P delen udregnes
    //  auto end = std::chrono::high_resolution_clock::now();
    //auto result = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    inte +=  err * dur.count();  // I delen udregnes
    //  start = std::chrono::high_resolution_clock::now();
    double Iout = ki * inte;
    double derivative = (err - last_err)/dur.count();
    double Dout = kd * derivative;

    double output = Pout + Dout ;//+ Iout;
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
