#include "motor.hpp"

void MotorInit()
{
   //std::cout << "Setting up WiringPi" << '\n';
   if (wiringPiSetup() == -1)
   {
      //std::cout << "WiringPi setup failed" << '\n';
      exit(1);
   }
   pinMode(1, PWM_OUTPUT);
   pinMode(23, PWM_OUTPUT);
   pinMode(2, OUTPUT);
   pinMode(3, OUTPUT);
   pinMode(24, OUTPUT);
   pinMode(25, OUTPUT);
}

void LeftMotor(direction dir, int speed)
{
   if (dir == BACK)
   {
      digitalWrite(2, HIGH);
      digitalWrite(3, LOW);

   }
   else
   {
      digitalWrite(2, LOW);
      digitalWrite(3, HIGH);

   }
   pwmWrite(1, speed);
}

void RightMotor(direction dir, int speed)
{
   if (dir == BACK)
   {
      digitalWrite(25, HIGH);
      digitalWrite(24, LOW);

   }
   else
   {
      digitalWrite(25, LOW);
      digitalWrite(24, HIGH);

   }
   pwmWrite(23, speed);
}

double inte     = 0;
int    last_err = 0;

double pid(int err, std::chrono::time_point <std::chrono::high_resolution_clock>&pid_start)
{
  err -= 8;
  auto pid_end = std::chrono::system_clock::now();
  std::chrono::duration<double, std::milli> dur = pid_end - pid_start;
  pid_start = std::chrono::system_clock::now();
  double Pout = kp * err; // P delen udregnes
  inte +=  err * (dur.count()/1000);  // I delen udregnes
  double Iout = ki * inte;
  double derivative = (err - last_err)/(dur.count()/1000);
  double Dout = kd * derivative;

  double output = Pout + Iout+ Dout;

  last_err=err;
  return output;
}

//Follow line function
void MotorFollowLine(int err, int speed, std::chrono::time_point <std::chrono::high_resolution_clock>&pid_start)
{

   double error = pid(err, pid_start);
   error=abs(error)+1;
   if (err < 0)
   {
     RightMotor(FORWARD, ((speed*2)/(error+1))*error);
     LeftMotor(FORWARD, (speed*2)/(error+1));
      return;
   }
   if (err > 0)
   {
     LeftMotor(FORWARD, ((speed*2)/(error+1))*error);
     RightMotor(FORWARD, (speed*2)/(error+1));
      return;
   }
   if (err == 0)
   {
      RightMotor(FORWARD, speed);
      LeftMotor(FORWARD, speed);
      return;
   }
}
