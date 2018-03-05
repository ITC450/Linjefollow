#include "motor.hpp"

void MotorInit()
{
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

void LeftMotor(direction dir, int speed)
{
   if (dir == BACK)
   {
      digitalWrite(2,HIGH);
      digitalWrite(3,LOW);
   }else{
     digitalWrite(2,LOW);
     digitalWrite(3,HIGH);
   }
   pwmWrite(1,speed);
}

void RightMotor(direction dir, int speed)
{
  if (dir == BACK)
  {
     digitalWrite(25,HIGH);
     digitalWrite(24,LOW);
  }else{
    digitalWrite(25,LOW);
    digitalWrite(24,HIGH);
  }
  pwmWrite(23,speed);
}
