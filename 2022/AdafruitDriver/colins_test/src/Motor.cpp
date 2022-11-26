// Motor.cpp
// by Andrew Kramer

// Provides low-level control of a the speed and direction of a single DC motor
// by means of a motor driver such as the TB6612FNG

#include "Arduino.h"
#include <SPI.h>
#include "Motor.h"
// accepts three ints as parameters: 
//    the pin numbers for the direction control pins
//    and the pin number of the pwm pin
Motor::Motor(int Mn)
{
  AFMS = Adafruit_MotorShield();
  motor = AFMS.getMotor(Mn);
  AFMS.begin();
  motor->setSpeed(0);
}

// sets the motor's direction to forward
void Motor::setFwd()
{
  motor->run(FORWARD);
}

// sets the motor's direction to backward
void Motor::setBack()
{
  motor->run(BACKWARD);
}

// sets the motor to freewheel
void Motor::setFree()
{
  motor->run(RELEASE);
}

// sets the motor to brake
void Motor::setStop()
{
  motor->run(BRAKE);
}

// accepts an int, the PWM level, as a parameter
// sets the PWM output to the motor to the given int
// level must be between 0 and 255 inclusive
// behavior is undefined if level is outside this range
void Motor::setPWM(int level)
{
  motor->setSpeed(level);
}

