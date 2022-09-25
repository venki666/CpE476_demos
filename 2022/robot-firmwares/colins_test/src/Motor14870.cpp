// Motor.cpp
// by Andrew Kramer

// Provides low-level control of a the speed and direction of a single DC motor
// by means of a motor driver such as the TB6612FNG

#include "Arduino.h"
#include "Motor14870.h"

// accepts three ints as parameters: 
//    the pin numbers for the direction control pins
//    and the pin number of the pwm pin
Motor14870::Motor14870(int dir1, int pwm)
{
	_dir1 = dir1;
	_pwm = pwm;
	pinMode(_dir1, OUTPUT);
	pinMode(_pwm, OUTPUT);
}

// sets the motor's direction to forward
void Motor14870::setFwd()
{
	digitalWrite(_dir1, LOW);
}

// sets the motor's direction to backward
void Motor14870::setBack()
{
	digitalWrite(_dir1, HIGH);
}

// sets the motor to freewheel
void Motor14870::setFree()
{
	analogWrite(_pwm, 3);
}

// sets the motor to brake
void Motor14870::setStop()
{
	analogWrite(_pwm, 0);
}

// accepts an int, the PWM level, as a parameter
// sets the PWM output to the motor to the given int
// level must be between 0 and 255 inclusive
// behavior is undefined if level is outside this range
void Motor14870::setPWM(int level)
{
	analogWrite(_pwm, level);
}
