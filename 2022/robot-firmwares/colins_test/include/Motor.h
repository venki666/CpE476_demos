// Motor
// by Andrew Kramer

// controls direction and pwm input to a DC motor

#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>

class Motor
{
public:
	Motor(int Mn);
	void setFwd();
	void setBack();
	void setFree();
	void setStop();
	void setPWM(int level);

private:
    Adafruit_MotorShield AFMS; 
    Adafruit_DCMotor *motor;  
};

#endif