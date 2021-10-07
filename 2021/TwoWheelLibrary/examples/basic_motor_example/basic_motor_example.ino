// We've also attached a daughter board to our Feather. This additional board.
// also known as a Wing, or a Shield if we were using an Arduino, needs a
// library to drive it. This statement includes this library so that we have access to
// it in the rest of our program. 
#include <Adafruit_MotorShield.h>

// We need to create an object representing the Adafruit Motor Shield. We do this by
// creating an object of the type "Adafruit_MotorShield" named AFMS. 
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// We're also creating two objects of the type "Adafruit_DCMotor", one for each of the 
// motors on our drawing robot. 
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(3);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(4);

void setup() {
  /* The setup section of your code runs once, when the 
  *  board first boots up. In this example, we only need to 
  *  do one thing, initalize the AFMS object. 
  */
  AFMS.begin();
}

void loop() {
  /* The loop section of your code will be run through 
   * continuously, from top to bottom. In this example,    
   * we want to robot to rotate in a circle. We set the 
   * speed for each of the motors and then tell each of the
   * motors to run. 
   */
  L_MOTOR->setSpeed(100);
  R_MOTOR->setSpeed(100);
  L_MOTOR->run(BACKWARD);
  R_MOTOR->run(FORWARD);
}
