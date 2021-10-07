/* 
 *  This is a very simple line following robot. 
 *  The only feature is that it rams up its speed 
 *  if it doesn't see a line. 
 *  
 *  Edwin Fallwell, MCPL 3/25/18
 */

// We've attached a daughter board to our Feather. This additional board.
// also known as a Wing, or a Shield if we were using an Arduino, needs a
// library to drive it. This statement includes this library so that we have access to
// it in the rest of our program. 
#include <Adafruit_MotorShield.h>

// We need to create an object representing the Adafruit Motor Shield. We do this by
// creating an object of the type "Adafruit_MotorShield" named AFMS. 
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// We're also creating two objects of the type "Adafruit_DCMotor", one for each of the 
// motors on our line following robot. 
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(3);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(4);


// Sensors
int leftSensor = A2;
int rightSensor = A1;

// This time, we're going to adjust our speed over time, so we need some variables to hold this
// information. We've got three valuse here to store our current speed, our maximum speed, 
// and our default speed. Each of these has been set up with an inital value that I found to work 
// from testing. 
int currentSpeed = 0; 
int maxSpeed = 100;
int defaultSpeed = 60;

// We also need some variables to store information used in line detection + logic. These are the
// threshold that determines if the reading we are getting from the sensor is detecting a line, and 
// a special type of variable called a boolean, which stores either the value true or false. We are using
// this to store the state of if the line has been detected. 
int lineThreshold = 400; //this set to something a little more logical. Testing will decide. 
bool lineDetectedFlag = false;

void setup() {
  /*  board first boots up. In this sketch, we need to 
  *  do two things, initalize the AFMS object + set the 
  *  currentSpeed to the defaultSpeed. 
  */  
  AFMS.begin();  // create with the default frequency 1.6KHz
  
 // turn on motors
  L_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);

  R_MOTOR->setSpeed(0);
  R_MOTOR->run(RELEASE);

  currentSpeed = defaultSpeed;
  // set the current speed to default. 

  Serial.begin(9600);
}

void loop() {
  /* The loop section of your code will be run through 
   * continuously, from top to bottom. In this example,    
   * we want to robot to rotate in a circle. We set the 
   * speed for each of the motors and then tell each of the
   * motors to run. 
   */
  
  // First, we set the speed of the motors. 
  L_MOTOR->setSpeed(currentSpeed);
  R_MOTOR->setSpeed(currentSpeed);


  // Then we check the sensors. If both the left & the 
  if((analogRead(rightSensor) < lineThreshold) && (analogRead(leftSensor) < lineThreshold)){
    //don't see no line at all
    L_MOTOR->run(FORWARD);
    R_MOTOR->run(FORWARD);
    lineDetectedFlag = false; 
  }
  
  else if((analogRead(rightSensor) < lineThreshold) && (analogRead(leftSensor) > lineThreshold)){
    //leftSensor detects line
    L_MOTOR->run(BACKWARD);
    R_MOTOR->run(FORWARD);
    lineDetectedFlag = true; 
  }
  
  else if((analogRead(rightSensor) > lineThreshold) && (analogRead(leftSensor) < lineThreshold)){
    //rightSensor detects line
    L_MOTOR->run(FORWARD);
    R_MOTOR->run(BACKWARD);
    lineDetectedFlag = true; 
  }
  
  else{
    //both see line/nothing
    L_MOTOR->run(RELEASE);
    R_MOTOR->run(RELEASE);
  }

  /* This part ramps up the speed until a line is detected. 
   * If a line is detected, drop the speed back down to default speed to avoid 
   * over shooting the line.
   */
  
//  if(lineDetectedFlag){ //if we've detected a line
//    currentSpeed = defaultSpeed; //slow back down
//  }
//  else{
//    if(currentSpeed < maxSpeed){ //if we're going slower than max speed
//      currentSpeed += 1; //accelerate by 1 
//    }
//  } 

//  Serial.print("right: ");
//  Serial.println(analogRead(rightSensor));
//  Serial.print("left: ");
//  Serial.println(analogRead(leftSensor));
}
