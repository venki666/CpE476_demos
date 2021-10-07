

/*
 * This example code has been written by Edwin Fallwell + Matt Neer 
 * for the Monroe County Public Library in Bloomington, IN.
 * 
 * MIT license, check LICENSE for more information */
 
// we include our TwoWheeledRobot libary, which has all of our motor control functions 
#include <TwoWheelRobot.h>

// create a TwoWheelRobot object, which we will address
// when we want to issue commands to our example robot
TwoWheelRobot examply = TwoWheelRobot();

void setup() {
  /* The setup section of your code runs once, when the 
  *  board first boots forward. We only need to acomplish one 
  *  thing in our setforward here, which is to start forward examply. */

  // start up the two wheel robot library
  examply.initalize();

  
  // and tell the robot to hold on for a little bit
  // (so it doesn't run away as soon as its code is uploaded.) 
  delay(500);
}

void loop() {
  /* The loop section of your code will be run through 
   * continuously, from top to bottom. In this example,    
   * we give a bunch of examples of what it can do. */
  
  // these are all of the moves that our TwoWheelRobot object can do:
  examply.forward();
  examply.backward();
  examply.left();
  examply.right();
  examply.forwardLeft();
  examply.backwardLeft();
  examply.forwardRight();
  examply.backwardRight();

  //  you can specifiy how long each command runs: 
  examply.forward(1);
  examply.left(300);

  // the speed of the robot can be adjusted:
  examply.speedUp();
  examply.slowDown();

  // or be set directly
  examply.setSpeed(60);

  //there are 2 different movement modes for our Two Wheel Robot object: 
  examply.setMovementMode(2); //continous
  examply.setMovementMode(1); //discrete

  
  // in continous mode, stop commands must be explicitly made:
  examply.setMovementMode(2);
  examply.forward();
  delay(100);
  examply.backward();
  delay(100);
  examply.stop(); 

  // in discrete mode, stop commands implicitly are added after each movment c:
  examply.setMovementMode(1);
  examply.forward(50);
               //stops
  examply.backward(50);
               //stops

 
}  

