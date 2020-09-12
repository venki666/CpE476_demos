// This example drives each motor on the Romi forward, then
// backward.  The yellow user LED is on when a motor should be
// running forward and off when a motor should be running
// backward.

#include <Romi32U4.h>
#include <Keyboard.h>

Romi32U4Motors motors;
Romi32U4ButtonA buttonA;

void setup()
{
  // Wait for the user to press button A.
  buttonA.waitForButton();
  Serial.begin(9600); // baud rate  
  Keyboard.begin();
  delay(1000);
}

void loop()
{
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    int speed = 300; // Local variable
    if( inByte != -1)
    {
      switch(inByte)
      {
        case 'w':
         motors.setSpeeds(speed, speed);
         delay(20);
         break;
        case 's':
         motors.setSpeeds(-speed, -speed);
         delay(20);
         break;
        case 'a': //Turn left
         motors.setSpeeds(speed/2, speed);
         delay(20);
         break;
        case 'd': //Turn Right
         motors.setSpeeds(speed, speed/2);
         delay(20);
         break;
        case 'x':  // stop
         motors.setSpeeds(0, 0);
         delay(20);
         break;
      }
    }
    else{
         motors.setSpeeds(0, 0);
    }

  } // if serial
  delay(500);
}
