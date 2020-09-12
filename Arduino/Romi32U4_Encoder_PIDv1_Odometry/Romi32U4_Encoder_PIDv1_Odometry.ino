// This program shows how to read the encoders on the Romi 32U4.
// The encoders can tell you how far, and in which direction each
// motor has turned.
//
// You can press button A on the Romi to drive both motors
// forward at full speed.  You can press button C to drive both
// motors in reverse at full speed.
//
// Encoder counts are printed to the LCD and to the serial
// monitor.
//
// On the LCD, the top line shows the counts from the left
// encoder, and the bottom line shows the counts from the right
// encoder.  Encoder errors should not happen, but if one does
// happen then the buzzer will beep and an exclamation mark will
// appear temporarily on the LCD.
//
// In the serial monitor, the first and second numbers represent
// counts from the left and right encoders, respectively.  The
// third and fourth numbers represent errors from the left and
// right encoders, respectively.

#include <Romi32U4.h>
#include <Odometer.h>
#include <PID_v1.h>

// Define robot geometrical properties
// Distance travelled per encoder tick
const float tickDist = .152505;
// Width between the two wheels
const float track = 142.5;


// Define motors max command
const float motorsMaxCommand = 255;

// Define motor trims in 1/40
const int leftTrim = 40;
const int rightTrim = 40;

// Define speed variables for acceleration control
double lastSpeedCmdLeft = 0;
double lastSpeedCmdRight = 0;

double SpeedCmdLeft = 0;
double SpeedCmdRight = 0;

double SpeedLeft = 0;
double SpeedRight = 0;

double OdoSpeedLeft = 0;
double OdoSpeedRight = 0;

// Define maximum speed command change per time step
const int accelMax = 20;

double aggKp=200, aggKi=700, aggKd=0.05;
double consKp=200, consKi=700, consKd=0.05;


Romi32U4Encoders encoders;
Odometer odometer(tickDist, track);
//TimedPID PIDLeft(Kp, Ki, Kd);
PID PIDLeft(&OdoSpeedLeft, &SpeedCmdLeft, &SpeedLeft, consKp, consKi, consKd, DIRECT);
//TimedPID PIDRight(Kp, Ki, Kd);
PID PIDRight(&OdoSpeedRight, &SpeedCmdRight, &SpeedRight, consKp, consKi, consKd, DIRECT);

Romi32U4Motors motors;
Romi32U4ButtonA btnA;
Romi32U4ButtonB btnB;
Romi32U4ButtonC btnC;

// Define variables used for target speed generation and time step calculation
unsigned long initialTime;
unsigned long lastTime;
char report[80];

void setup()
{
  Serial.begin(9600);

  // Set PID controllers command range
  PIDLeft.setOutputLimits(0, motorsMaxCommand);
  PIDRight.setOutputLimits(0, motorsMaxCommand);

  // Set PID controllers command range
  PIDLeft.setMode(AUTOMATIC);
  PIDRight.setMode(AUTOMATIC);

  // Initialize time variables
  initialTime = micros();
  lastTime = initialTime;
  delay(10);
}

void loop() {

  // Calculate the time step between passes of the main loop
  // Get current time
  unsigned long currentTime = micros();
  // Calculate time step in seconds (micro seconds / 1 000 000)
  float timeStep = float(currentTime - lastTime) / 1E6;
  // Store current time as last time for next pass of the loop
  lastTime = currentTime;

  // Define the target speed as function of time
  float targetSpeed = 0;
  if (currentTime - initialTime > 8E6){
    // Reset initial time
    initialTime = currentTime;
  }
  else if (currentTime - initialTime > 7E6)
  {
    targetSpeed = 250;
  }
    else if (currentTime - initialTime > 5E6)
  {
    targetSpeed = 200;
  }
  else if (currentTime - initialTime > 3E6) {
    targetSpeed = 250;
  }
  // Add continuous variation of the target speed
  targetSpeed += 25 * sin(micros() / 2.5E5);

  // Print actual motor speeds from odometer
  Serial.print(odometer.getSpeedRight());
  Serial.print("\t");
  Serial.print(odometer.getSpeedLeft());

  // Press button A to control motors with a variable target speed
  if (btnA.isPressed())
  {
    setMotorSpeeds(targetSpeed, targetSpeed);
    Serial.print("\t");
    Serial.println(targetSpeed);
  }
  // Press button B to control motors with a fixed target speed
  else if (btnB.isPressed())
  {
    setMotorSpeeds(300, 300);
    Serial.print("\t");
    Serial.println(300);
  }
  else
  {
    setMotorSpeeds(0, 0);
    Serial.print("\t");
    Serial.println(0);
  }

  // Ensure a constant time step of the main loop (10 milliseconds)
  while (micros() - currentTime < 10000)
  {
    // Wait until time step is reached
  }
}

// Sets the motor speeds using PID controllers
void setMotorSpeeds(int speedLeft, int speedRight)
{
  // Read odometer counts
  int countsLeft = encoders.getCountsLeft();
  int countsRight = encoders.getCountsRight();

  // Update odometer
  odometer.update(countsLeft, countsRight);
  // get speed command from PID controllers
  //int speedCmdLeft = PIDLeft.getCmdAutoStep(speedLeft, odometer.getSpeedLeft());
  OdoSpeedLeft = odometer.getSpeedLeft();
  if(abs(speedLeft - OdoSpeedLeft) < accelMax)
   PIDLeft.SetTunings(consKp, consKi, consKd);
  else 
   PIDLeft.SetTunings(aggKp, aggKi, aggKd);
  PIDLeft.Compute();
  //int speedCmdRight = PIDRight.getCmdAutoStep(speedRight, odometer.getSpeedRight());
  OdoSpeedRight = odometer.getSpeedRight();
    if(abs(speedRight - OdoSpeedRight) < accelMax)
   PIDRight.SetTunings(consKp, consKi, consKd);
  else 
   PIDRight.SetTunings(aggKp, aggKi, aggKd);
  PIDRight.Compute();

  // Stop immediately if target speed is zero
  if (speedLeft == 0)
  {
    speedCmdLeft = 0;
    initialTime = micros();
    lastTime = initialTime;
  }
  if (speedRight == 0)
  {
    speedCmdRight = 0;
    initialTime = micros();
    lastTime = initialTime;
  }

  // Set motor speeds
  motors.setSpeeds(speedCmdLeft * leftTrim / 40, speedCmdRight * rightTrim / 40);

  lastSpeedCmdLeft = speedCmdLeft;
  lastSpeedCmdRight = speedCmdRight;
}
