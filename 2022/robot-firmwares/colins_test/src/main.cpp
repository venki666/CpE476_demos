#include <Arduino.h>

// colin_controller.ino
// by Andrew Kramer

// Control program for a differential drive robot (Colin)
// Allows for control of the robot by a higher-level program 
// running on a Raspberry Pi using a serial communication protocol

// COMMAND PACKETS
// Expects to recieve command packets from the Raspberry Pi at least once every second
// Command packets should contain two 16 bit ints representing the commanded 
// translational speed and angular velocity
// The two ints should be broken into bytes, least significant byte (LSB) first
// To avoid problems with float representations, angular velocity is expected to be multiplied by 1000
// and casted to an int before sending
// Command packet format is as follows:
//         byte 0         |        byte 1       |       byte 2         |       byte 3
//    translational (LSB) | translational (MSB) | angular * 1000 (LSB) | angular * 1000 (MSB)

// If a command packet is not received for more than one second, the robot assumes there is a communication
// problem and stops moving until another command is received

// SENSOR PACKETS
// After successfully receiving a command and updating speeds, the robot updates 
// its sensors and sends a packet containing the sensor values in response.
// Sensor packets will contain NUM_SONAR + 3 16 bit ints broken into bytes, least significant byte (LSB) first
// Indices 0 through NUM_SONAR * 2 - 1 of the packet will contain sonar distance readings
// Indices NUM_SONAR * 2 through NUM_SONAR * 2 + 5 will contain the robot's x position in cm,
// y position in cm, and heading in radians
// To avoid problems with float representation, heading is multiplied by 1000 and casted to an int before sending
// Sensor packet format is as follows:
//     byte 0       |     byte 1      |    byte 2      | ... |  byte NUM_SONAR * 2   | byte NUM_SONAR * 2 + 1 | byte NUM_SONAR * 2 + 2 | byte NUM_SONAR * 2 + 3 | byte NUM_SONAR * 2 + 4 | byte NUM_SONAR * 2 + 5
//   sonar 0 (LSB)  |  sonar 0 (MSB)  |  sonar 1 (LSB) | ... |   x position (LSB)    |    x position (MSB)    |    y position (LSB)    |    y position (MSB)    |  heading * 1000 (LSB)  |  heading * 1000 (MSB)  

#include <DifferentialDrive.h>
#include <Colin.h>
#include <TimerOne.h>
#include <Wire.h>

#define SONAR_ADDRESS        0x4 // address of the sonar controller on the I2C bus
#define OWN_ADDRESS          0x6 // this controller's own I2C address
#define NUM_SONAR              8
#define SONAR_PER_CONTROLLER   8
#define NUM_CONTROLLERS        1
#define LED_PIN                13

int sonarDistances[NUM_SONAR]; // array of ping times from sonar sensors
const uint8_t trig = 1; // meaningless value to trigger update
const uint8_t sonarArrayRadius = 175; // in microseconds
const double speedOfSound = 0.0343; // in cm/microsecond

Motor rhMotor(1);
Encoder rhEncoder(RH_ENCODER_A, RH_ENCODER_B, deltaT, ticksPerRev);
SpeedControl rhSpeedControl(&rhMotor, &rhEncoder);
PositionControl rhPosition(&rhSpeedControl);

Motor lhMotor(2);
Encoder lhEncoder(LH_ENCODER_A, LH_ENCODER_B, deltaT, ticksPerRev);
SpeedControl lhSpeedControl(&lhMotor, &lhEncoder);
PositionControl lhPosition(&lhSpeedControl);

DifferentialDrive colin(&lhPosition, &rhPosition, 
                        wheelCirc, wheelDist);

double x, y; 
double theta;
unsigned long lastCommandTime, currentTime;

int translational; // current translational speed in cm/s
double angular; // current angular velocity in rad/s 




// updates the left hand encoder when a left hand encoder event is triggered
void readLHEncoder()
{
  lhEncoder.updateCount();
}

// updates the right hand encoder when a right hand encoder event is triggered
void readRHEncoder()
{
  rhEncoder.updateCount();
}

// updates colin's position and motor speeds
// should be run every 50ms using a timer interrupt
void adjust()
{
  colin.update();
}

void setup() {
  Serial.begin(9600); // start serial with Raspberry Pi
  Serial.setTimeout(100); 
  
  // blink LED to signal startup
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  // start timer and hardware interrupts
  Timer1.initialize(deltaT);
  Timer1.attachInterrupt(adjust);
  attachInterrupt(0, readLHEncoder, CHANGE);
  attachInterrupt(1, readRHEncoder, CHANGE);

  // set PID gains for each motor
  rhSpeedControl.setGains(kP, kI, kD);
  lhSpeedControl.setGains(kP, kI, kD);

  lastCommandTime = millis();
  currentTime = millis();
  translational = 0;
  angular = 0.0;
}


void loop() 
{ 
  currentTime = millis();
  
  // stop colin if a command packet has not been received for 1 second
  if (currentTime - lastCommandTime > 1000)
  {
    lastCommandTime = millis();
    colin.drive(2, 0.0);
  }
}