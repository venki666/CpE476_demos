#include <Wire.h>
#include "Grove_Motor_Driver_TB6612FNG.h"
//#include <Adafruit_SSD1306.h>

// Connect to the two encoder outputs!
#define ENCODER_A   5
#define ENCODER_B   7

// These let us convert ticks-to-RPM
#define GEARING     48
#define ENCODERMULT 16

MotorDriver motor;
// Create an IntervalTimer object 
IntervalTimer myTimer;

volatile float pulseA = 0;
volatile uint32_t lastA = 0;
volatile float pulseB = 0;
volatile uint32_t lastB = 0;

void interruptA() {

  uint32_t currA = micros();
  if (lastA < currA) {
    // did not wrap around
    float rev = currA - lastA;  // us
    pulseA = rev;
  }
  lastA = currA;
}

void interruptB() {

  uint32_t currB = micros();
  if (lastB < currB) {
    // did not wrap around
    float rev = currB - lastB;  // us
    pulseB = rev;
  }
  lastB = currB;
}

void printpulse() {
    Serial.print((int)pulseA); Serial.print("\t"); 
    Serial.println((int)pulseB);
}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), interruptA, RISING);
  pinMode(ENCODER_B , INPUT);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), interruptB, RISING);
  delay(100);
  Wire.begin();
  motor.init();

  myTimer.begin(printpulse, 100000);  // blinkLED to run every 0.1 seconds

  // turn on motor M1
  motor.dcMotorStop(MOTOR_CHA);
  motor.dcMotorStop(MOTOR_CHB);
  
}



int i;
void loop() {
 
    motor.dcMotorRun(MOTOR_CHA, 200);
    motor.dcMotorRun(MOTOR_CHB, 200);
    delay(10000);


    motor.dcMotorStop(MOTOR_CHA);
    motor.dcMotorStop(MOTOR_CHB);
    delay(1000);

    motor.dcMotorRun(MOTOR_CHA, -200);
    motor.dcMotorRun(MOTOR_CHB, -200);
    delay(10000);

    motor.dcMotorStop(MOTOR_CHA);
    motor.dcMotorStop(MOTOR_CHB);
    delay(1000);

}