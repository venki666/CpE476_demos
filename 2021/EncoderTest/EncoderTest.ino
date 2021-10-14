#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include <Adafruit_SSD1306.h>

// Connect to the two encoder outputs!
#define ENCODER_A   12
#define ENCODER_B   11

// These let us convert ticks-to-RPM
#define GEARING     48
#define ENCODERMULT 16

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// And connect a DC motor to port M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);
// We'll display the speed/direction on the OLED
//Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

volatile float pulse = 0;
volatile uint32_t lastA = 0;
volatile bool motordir = FORWARD;

void interruptA() {

  uint32_t currA = micros();
  if (lastA < currA) {
    // did not wrap around
    float rev = currA - lastA;  // us
    pulse = rev;
  }
  lastA = currA;
}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  pinMode(ENCODER_A, INPUT_PULLUP);
  attachInterrupt(ENCODER_A, interruptA, RISING);

  delay(100);

  //Serial.println("MMMMotor party!");
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");  Serial.println("Begun");
  // turn on motor M1
  myMotor->setSpeed(0);
}

void printpulse() {
    Serial.print((int)pulse); Serial.println(" us");
}

int i;
void loop() {
  delay(50);
  myMotor->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    delay(20);
    printpulse();
  }

  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    delay(20);
    printpulse();
  }

  myMotor->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    delay(20);
    printpulse();
  }

  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    delay(20);
    printpulse();
  }
}
