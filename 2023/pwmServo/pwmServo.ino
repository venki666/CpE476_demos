#include <Servo.h>
Servo pwmServo;

#define PSERVO_PIN 4

int pwmServoChannel = 7;
int pwmServoInitPos = 90;


void pwmServoInit(){
  pwmServo.attach(PSERVO_PIN);
}

void setup() {
  pwmServoInit();
}

void loop() {
  pwmServo.write(50);
  delay(1000);
  pwmServo.write(90);
  delay(1000);
}
