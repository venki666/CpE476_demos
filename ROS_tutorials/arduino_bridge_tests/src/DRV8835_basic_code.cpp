//DRV8835 - Arduino code

/////////////////////////////////////////////////////////
//main code//
/////////////////////////////////////////////////////////

int left_motor_in = 10; //PWM pin
int left_motor_out = 16;

int right_motor_in = 9; //PWM pin
int right_motor_out = 15;

void setup() {
  // put your setup code here, to run once:
  pinMode(left_motor_in, OUTPUT);    pinMode(left_motor_out, OUTPUT);
  pinMode(right_motor_in, OUTPUT);  pinMode(right_motor_out, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  left_motor_go(100);
  right_motor_go(100);
}

/////////////////////////////////////////////////////////
//user function
/////////////////////////////////////////////////////////

// 0<parameter value<255
void left_motor_go (int motor_speed) {
  analogWrite(left_motor_in, motor_speed);
  digitalWrite(left_motor_out, LOW);
}

void right_motor_go (int motor_speed) {
  analogWrite(right_motor_in, motor_speed);
  digitalWrite(right_motor_out, LOW);
}

