const uint16_t PWMA = 25;         
const uint16_t AIN2 = 17;        
const uint16_t AIN1 = 21;         
const uint16_t BIN1 = 22;       
const uint16_t BIN2 = 23;        
const uint16_t PWMB = 26;   

const uint16_t ANALOG_WRITE_BITS = 8;

int freq = 100000;
int channel_A = 0;
int channel_B = 1;
int resolution = ANALOG_WRITE_BITS;

void initMotors(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  ledcSetup(channel_A, freq, resolution);
  ledcAttachPin(PWMA, channel_A);

  ledcSetup(channel_B, freq, resolution);
  ledcAttachPin(PWMB, channel_B);

}

void forwardA(uint16_t pwm){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(channel_A, pwm);
}

void forwardB(uint16_t pwm){
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(channel_B, pwm);
}

void setup() {
  initMotors();
}

void loop() {
  forwardA(400);
  forwardB(400);
}
