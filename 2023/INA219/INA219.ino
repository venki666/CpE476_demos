// <<<<<<<<<<=========INA219: 0x42===========>>>>>>>>>>
#define S_SDA 32
#define S_SCL 33
#include <INA219_WE.h>
#include <Wire.h>
#define INA219_ADDRESS 0x42
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0; 
bool ina219_overflow = false;

void InitINA219(){
  if(!ina219.init()){
    Serial.println("INA219 not connected!");
  }
  ina219.setADCMode(BIT_MODE_9);
  ina219.setPGain(PG_320);
  ina219.setBusRange(BRNG_16);
  ina219.setShuntSizeInOhms(0.01); // used in INA219.
}

void InaDataUpdate(){
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
}

void allDataUpdate(){
    Serial.print("battery:");
    Serial.println(loadVoltage_V);
    Serial.print("current_mA:");
    Serial.println(current_mA);
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin(S_SDA,S_SCL);
  Serial.begin(115200);
  while(!Serial){}
  InitINA219();
}

void loop() {
  // put your main code here, to run repeatedly:
  InaDataUpdate();
  allDataUpdate();
  delay(1000);
}
