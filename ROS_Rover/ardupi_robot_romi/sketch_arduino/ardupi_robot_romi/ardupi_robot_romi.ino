
//***************
/*
   ardupi_robot_romi
   
   Programme which interface romi motors and encoders to Raspberry PI by ROS messages
   
   authors :  Sylvain Bertrand | Alexis Reis
*/
//***************

// includes
#include <ros.h>                        // to use ros on arduino
#include <ardupi_robot_romi/Int32Stamped.h>  // header which define the message int32stamped in order to use it

// constants
#define WHEEL_DIAMETER 0.07             // in SI unit
#define ANGULAR_RES 0.52359

#define LEFT_ENCODER_PINA 2    // left encoder on Digital Pin 2  - Arduino MEGA
#define LEFT_ENCODER_PINB 3    // left encoder on Digital Pin 3  - Arduino MEGA
#define RIGHT_ENCODER_PINA 20  // left encoder on Digital Pin 20 - Arduino MEGA
#define RIGHT_ENCODER_PINB 21  // left encoder on Digital Pin 21 - Arduino MEGA

#define LEFT_ENCODER_INTERRUPT_A 0  //number of the interruption - Arduino MEGA - 0 n° interruption, pin = 2
#define LEFT_ENCODER_INTERRUPT_B 1  //number of the interruption - Arduino MEGA - 1 n° interruption, pin = 3
#define RIGHT_ENCODER_INTERRUPT_A 3 //number of the interruption - Arduino MEGA - 2 n° interruption, pin = 20
#define RIGHT_ENCODER_INTERRUPT_B 2 //number of the interruption - Arduino MEGA - 3 n° interruption, pin = 21

#define LEFT_MOTOR_PWM_PIN 6  // motor A blue motorshield
#define LEFT_MOTOR_DIR_PIN 7
#define RIGHT_MOTOR_PWM_PIN 9 // motor B blue motorshield
#define RIGHT_MOTOR_DIR_PIN 8

// declaration of callbacks interrupts of encoders
void callBackInterruptRightEncoderCount();
void callBackInterruptLeftEncoderCount();

// declaration of callbacks for cmd msg reception
void callBackCmdLeftMotor( const ardupi_robot_romi::Int32Stamped& msg); 
void callBackCmdRightMotor( const ardupi_robot_romi::Int32Stamped& msg);

// ROS node
ros::NodeHandle nh;

// global variables

ardupi_robot_romi::Int32Stamped RightEncoderCount;
ardupi_robot_romi::Int32Stamped LeftEncoderCount;

ardupi_robot_romi::Int32Stamped ctrlLeftMotor;
ardupi_robot_romi::Int32Stamped ctrlRightMotor;

int leftWheelRotationDir = 1;  // 0: stop, +1: forward, -1: backward
int rightWheelRotationDir = 1; // 0: stop, +1: forward, -1: backward

double timeOfLastChangeLeftEncoder = 0.0;
double timeOfLastChangeRightEncoder = 0.0;

double publicationPeriodEncoders = 0.05;
double timeOfLastPubEncoders = 0.0;

// varaibles use ti detect the direction give by the encoder
int LstateRight;
byte encoderRightALast;
int valR;
boolean DirectionRight;

int LstateLeft;
byte encoderLeftALast;
int valL;
boolean DirectionLeft;

// publishers
ros::Publisher pubCountRightEncoderA("ardupi_robot/encoderCount/right", &RightEncoderCount);
ros::Publisher pubCountLeftEncoderA("ardupi_robot/encoderCount/left", &LeftEncoderCount);

// suscribers
ros::Subscriber<ardupi_robot_romi::Int32Stamped> subCmdLeftMotor("ardupi_robot/cmdMotor/left", &callBackCmdLeftMotor );
ros::Subscriber<ardupi_robot_romi::Int32Stamped> subCmdRightMotor("ardupi_robot/cmdMotor/right", &callBackCmdRightMotor );

// ****** executed once at Arduino power on or reset ******
void setup()
{
 //setup ROS
 nh.initNode(); 
 nh.advertise(pubCountRightEncoderA);
 nh.advertise(pubCountLeftEncoderA);

 nh.subscribe(subCmdLeftMotor);
 nh.subscribe(subCmdRightMotor);
 
 //setup encoder pin
 pinMode(LEFT_ENCODER_PINA, INPUT_PULLUP);
 pinMode(LEFT_ENCODER_PINB, INPUT_PULLUP);
 pinMode(RIGHT_ENCODER_PINA, INPUT_PULLUP);
 pinMode(RIGHT_ENCODER_PINB, INPUT_PULLUP);

 
 //setup interruptions
 attachInterrupt(LEFT_ENCODER_INTERRUPT_A, callBackInterruptLeftEncoderCount, CHANGE);
 attachInterrupt(RIGHT_ENCODER_INTERRUPT_A, callBackInterruptRightEncoderCount, CHANGE);
 
 //setup motors pin
 pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
 pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
 pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
 pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
 
 RightEncoderCount.data=0;
 DirectionRight=true;
 
 LeftEncoderCount.data=0;
 DirectionLeft=true;
 
 Serial.begin(57600);
}

// ****** infinite main loop on Arduino ******
void loop()
{
 // Timer for encoder msg publication  
 if( ((millis()/1000.0) - timeOfLastPubEncoders ) > publicationPeriodEncoders)
 {
    // publish messages
    RightEncoderCount.header.stamp = nh.now();
    LeftEncoderCount.header.stamp =RightEncoderCount.header.stamp;
    
    pubCountRightEncoderA.publish(&RightEncoderCount);
    pubCountLeftEncoderA.publish(&LeftEncoderCount);
     
    // Reset timer
    // if timer expired several times
    // => reset to the integer part of the number of timer periods from last reset of the timer
    timeOfLastPubEncoders = timeOfLastPubEncoders + publicationPeriodEncoders*floor( (millis()/1000.0 - timeOfLastPubEncoders) / publicationPeriodEncoders);
  }
      
  nh.spinOnce(); 
  
}

// definition of callback functions

void callBackCmdLeftMotor( const ardupi_robot_romi::Int32Stamped& msg)
{
  int u = msg.data;
  
  // check saturation
  if (u>255)
    u = 255;
  if (u<-255)
    u = -255;
	
  // write rotation speed module
  analogWrite(LEFT_MOTOR_PWM_PIN, abs(u));

  // write rotation speed direction
  if (u>=0)
  {
    digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);  // forward or stop

    if (u==0)
      leftWheelRotationDir = 0;  // stop
    else
      leftWheelRotationDir = 1;  // forward    
  }
  else
  {
    digitalWrite(LEFT_MOTOR_DIR_PIN, HIGH);  // backward
    leftWheelRotationDir = -1;
  }
}

void callBackCmdRightMotor( const ardupi_robot_romi::Int32Stamped& msg)
{
  int u = msg.data;
   
  // saturation
  if (u>255)
    u = 255;
  if (u<-255)
    u = -255;
	
  // write rotation speed module
  analogWrite(RIGHT_MOTOR_PWM_PIN, abs(u));

  // write rotation speed direction
  if (u>=0)
  {
    digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);  // forward or stop
    if (u==0)
      rightWheelRotationDir = 0;  // stop
    else
      rightWheelRotationDir = 1;  // forward
    
  }
  else
  {
    digitalWrite(RIGHT_MOTOR_DIR_PIN, HIGH);  // backward
    rightWheelRotationDir = -1;
  }
}


void callBackInterruptRightEncoderCount()
{
  LstateRight=digitalRead(RIGHT_ENCODER_PINA);
  
  if((encoderRightALast==LOW) && LstateRight==HIGH)
  {
    valR=digitalRead(RIGHT_ENCODER_PINB);
    
    if(valR==LOW && DirectionRight)
      DirectionRight=false; //reverse
    else if(valR==HIGH && !DirectionRight)
      DirectionRight=true; //forward
  }
  
  encoderRightALast=LstateRight;
  
  if(DirectionRight)
    RightEncoderCount.data++;
  else
    RightEncoderCount.data--;
}


void callBackInterruptLeftEncoderCount()
{
  LstateLeft=digitalRead(LEFT_ENCODER_PINA);
  
  if((encoderLeftALast==LOW) && LstateLeft==HIGH)
  {
    valL=digitalRead(LEFT_ENCODER_PINB);
    
    if(valL==LOW && DirectionLeft)
      DirectionLeft=false; //reverse
    else if(valL==HIGH && !DirectionLeft)
      DirectionLeft=true; //forward
  }
  
  encoderLeftALast=LstateLeft;
  
  if(DirectionLeft)
    LeftEncoderCount.data++;
  else
    LeftEncoderCount.data--;
}
