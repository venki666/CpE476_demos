#include <Romi32U4.h>
#include <PololuRPiSlave.h>
#define LOG Serial
#include <log.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

Romi32U4Motors motors;
Romi32U4Encoders encoders;
Romi32U4ButtonA buttonA;

float _debug_linear_ms = 0.0;
float _debug_angle_rs = 0.0;

void messageCb( const geometry_msgs::Twist& msg){
  _debug_angle_rs = msg.angular.z;
  _debug_linear_ms = msg.linear.x;
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

 
void _DEBUG_PID_CONTROL() {
  //static float _linear_ms_change = 0.1;
  set_twist_target(_debug_linear_ms, _debug_angle_rs);
}

void setup() {
  Serial.begin(57600);
  set_twist_target(_debug_linear_ms, _debug_angle_rs);
  nh.initNode(); 
  nh.subscribe(sub);
}



void loop() {
  // put your main code here, to run repeatedly:
  if (everyNmillisec(10)) {
    // ODOMETRY
    calculateOdom();
    doPID();
  }
 nh.spinOnce();
}
