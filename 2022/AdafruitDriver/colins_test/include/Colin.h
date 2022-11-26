//-----------------------------------------------------------------------------
// Colin's Header
// Author: Andrew Kramer
// 12/9/2016

// This header contains constants used in programs for Colin the robot
// including the nubers used for motor control pins and sensor pins

#ifndef Colin_h
#define Colin_h

#define RH_PWM 15 // PWM pin for right hand motor
#define RH_DIR1 16 // direction control for right hand motor BIN1 pin on motor controller
#define RH_DIR2 17 // direction control for right hand motor BIN2 pin on motor controller

#define LH_PWM 14 // PWM pin for left hand motor
#define LH_DIR1 20 // direction control for right hand motor AIN1 pin on motor controller
#define LH_DIR2 21 // direction control for left hand motor AIN2 pin on motor controller

// gain values for PID motor control
const double kP = 0.028, kI = 0.001, kD = 0.0006;

// delta T for speed calculation and PID adjustment
const long deltaT = 50000;

const int ticksPerRev = 1024, wheelCirc = 515, wheelDist = 368;

// pins for the encoder inputs
#define RH_ENCODER_A 7 
#define RH_ENCODER_B 8
#define LH_ENCODER_A 5
#define LH_ENCODER_B 6

#endif