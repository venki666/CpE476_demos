/*
 *  Square.cpp
 *  Example for driving a 50 cm square using CarPWMMotorControl class
 *
 *  Copyright (C) 2020-2021  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/PWMMotorControl.
 *
 *  PWMMotorControl is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

/*
 * You will need to change these values according to your motor, H-bridge and motor supply voltage.
 * You must specify this before the include of "CarPWMMotorControl.hpp"
 */
//#define USE_ENCODER_MOTOR_CONTROL  // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define USE_ADAFRUIT_MOTOR_SHIELD  // Activate this if you use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
//#define USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD  // Activate this to force using of Adafruit library. Requires 694 bytes program memory.
#define VIN_2_LIPO                 // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
//#define VIN_1_LIPO                 // Or if you use a Mosfet bridge, 1 LIPO (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT   6000  // Default. For 4 x AA batteries (6 volt).
//#define MOSFET_BRIDGE_USED  // Activate this, if you use a (recommended) mosfet bridge instead of a L298 bridge, which has higher losses.
//#define DEFAULT_DRIVE_MILLIVOLT       2000 // Drive voltage -motors default speed- is 2.0 volt
#define DO_NOT_SUPPORT_RAMP  // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program space.
#define DEFAULT_DRIVE_SPEED_PWM 255


#include "CarPWMMotorControl.hpp"

#include "PinDefinitionsAndMore.h"

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT    0

CarPWMMotorControl RobotCarPWMMotorControl;
#define SIZE_OF_SQUARE_MILLIMETER  400

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    // For Adafruit Motor Shield v2
    RobotCarPWMMotorControl.init();
#else
    RobotCarPWMMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
    LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#endif

    /*
     * You will need to change these values according to your motor, wheels and motor supply voltage.
     */
    RobotCarPWMMotorControl.setDriveSpeedAndSpeedCompensationPWM(DEFAULT_DRIVE_SPEED_PWM, SPEED_PWM_COMPENSATION_RIGHT); // Set compensation
#if defined(CAR_HAS_4_WHEELS)
    RobotCarPWMMotorControl.setFactorDegreeToMillimeter(FACTOR_DEGREE_TO_MILLIMETER_4WD_CAR_DEFAULT);
#else
    RobotCarPWMMotorControl.setFactorDegreeToMillimeter(FACTOR_DEGREE_TO_MILLIMETER_2WD_CAR_DEFAULT);
#endif
    // Print info
    PWMDcMotor::printSettings(&Serial);

    delay(5000);
}

void loop() {
    static uint8_t sMotorDirection = DIRECTION_FORWARD;

    for (int i = 0; i < 4; ++i) {
        /*
         * Try to go 40 cm with speed DEFAULT_DRIVE_SPEED.
         * You can adjust the speed as well as the distance to time factor above, to get better results.
         * If you have have slot type photo interrupters assembled, you require no factor if defining USE_ENCODER_MOTOR_CONTROL in PWMDCMotor.h
         */
        RobotCarPWMMotorControl.goDistanceMillimeter(SIZE_OF_SQUARE_MILLIMETER, sMotorDirection);
        delay(400);
        /*
         * Try to turn by 90 degree.
         */
        RobotCarPWMMotorControl.rotate(90, sMotorDirection, NULL, true);
        delay(400);
    }

    /*
     * Turn car around and switch direction
     */
    RobotCarPWMMotorControl.rotate(180, TURN_IN_PLACE, NULL, true);
    sMotorDirection = oppositeDIRECTION(sMotorDirection);
    delay(2000);
}
