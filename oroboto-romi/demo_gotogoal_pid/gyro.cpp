#include <Romi32U4.h>
#include "gyro.h"
#include "motors.h"
#include "pose_snapshotter.h"
#include "sound.h"
#include "pose.h"
#include "defaults.h"
#include "debug.h"

Gyro::Gyro(Sound* sound, Motors* motors, PoseSnapshotter* poseSnapshotter) : sound(sound), motors(motors), poseSnapshotter(poseSnapshotter)
{
    gyroOffset = 0;           
    gyroLastUpdateMicros = 0;
    gyroAngleDifference = 0;

    resetAngles();
}


/**
 * Whatever our current heading is is now considered 0 degrees.
 */
void Gyro::resetAngles()
{
    gyroAngle = 0;
    gyroAngleRad = 0;  
}


void Gyro::resetLastUpdateTimeToNow()
{
    gyroLastUpdateMicros = micros();
}


double Gyro::getAngleRad()
{
    return gyroAngleRad;
}


/**
 * Update gyroscope heading.
 * 
 * Similar to getHeading(), this will always update gyroAngleRad to a value between 0 and 2*PI where the value grows in the
 * counter-clockwise direction.
 */
void Gyro::updateHeading()
{
    while ( ! imu.readReg(LSM6::STATUS_REG) & 0x08);
    imu.read();

    if (imu.timeoutOccurred())
    {
       sound->alarm(true);
    }

    // Update the orientation as determined by the gyroscope
    int16_t gyroTurnRate = imu.g.z - gyroOffset;      // int16_t

    unsigned long now = micros();                     // uint32_t
    unsigned long dt = now - gyroLastUpdateMicros;
    gyroLastUpdateMicros = now;

    // Determine how much we've rotated around Z axis since last measurement
    int32_t rotation = (int32_t)gyroTurnRate * dt;    // signed 16 bits -> signed 32 bits multiplied by unsigned long

    // rotation is measured in gyro digits * micro-seconds. convert to degrees and add to the current angle
    gyroAngleDifference = ((double)rotation * GYRODIGITS_TO_DPS) * 0.000001;
    gyroAngle += gyroAngleDifference;  

    while (gyroAngle < 0)
    {
        gyroAngle += 360;
    }
    while (gyroAngle > 360)
    {
        gyroAngle -= 360;
    }

    gyroAngleRad = gyroAngle * (2*M_PI / 360.0);
}


/**
 * Our heading is so wrong we should correct with a pivot (on-the-spot) rotation rather than PID (using gyroscope).
 */
void Gyro::correctHeadingWithPivotTurn(Pose currentPose, double headingError, uint8_t pivotTurnSpeed, bool enableRanging)
{
    motors->setSpeeds(0, 0);

    updateHeading();
    double diff, delaySafeHeadingError = 0.75 * headingError, gyroStartAngleRad = gyroAngleRad;
    bool skip;
    int i = 0;

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTING [%s] rad %s, gyroStartAngleRad [%s]"), ftoa(floatBuf1, headingError), ((headingError > 0) ? "CCW" : "CW"), ftoa(floatBuf2, gyroStartAngleRad)); 
    Serial.println(report);      
#endif

    delay(PIVOT_TURN_SLEEP_MS);
    
    if (headingError > 0)
    {
        // left backwards, right forwards (counter clockwise ("left")): radians are positive CCW
        motors->setSpeeds(-1 * pivotTurnSpeed, pivotTurnSpeed); 

        do
        {
           delay(10);
           updateHeading();
           skip = false;

           // Ignore noise, the angle SHOULD be increasing
           if ((fabs(gyroStartAngleRad - gyroAngleRad) < 0.02) || (fabs(gyroStartAngleRad - gyroAngleRad) > ((2*M_PI) - 0.02)))
           {
#ifdef __DEBUG__                                
//            Serial.println("Ignoring noise");
#endif
              skip = true;
              continue;
           }
          
           if (gyroAngleRad > gyroStartAngleRad)
           {
              diff = gyroAngleRad - gyroStartAngleRad;
           }
           else
           {
              diff = ((2*M_PI) - gyroStartAngleRad) + gyroAngleRad;
           }

           // Only record a snapshot (which requires an ADC read, which can impact timing) during the first 75% of the rotation and even then only every 400ms
           if ((diff < delaySafeHeadingError) && ((i++ % 40) == 0))
           {
              poseSnapshotter->recordSnapshot(currentPose.heading + diff, currentPose.x, currentPose.y, enableRanging, NULL);
           }

#ifdef __DEBUG__                    
//           Serial.println(diff);
#endif
        }
        while (skip || (diff < headingError));
    }
    else
    {
        // left forwards, right backwards (clockwise, ("right"))
        motors->setSpeeds(pivotTurnSpeed, -1 * pivotTurnSpeed);

        do
        {
            delay(10);
            updateHeading();
            skip = false;

            // Ignore noise, the angle SHOULD be decreasing
            if ((fabs(gyroStartAngleRad - gyroAngleRad) < 0.02) || (fabs(gyroStartAngleRad - gyroAngleRad) > ((2*M_PI) - 0.02)))
            {
#ifdef __DEBUG__                                  
//              Serial.println("Ignoring noise");
#endif
                skip = true;
                continue;
            }
            
            if (gyroAngleRad < gyroStartAngleRad)
            {
                diff = gyroAngleRad - gyroStartAngleRad;
            }
            else
            {
                diff = -1.0 * (gyroStartAngleRad + ((2*M_PI) - gyroAngleRad)); 
            }

           // Only record a snapshot (which requires an ADC read, which can impact timing) during the first 75% of the rotation and even then only every 400ms
           if ((diff > delaySafeHeadingError) && ((i++ % 40) == 0))
           {
              poseSnapshotter->recordSnapshot(currentPose.heading + diff, currentPose.x, currentPose.y, enableRanging, NULL);
           }

#ifdef __DEBUG__                    
//            Serial.println(diff);
#endif
        }
        while (skip || (diff > headingError));
    }
    
    motors->setSpeeds(0, 0);

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTED heading error of [%s], current gyroAngleRad [%s] (diff [%s])"), ftoa(floatBuf1, headingError), ftoa(floatBuf2, gyroAngleRad), ftoa(floatBuf3, diff)); 
    Serial.println(report);      
#endif    

    delay(PIVOT_TURN_SLEEP_MS); 
}


/****************************************************************************************************************
 * Calibration 
 ****************************************************************************************************************/

/**
 * Calibrate fixed offset out of the gyroscope.
 */
void Gyro::calibrate()
{
    delay(5000);                          // give user time to place device at rest
    sound->alarm(true);

#ifdef __DEBUG__                    
    Serial.println("Calibrating gyroscope ...");
#endif
    
    if ( ! imu.init())
    {
#ifdef __DEBUG__                          
        Serial.println("Failed to detect or initialize IMU, halting.");
#endif        
        while(1);
    }

    imu.enableDefault();
    delay(500);

    int32_t gyroTotal = 0;
    for (uint16_t i = 0; i < 1024; i++)
    {
        while ( ! imu.readReg(LSM6::STATUS_REG) & 0x08);
        imu.read();
        gyroTotal += imu.g.z;             // gyro values are int16_t (signed)
    }

    gyroOffset = gyroTotal / 1024;        // average across all 1024 samples

#ifdef __DEBUG__                        
    Serial.println("Gyroscope calibration complete");
#endif
    
    sound->alarm(true);
}

