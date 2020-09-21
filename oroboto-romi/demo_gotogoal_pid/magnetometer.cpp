#include <Romi32U4.h>
#include "magnetometer.h"
#include "sound.h"
#include "motors.h"
#include "debug.h"

#ifdef __ENABLE_MAGNETOMETER__

Magnetometer::Magnetometer(Sound* sound, Motors* motors) : sound(sound), motors(motors)
{
    magnetoAngle = 0;
    magnetoAngleRad = 0;  
}


/**
 * Update magnetometer heading (blocks until magnetometer ready)
 * 
 * Similar to getHeading(), this will always update magnetoAngleRad to a value between 0 and 2*PI where the value grows in the
 * counter-clockwise direction.
 */
void Magnetometer::updateHeading()
{
    while ( ! compass.ready())
    {
        delay(1);
    }

    // 360 - heading converts compass so that it grows from 0..360 CCW (ie. opposite to actual compass)
    magnetoAngle = 360 - compass.readHeading();
    magnetoAngleRad = magnetoAngle * ((2*M_PI) / 360.0);
}


/**
 * Our heading is so wrong we should correct with a pivot (on-the-spot) rotation rather than PID (using magnetometer).
 */
void Magnetometer::correctHeadingWithPivotTurn(double headingError, uint8_t pivotTurnSpeed)
{
    motors->setSpeeds(0, 0);

    updateHeading();
    double diff, magnetoStartAngleRad = magnetoAngleRad;
    bool skip;

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTING [%s] rad %s, magnetoStartAngleRad [%s]"), ftoa(floatBuf1, headingError), ((headingError > 0) ? "CCW" : "CW"), ftoa(floatBuf2, magnetoStartAngleRad)); 
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
           if ((fabs(magnetoStartAngleRad - magnetoAngleRad) < 0.04) || (fabs(magnetoStartAngleRad - magnetoAngleRad) > ((2*M_PI) - 0.04)))
           {
#ifdef __DEBUG__                    
//            Serial.println("Ignoring noise");
#endif
              skip = true;
              continue;
           }
          
           if (magnetoAngleRad > magnetoStartAngleRad)
           {
              diff = magnetoAngleRad - magnetoStartAngleRad;
           }
           else
           {
              diff = ((2*M_PI) - magnetoStartAngleRad) + magnetoAngleRad;
           }

#ifdef __DEBUG__                    
           Serial.println(diff);
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
            if ((fabs(magnetoStartAngleRad - magnetoAngleRad) < 0.04) || (fabs(magnetoStartAngleRad - magnetoAngleRad) > ((2*M_PI) - 0.04)))
            {
#ifdef __DEBUG__                                  
//              Serial.println("Ignoring noise");
#endif
                skip = true;
                continue;
            }
            
            if (magnetoAngleRad < magnetoStartAngleRad)
            {
                diff = magnetoAngleRad - magnetoStartAngleRad;
            }
            else
            {
                diff = -1.0 * (magnetoStartAngleRad + ((2*M_PI) - magnetoAngleRad)); 
            }

#ifdef __DEBUG__                    
            Serial.println(diff);
#endif            
        }
        while (skip || (diff > headingError));
    }
    
    motors->setSpeeds(0, 0);

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTED heading error of [%s], current magnetoAngleRad [%s] (diff [%s])"), ftoa(floatBuf1, headingError), ftoa(floatBuf2, magnetoAngleRad), ftoa(floatBuf3, diff)); 
    Serial.println(report);      
#endif

    delay(PIVOT_TURN_SLEEP_MS); 
}


/****************************************************************************************************************
 * Calibration 
 ****************************************************************************************************************/

/**
 * Calibrate magnetometer
 */
void Magnetometer::calibrateMagnetometer()
{
    delay(5000);                          // give user time to pick up device
    sound->alarm(true);

#ifdef __DEBUG__                    
    Serial.println("Calibrating compass, please move device around all axes ...");
#endif

    compass.init();
    compass.resetCalibration();

    int calibrationTime = 0;
    while (calibrationTime < 20000)
    {
        compass.readHeading();
        delay(200);
        calibrationTime += 200;
    }

#ifdef __DEBUG__                    
    Serial.println("Compass calibration complete");
#endif    

    sound->alarm();
}

#endif  // __ENABLE_MAGNETOMETER__

