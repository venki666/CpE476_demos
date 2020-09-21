#include "motors.h"
#include "defaults.h"
#include "sound.h"
#include "gyro.h"
#include "pose.h"
#include "debug.h"

int16_t motorCalibrationBuckets[] = {30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240};     // Motor speed range is -300 to 300 (reverse to forward)

Motors::Motors(Sound* sound) : sound(sound)
{
    motorCalibrationRightToLeftRatio = NULL;
    motorCalibrationAdjustedBuckets = 0;
    motorCalibrationBucketCount = sizeof(motorCalibrationBuckets) / sizeof(int16_t);  
}


void Motors::setSpeeds(int left, int right)
{
    motors.setSpeeds(left, right);
}


void Motors::resetEncoders()
{
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
}


void Motors::getAndResetEncoderCounts(int16_t* left, int16_t* right)
{
    *left  = encoders.getCountsAndResetLeft();
    *right = encoders.getCountsAndResetRight();
    
    if (encoders.checkErrorLeft())
    {
       sound->alarm(false);
    }

    if (encoders.checkErrorRight())
    {
       sound->alarm(false);      
    }         
}


/**
 * Our heading is so wrong we should correct with a pivot (on-the-spot) rotation rather than PID (using odometry).
 */
void Motors::correctHeadingWithPivotTurn(double headingError, uint8_t pivotTurnSpeed)
{
    motors.setSpeeds(0, 0);
    delay(PIVOT_TURN_SLEEP_MS);
    
    double arcLength = ((headingError / (2*M_PI)) * (2*M_PI*BASERADIUS));

#ifdef __DEBUG__                        
    snprintf_P(report, sizeof(report), PSTR("   CORRECTING [%s] rad, spin arc length [%s]"), ftoa(floatBuf1, headingError), ftoa(floatBuf2, arcLength)); 
    Serial.println(report);      
#endif
    
    encoders.getCountsAndResetLeft();
    double spinDist = 0.0;
    arcLength = fabs(arcLength);
    
    if (headingError > 0)
    {
        // left backwards, right forwards (counter clockwise ("left")): radians are positive CCW
        motors.setSpeeds(-1 * pivotTurnSpeed, pivotTurnSpeed); 
    }
    else
    {
        // left forwards, right backwards (clockwise, ("right"))
        motors.setSpeeds(pivotTurnSpeed, -1 * pivotTurnSpeed);
    }
    
    while (spinDist < arcLength)
    {
        int16_t countsLeft = abs(encoders.getCountsLeft());
        int16_t countsRight = abs(encoders.getCountsRight());
        int16_t counts = (countsLeft + countsRight) / 2;
        
        unsigned long absCounts = abs(counts);
        spinDist = ((double)absCounts / ENCODER_COUNTS_PER_REVOLUTION) * (2*M_PI * WHEELRADIUS); 
    }
    
    motors.setSpeeds(0, 0);

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTED arc length [%s]"), ftoa(floatBuf2, spinDist)); 
    Serial.println(report);      
#endif
    
    delay(PIVOT_TURN_SLEEP_MS);    
}


/**
 * Get the calibrated motor speed for the left motor. 
 */
int16_t Motors::getCalibratedLeftMotorSpeed(int16_t desiredSpeed)
{
   if ( ! motorCalibrationRightToLeftRatio)
   {
#ifdef __DEBUG__    
      Serial.println("   Motor calibration has not been performed");
#endif      
      return desiredSpeed;
   }
   
   if (desiredSpeed < motorCalibrationBuckets[0] || desiredSpeed > motorCalibrationBuckets[motorCalibrationBucketCount - 1])
   {
#ifdef __DEBUG__        
       snprintf_P(report, sizeof(report), PSTR("No calibration data for speed %d"), desiredSpeed);
       Serial.println(report); 
#endif

       return desiredSpeed;
   }

   // Assume the buckets are equally divided into groups of 10
   uint8_t bucketNumber = (desiredSpeed - motorCalibrationBuckets[0]) / 10;
   int16_t leftSpeed = (float)desiredSpeed * motorCalibrationRightToLeftRatio[bucketNumber];

#ifdef __DEBUG__    
   snprintf_P(report, sizeof(report), PSTR("Using bucket %d (speed: %d) to adjust rightSpeed %d to leftSpeed %d (ratio: %s)"), bucketNumber, motorCalibrationBuckets[bucketNumber], desiredSpeed, leftSpeed, ftoa(floatBuf1, motorCalibrationRightToLeftRatio[bucketNumber]));
   Serial.println(report); 
#endif

   return leftSpeed;
}


/**
 * Convert velocity expressed in cm/s to a motor speed integer in the range[-300, 300].
 */
int Motors::convertVelocityToMotorSpeed(double velocity)
{
    int speed = (velocity / MAX_SPEED_CM_PER_SEC) * MAX_SPEED;

    if (speed > 0)
    {
        if (speed < MIN_SPEED)
        {
           speed = MIN_SPEED;          
        }
        else if (speed > MAX_SPEED)
        {
            speed = MAX_SPEED;
        }
    }
    else if (speed < 0)
    {
       if (speed > (-1 * MIN_SPEED))
       {
          speed = -1 * MIN_SPEED;
       }
       else if (speed < (-1 * MAX_SPEED))
       {
          speed = -1 * MAX_SPEED;
       }
    }

    return speed;
}


/****************************************************************************************************************
 * Calibration 
 ****************************************************************************************************************/

/**
 * Calibrate motor speeds. We will adjust the left motor speed to try and balance it with the right.
 */
void Motors::calibrate(uint8_t sampleCount, uint8_t pivotTurnSpeed, Gyro* gyro) 
{
   uint16_t i;
   uint32_t* leftTotals, *rightTotals, countTravelled = 0;
   int16_t leftCount;

#ifdef __DEBUG__                    
   Serial.println("Calibrating motors ...");
#endif

   motorCalibrationRightToLeftRatio = (float*)malloc(motorCalibrationBucketCount * sizeof(float));
   leftTotals = (uint32_t*)malloc(motorCalibrationBucketCount * sizeof(uint32_t));
   rightTotals = (uint32_t*)malloc(motorCalibrationBucketCount * sizeof(uint32_t));

   memset(leftTotals, 0, motorCalibrationBucketCount * sizeof(uint32_t));
   memset(rightTotals, 0, motorCalibrationBucketCount * sizeof(uint32_t));

   sound->alarm(true);

   for (i = 0; i < motorCalibrationBucketCount * sampleCount; i++)
   {
       uint16_t speedBucket = i % motorCalibrationBucketCount;

       delay(500);
       
       encoders.getCountsAndResetLeft();
       encoders.getCountsAndResetRight();
       
       motors.setSpeeds(motorCalibrationBuckets[speedBucket], motorCalibrationBuckets[speedBucket]);
       delay(3000);
       motors.setSpeeds(0, 0);

       leftCount = encoders.getCountsAndResetLeft();

       leftTotals[speedBucket]  += leftCount;
       rightTotals[speedBucket] += encoders.getCountsAndResetRight();
       countTravelled           += abs(leftCount);

       if (((countTravelled / ENCODER_COUNTS_PER_REVOLUTION) * WHEELRADIUS * 2 * M_PI) > 70)
       {
          if (gyro)
          {
              Pose dummyPose;
              gyro->correctHeadingWithPivotTurn(dummyPose /* only required for snapshot recording */, M_PI / 2, pivotTurnSpeed, false);            
          }
          else
          {
              correctHeadingWithPivotTurn(M_PI / 2, pivotTurnSpeed);
          }

          countTravelled = 0;
       }
   }

   // Calculate required adjustment factors
   for (i = 0; i < motorCalibrationBucketCount; i++)
   {
       motorCalibrationRightToLeftRatio[i] = 1.0;

       if (leftTotals[i])
       {
           float observedRatio = (float)rightTotals[i] / (float)leftTotals[i];

           // What would left's new speed if using the adjustment factor?
           int left = (float)motorCalibrationBuckets[i] * observedRatio;

           // We only have integer granularity: if the difference between the two speeds is more than the adjustment factor, don't adjust or we'll overcook it.
           float correctedDiff = abs(left - motorCalibrationBuckets[i]) / (float)motorCalibrationBuckets[i];
           float observedDiff = fabs(1 - observedRatio);

           if ((left == motorCalibrationBuckets[i]) || (correctedDiff > observedDiff))
           {
#ifdef __DEBUG__                                
               snprintf_P(report, sizeof(report), PSTR("bucket[%d] l: %d diff: %s > %s, ignoring"), motorCalibrationBuckets[i], left, ftoa(floatBuf1, correctedDiff), ftoa(floatBuf2, observedDiff));
               Serial.println(report);
#endif                
           }
           else
           {
#ifdef __DEBUG__                                
               snprintf_P(report, sizeof(report), PSTR("bucket[%d] l: %d diff: %s <= %s, ADJUSTING"), motorCalibrationBuckets[i], left, ftoa(floatBuf1, correctedDiff), ftoa(floatBuf2, observedDiff));
               Serial.println(report); 
#endif

               motorCalibrationRightToLeftRatio[i] = observedRatio;
               motorCalibrationAdjustedBuckets++;
           }
       }
   }

   free(leftTotals);
   free(rightTotals);

   sound->alarm(true);

#ifdef __DEBUG__                    
   Serial.println("Motor calibration complete ...");
#endif   
}
