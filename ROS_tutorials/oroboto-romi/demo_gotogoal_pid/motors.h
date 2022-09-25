#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <Romi32U4.h>

#define MIN_SPEED            20                 // motors stall below this point
#define MAX_SPEED            300                // on-floor tests show 100 =~ 26.13cm/s, 200 =~ 44.24 cm/s, 300 =~ 65.49cm/s
#define MAX_SPEED_CM_PER_SEC 65.49

class Sound;
class Gyro;

class Motors
{
  public:
    Motors(Sound* sound);

    void    setSpeeds(int left, int right);
    void    correctHeadingWithPivotTurn(double headingError, uint8_t pivotTurnSpeed);
    int     convertVelocityToMotorSpeed(double velocity);

    void    resetEncoders();
    void    getAndResetEncoderCounts(int16_t* left, int16_t* right);

    void    calibrate(uint8_t sampleCount, uint8_t pivotTurnSpeed, Gyro* gyro);
    int16_t getCalibratedLeftMotorSpeed(int16_t desiredSpeed);

  private:
    Sound*            sound;
    Romi32U4Motors    motors;
    Romi32U4Encoders  encoders;

    float * motorCalibrationRightToLeftRatio;
    uint8_t motorCalibrationAdjustedBuckets;
    uint8_t motorCalibrationBucketCount;

};

#endif // __MOTORS_H__
