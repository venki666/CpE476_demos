#ifndef __GYRO_H__
#define __GYRO_H__

#include <LSM6.h>

#define GYRODIGITS_TO_DPS 0.00875

class Sound;
class Motors;
class PoseSnapshotter;
class Pose;

class Gyro
{
  public:
    Gyro(Sound* sound, Motors* motors, PoseSnapshotter* poseSnapshotter);

    void    resetAngles();
    void    resetLastUpdateTimeToNow();
    
    void    calibrate();
    void    correctHeadingWithPivotTurn(Pose currentPose, double headingError, uint8_t pivotTurnSpeed, bool enableRanging);

    double  getAngleRad();
    void    updateHeading();

  private:    
    LSM6              imu;
    Sound*            sound;
    Motors*           motors;
    PoseSnapshotter*  poseSnapshotter;

    int16_t           gyroOffset;           // average reading on gyro Z axis during calibration
    unsigned long     gyroLastUpdateMicros; // helps calculate dt for gyro readings (in microseconds)
    double            gyroAngle;
    double            gyroAngleRad;
    double            gyroAngleDifference;  
};

#endif // __GYRO_H__
