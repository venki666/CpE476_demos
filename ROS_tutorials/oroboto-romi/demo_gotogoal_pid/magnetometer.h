#ifndef __MAGNETOMETER_H__
#define __MAGNETOMETER_H__

#ifdef __ENABLE_MAGNETOMETER__

#include "QMC5883L.h"

class Sound;
class Motors;

class Magnetometer
{
  public:
    Magnetometer(Sound* sound, Motors* motors);

    void  calibrateMagnetometer();
    void  correctHeadingWithPivotTurn(double headingError, uint8_t pivotTurnSpeed);
    
  private:
  
    void  updateHeading();
    
    QMC5883L  compass;
    Sound*    sound;
    Motors*   motors;

    double    magnetoAngle;
    double    magnetoAngleRad;
};

#endif // __ENABLE_MAGNETOMETER__

#endif // __MAGNETOMETER_H__
