#include <Romi32U4.h>
#include <Wire.h>
#include "defaults.h"
#include "sound.h"
#include "motors.h"
#include "gyro.h"
#include "i2c_interface.h"
#include "pose_snapshotter.h"
#include "waypoint_navigator.h"
#include "debug.h"

Sound                 sound;
I2CInterface          i2c(&sound);
PoseSnapshotter       poseSnapshotter(&i2c);
Motors                motors(&sound);
Gyro                  gyro(&sound, &motors, &poseSnapshotter);
WaypointNavigator     waypointNavigator(&sound, &motors, &gyro, &poseSnapshotter);

#ifdef __ENABLE_MAGNETOMETER__
Magnetometer        magnetometer(&sound, &motors);
#endif

/*****************************************************************************************************
 * Entrypoint
 *****************************************************************************************************/

void setup() 
{
    Serial.begin(9600);
    Wire.begin();

    sound.boot();

    if (USE_GYRO_FOR_HEADING || USE_GYRO_FOR_PIVOT_TURN)
    {
#ifdef __ENABLE_MAGNETOMETER__      
        magnetometer.calibrate();      // far less accurate than the gyro until we can use it for fusion
#endif
        gyro.calibrate();
    }    
}

void loop() 
{
   if (i2c.pollForCommands())
   {
      switch (i2c.cmdCtx.cmd)
      {
          case BotCmd::CalibrateMotors:
#ifdef __DEBUG__
            Serial.println("CMD: CalibrateMotors");
#endif                             
            motors.calibrate(2, i2c.cmdCtx.pivotTurnSpeed, &gyro);
            break;

          case BotCmd::ResetToOrigin:
#ifdef __DEBUG__
            Serial.println("CMD: ResetToOrigin");
#endif         
            waypointNavigator.resetToOrigin();
            break;

          case BotCmd::SetPidParameters:
#ifdef __DEBUG__
            Serial.println("CMD: SetPidParameters");
#endif                   
            waypointNavigator.setPIDParameters(i2c.cmdCtx.pidProportional, i2c.cmdCtx.pidIntegral, 0, i2c.cmdCtx.pidLoopIntervalMs);
            break;

          case BotCmd::Rotate:
#ifdef __DEBUG__
            snprintf_P(report, sizeof(report), PSTR("CMD: Rotate [%s rad]"), ftoa(floatBuf1, i2c.cmdCtx.rotationRadians));      
            Serial.println(report);  
#endif                   
            waypointNavigator.executeRotation(i2c.cmdCtx);
            break;

          case BotCmd::TransitViaWaypoints:
            waypointNavigator.executeTransit(i2c.cmdCtx);
            break;

          case BotCmd::ReportStatus:
            i2c.reportStatus();
            break;
      }
   }

   delay(100);
}
 

