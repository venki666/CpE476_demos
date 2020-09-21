#ifndef __WAYPOINT_NAVIGATOR__
#define __WAYPOINT_NAVIGATOR__

#include "pose.h"

#define PID_PROPORTIONAL      50.00             // was 0.9
#define PID_INTEGRAL          0.001             // was 0.0005
#define PID_DERIVATIVE        0.00
#define PID_LOOP_INTERVAL_MS  50

class   Sound;
class   Motors;
class   Gyro;
class   PoseSnapshotter;
class   BotCmdCtx;

class WaypointNavigator
{
  public:
    WaypointNavigator(Sound* sound, Motors* motors, Gyro* gyro, PoseSnapshotter* poseSnapshotter);

    void    resetToOrigin();
    void    setPIDParameters(float proportional, float integral, float derivative, uint16_t loopInterval);

    void    executeTransit(BotCmdCtx cmdCtx);
    void    executeRotation(BotCmdCtx cmdCtx);

  private:
    bool    goToWaypoint(double x, double y, BotCmdCtx cmdCtx);
    void    rotate(double rad, uint8_t pivotTurnSpeed, bool enableRanging);

    double  getHeading(double toX, double toY, double fromX, double fromY, double currentHeading);
    void    resetDistancesForNewWaypoint();
  
    Sound*            sound;
    Motors*           motors;
    Gyro*             gyro;
    PoseSnapshotter*  poseSnapshotter;

    float    pidProportional;
    float    pidIntegral;
    uint16_t pidLoopIntervalMs;

    struct Pose currentPose;          // (believed) current position and heading
    struct Pose referencePose;        // pose of reference waypoint (heading is heading required from currentPose)

    double  headingError;              // for PID proportional term
    double  headingErrorPrev;          // for PID derivative term
    double  headingErrorIntegral;      // for PID integral term

    double  distLeft;                  // current distance travelled by left wheel in this waypoint segment
    double  distLeftPrev;              // previous current distance travelled by left wheel in this waypoint segment
    double  distRight;
    double  distRightPrev;
    double  distTotal;                 // total distance travelled in this waypoint segment
    double  distTotalPrev;             // previous total distance travelled in this waypoint segment

    uint8_t consecutiveCloseProximityReadings;

};

#endif // __WAYPOINT_NAVIGATOR__
