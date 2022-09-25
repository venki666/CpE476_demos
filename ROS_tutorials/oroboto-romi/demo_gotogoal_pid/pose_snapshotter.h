#ifndef __POSE_SNAPSHOTTER_H__
#define __POSE_SNAPSHOTTER_H__

#include "pose.h"
#include "ultrasonic.h"

#define MAX_POSE_SNAPSHOTS 48

class I2CInterface;

class PoseSnapshotter
{
  public:
    PoseSnapshotter(I2CInterface* i2c);

    void  resetSnapshotter();
    bool  recordSnapshot(double heading, double x, double y, bool enableRanging, double* distanceToObstacle);
    void  reportPoseSnapshots(Pose currentPose, bool enableRanging, bool abortedDueToObstacle, bool lastWaypointOfJourney);

  private:
    UltrasonicTransducer    ultrasonic;
    I2CInterface*           i2c;
    
    uint8_t       poseSnapshotCount;
    struct Pose   poseSnapshots[MAX_POSE_SNAPSHOTS];  // capture snapshots of our pose along each waypoint path for reporting
    uint32_t      waypointStartTime;
};

#endif // __POSE_SNAPSHOTTER_H__
