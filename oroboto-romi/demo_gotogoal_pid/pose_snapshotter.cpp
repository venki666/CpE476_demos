#include "pose_snapshotter.h"
#include "ultrasonic.h"
#include "i2c_interface.h"
#include "debug.h"

PoseSnapshotter::PoseSnapshotter(I2CInterface* i2c) : i2c(i2c)
{
   resetSnapshotter();
}

void PoseSnapshotter::resetSnapshotter()
{  
   poseSnapshotCount = 0;  
   waypointStartTime = millis();
}

/**
 * Record a snapshot of the current position and a range finding to the nearest obstacle.
 */
bool PoseSnapshotter::recordSnapshot(double heading, double x, double y, bool enableRanging, double* pDistanceToObstacle)
{
    bool    recordedSnapshot = false;
    double  distanceToObstacle = 0;
    
    if (poseSnapshotCount < MAX_POSE_SNAPSHOTS)
    {
        recordedSnapshot = true;
        
        poseSnapshots[poseSnapshotCount].timestamp = (uint16_t)(millis() - waypointStartTime);
        poseSnapshots[poseSnapshotCount].heading = heading;
        poseSnapshots[poseSnapshotCount].x = x;
        poseSnapshots[poseSnapshotCount].y = y;

        if (enableRanging)
        {
            distanceToObstacle = ultrasonic.getSonarRangedDistance();
        }

        poseSnapshots[poseSnapshotCount].distanceToObstacle = distanceToObstacle;

        if (pDistanceToObstacle)
        {
            *pDistanceToObstacle = distanceToObstacle;
        }
        
        poseSnapshotCount++;          
    }

    return recordedSnapshot;
}

/**
 * Report the pose snapshots from the last waypoint to the Raspberry Pi.
 */
void PoseSnapshotter::reportPoseSnapshots(Pose currentPose, bool enableRanging, bool abortedDueToObstacle, bool lastWaypointOfJourney)
{
    // The last snapshot is always our current position
    if (poseSnapshotCount >= MAX_POSE_SNAPSHOTS)
    {
        poseSnapshotCount--;
    }

    recordSnapshot(currentPose.heading, currentPose.x, currentPose.y, enableRanging, NULL);

#ifdef __DEBUG__            
    snprintf_P(report, sizeof(report), PSTR("Reporting %d pose snapshots:"), poseSnapshotCount);      
    Serial.println(report);  
#endif

    i2c->reportPoseSnapshots(poseSnapshotCount, poseSnapshots, abortedDueToObstacle, lastWaypointOfJourney);
}
