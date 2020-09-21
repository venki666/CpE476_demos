#ifndef __POSE_H__
#define __POSE_H__

#include <Romi32U4.h>

struct Pose {
  double        x;
  double        y;
  double        heading;
  double        distanceToObstacle;
  uint16_t      timestamp;  
};

#endif // __POSE_H__

