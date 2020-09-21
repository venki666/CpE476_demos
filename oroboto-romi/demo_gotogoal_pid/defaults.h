#ifndef __DEFAULTS_H__
#define __DEFAULTS_H__

#define WHEELBASE        14.9225                // cm
#define BASERADIUS       7.46125                // cm
#define WHEELRADIUS      3.4925                 // cm (was 3.65125)

#define ENCODER_COUNTS_PER_REVOLUTION 1440.0            // number of encoder counts per wheel revolution (should be ~1440)

#define PIVOT_TURN true                         // pivot turn instead of PID mode for large heading corrections
#define PIVOT_TURN_SPEED 30                     // known good speed is 30
#define PIVOT_TURN_THRESHOLD 12.0               // use pivot if heading correction > 2*PI / PIVOT_TURN_THRESHOLD
#define PIVOT_TURN_SLEEP_MS 200                 // known good sleep is 500

#define USE_GYRO_FOR_HEADING false
#define USE_GYRO_FOR_PIVOT_TURN true

/**
 * Waypoint Navigation Configuration
 */
#define MAX_VELOCITY         5                  // cm/s (slowest known good max velocity is "5")
#define VELOCITY_ON_APPROACH 60                 // was 1.0, if this is too low (ie. 1.0) we end up stalling before hitting the waypoint, and fall short of it (at WAYPOINT_PROXIMITY_APPROACHING we stall, so normally around 5.0cm short)

#define CAP_ANGULAR_VELOCITY_SIGNAL true        // cap angular velocity correction allowed in PID mode
#define ANGULAR_VELOCITY_SIGNAL_LIMIT 0.2

#define ABORT_WAYPOINT_AFTER_DISTANCE_INCREASES false   // true
#define DISTANCE_INCREASE_ITERATION_THRESHOLD 1
#define DISTANCE_INCREASE_DRIFT_FROM_INITIAL_VECTOR_THRESHOLD 0.05    // used when ABORT_WAYPOINT_AFTER_DISTANCE_INCREASES is false

#define PERIODIC_REFERENCE_HEADING_RESET true   // should we periodically recalculate required reference heading during waypoint finding?
#define PERIODIC_REFERENCE_HEADING_INTERVAL 5   // reset every n iterations of the PID loop (10 is known good)

#define CLOSE_PROXIMITY_THRESHOLD 17.0
#define MAX_CONSECUTIVE_CLOSE_PROXIMITY_READINGS 2
#define OBSTACLE_AVOIDANCE_DISTANCE 30.0

#define WAYPOINT_PROXIMITY_APPROACHING 5.0      // was 10.0
#define WAYPOINT_PROXIMITY_REACHED 1.0          // known good is 2.0
#define POST_WAYPOINT_SLEEP_MS 300              // known good sleep is 1000

#endif // __DEFAULTS_H__
