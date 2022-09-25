#include <Romi32U4.h>
#include "waypoint_navigator.h"
#include "i2c_interface.h"
#include "sound.h"
#include "motors.h"
#include "gyro.h"
#include "pose_snapshotter.h"
#include "defaults.h"
#include "debug.h"

WaypointNavigator::WaypointNavigator(Sound* sound, Motors* motors, Gyro* gyro, PoseSnapshotter* poseSnapshotter) : sound(sound), motors(motors), gyro(gyro), poseSnapshotter(poseSnapshotter)
{
    pidProportional = PID_PROPORTIONAL;
    pidIntegral = PID_INTEGRAL;
    pidLoopIntervalMs = PID_LOOP_INTERVAL_MS;  

    resetToOrigin();
}

/**
 * Reset pose to (0,0) with heading of 0 rad.
 */
void WaypointNavigator::resetToOrigin() 
{
    headingError = 0.0;
    headingErrorPrev = 0.0;
    headingErrorIntegral = 0.0;

    currentPose.x = 0.0;
    currentPose.y = 0.0;
    currentPose.heading = 0.0;
    currentPose.timestamp = 0;

    referencePose.x = 0.0;
    referencePose.y = 0.0;
    referencePose.heading = 0.0;
    referencePose.timestamp = 0;

    gyro->resetAngles();              // assume we're heading 0 degrees

    resetDistancesForNewWaypoint();
}


/**
 * Reset distance travelled measurements. This must be done at the start of each waypoint segment. 
 * 
 * NOTE: We don't reset our current pose. This allows us to maintain location state within the world during consecutive waypoint transits.
 */
void WaypointNavigator::resetDistancesForNewWaypoint() 
{
    distLeft = 0.0;
    distLeftPrev = 0.0;
    distRight = 0.0;
    distRightPrev = 0.0;
    distTotal = 0.0;
    distTotalPrev = 0.0;  
    consecutiveCloseProximityReadings = 0;

    gyro->resetLastUpdateTimeToNow();
    poseSnapshotter->resetSnapshotter();
}


/**
 * Set the PID parameters used for transits.
 */
void WaypointNavigator::setPIDParameters(float proportional, float integral, float derivative, uint16_t loopInterval)
{
    pidProportional = proportional;
    pidIntegral = integral;
    pidLoopIntervalMs = loopInterval;
}


/**
 * Execute a transit between the waypoints specified in the command context.
 */
void WaypointNavigator::executeTransit(BotCmdCtx cmdCtx)
{      
#ifdef __DEBUG__
    snprintf_P(report, sizeof(report), PSTR("START: range[%d] abort[%d] cap[%d] refReset[%d] P[%s] I[%s] dt[%d]"), cmdCtx.enableRanging, cmdCtx.abortAfterDistanceToWaypointIncreases, cmdCtx.capAngularVelocity, cmdCtx.periodicReferenceHeadingReset, ftoa(floatBuf1, pidProportional), ftoa(floatBuf2, pidIntegral), pidLoopIntervalMs);    
    Serial.println(report);
#endif           

    ledYellow(1);
      
    for (int8_t i = 0; i < cmdCtx.waypointPayloadCount; i++)
    {
        bool abortedDueToObstacle = goToWaypoint((double)cmdCtx.waypointPayload[(i*2)], (double)cmdCtx.waypointPayload[(i*2)+1], cmdCtx);

        poseSnapshotter->reportPoseSnapshots(currentPose, cmdCtx.enableRanging, abortedDueToObstacle, i == (cmdCtx.waypointPayloadCount - 1));

        if (abortedDueToObstacle && cmdCtx.avoidObstacles)
        {
           uint8_t attempts = 0;

           while (attempts < 7 && abortedDueToObstacle)
           {
              // Rotate 45 degrees clockwise and try again
              double avoidanceHeading = atan2(sin(currentPose.heading - (M_PI / 4)), cos(currentPose.heading - (M_PI / 4)));
              double avoidanceWaypointX = currentPose.x + (OBSTACLE_AVOIDANCE_DISTANCE * cos(avoidanceHeading));
              double avoidanceWaypointY = currentPose.y + (OBSTACLE_AVOIDANCE_DISTANCE * sin(avoidanceHeading));

#ifdef __DEBUG__
              snprintf_P(report, sizeof(report), PSTR("Collision avoidance attempt[%d], target (%s,%s)"), attempts, ftoa(floatBuf1, avoidanceWaypointX), ftoa(floatBuf2, avoidanceWaypointY));    
              Serial.println(report);
#endif

              abortedDueToObstacle = goToWaypoint(avoidanceWaypointX, avoidanceWaypointY, cmdCtx);   
              poseSnapshotter->reportPoseSnapshots(currentPose, cmdCtx.enableRanging, abortedDueToObstacle, false /* last waypoint of journey can never be an obstacle avoidance waypoint */); 

              attempts++;
           }

#ifdef __DEBUG__
           snprintf_P(report, sizeof(report), PSTR("Collision avoidance finished %s after %d attempts"), abortedDueToObstacle ? "unsuccessfully" : "successfully", attempts);    
           Serial.println(report);
#endif

           if (attempts == 7 && abortedDueToObstacle)
           {
              sound->failedObstacleAvoidance();
              break;   // we're blocked on all sides!
           }
           else
           {
              i--;     // try the current waypoint again
           }
        }
   }

   sound->finished();

   ledYellow(0);
}


/**
 * Go to the specified waypoint from the current waypoint.
 */
bool WaypointNavigator::goToWaypoint(double x, double y, BotCmdCtx cmdCtx) 
{
    double    targetVectorMagnitude;                    // current distance between where we think we are and the waypoint
    double    targetVectorMagnitudeLast = 0.0;          // last distance between where we think we were and the waypoint
    double    targetVectorMagnitudeInitial = 0.0;       // initial distance between where we think were are the waypoint
    double    targetVectorMagnitudeAtStartOfDrift;      // if we begin to get further from target (rather than closer), what was our distance to the target when this started?
    uint8_t   consecutiveIncreasingDistances = 0;
    bool      approachingTarget = false;                // once we start getting close to waypoint, don't forget that
    bool      abortedDueToObstacle = false;

    double    velocityLeft = 0.0, velocityRight = 0.0;  // current velocity of left and right wheels

    resetDistancesForNewWaypoint();

    // @todo Should we really throw away PID error from the previous waypoint here?
    headingErrorPrev = 0.0;
    headingErrorIntegral = 0.0;

#ifdef __DEBUG__
    snprintf_P(report, sizeof(report), PSTR("************ WAYPOINT (%s,%s) from (%s,%s head [%s]) ************"), ftoa(floatBuf1, x), ftoa(floatBuf2, y), ftoa(floatBuf3, currentPose.x), ftoa(floatBuf4, currentPose.y), ftoa(floatBuf5, currentPose.heading));    
    Serial.println(report);      
#endif

    referencePose.x = x;
    referencePose.y = y;

    // We need to keep our heading as close to this reference heading as possible to reach the waypoint
    referencePose.heading = getHeading(referencePose.x, referencePose.y, currentPose.x, currentPose.y, currentPose.heading);

    uint32_t      iteration = 0;
    double        dt = pidLoopIntervalMs / 1000.0;
    unsigned long lastMillis = 0;
    int           lastLeftSpeed = 0, lastRightSpeed = 0;

    // Reset wheel encoder counts to zero, we don't care about their current values
    motors->resetEncoders();
    
    while (true)
    {
        if (iteration != 0)
        {
            dt = (millis() - lastMillis) / 1000.0;

            // Recalculate reference heading every now and then as our current position changes
            if (cmdCtx.periodicReferenceHeadingReset && (iteration % PERIODIC_REFERENCE_HEADING_INTERVAL == 0))
            {
                referencePose.heading = getHeading(referencePose.x, referencePose.y, currentPose.x, currentPose.y, currentPose.heading);

#ifdef __DEBUG__                
                snprintf_P(report, sizeof(report), PSTR("corrected reference heading to [%s])"), ftoa(floatBuf1, referencePose.heading));    
                Serial.println(report);      
#endif                
            }
        }

#ifdef __DEBUG__
//      snprintf_P(report, sizeof(report), PSTR("[%3d] to (%s, %s, head [%s]) dist: %s"), iteration, ftoa(floatBuf1, referencePose.x), ftoa(floatBuf2, referencePose.y), ftoa(floatBuf3, referencePose.heading), ftoa(floatBuf4, targetVectorMagnitudeInitial)); 
//      Serial.println(report);      
//      snprintf_P(report, sizeof(report), PSTR("   was (%s, %s, head [%s]) distTgt[%s]"), ftoa(floatBuf1, currentPose.x), ftoa(floatBuf2, currentPose.y), ftoa(floatBuf3, currentPose.heading), ftoa(floatBuf4, targetVectorMagnitudeLast));    
//      Serial.println(report);      
#endif

        int16_t countsLeft, countsRight;
        motors->getAndResetEncoderCounts(&countsLeft, &countsRight);  // 1440 per revolution = 2*PI*radius cm travel

        if (iteration != 0)
        {
            //
            // Update heading (using odometry and/or gyroscope if this isn't the first iteration of the waypoint (ie. we've actually travelled somewhere)
            //

            if (USE_GYRO_FOR_HEADING)
            {
               gyro->updateHeading();
            }
            
            distLeft  += ((double)countsLeft / ENCODER_COUNTS_PER_REVOLUTION)  * (2*M_PI * WHEELRADIUS);        // cm travelled by left wheel (total, note the increment)
            distRight += ((double)countsRight / ENCODER_COUNTS_PER_REVOLUTION) * (2*M_PI * WHEELRADIUS);        // cm travelled by right wheel (total, note the increment)
            distTotal = (distLeft + distRight) / 2.0;                                                   // cm travelled forward (total)

#ifdef __DEBUG__
            snprintf_P(report, sizeof(report), PSTR("travelled L[%s] R[%s] dist[%s] distPrev[%s] dt[%s] [%d]"), ftoa(floatBuf1, distLeft), ftoa(floatBuf2, distRight), ftoa(floatBuf3, distTotal), ftoa(floatBuf4, distTotalPrev), ftoa(floatBuf5, dt), iteration); 
            Serial.println(report);      
#endif

            // How far have we travelled in this last iteration?
            double distDelta      = distTotal - distTotalPrev;
            double distLeftDelta  = distLeft - distLeftPrev;
            double distRightDelta = distRight - distRightPrev;

            currentPose.x += distDelta * cos(currentPose.heading);
            currentPose.y += distDelta * sin(currentPose.heading);

            // Update the heading as it has changed based on the distance travelled too
            currentPose.heading += ((distRightDelta - distLeftDelta) / WHEELBASE);

            if (USE_GYRO_FOR_HEADING)
            {
                currentPose.heading = gyro->getAngleRad();
            }

            // Ensure our heading remains sane (between -pi and +pi)
            currentPose.heading = atan2(sin(currentPose.heading), cos(currentPose.heading));
            
            distTotalPrev = distTotal;
            distLeftPrev  = distLeft;
            distRightPrev = distRight;
        }
        
        // What's the error between our required heading and our heading?
        double headingErrorRaw = referencePose.heading - currentPose.heading;
        headingError = atan2(sin(headingErrorRaw), cos(headingErrorRaw));

#ifdef __DEBUG__
        if (USE_GYRO_FOR_HEADING)
        {
            snprintf_P(report, sizeof(report), PSTR("head curr[%s] gyro[%s rad] ref[%s] err[%s] (raw %s)"), ftoa(floatBuf1, currentPose.heading), ftoa(floatBuf2, gyro->getAngleRad()), ftoa(floatBuf3, referencePose.heading), ftoa(floatBuf4, headingError), ftoa(floatBuf5, headingErrorRaw));          
        }
        else
        {
            snprintf_P(report, sizeof(report), PSTR("head curr[%s] ref[%s] err[%s] (raw %s)"), ftoa(floatBuf1, currentPose.heading), ftoa(floatBuf2, referencePose.heading), ftoa(floatBuf3, headingError), ftoa(floatBuf4, headingErrorRaw));
        }
        Serial.println(report);
#endif

        if (PIVOT_TURN && ((headingError > ((2*M_PI) / PIVOT_TURN_THRESHOLD)) || (headingError < -((2*M_PI) / PIVOT_TURN_THRESHOLD))))
        {
            rotate(headingError, cmdCtx.pivotTurnSpeed, cmdCtx.enableRanging);

            // If using gyroscope, assume we got to whatever heading it is currently measuring, rather than what we asked for
            if ( ! USE_GYRO_FOR_HEADING)
            {
                // @todo We SHOULD set the current heading based on the arc length ACTUALLY travelled rather than that requested (because we can overshoot)
                currentPose.heading = referencePose.heading;
            }
            
            headingError = 0.0;   // @todo: if using the gyro we should update this for the current error NOW that we've spun via the gyro (and its newly reported heading)

            // Reset counters as if we'd never done this since last checking them
            motors->resetEncoders();

            // @todo Should we really throw away PID error from the current waypoint here?
            headingErrorPrev = 0.0;
            headingErrorIntegral = 0.0;
        }

        if (iteration % 8 == 0)   // don't poll for distance more often than every 400ms (loop has delay ~50ms)
        {
            double distanceToObstacle;
            bool recordedSnapshot = poseSnapshotter->recordSnapshot(currentPose.heading, currentPose.x, currentPose.y, cmdCtx.enableRanging, &distanceToObstacle);

            if (recordedSnapshot && cmdCtx.enableRanging)
            {
                if (distanceToObstacle < CLOSE_PROXIMITY_THRESHOLD)
                {
                    sound->pip();
                    
                    consecutiveCloseProximityReadings++;

                    if (consecutiveCloseProximityReadings >= MAX_CONSECUTIVE_CLOSE_PROXIMITY_READINGS)
                    {
                        motors->setSpeeds(0, 0);
#ifdef __DEBUG__
                        Serial.println("   !!! ABORT: OBSTACLE DETECTED");      
#endif
                        sound->abortedWaypointDueToObstacle();
                        abortedDueToObstacle = true;               
                        break;
                    }
                }
                else
                {
                    consecutiveCloseProximityReadings = 0;
                }
            }
        }

        // What is the magnitude of the vector between us and our target?
        targetVectorMagnitude = sqrt(sq(referencePose.x - currentPose.x) + sq(referencePose.y - currentPose.y));
        if (iteration == 0)
        {
            targetVectorMagnitudeInitial = targetVectorMagnitude;        
        }

#ifdef __DEBUG__
        snprintf_P(report, sizeof(report), PSTR("   now (%s,%s) head [%s] err[%s] distTgt[%s]"), ftoa(floatBuf1, currentPose.x), ftoa(floatBuf2, currentPose.y), ftoa(floatBuf3, currentPose.heading), ftoa(floatBuf4, headingError), ftoa(floatBuf5, targetVectorMagnitude));    
        Serial.println(report);      
#endif

        // Give up out of out-of-control situations
        if (iteration != 0)
        {
            if (targetVectorMagnitude >= targetVectorMagnitudeLast)
            {
                bool abortWaypoint = false;
                
                if (consecutiveIncreasingDistances == 0)
                {
                    targetVectorMagnitudeAtStartOfDrift = targetVectorMagnitude;
                }

                consecutiveIncreasingDistances++;

                if (cmdCtx.abortAfterDistanceToWaypointIncreases && consecutiveIncreasingDistances >= DISTANCE_INCREASE_ITERATION_THRESHOLD)
                {
                    abortWaypoint = true;
                }
                else
                {
                    double distanceCoveredSinceStartOfDrift = targetVectorMagnitude - targetVectorMagnitudeAtStartOfDrift;
                    if (distanceCoveredSinceStartOfDrift >= (DISTANCE_INCREASE_DRIFT_FROM_INITIAL_VECTOR_THRESHOLD * targetVectorMagnitudeInitial))
                    {
                        abortWaypoint = true;
                    }
                }

                if (abortWaypoint)
                {
                    motors->setSpeeds(0, 0);
#ifdef __DEBUG__
                    Serial.println("   !!! ABORT: DISTANCE TO TARGET INCREASED");      
#endif
                    sound->abortedWaypointDueToDistanceIncrease();
                    break;                  
                }
            }
            else
            {
                consecutiveIncreasingDistances = 0;
            }
        }
        
        double forwardVelocity = cmdCtx.maxVelocity;

        if (targetVectorMagnitude < WAYPOINT_PROXIMITY_APPROACHING || approachingTarget)
        {
            approachingTarget = true;

#ifdef __DEBUG__
            Serial.println("   >>> slowing...");      
#endif

            forwardVelocity = VELOCITY_ON_APPROACH;
        }

        if (targetVectorMagnitude <= WAYPOINT_PROXIMITY_REACHED)
        {
#ifdef __DEBUG__          
            Serial.println("   >>> WAYPOINT REACHED <<<");      
#endif            
            break;
        }

        targetVectorMagnitudeLast = targetVectorMagnitude;

        // Maintain the PID variables
        double headingErrorDerivative  = (headingError - headingErrorPrev) / dt;
        headingErrorIntegral          += (headingError * dt);
        headingErrorPrev               = headingError;

        double pidP = pidProportional * headingError;
        double pidI = pidIntegral * headingErrorIntegral;
        double pidD = PID_DERIVATIVE * headingErrorDerivative;

        // PID, this gives us the control signal, u, this is our required angular velocity to achieve our desired heading
        double u = pidP + pidI + pidD;

        if (cmdCtx.capAngularVelocity)
        {
            if (u > ANGULAR_VELOCITY_SIGNAL_LIMIT)
            {
                u = ANGULAR_VELOCITY_SIGNAL_LIMIT;

#ifdef __DEBUG__                
                Serial.println("   !!! Capped angular velocity signal");      
#endif                
            }
            else if (u < (-1.0 * ANGULAR_VELOCITY_SIGNAL_LIMIT))
            {
                u = -1.0 * ANGULAR_VELOCITY_SIGNAL_LIMIT;

#ifdef __DEBUG__                
                Serial.println("   !!! Capped angular velocity signal");                      
#endif                
            }
        }

        // Angular velocity gives us new wheel velocities. We assume a constant forward velocity for simplicity.
        velocityRight = ((2.0 * forwardVelocity) + (u * WHEELBASE)) / (2.0 * WHEELRADIUS);    // cm/s
        velocityLeft  = ((2.0 * forwardVelocity) - (u * WHEELBASE)) / (2.0 * WHEELRADIUS);    // cm/s

        int leftSpeed  = motors->convertVelocityToMotorSpeed(velocityLeft);
        int rightSpeed = motors->convertVelocityToMotorSpeed(velocityRight); 

        if (approachingTarget)
        {
           // Avoid low speed oscillations that can occur and stall at low speed
           if (leftSpeed == lastRightSpeed && rightSpeed == lastLeftSpeed)
           {
              motors->setSpeeds(0, 0);
#ifdef __DEBUG__
              Serial.println("   !!! Stalled at low speed, aborting!"); 
#endif
              sound->abortedWaypointDueToStall();
              break;                                
           }
        }

        lastLeftSpeed = leftSpeed;
        lastRightSpeed = rightSpeed;

        leftSpeed = motors->getCalibratedLeftMotorSpeed(leftSpeed);

#ifdef __DEBUG__
//      snprintf_P(report, sizeof(report), PSTR("   P[%s] I[%s] D[%s] u[%s] -> required velocities L%s (%4d) R%s (%4d)"), ftoa(floatBuf1, pidP), ftoa(floatBuf2, pidI), ftoa(floatBuf3, pidD), ftoa(floatBuf4, u), ftoa(floatBuf5, velocityLeft), leftSpeed, ftoa(floatBuf6, velocityRight), rightSpeed);    
        snprintf_P(report, sizeof(report), PSTR("   u[%s] required velocities L%s (%4d) R%s (%4d)"), ftoa(floatBuf1, u), ftoa(floatBuf2, velocityLeft), leftSpeed, ftoa(floatBuf3, velocityRight), rightSpeed);    
        Serial.println(report);      
#endif

        motors->setSpeeds(leftSpeed, rightSpeed);

        iteration++;

        lastMillis = millis();
        delay(pidLoopIntervalMs);

        // Time passes, wheels respond to new control signal and begin moving at new velocities
    }

    motors->setSpeeds(0, 0);
    delay(POST_WAYPOINT_SLEEP_MS);

    return abortedDueToObstacle;
}


void WaypointNavigator::executeRotation(BotCmdCtx cmdCtx)
{
    ledYellow(1);

    gyro->resetLastUpdateTimeToNow();
    poseSnapshotter->resetSnapshotter();

    rotate(cmdCtx.rotationRadians, cmdCtx.pivotTurnSpeed, cmdCtx.enableRanging);
      
    poseSnapshotter->reportPoseSnapshots(currentPose, cmdCtx.enableRanging, false /* no obstacle */, false /* rotation never be an obstacle avoidance waypoint */); 

    ledYellow(0);
}


void WaypointNavigator::rotate(double rad, uint8_t pivotTurnSpeed, bool enableRanging)
{
    if (USE_GYRO_FOR_PIVOT_TURN)
    {
        gyro->correctHeadingWithPivotTurn(currentPose, rad, pivotTurnSpeed, enableRanging);
    }
    else
    {
        motors->correctHeadingWithPivotTurn(rad, pivotTurnSpeed);              
    }  

    currentPose.heading += rad;
    currentPose.heading = atan2(sin(currentPose.heading), cos(currentPose.heading));
}


/**
 * Determine heading required to get from a given global co-ordinate (facing in a given heading) to 
 * another global go-ordinate.
 * 
 * The current position and heading is always translated to (0,0) 0 radians (ie. looking along the x-axis from (0,0)) and
 * this function will return a value between 0..6.28 where radians grows counter-clockwise:
 * 
 * (10,0)     0 rad
 * (10,10)    0.78 rad (pi/4)
 * (0,10)     1.57 rad (pi/2)     
 * (-10,10)   2.36 rad (3pi/4) 
 * (-10,0)    3.14 rad (pi)
 * (-10,-10)  3.92 rad (5pi/4)
 * (0,-10)    4.71 rad (3pi/2)
 * (10,-10)   5.49 rad (7pi/4)
 * 
 * @param double toX              destination global x co-ord
 * @param double toY              destination global y co-ord
 * @param double fromX            current global x co-ord
 * @param double fromY            current global y co-ord
 * @param double currentHeading   current heading
 */
double WaypointNavigator::getHeading(double toX, double toY, double fromX, double fromY, double currentHeading) 
{
    // Translate current pose to (0,0, 0 rad)
    double rad = currentHeading;
    double x = toX - fromX;
    double y = toY - fromY;

#ifdef __DEBUG__          
//  snprintf_P(report, sizeof(report), PSTR("to global   (%s, %s) from (%s, %s head [%s])"), ftoa(floatBuf1, toX), ftoa(floatBuf2, toY), ftoa(floatBuf3, fromX), ftoa(floatBuf4, fromY), ftoa(floatBuf5, currentHeading));    
//  Serial.println(report);
//  snprintf_P(report, sizeof(report), PSTR("to relative (%s, %s)"), ftoa(floatBuf1, x), ftoa(floatBuf2, y));    
//  Serial.println(report);
#endif

    if (x <= 0.01 && x >= -0.01)            // x == 0
    {
        if (y < 0.0)
        {
            rad = (double)(3.0*M_PI)/2.0;
        }
        else if (y > 0.0)
        {
            rad = M_PI / 2.0;
        }
#ifdef __DEBUG__                    
        else
        {
            snprintf_P(report, sizeof(report), PSTR("-> keeping current heading"));    
            Serial.println(report);      
            
        }
#endif        
    }
    else
    {
        rad = atan(y / x);

        if (x > 0)
        {
            rad += (2.0*M_PI);

            if (rad > (2.0*M_PI))
            {
                rad -= (2.0*M_PI); 
            }
        }
        else if (x < 0)
        {
            rad += M_PI;          
        }
    }

    /**
     * BUG: I think we're missing a condition here where y = 0. If y = 0 and x > 0 then we correctly maintain the current pose, but if x < 0 then we need to do a 180.
     */

    return rad;
}

