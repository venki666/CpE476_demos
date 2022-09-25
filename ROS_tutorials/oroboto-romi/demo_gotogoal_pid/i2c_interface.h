#ifndef __I2C_INTERFACE_H__
#define __I2C_INTERFACE_H___H__

/**
 * Commands that can be sent from the Raspberry Pi 
 */
#define I2C_PI_ADDR 0b10011                     // 0x0A
#define I2C_PI_CMD_GETNEXTBOTCMD 'p'            // ping!
#define I2C_PI_CMD_REPORTSNAPSHOTS 'r'       
#define I2C_PI_CMD_REPORTSTATUS 's'

#define I2C_MARKER_SEGMENT_START 0xA0
#define I2C_MARKER_SEGMENT_END 0xA1
#define I2C_MARKER_PAYLOAD_START 0xA2
#define I2C_MARKER_PAYLOAD_END 0xA3

// Valid bitmasks for first option byte
#define OPTION1_ENABLE_RANGING 0x01
#define OPTION1_ENABLE_ABORT_AFTER_DISTANCE_INCREASES 0x02
#define OPTION1_ENABLE_CAP_ANGULAR_VELOCITY_SIGNAL 0x04
#define OPTION1_ENABLE_PERIODIC_REFERENCE_HEADING_RESET 0x08
#define OPTION1_ENABLE_OBSTACLE_AVOIDANCE 0x10
#define OPTION1_OVERRIDE_CALIBRATE_MOTORS 0xB0
#define OPTION1_OVERRIDE_RESET_TO_ORIGIN  0xB1
#define OPTION1_OVERRIDE_SET_PID_PARAMETERS 0xB2

#define OPTION2_OVERRIDE_ROTATE 0xB3
#define OPTION2_OVERRIDE_REPORT_STATUS 0xB4

#define I2C_WAYPOINTS_MAX 16                    // max waypoints that can be sent per I2C_PI_CMD_GETNEXTBOTCMD
#define I2C_WAYPOINT_SIZE 4                     // 2 bytes each (signed short) for x and y co-ordinates 
#define I2C_WAYPOINTS_PER_SEGMENT 2             // number of waypoints that can be sent per segment for I2C_PI_CMD_GETNEXTBOTCMD (don't exceed 16 byte max segment size)

#define I2C_SNAPSHOT_SIZE 11                      // 2 bytes (signed short) for x, y, 2 bytes for heading, 2 bytes (unsigned short) for distance, 2 bytes (unsigned short) for timestamp, 1 byte for extra detail (ie. aborted due to obstacle)
#define I2C_SNAPSHOTS_PER_SEGMENT 1               // number of snapshots that can be sent per segment for I2C_PI_CMD_REPORTSNAPSHOTS (don't exceed 16 byte max segment size)
#define I2C_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_JOURNEY 0x01      // reported on the last snapshot of the last waypoint as long as that waypoint was not interrupted by an obstacle: if we never reach the last waypoint this will never be sent
#define I2C_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_WAYPOINT 0x02     // reported on the last snapshot of every waypoint or obstacle avoidance maneuver
#define I2C_SNAPSHOT_DETAILBYTE_ABORTED_WAYPOINT  0x04              // reported on the last snapshot if the current waypoint or obstacle avoidance maneuver itself was aborted due to an obstacle

struct Pose;
class  Sound;

/**
 * Commands that can be received from the Raspberry Pi 
 */
enum class BotCmd 
{
  TransitViaWaypoints = 0,
  Rotate,
  CalibrateMotors,
  ResetToOrigin,
  SetPidParameters,
  ReportStatus
};

class BotCmdCtx
{
  public:  
    BotCmd  cmd;
    
    uint8_t waypointPayloadCount;
    int16_t waypointPayload[I2C_WAYPOINTS_MAX * 2]; // all the waypoints received via an I2C_PI_CMD_GETNEXTBOTCMD are stored here

    uint8_t maxVelocity;
    uint8_t pivotTurnSpeed;

    bool    capAngularVelocity;
    bool    periodicReferenceHeadingReset;    
    bool    enableRanging;
    bool    avoidObstacles;
    bool    abortAfterDistanceToWaypointIncreases;  

    double  rotationRadians;

    // For BotCmd::SetPidParameters only, not command-specific
    float     pidProportional;
    float     pidIntegral;
    uint16_t  pidLoopIntervalMs;
};

class I2CInterface
{
  public:
    I2CInterface(Sound* sound);

    bool pollForCommands();
    void reportPoseSnapshots(int poseSnapshotCount, struct Pose* poseSnapshots, bool abortedDueToObstacle, bool lastWaypointOfJourney);
    void reportStatus();

    BotCmdCtx cmdCtx;

  private:
    Sound*          sound;
    bool            waypointPayloadInProgress;  
    uint8_t         waypointPayloadExpectedCount;
    uint8_t         checksum, checksumComputed;
    uint8_t         optionByte1, optionByte2;    
};

#endif // __I2C_INTERFACE_H__

