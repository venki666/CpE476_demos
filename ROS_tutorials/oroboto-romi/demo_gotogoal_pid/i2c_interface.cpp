#include <Romi32U4.h>
#include <Wire.h>
#include "i2c_interface.h"
#include "sound.h"
#include "defaults.h"
#include "pose.h"
#include "debug.h"

I2CInterface::I2CInterface(Sound* sound) : sound(sound)
{
    // Initialise bot command context
    cmdCtx.waypointPayloadCount = 0;

    cmdCtx.maxVelocity = MAX_VELOCITY;
    cmdCtx.pivotTurnSpeed = PIVOT_TURN_SPEED;
    cmdCtx.enableRanging = true;
    cmdCtx.abortAfterDistanceToWaypointIncreases = ABORT_WAYPOINT_AFTER_DISTANCE_INCREASES;
    cmdCtx.capAngularVelocity = CAP_ANGULAR_VELOCITY_SIGNAL;
    cmdCtx.periodicReferenceHeadingReset = PERIODIC_REFERENCE_HEADING_RESET;    

    waypointPayloadInProgress = false;
    waypointPayloadExpectedCount = 0;
}

/**
 * Poll the Raspberry Pi (acting as an I2C slave) for next command
 */
bool I2CInterface::pollForCommands()
{
   bool receivedFullCommand = false;
   
   Wire.beginTransmission(I2C_PI_ADDR);
   Wire.write(I2C_PI_CMD_GETNEXTBOTCMD);
   Wire.endTransmission();

   // The Pi's BSC FIFO is 16 bytes deep, keep the maximum transmit segment below that (10 bytes)
   Wire.requestFrom((int)I2C_PI_ADDR, (int)(I2C_WAYPOINTS_PER_SEGMENT * I2C_WAYPOINT_SIZE) + 2);  // 4 bytes per waypoint (x,y) + start and end markers

  // Wire.requestFrom((uint8_t)I2C_PI_ADDR, (uint8_t)(I2C_WAYPOINTS_PER_SEGMENT * I2C_WAYPOINT_SIZE) + 2);  // 4 bytes per waypoint (x,y) + start and end markers
   uint8_t startMarker, endMarker, i, bufferOffset = 0, maxWaypointsPerSegment = I2C_WAYPOINTS_PER_SEGMENT;
   uint8_t waypointsBuffer[I2C_WAYPOINTS_PER_SEGMENT * I2C_WAYPOINT_SIZE];

   startMarker = Wire.read();
   for (i = 0; i < (I2C_WAYPOINTS_PER_SEGMENT * I2C_WAYPOINT_SIZE); i++)
   {
      waypointsBuffer[i] = Wire.read();
   }
   endMarker = Wire.read();
   Wire.endTransmission();  

   if (startMarker == I2C_MARKER_SEGMENT_START && endMarker == I2C_MARKER_SEGMENT_END)
   {
#ifdef __DEBUG__    
      Serial.println("Received valid I2C segment");
#endif      
      sound->ok();    

      // Valid payload, either part of an existing waypoint payload, or the start of a new one
      if (waypointsBuffer[0] == I2C_MARKER_PAYLOAD_START && waypointsBuffer[1] == I2C_MARKER_PAYLOAD_START)
      {
         ledRed(1);

#ifdef __DEBUG__
         Serial.println(" - Segment starts new waypoint payload");
#endif

         if (waypointsBuffer[2] == 0 || waypointsBuffer[2] > I2C_WAYPOINTS_MAX)
         {
#ifdef __DEBUG__          
            Serial.println(" - ERROR: mismatched payload count indicator");
#endif            
            ledRed(0);
         }
         else
         {
            waypointPayloadExpectedCount = waypointsBuffer[2]; 
            
            cmdCtx.maxVelocity = waypointsBuffer[3];
            cmdCtx.pivotTurnSpeed = waypointsBuffer[4];

            optionByte1 = waypointsBuffer[5];
            optionByte2 = waypointsBuffer[6];
            checksum = waypointsBuffer[7];
            
            checksumComputed = waypointPayloadExpectedCount + cmdCtx.maxVelocity + cmdCtx.pivotTurnSpeed + optionByte1 + optionByte2;

            waypointPayloadInProgress = true;

#ifdef __DEBUG__
            snprintf_P(report, sizeof(report), PSTR(" - Expect %d waypoints (maxVelocity:%d, pivotTurnSpeed:%d, o1:%x, o2:%x, chk:%x)"), waypointPayloadExpectedCount, cmdCtx.maxVelocity, cmdCtx.pivotTurnSpeed, optionByte1, optionByte2, checksum);      
            Serial.println(report);  
#endif

            cmdCtx.waypointPayloadCount = 0;
            memset(cmdCtx.waypointPayload, 0, sizeof(cmdCtx.waypointPayload));

            return false;  // expecting another segment with the first (and next) waypoints
         }
      }
      else
      {
         if ( ! waypointPayloadInProgress)
         {
#ifdef __DEBUG__
            Serial.println(" - Segment is a payload extension but no payload build is in progress, ignoring");
#endif            
            ledRed(0);
            
            return false;
         }
#ifdef __DEBUG__          
         else
         {
            Serial.println(" - Segment is a payload extension");
         }
#endif            
      }

      for (i = 0; i < maxWaypointsPerSegment && cmdCtx.waypointPayloadCount < waypointPayloadExpectedCount; i++, cmdCtx.waypointPayloadCount++)
      {
          int16_t x = (waypointsBuffer[bufferOffset + (i * I2C_WAYPOINT_SIZE)] << 8) | waypointsBuffer[bufferOffset + (i * I2C_WAYPOINT_SIZE) + 1];
          int16_t y = (waypointsBuffer[bufferOffset + (i * I2C_WAYPOINT_SIZE) + 2] << 8) | waypointsBuffer[bufferOffset + (i * I2C_WAYPOINT_SIZE) + 3];

          cmdCtx.waypointPayload[(cmdCtx.waypointPayloadCount * sizeof(int16_t))]   = x;
          cmdCtx.waypointPayload[(cmdCtx.waypointPayloadCount * sizeof(int16_t))+1] = y;  

          checksumComputed += ((x >> 8) & (x & 0xFF));
          checksumComputed += ((y >> 8) & (y & 0xFF));
      }

      if (cmdCtx.waypointPayloadCount == waypointPayloadExpectedCount)
      {
#ifdef __DEBUG__                  
          Serial.println(" - Waypoint payload is full, ignoring further waypoint data");
#endif    
          if (checksum == checksumComputed)
          {
              cmdCtx.enableRanging = (optionByte1 & OPTION1_ENABLE_RANGING);
              cmdCtx.avoidObstacles = (optionByte1 & OPTION1_ENABLE_OBSTACLE_AVOIDANCE);
              cmdCtx.abortAfterDistanceToWaypointIncreases = (optionByte1 & OPTION1_ENABLE_ABORT_AFTER_DISTANCE_INCREASES);
              cmdCtx.capAngularVelocity = (optionByte1 & OPTION1_ENABLE_CAP_ANGULAR_VELOCITY_SIGNAL);
              cmdCtx.periodicReferenceHeadingReset = (optionByte1 & OPTION1_ENABLE_PERIODIC_REFERENCE_HEADING_RESET);
              cmdCtx.cmd = BotCmd::TransitViaWaypoints;
              
              receivedFullCommand = true; 

              // Overrides
              if (optionByte1 == OPTION1_OVERRIDE_CALIBRATE_MOTORS)
              {
                 cmdCtx.cmd = BotCmd::CalibrateMotors;
              }
              else if (optionByte1 == OPTION1_OVERRIDE_RESET_TO_ORIGIN)
              {
                 cmdCtx.cmd = BotCmd::ResetToOrigin;
              }
              else if (optionByte2 == OPTION2_OVERRIDE_ROTATE)
              {
                 cmdCtx.cmd = BotCmd::Rotate;

                 // packing format: MSB (16th bit) = sign, 7 bits of integer, 8 bits of decimal point
                 cmdCtx.rotationRadians = (uint8_t)((cmdCtx.waypointPayload[0] >> 8) & 0x7F) + ((uint8_t)(cmdCtx.waypointPayload[0] & 0xFF) / 100.0);
                 if (cmdCtx.waypointPayload[0] & 0x8000)
                 {
                    cmdCtx.rotationRadians *= -1.0;
                 }
              }
              else if (optionByte2 == OPTION2_OVERRIDE_REPORT_STATUS)
              {
                 cmdCtx.cmd = BotCmd::ReportStatus;
              }
              else if (optionByte1 == OPTION1_OVERRIDE_SET_PID_PARAMETERS)
              {
                 cmdCtx.cmd = BotCmd::SetPidParameters;
                 
                 // When setting PID parameters the waypoints stand in as command bytes
                 cmdCtx.pidLoopIntervalMs = cmdCtx.maxVelocity;           // interval up to 255ms can be set via I2C
                 cmdCtx.pidProportional = (uint8_t)(cmdCtx.waypointPayload[0] >> 8) + ((uint8_t)(cmdCtx.waypointPayload[0] & 0xFF) / 100.0);
                 cmdCtx.pidIntegral = (uint8_t)(cmdCtx.waypointPayload[1] >> 8) + ((uint8_t)(cmdCtx.waypointPayload[1] & 0xFF) / 100.0);

#ifdef __DEBUG__
                 snprintf_P(report, sizeof(report), PSTR(" - Updated PID. Interval %ums, P: %s I: %s"), cmdCtx.pidLoopIntervalMs, ftoa(floatBuf1, cmdCtx.pidProportional), ftoa(floatBuf2, cmdCtx.pidIntegral));      
                 Serial.println(report);
#endif 
              }
          }
          else
          {
#ifdef __DEBUG__                  
              snprintf_P(report, sizeof(report), PSTR(" - Invalid checksum [%x] expected [%x])"), checksumComputed, checksum);      
              Serial.println(report);
#endif                
          }

          ledRed(0);
      }
   }
   
   return receivedFullCommand;
}


/**
 * Report bot status (currently just battery voltage)
 */
void I2CInterface::reportStatus()
{
   uint16_t mv = readBatteryMillivolts();
   Wire.beginTransmission(I2C_PI_ADDR);
   Wire.write(I2C_PI_CMD_REPORTSTATUS);
   Wire.write(mv >> 8);
   Wire.write(mv & 0xFF);
   Wire.endTransmission();
}


/**
 * Report the pose snapshots from the last waypoint to the Raspberry Pi.
 * 
 * Snapshots are sent to the Raspberry Pi, acting as an I2C slave, encapsulated in a similar way
 * to how the Pi sends waypoint commands to the Arduino. Each snapshot buffer segment contains 
 * one snapshot (8 bytes) and the 1st and 10th byte are segment markers.
 * 
 * A snapshot consists of:
 *  x position (converted from double to signed 16-bit integer)
 *  y position (converted from double to signed 16-bit integer)
 *  heading (converted from double to 2 8-bit integers, the first considered signed and the second used for the floating point)
 *  distance to obstacle (converted from double to unsigned 16-bit integer)
 */
void I2CInterface::reportPoseSnapshots(int poseSnapshotCount, struct Pose* poseSnapshots, bool abortedDueToObstacle, bool lastWaypointOfJourney)
{
    ledGreen(1);

    unsigned char snapshotBuffer[(I2C_SNAPSHOTS_PER_SEGMENT * I2C_SNAPSHOT_SIZE) + 2];           // 1 snapshot + header / trailer markers
    size_t snapshotBufferSize = sizeof(snapshotBuffer);
    int i;
    
    for (i = 0; i < poseSnapshotCount; i++)
    {
        int bufferIndex = i % I2C_SNAPSHOTS_PER_SEGMENT;                     // allows for smaller snapshot reports in future

        uint8_t heading = ((uint8_t)(fabs(poseSnapshots[i].heading)) & 0x7F) | ((poseSnapshots[i].heading < 0) ? 0x80 : 0);      // 7-bits only, MSB is sign
        uint8_t headingFloat = (poseSnapshots[i].heading >= 0) ? ((uint8_t)((int)(poseSnapshots[i].heading * 100.0) % 100)) : ((uint8_t)((int)(poseSnapshots[i].heading * -100.0) % 100));

#ifdef __DEBUG__                  
        snprintf_P(report, sizeof(report), PSTR("   Adding snapshot [%d]: (%d,%d at %s [%u.%u]) dist %d ts: %u"), i, (int16_t)poseSnapshots[i].x, (int16_t)poseSnapshots[i].y, ftoa(floatBuf1, poseSnapshots[i].heading), heading, headingFloat, (uint16_t)poseSnapshots[i].distanceToObstacle, poseSnapshots[i].timestamp);      
        Serial.println(report);  
#endif

        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+0] = (unsigned char)((int16_t)poseSnapshots[i].x >> 8);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+1] = (unsigned char)((int16_t)poseSnapshots[i].x & 0xFF);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+2] = (unsigned char)((int16_t)poseSnapshots[i].y >> 8);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+3] = (unsigned char)((int16_t)poseSnapshots[i].y & 0xFF);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+4] = (unsigned char)heading;       // 7-bits only, MSB is sign
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+5] = (unsigned char)headingFloat;        
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+6] = (unsigned char)((int16_t)poseSnapshots[i].distanceToObstacle >> 8);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+7] = (unsigned char)((int16_t)poseSnapshots[i].distanceToObstacle & 0xFF);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+8] = (unsigned char)(poseSnapshots[i].timestamp >> 8);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+9] = (unsigned char)(poseSnapshots[i].timestamp & 0xFF);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+10] = 0;

        if (i == (poseSnapshotCount - 1))
        {
           snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+10] |= I2C_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_WAYPOINT;

           if (abortedDueToObstacle)
           {
              // If the waypoint was aborted due to finding an obstacle, report that on the last snapshot (as that represents 
              // our current position and hence its distance measurement will have the best estimate for the obstacle location)
              snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+10] |= I2C_SNAPSHOT_DETAILBYTE_ABORTED_WAYPOINT;
           }
           else if (lastWaypointOfJourney)
           {
              // This is never reported if the last waypoint can't be reached due to an obstacle
              snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+10] |= I2C_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_JOURNEY;            
           }
        }

        if ((I2C_SNAPSHOTS_PER_SEGMENT == 1) || (i % (I2C_SNAPSHOTS_PER_SEGMENT - 1) == 0))
        {
#ifdef __DEBUG__                    
            Serial.println("   Writing snapshot buffer...");
#endif

            snapshotBuffer[0] = I2C_MARKER_SEGMENT_START;
            snapshotBuffer[snapshotBufferSize-1] = I2C_MARKER_SEGMENT_END;

            Wire.beginTransmission(I2C_PI_ADDR);
            Wire.write('r');                             // report
            for (int j = 0; j < snapshotBufferSize; j++)
            {
               Wire.write(snapshotBuffer[j]);
            }
            Wire.endTransmission();

            if (I2C_SNAPSHOTS_PER_SEGMENT > 1)
            {
                memset(snapshotBuffer, 0, snapshotBufferSize);
            }

            delay(50);    // don't write too fast
        }
    }

    if (i % I2C_SNAPSHOTS_PER_SEGMENT != 0)
    {
#ifdef __DEBUG__                
        snprintf_P(report, sizeof(report), PSTR("   Writing trailing snapshot buffer with %d snapshots"), i % I2C_SNAPSHOTS_PER_SEGMENT);      
        Serial.println(report);  
#endif

        snapshotBuffer[0] = I2C_MARKER_SEGMENT_START;
        snapshotBuffer[snapshotBufferSize-1] = I2C_MARKER_SEGMENT_END;

        Wire.beginTransmission(I2C_PI_ADDR);
        Wire.write('r');                             // report
        for (int j = 0; j < snapshotBufferSize; j++)
        {
           Wire.write(snapshotBuffer[j]);
        }
        Wire.endTransmission();

        delay(50);    // don't write too fast
    }    

    ledGreen(0);
}
