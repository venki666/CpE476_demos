import pigpio
import struct
import math
import udp
import globals
import enums
import commands
import config

POSE_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_JOURNEY = 0x01
POSE_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_WAYPOINT = 0x02
POSE_SNAPSHOT_DETAILBYTE_ABORTED_WAYPOINT = 0x04

localPi = 0
slaveAddr = 0
eventHandler = 0

transmitSegments = []
segmentsSent = 0


def buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2):
    """Builds an array of I2C segments to transmit a list of waypoints to the bot.

    :param waypoints: An array of (x,y) tuples specifying the waypoints to visit.
    :return: An array of I2C segments to transmit.
    """
    segments = []

    if len(waypoints) > 16:
        print('Only 16 waypoints can be defined!')
        return segments

    print('Building transmit segments for %d waypoints' % len(waypoints))

    checksum = len(waypoints) + maxVelocity + pivotTurnSpeed + optionByte1 + optionByte2

    for i in range(len(waypoints)):
        waypoint = waypoints[i]
        checksum += (waypoint[0] >> 8) & (waypoint[0] & 0xFF)
        checksum += (waypoint[1] >> 8) & (waypoint[1] & 0xFF)

    checksum = checksum % 256

    currentSegment = bytearray(struct.pack('>BBBBBBBBBB', 0xA0, 0xA2, 0XA2, len(waypoints), maxVelocity, pivotTurnSpeed, optionByte1, optionByte2, checksum, 0xA1))
    segments.append(currentSegment)

    currentSegment = bytearray(struct.pack('>B', 0xA0))

    for i in range(len(waypoints)):
        waypoint = waypoints[i]
        print('Processing waypoint %d: (%d,%d)' % (i, waypoint[0], waypoint[1]))

        currentSegment.extend(struct.pack('>hh', waypoint[0], waypoint[1]))

        if len(currentSegment) == 9 or (i == len(waypoints) - 1):
#           print('Closing current segment')
            for j in range(9 - len(currentSegment)):
#               print(' + packing')
                currentSegment.extend(struct.pack('>B', 0x00))
            currentSegment.extend(struct.pack('>B', 0xA1))
            segments.append(currentSegment)
            currentSegment = bytearray(struct.pack('>B', 0xA0))

    print('Created %d transmit segments' % len(segments))
    return segments


def registerTransmitSegments(segments):
    global transmitSegments
    global segmentsSent

    transmitSegments = segments
    segmentsSent = 0


def i2cInterrupt(id, tick):
    """Handle an I2C interrupt to either send or receive data from the bot.
    """

    global localPi
    global slaveAddr
    global transmitSegments
    global segmentsSent

    s, b, d = localPi.bsc_i2c(slaveAddr)

    if b:
        if d[0] == ord('p') and len(transmitSegments) != 0 and segmentsSent < len(transmitSegments):
            """CMD: PING: Send the next waypoint I2C segment."""
            s, b, d = localPi.bsc_i2c(slaveAddr, transmitSegments[segmentsSent])
            segmentsSent += 1

        elif d[0] == ord('s') and b == 3:
            """CMD: REPORT STATUS: """
            mv = struct.unpack_from('>H', d, 1)[0]

            msg = 'Battery Level: %d mV' % mv
            print(msg)
            udp.sendPong('255.255.255.255', config.udpBotLabPort, mv)

        elif d[0] == ord('r'):
            """CMD: REPORT SNAPSHOT: Receive the next pose snapshot report."""
            if b == 14:
                x = struct.unpack_from('>h', d, 2)[0]
                y = struct.unpack_from('>h', d, 4)[0]
                headingPacked = struct.unpack_from('B', d, 6)[0]
                headingFloat = struct.unpack_from('B', d, 7)[0]
                distanceToObstacle = struct.unpack_from('>H', d, 8)[0]
                timestamp = struct.unpack_from('>H', d, 10)[0]
                detailByte = struct.unpack_from('>B', d, 12)[0]

                if headingPacked & 0x80:
                    heading = 0
                    heading -= (headingFloat / 100.0)
                    heading -= (headingPacked & 0x7F)
                else:
                    heading = (headingPacked & 0x7F) + (headingFloat / 100.0)

                # If ranging was disabled the distance to obstacle reported will always be 0
                if distanceToObstacle:
                    obstacleX = x + (distanceToObstacle * math.cos(heading))
                    obstacleY = y + (distanceToObstacle * math.sin(heading))
                else:
                    obstacleX = 0    # ensure that botlab doesn't draw any obstacles, we weren't measuring them
                    obstacleY = 0    # ensure that botlab doesn't draw any obstacles, we weren't measuring them

                print('%lu\t%d\t%d\t%f\t%d\t%d\t%d\t%u' % (timestamp, x, y, heading, distanceToObstacle, obstacleX, obstacleY, detailByte))

                msg = ('%lu\t%d\t%d\t%.2f\t%d\t%d\t%u' % (timestamp, x, y, heading, obstacleX, obstacleY, detailByte))
                udp.logToBotlab(msg, True)

                globals.currentX = x
                globals.currentY = y
                globals.currentHeading = heading

                if globals.currentCommand == enums.PI_CMD_TRANSIT_WAYPOINTS:
                    # If this is the last snapshot of a waypoint and it wasn't interrupted by an obstacle, it's a good one to follow
                    if (detailByte & POSE_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_WAYPOINT) and not (detailByte & POSE_SNAPSHOT_DETAILBYTE_ABORTED_WAYPOINT):
                        print('  above snapshot is follow me candidate')
                        waypoint = (x, y)
                        globals.followMeWaypoints.append(waypoint)

                    # If the last snapshot of the last waypoint and follow me mode is on, send all good waypoints
                    if (detailByte & POSE_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_JOURNEY) and globals.followMe:
                        # TODO: propagate optionByte1
                        udp.sendFollowMeCommand(globals.followMeWaypoints, globals.followMeMaxVelocity, globals.followMePivotTurnSpeed, 0)
                elif globals.currentCommand == enums.PI_CMD_ROTATE_AND_DRIVE:
                    if detailByte & POSE_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_WAYPOINT:
                        commands.executeRotateAndDrive(globals.currentCommandPayload)
                elif globals.currentCommand == enums.PI_CMD_FIND_OBJECT:
                    if detailByte & POSE_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_WAYPOINT:
                        commands.executeFindObject(globals.currentCommandPayload)


#           else:
#             print("read %d bytes for snapshot report, too short!" % b)
#       else:
#             print("read unknown command %x" % d[0])
#   else:
#       print("received empty i2c interrupt")


def registerI2CSlave(addr):
    global slaveAddr
    global eventHandler
    global localPi

    localPi = pigpio.pi()
    slaveAddr = addr

    if not localPi.connected:
        print('Cannot initialize pigpio!')
        return 0

    eventHandler = localPi.event_callback(pigpio.EVENT_BSC, i2cInterrupt) # register for I2C callbacks
    localPi.bsc_i2c(slaveAddr)                                            # configure BSC device as I2C slave to Arduino

    print('Registered I2C slave')

def stopI2CSlave():
    global eventHandler
    global localPi

    eventHandler.cancel()

    localPi.bsc_i2c(0)   # Disable BSC peripheral
    localPi.stop()

    print('Disabled I2C slave')