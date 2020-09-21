import i2c
import udp
import globals
import enums
import math
import config

if config.supportsCamera:
    import imgrecognition

def executeSetBotOption(commandPayload):
    maxVelocity = int(commandPayload[0])
    pivotTurnSpeed = int(commandPayload[1])
    optionByte1 = int(commandPayload[2])
    optionByte2 = int(commandPayload[3])
    piOptionByte1 = int(commandPayload[4])

    packedOptions = commandPayload[5:]
    waypoints = []

    if len(packedOptions) % 2 != 0:
        return (False, 'Received invalid packed options from botLab')

    packedOptionCount = int(len(packedOptions) / 2)

    print('Received %d packed options [%s]' % (packedOptionCount, packedOptions))

    for i in range(packedOptionCount):
        packedOption = (int(packedOptions[i*2]), int(packedOptions[(i*2)+1]))
        waypoints.append(packedOption)

    msg = ''

    if optionByte1 == enums.BOT_OPTION_CALIBRATE:
        msg = ('Executing bot calibration [pivotSpeed: %d]' % (pivotTurnSpeed))
    elif optionByte1 == enums.BOT_OPTION_RESET_TO_ORIGIN:
        msg = 'Reset to origin'
        globals.currentX = 0
        globals.currentY = 0
        globals.currentHeading = 0
    elif optionByte1 == enums.BOT_OPTION_SET_PID_PARAMETERS:
        msg = 'Updated PID parameters'
    else:
        return (False, 'Unknown bot option')

    print(msg)
    udp.logToBotlab(msg, False)

    transmitSegments = i2c.buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2)
    i2c.registerTransmitSegments(transmitSegments)

    return (True, '')


def executeReportStatus(commandPayload):
    waypoints = [(0,0)]
    transmitSegments = i2c.buildTransmitSegments(waypoints, 0, 0, 0, enums.BOT_OPTION2_REPORTSTATUS)
    i2c.registerTransmitSegments(transmitSegments)

    return (True, '')


def executeCalibrateCamera(commandPayload):
    acceptableHueRange = int(commandPayload[0])
    piOptionByte1 = int(commandPayload[4])

    imgrecognition.calibrateCameraUntilObjectFound(acceptableHueRange, piOptionByte1 & enums.PI_OPTION_UPLOAD_SNAPSHOTS)

    return (True, '')


def executeTransit(commandPayload):
    maxVelocity = int(commandPayload[0])
    pivotTurnSpeed = int(commandPayload[1])
    optionByte1 = int(commandPayload[2])
    optionByte2 = int(commandPayload[3])
    piOptionByte1 = int(commandPayload[4])

    globals.followMeWaypoints = []
    globals.followMeMaxVelocity = maxVelocity
    globals.followMePivotTurnSpeed = pivotTurnSpeed

    points = commandPayload[5:]
    waypoints = []

    if len(points) % 2 != 0:
        return (False, 'Received invalid waypoint instructions from botLab')

    waypointCount = int(len(points) / 2)

    print('Received %d waypoints [%s]' % (waypointCount, points))

    for i in range(waypointCount):
        waypoint = (int(points[i*2]), int(points[(i*2)+1]))
        waypoints.append(waypoint)

    if piOptionByte1 & enums.PI_OPTION_FOLLOW_ME_MODE:
        globals.followMe = True
    else:
        globals.followMe = False

    msg = ('Transit between %d waypoints [maxVelocity: %d, pivotSpeed: %d, opt1: %2X, opt2: %2X, piOpt1: %2X]' % (waypointCount, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2, piOptionByte1))
    print(msg)
    udp.logToBotlab(msg, False)

    globals.currentCommand = enums.PI_CMD_TRANSIT_WAYPOINTS

    transmitSegments = i2c.buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2)
    i2c.registerTransmitSegments(transmitSegments)

    return (True, '')


def executeRotateAndDrive(commandPayload):
    maxVelocity = int(commandPayload[0])
    pivotTurnSpeed = int(commandPayload[1])
    optionByte1 = int(commandPayload[2])
    optionByte2 = int(commandPayload[3])
    piOptionByte1 = int(commandPayload[4])
    rotationAngle = int(commandPayload[5])
    distance = int(commandPayload[6])

    waypoints = []

    if globals.currentCommandPhase == 0:
        globals.currentCommand = enums.PI_CMD_ROTATE_AND_DRIVE
        globals.currentCommandPhase = 1
        globals.currentCommandPayload = commandPayload

        waypoint = (rotationAngle, 0)
        waypoints.append(waypoint)

        msg = ('R&D P1: Rotating [pivotSpeed: %d, opt1: %2X, opt2: %2X]' % (pivotTurnSpeed, optionByte1, optionByte2))
    elif globals.currentCommandPhase == 1:
        globals.currentCommand = 0
        globals.currentCommandPayload = 0

        if distance:
            targetX = globals.currentX + (distance * math.cos(globals.currentHeading))
            targetY = globals.currentY + (distance * math.sin(globals.currentHeading))

            waypoint = (int(targetX), int(targetY))
            waypoints.append(waypoint)
            optionByte2 = 0   # this carries the rotate command, need to switch back to waypoint transit

            msg = ('R&D P2: Driving %d cm from (%d,%d) to (%d,%d) [maxVelocity: %d]' % (distance, globals.currentX, globals.currentY, targetX, targetY, maxVelocity))
        else:
            return (True, '')   # no drive phase
    else:
        globals.currentCommand = 0
        globals.currentCommandPhase = 0
        return (False, 'Unknown phase')

    print(msg)
    udp.logToBotlab(msg, False)

    transmitSegments = i2c.buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2)
    i2c.registerTransmitSegments(transmitSegments)

    return (True, '')


def executeFindObject(commandPayload):
    maxVelocity = int(commandPayload[0])
    pivotTurnSpeed = int(commandPayload[1])
    optionByte1 = int(commandPayload[2])
    optionByte2 = int(commandPayload[3])
    piOptionByte1 = int(commandPayload[4])
    rotationAngle = int(commandPayload[5])
    rotationIterations = int(commandPayload[6])
    objectName = commandPayload[7]
    distance = int(commandPayload[8])

    if not config.supportsCamera:
        return ('False', 'Camera not supported')

    if globals.currentCommandPhase >= rotationIterations:
        globals.currentCommand = 0
        globals.currentCommandPhase = 0
        return (False, 'Object not found')

    msg = ('Looking for [%s] (attempt %d of %d)') % (objectName, globals.currentCommandPhase + 1, rotationIterations)
    print(msg)
    udp.logToBotlab(msg, False)

    foundObject = False
    (targetX, targetY) = (0, 0)
    labels = ''

    # Look for the object, there may be no need to rotate
    if objectName == 'redbox':
        (foundObject, targetX, targetY) = imgrecognition.detectRedBox(globals.currentX, globals.currentY, globals.currentHeading, piOptionByte1 & enums.PI_OPTION_UPLOAD_SNAPSHOTS)
    else:
        (foundObject, labels) = imgrecognition.detectNamedObjectInSnapshot(objectName, piOptionByte1 & enums.PI_OPTION_UPLOAD_SNAPSHOTS)

        if foundObject and distance:
            # We've been commanded to travel this far toward the object we've found
            targetX = globals.currentX + (distance * math.cos(globals.currentHeading))
            targetY = globals.currentY + (distance * math.sin(globals.currentHeading))

    if foundObject:
        globals.currentCommand = 0
        globals.currentCommandPhase = 0

        if objectName == 'redbox':
            msg = 'Found [redbox] at (%d, %d)' % (targetX, targetY)
        else:
            msg = 'Found [%s]' % objectName

        if distance:
            waypoints = []
            waypoint = (int(targetX), int(targetY))
            waypoints.append(waypoint)

            if piOptionByte1 & enums.PI_OPTION_FOLLOW_ME_MODE:
                # Send another bot there, not me
                msg = ('Found [%s], directing others to (%d,%d) [maxVelocity: %d, pivotSpeed: %d, optionByte1: %d]' % (objectName, targetX, targetY, maxVelocity, pivotTurnSpeed, optionByte1))
                udp.sendFollowMeCommand(waypoints, maxVelocity, pivotTurnSpeed, optionByte1)
            else:
                msg = ('Found [%s], driving %d cm from (%d,%d) to (%d,%d) [maxVelocity: %d]' % (objectName, distance, globals.currentX, globals.currentY, targetX, targetY, maxVelocity))
                transmitSegments = i2c.buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2)
                i2c.registerTransmitSegments(transmitSegments)

        print(msg)
        udp.logToBotlab(msg, False)

        return (True, '')
    else:
        msg = 'Saw: %s' % labels
        udp.logToBotlab(msg, False)

    globals.currentCommand = enums.PI_CMD_FIND_OBJECT
    globals.currentCommandPayload = commandPayload
    globals.currentCommandPhase += 1

    waypoints = []
    waypoint = (rotationAngle, 0)
    waypoints.append(waypoint)

    transmitSegments = i2c.buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, enums.BOT_OPTION2_ROTATE)
    i2c.registerTransmitSegments(transmitSegments)

    return (True, '')