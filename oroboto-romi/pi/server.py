#!/usr/bin/env python

import config
import i2c
import udp
import globals
import enums
import commands


def botLabCallback(data):
    dataString = data.decode('ascii')
    payload = dataString.split()

    if len(payload) < 7:   # name + command + maxVelocity + pivotTurnSpeed + 2x bot option bytes + 1x Pi option byte
        print('Received invalid command payload: [%s]' % dataString)
        return

    if payload[0] == config.name:
        print('Ignoring broadcast command from myself')
        return

    command = int(payload[1])
    globals.currentCommandPhase = 0
    success = False
    errMsg  = ''

    if command == enums.PI_CMD_TRANSIT_WAYPOINTS:
        (success, errMsg) = commands.executeTransit(payload[2:])
    elif command == enums.PI_CMD_SET_BOT_OPTION:
        (success, errMsg) = commands.executeSetBotOption(payload[2:])
    elif command == enums.PI_CMD_ROTATE_AND_DRIVE:
        (success, errMsg) = commands.executeRotateAndDrive(payload[2:])
    elif command == enums.PI_CMD_FIND_OBJECT:
        (success, errMsg) = commands.executeFindObject(payload[2:])
    elif command == enums.PI_CMD_CALIBRATE_CAMERA:
        (success, errMsg) = commands.executeCalibrateCamera(payload[2:])
    elif command == enums.PI_CMD_REPORT_STATUS:
        (success, errMsg) = commands.executeReportStatus(payload[2:])
    else:
        errMsg = 'Unknown command'

    if not success:
        print('Failed to execute command [%2X]: %s (payload: %s)' % (command, errMsg, dataString))
        msg = 'Failed to execute cmd [%2X]: %s' % (command, errMsg)
        udp.logToBotlab(msg, False)

    return 0


if __name__ == '__main__':
    i2c.registerI2CSlave(config.i2cSlaveAddr)

    udp.sendPong('255.255.255.255', config.udpBotLabPort, 0)
    udp.listenForBotLab(config.udpLocalPort, config.udpBotLabPort, config.name, config.colour, botLabCallback)

    i2c.stopI2CSlave()