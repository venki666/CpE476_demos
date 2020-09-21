import socket
import config
import enums
import commands

BUFSIZ = 1024

botLabAddr = 0
botLabPort = 0


def listenForBotLab(listenPort, useBotLabPort, botName, botColour, callback):
    global botLabAddr
    global botLabPort

    if botLabPort == 0:
        botLabPort = useBotLabPort

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('', listenPort))
    while 1:
        data, addr = s.recvfrom(BUFSIZ)

        if botLabAddr == 0:
            botLabAddr = addr[0]

        if len(data) == 4 and data.decode('ascii') == 'ping':
            sendPong(botLabAddr, botLabPort, 0)
            commands.executeReportStatus(None)
        else:
            callback(data)


def sendPong(botLabAddr, botLabPort, batteryMillivolts):
    pong = bytes('pong %s %s %d %s %s' % (config.name, config.colour, batteryMillivolts, (1 if config.supportsSonar else 0), (1 if config.supportsCamera else 0)), 'ascii')

    if botLabAddr == '255.255.255.255':
        sendToAddr(botLabAddr, botLabPort, pong, True)
    else:
        sendToAddr(botLabAddr, botLabPort, pong, False)


def sendFollowMeCommand(waypoints, maxVelocity, pivotTurnSpeed, optionByte1):
    waypointsMsg = ' '.join('%d %d' % tuple for tuple in waypoints)
    commandMsg = '%s %d %d %d %d 0 0 ' % (config.name, enums.PI_CMD_TRANSIT_WAYPOINTS, maxVelocity, pivotTurnSpeed, optionByte1)
    followMeCommand = commandMsg + waypointsMsg

    print('broadcasting follow me command [%s]' % followMeCommand)
    sendToAddr('255.255.255.255', config.udpLocalPort, bytes(followMeCommand, 'ascii'), True)


def logToBotlab(msg, msgIsPoseSnapshot):
    """Sends a log back to botLab.

    :param msg: The log message.
    """

    global botLabAddr
    global botLabPort

    msg = '# ' + msg if msgIsPoseSnapshot else '> ' + msg

    if botLabAddr:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(bytes(msg, 'ascii'), (botLabAddr, botLabPort))


def getBotLabAddr():
    global botLabAddr

    return botLabAddr


def sendToAddr(addr, port, msg, broadcast):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if broadcast:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.sendto(msg, (addr, port))