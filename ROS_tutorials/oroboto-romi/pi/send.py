import config
import udp
import sys

#    transmitSegments = buildTransmitSegments([(100,100),(-200,500),(233,992)])
#    transmitSegments = buildTransmitSegments([(50,0),(50,50),(0,50),(0,0)])
#    transmitSegments = buildTransmitSegments([(100,0),(100,100),(0,100),(0,0)])
#    transmitSegments = buildTransmitSegments([(100,0),(100,100)])
#    transmitSegments = buildTransmitSegments([(100,0),(100,-100),(0,-100),(0,0)])
#    transmitSegments = buildTransmitSegments([(100,0)])
#    transmitSegments = buildTransmitSegments([(50,0),(50,50),(0,50)])

if __name__ == '__main__':
    msg = '2 50 0 1 100 0 100 100 0 100 0 0'
    udp.sendToAddr(sys.argv[1], config.udpLocalPort, msg, 1)
