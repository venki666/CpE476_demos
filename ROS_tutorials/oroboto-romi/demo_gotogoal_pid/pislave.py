#!/usr/bin/env python

import time
import pigpio
import sys
import struct

I2C_ADDR=0x13
transmitSegments = []
segmentsSent = 0

def buildTransmitSegments(waypoints):
   transmitSegments = []

   if len(waypoints) > 16:
      print("only 16 waypoints can be defined!")
      return transmitSegments

   print("processing %d waypoints" % len(waypoints))

   currentSegment = bytearray(struct.pack(">BBBBB", 0xA0, 0xA2, 0XA2, len(waypoints), len(waypoints)))

   for i in range(len(waypoints)):
      waypoint = waypoints[i]
      print("processing waypoint(%d,%d)" % (waypoint[0], waypoint[1]))

      currentSegment.extend(struct.pack(">hh", waypoint[0], waypoint[1]))

      if len(currentSegment) == 9 or (i == len(waypoints) - 1):
         print("closing current segment")
         for j in range(9 - len(currentSegment)):
            print(" + packing")
            currentSegment.extend(struct.pack(">B", 0x00))
         currentSegment.extend(struct.pack(">B", 0xA1))
         transmitSegments.append(currentSegment)
         currentSegment = bytearray(struct.pack(">B", 0xA0))

   print("created %d transmit segments" % len(transmitSegments))
  
   return transmitSegments


def i2c(id, tick):
    global pi
    global transmitSegments
    global segmentsSent

    print("I2C interrupt")

    s, b, d = pi.bsc_i2c(I2C_ADDR)
    if b:
        if d[0] == ord('p') and segmentsSent < len(transmitSegments):
            s, b, d = pi.bsc_i2c(I2C_ADDR, transmitSegments[segmentsSent])
            segmentsSent += 1

        elif d[0] == ord('r'):
           if b == 11:
              x = struct.unpack_from(">h", d, 2)[0]
              y = struct.unpack_from(">h", d, 4)[0]
              heading = struct.unpack_from("b", d, 6)[0]
              headingFloat = struct.unpack_from("B", d, 7)[0]
              distanceToObstacle = struct.unpack_from(">H", d, 8)[0]

              if heading >= 0:
                 heading += (headingFloat / 100.0)
              else:
                 heading -= (headingFloat / 100.0)

              print("x: %d\t y: %d\t head: %f\t dist: %d" % (x, y, heading, distanceToObstacle))


pi = pigpio.pi()

if not pi.connected:
    exit()

#transmitSegments = buildTransmitSegments([(100,100),(-200,500),(233,992)])
transmitSegments = buildTransmitSegments([(50,50)])

# Respond to BSC slave activity

e = pi.event_callback(pigpio.EVENT_BSC, i2c)
pi.bsc_i2c(I2C_ADDR) # Configure BSC as I2C slave
time.sleep(60)
e.cancel()
pi.bsc_i2c(0) # Disable BSC peripheral
pi.stop()

