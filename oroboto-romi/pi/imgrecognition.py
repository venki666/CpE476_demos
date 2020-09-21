import time
import boto3
import config
import udp
import socket
import struct
import cv2
import numpy as np
import math

from PIL import Image
from picamera import PiCamera

camera = None
rekognitionClient = None
imageServerSocket = None

lowerColourRange = None
upperColourRange = None
cameraDimX = 0


def getCamera():
    global camera

    if not camera:
        camera = PiCamera()
        camera.resolution = (1024, 768)
        camera.rotation = 180
        camera.start_preview()
        time.sleep(2)           # let AWB and auto-gain settle

    return camera


def detectRedBox(currentX, currentY, currentHeading, uploadSnapshots):
    global lowerColourRange
    global upperColourRange
    global cameraDimX

    if lowerColourRange == None or upperColourRange == None:
        msg = 'Camera not calibrated'
        print(msg)
        udp.logToBotlab(msg, False)
        return (False, 0, 0)

    camera = getCamera()

    timestamp = time.time()
    imageName = '%s/%u.jpg' % (config.imagePath, timestamp)

    camera.capture(imageName, resize=(320,240))

    bgrImg = cv2.imread(imageName)
    hsvImg = cv2.cvtColor(bgrImg, cv2.COLOR_BGR2HSV)

    (foundObject, x, y, width, height, aspectRatio) = detectTrackedObject(hsvImg, bgrImg, lowerColourRange, upperColourRange)

    if uploadSnapshots:
        labelledImageName = '%s/%u-labelled.jpg' % (config.imagePath, timestamp)
        cv2.imwrite(labelledImageName, bgrImg)
        uploadSnapshot(labelledImageName)

    (targetX, targetY) = (0, 0)

    if foundObject:
        (targetX, targetY) = calculatePositionOfTrackedObject(currentX, currentY, currentHeading, x, y, width, cameraDimX)

    return (foundObject, targetX, targetY)


def detectNamedObjectInSnapshot(objectName, uploadSnapshots):
    global rekognitionClient

    if not rekognitionClient:
        rekognitionClient = boto3.client(
            'rekognition',
            region_name=config.awsRegionName,
            aws_access_key_id=config.awsAccessKeyId,
            aws_secret_access_key=config.awsAccessKeySecret
        )

    camera = getCamera()

    timestamp = time.time()
    imageName = '%s/%u.jpg' % (config.imagePath, timestamp)
    imageNameCropped = '%s/%u-crop.jpg' % (config.imagePath, timestamp)

    camera.capture(imageName, resize=(320,240))

    image = Image.open(imageName)
    cropped = image.crop((106, 0, 213, 240))
    cropped.save(imageNameCropped)

    fd = open(imageNameCropped, 'rb')
    bytes = fd.read()
    response = rekognitionClient.detect_labels(
        Image={
            'Bytes': bytes
        },
        MaxLabels=10,
        MinConfidence=0.5
    )
    fd.close()

    if uploadSnapshots:
        uploadSnapshot(imageName)

    labels = ''

    if response['ResponseMetadata']['HTTPStatusCode'] == 200:
        for label in response['Labels']:
            labels += (' %s' % label['Name'].lower())

            if objectNameMatchesLabel(objectName, label['Name']):
                return (True, '')

    return (False, labels)


def objectNameMatchesLabel(objectName, label):
    if objectName.lower() == label.lower():
        return True

    return False


def uploadSnapshot(snapshot_file_path):
    global imageServerSocket

    if not imageServerSocket:
        connectToImageServer()

    if not imageServerSocket:
        print('Could not connect to ImageServer')
        return

    fd = open(snapshot_file_path, 'rb')
    imageBytes = fd.read()
    fd.close()

    command = bytearray(struct.pack('>BBBBI', 0x42, 0x42, 0x42, 0x42, len(imageBytes)))
    command.extend(imageBytes)

    sent = 0
    while sent < len(command):
        try:
            n = imageServerSocket.send(command[sent:])
        except socket.error:
            n = 0
            imageServerSocket.close()
            imageServerSocket = None

        if n == 0:
            print('Failed to write to ImageServer')
            return

        sent += n

    return


def connectToImageServer():
    global imageServerSocket

    if imageServerSocket:
        return

    imageServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    imageServerSocket.connect((udp.getBotLabAddr(), config.tcpBotLabImagePort))


def calibrateCameraUntilObjectFound(acceptableHueRange, uploadSnapshots):
    global lowerColourRange
    global upperColourRange
    global cameraDimX

    if acceptableHueRange < 0:
        acceptableHueRange = 40

    camera = getCamera()
    foundObject = False

    while not foundObject:
        timestamp = time.time()
        imageName = '%s/%u.jpg' % (config.imagePath, timestamp)
        camera.capture(imageName, resize=(320,240))

        bgrImg = cv2.imread(imageName)
        hsvImg = cv2.cvtColor(bgrImg, cv2.COLOR_BGR2HSV)

        (cameraDimX, cameraDimY, averageHue, lowerColourRange, upperColourRange) = calibrateCamera(hsvImg, acceptableHueRange)
        (foundObject, x, y, width, height, aspectRatio) = detectTrackedObject(hsvImg, bgrImg, lowerColourRange, upperColourRange)

        if foundObject:
            if uploadSnapshots:
                labelledImageName = '%s/%u-labelled.jpg' % (config.imagePath, timestamp)
                cv2.imwrite(labelledImageName, bgrImg)
                uploadSnapshot(labelledImageName)

            msg = 'Calibrated with object of aspect ratio %.2f and average hue %.2f (scene width: %dpx)' % (aspectRatio, averageHue, cameraDimX)
            print(msg)
            udp.logToBotlab(msg, False)
        else:
            time.sleep(1)


def calibrateCamera(hsvImg, acceptableHueRange):
    (cameraDimY, cameraDimX, channels) = hsvImg.shape

    (min, max, avg) = (0, 0, 0)
    (midX, midY) = (int(cameraDimX / 2), int(cameraDimY / 2))
    for x in range(midX - 5, midX + 5):
        for y in range(midY - 5, midY + 5):
            if hsvImg[x,y, 0] < min:
                min = hsvImg[x, y, 0]

            if hsvImg[x, y, 0] > max:
                max = hsvImg[x, y, 0]

            avg += hsvImg[x, y, 0]

    averageHue = avg / 100

    lowerColourRange = np.array([averageHue - acceptableHueRange, 50, 50])
    upperColourRange = np.array([averageHue + acceptableHueRange, 255, 255])

    print('cameraDims: (%d, %d), averageHue: %d' % (cameraDimX, cameraDimY, averageHue))

    return (cameraDimX, cameraDimY, averageHue, lowerColourRange, upperColourRange)


def detectTrackedObject(hsvInputImg, bgrInputImg, lowerColourRange, upperColourRange):
    foundObject = False
    (foundX, foundY, foundWidth, foundHeight) = (0, 0, 0, 0)
    foundAspectRatio = 0

    grayMaskImg = cv2.inRange(hsvInputImg, lowerColourRange, upperColourRange)
    #   cv2.imwrite('/tmp/mask.jpg', grayMaskImg)
    graySmoothImg = cv2.GaussianBlur(grayMaskImg, (config.gaussianKernelSize, config.gaussianKernelSize), 0)
    #   cv2.imwrite('/tmp/blurred.jpg', graySmoothImg)

    # Find extreme outer contours only, find only end points of each segment (simple mode)
    im2, contours, hierarchy = cv2.findContours(graySmoothImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(contour, 0.04 * perimeter, True)                          # approximate a polygon on top of the contour, ensure it's closed

        if len(polygon) == 4:   # we only care about squares or rectangles (those with 4 vertices)
            (x, y, width, height) = cv2.boundingRect(polygon)                                # get its position and size within the image

            aspectRatio = width / float(height)

            if width >= 10 and aspectRatio >= config.minAspectRatio and aspectRatio <= config.maxAspectRatio:
                print('Found object at (%d, %d), width: %d, aspect ratio: %.2f' % (x, y, width, aspectRatio))
                cv2.rectangle(bgrInputImg, (x, y), (x + width, y + height), (255, 0, 0), 2)      # mark it

                # This is a candidate object, but maybe not the best one
                if not foundObject or (math.fabs(1.0 - foundAspectRatio) > math.fabs(1.0 - aspectRatio)):
                    print('Aspect ratio is better than previously found (%.2f), using this object' % foundAspectRatio)
                    (foundX, foundY, foundWidth, foundHeight, foundAspectRatio) = (x, y, width, height, aspectRatio)

                foundObject = True

    return (foundObject, foundX, foundY, foundWidth, foundHeight, foundAspectRatio)


def calculateDistanceToObject(width):
    distanceFromFrontOfBot = (width / 958.657682) ** (-1.20494147341707)  # from linear regression on sample images at known distances measured from FRONT of bot
    return distanceFromFrontOfBot + config.baseRadiusCm


def calculatePositionOfTrackedObject(currentX, currentY, currentHeading, trackedObjImgX, trackedObjImgY, trackedObjWidthPx, imgWidthPx):
    r = calculateDistanceToObject(trackedObjWidthPx)

    (midpointX, midpointY) = (currentX + (r * math.cos(currentHeading)), currentY + (r * math.sin(currentHeading)))
    cmPerPx = config.trackedObjWidthCm / trackedObjWidthPx             # object has known size, therefore each px represents cmPerPx cm
    pxFromMidpoint = (imgWidthPx / 2) - (trackedObjImgX + (trackedObjWidthPx / 2))  # positive value is left of midpoint, negative is right of midpoint
    cmFromMidpoint = pxFromMidpoint * cmPerPx                   # od
    trackedObjHeadingOffset = math.atan(cmFromMidpoint / r)     # theta
    rPrime = cmFromMidpoint / math.sin(trackedObjHeadingOffset) # r'

    trackedObjX = currentX + (rPrime * math.cos(currentHeading + trackedObjHeadingOffset))
    trackedObjY = currentY + (rPrime * math.sin(currentHeading + trackedObjHeadingOffset))

    print('Detected tracked object at (%d %d) px, %dpx width [%.2f cm/px, %.2f px from midpoint) approx. %.2f cm away (r) [%.2f cm from midpoint (%d, %d)] -> (%d, %d) [offsetFromHeading: %.2f, rPrime: %.2f]' % (trackedObjImgX, trackedObjImgY, trackedObjWidthPx, cmPerPx, pxFromMidpoint, r, cmFromMidpoint, midpointX, midpointY, trackedObjX, trackedObjY, trackedObjHeadingOffset, rPrime))

    return (trackedObjX, trackedObjY)
