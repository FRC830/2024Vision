# imports
from networktables import NetworkTables
from dt_apriltags import Detector
import cv2
import numpy as np
import math
import time
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink, CvSource, VideoMode

newFT=0
prevFT=0

rTc = [[0.831, 0, 0.556, 9.5], [0, 1, 0, -0.5], [-0.556, 0, 0.831, 8.5], [0, 0, 0, 1]]

params = {678.154, 678.17, 318.135, 228.374}
dim = (640, 480)
videoSource = 0
numOfATags = 0
NetworkTables.initialize(server='roborio-830-frc.local')
table = NetworkTables.getTable("SmartDashboard")
#table = tables.getSubTable('vision')
camera = CameraServer.startAutomaticCapture()
camera.setResolution(640, 480)
cvsinkVIDEO = CameraServer.getVideo()
cvSource = cvsinkVIDEO.getSource()
cvSource.setFPS(30)
emptyNArray = np.zeros((640, 480, 3))
videoOutput = CameraServer.putVideo("I HATE PYTHON", 640, 480)
binOutput = CameraServer.putVideo("Tuning", 640, 480)
detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

def resetApriltagstuff():
    for i in {3, 5, 6, 7}:
        table.putString("Apriltag ID:{a}".format(a=str(i)), "NOT DETECTED!")

def updateDashboardTag(id, poseT):
    X = pose[2] * 39.37
    Y = -pose[1] * 39.37
    Z = pose[0] * 39.37

    x = (0.831 * X) + (0.551 * Z) + (2 * 9.5)
    y = Y + (-0.5)

    table.putBoolean("Apriltag {a} Detected".format(a=id), True)
    table.putNumber("Apriltag {a} X: ".format(a=id), x)
    table.putNumber("Apriltag {a} Y: ".format(a=id), y)

def updateDashboard(tagCounts, fps):
    table.putNumber("Tags Detected: ", tagCounts)
    table.putNumber("FPS", fps)

def clearDashboard(id):
    table.putBoolean("Apriltag {a}".format(a=id), False)
    table.putNumber("Apriltag {a} X: ".format(a=id), 0.0)
    table.putNumber("Apriltag {a} Y: ".format(a=id), 0.0)


def drawBoxesAndStuff(r,processedImage):
    updateDashboardTag(r.tag_id, pose)
    (ptA, ptB, ptC, ptD) = r.corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))
    cv2.line(processedImage, ptA, ptB, (0, 255, 0), 2)
    cv2.line(processedImage, ptB, ptC, (0, 255, 0), 2)
    cv2.line(processedImage, ptC, ptD, (0, 255, 0), 2)
    cv2.line(processedImage, ptD, ptA, (0, 255, 0), 2)
    (cX, cY) = (int(r.center[0]), int(r.center[1]))
    cv2.putText(processedImage, str(r.tag_id), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_4, False)
    return processedImage

while True:
    time.sleep(.01)
    delete, image = cvsinkVIDEO.grabFrame(emptyNArray)
    if (image is None):
        continue
    image32 = np.float32(image)
    image32 = cv2.rotate(image32, cv2.ROTATE_90_COUNTERCLOCKWISE)
    gray = cv2.cvtColor(image32, cv2.COLOR_BGR2GRAY)
    gray = gray.astype(np.uint8)
    temp, bin = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    processedImage = image32
    results = detector.detect(gray, True, params, 0.1524)
    numOfATags = len(results)
    newFT = time.time()
    fps = 1 / (newFT - prevFT)
    prevFT = newFT
    if len(results) == 0:
        for i in range(1, 17):
            clearDashboard(i)
    else:
        updateDashboard(len(results), fps)
    for r in results:
        pose = r.pose_t
        drawBoxesAndStuff(r, processedImage)
        drawBoxesAndStuff(r, bin)
    binOutput.putFrame(bin)
    videoOutput.putFrame(processedImage)
