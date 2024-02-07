# imports
from networktables import NetworkTables
from dt_apriltags import Detector
import cv2
import numpy as np
import math
import time
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink, CvSource, VideoMode

curCorrectionFactor = {
0.3139 : 12,
0.5860 : 24,
0.8690 : 36,
1.1447 : 48,
1.4221 : 60,
1.7142 : 72,
1.9760 : 84, 
2.2450 : 96,
2.5146 : 108,
2.8159 : 120,
3.0723 : 132,
3.3908 : 144,
3.6432 : 156,
3.9622 : 168,
4.2288 : 180,
4.5957 : 192,
4.9070 : 204,
5.1527 : 216,
}
params = {678.154, 678.17, 318.135, 228.374}
dim = (640, 480)
videoSource = 0
prev_frame_time = 0
new_frame_time = 0
numOfATags = 0
NetworkTables.initialize(server='roborio-830-frc.local')
tables = NetworkTables.getTable("SmartDashboard")
table = tables.getSubTable('vision')
camera = CameraServer.startAutomaticCapture()
camera.setResolution(640, 480)
cvsinkVIDEO = CameraServer.getVideo()
emptyNArray = np.zeros((640, 480, 3))
videoOutput = CameraServer.putVideo("I HATE PYTHON", 640, 480)
detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

def findAB(target):
    a = 0
    if target > 5.1227:
        return -1
    for i in curCorrectionFactor: 
        b = i
        if a <= target and target <= b: 
            break
        else:
            a = b
    return a, b

def correct(target): 
    a, b = findAB(target)
    return (target / (b - a)) * (1 / (curCorrectionFactor[b] - curCorrectionFactor[a]))


def getCorrectionFactors():
    correctionFactor = {
        24 : 6.94467694627
    }
    return table.getNumberArray("CorrectonFactors", correctionFactor)

def resetApriltagstuff():
    for i in {3, 5, 6, 7}:
        table.putString("Apriltag ID:{a}".format(a=str(i)), "NOT DETECTED!")

def updateApriltagStuff(id, poseT):
    x = pose[0]
    y = pose[2]
    table.putBoolean("Apriltag {a} Detected".format(a=id), True)
    table.putNumber("Apriltag {a} X: ".format(a=id), x)
    table.putNumber("Apriltag {a} Y: ".format(a=id), y)
    table.putNumber("Apriltag {a} Z: ".format(a=id), math.sqrt(pow(x, 2) +  pow(y, 2)))
    table.putNumber("ApriltagCorrected {a} X: ".format(a=id), correct(x))
    table.putNumber("ApriltagCorrected {a} Y: ".format(a=id), correct(y))
    table.putNumber("ApriltagCorrected {a} Z: ".format(a=id), math.sqrt(pow(correct(x), 2) +  pow(correct(y), 2)))

def updateApriltagStuffgay(id):
    table.putBoolean("Apriltag {a}".format(a=id), False)
    table.putNumber("Apriltag {a} X: ".format(a=id), 0.0)
    table.putNumber("Apriltag {a} Y: ".format(a=id), 0.0)
    table.putNumber("Apriltag {a} Z: ".format(a=id), 0.0)
    table.putNumber("ApriltagCorrected {a} X: ".format(a=id), 0.0)
    table.putNumber("ApriltagCorrected {a} Y: ".format(a=id), 0.0)
    table.putNumber("ApriltagCorrected {a} Z: ".format(a=id), 0.0)

while True:
    curCorrectionFactor = getCorrectionFactors()
    time.sleep(.01)
    delete, image = cvsinkVIDEO.grabFrame(emptyNArray)
    if (image is None):
        continue
    print(image.shape)
    image32 = np.float32(image)
    gray = cv2.cvtColor(image32, cv2.COLOR_BGR2GRAY)
    gray = gray.astype(np.uint8)
    processedImage = image32
    results = detector.detect(gray, True, params, 0.1524)
    numOfATags = len(results)
    new_frame_time = time.time()
    fps = 1/ (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    table.putNumber("FPS", fps)
    if len(results) == 0:
        updateApriltagStuffgay(3)
        updateApriltagStuffgay(5)
        updateApriltagStuffgay(6)
        updateApriltagStuffgay(7)
    for r in results:
        if(r.tag_id == 3 or r.tag_id == 5 or r.tag_id == 6 or r.tag_id == 7):
            table.putString("Objective AprilTag Detected?", "Detected!")
        else:
            table.putString("Objective AprilTag Detected?", "Not Detected!")
            continue
        pose = r.pose_t
        updateApriltagStuff(r.tag_id, pose)
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
        print(ptA, ptB, ptC, ptD)
    videoOutput.putFrame(processedImage)
