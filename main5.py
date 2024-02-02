# imports
from networktables import NetworkTables
from dt_apriltags import Detector
import cv2
import numpy as np
import math
import time



curCorrectionFactor = {
    24 : 6.94467694627
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


detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


source = cv2.VideoCapture(videoSource)

def updateCorrectionFactors(factor):
    table.putNumberArray("CorrectonFactors", factor)


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

def updateApriltagStuffgay(id):
    table.putBoolean("Apriltag {a}".format(a=id), False)
    table.putNumber("Apriltag {a} X: ".format(a=id), 0.0)
    table.putNumber("Apriltag {a} Y: ".format(a=id), 0.0)
    table.putNumber("Apriltag {a} Z: ".format(a=id), 0.0)


def drawBoxesAndLabelStuff(r, image):
    # extract R bounding box (x, y)-coordinates for the AprilTag
    # and convert each of the (x, y)-coordinate pairs to integers
    (ptA, ptB, ptC, ptD) = r.corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))
    # draw the bounding box of the AprilTag detection
    cv2.line(image, ptA, ptB, (0, 255, 0), 2)
    cv2.line(image, ptB, ptC, (0, 255, 0), 2)
    cv2.line(image, ptC, ptD, (0, 255, 0), 2)
    cv2.line(image, ptD, ptA, (0, 255, 0), 2)
    # draw the center (x, y)-coordinates of the AprilTag
    (cX, cY) = (int(r.center[0]), int(r.center[1]))
    cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
    # draw the tag family on the image
    tagFamily = r.tag_family.decode("utf-8")
    cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    # new code
    # find distance
    # print(str(r.tag_id))
    # write a number
    cv2.putText(image, str(r.tag_id), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 255, 0), 2, cv2.LINE_4, False)


while True:
    curCorrectionFactor = getCorrectionFactors()
    time.sleep(.01)
    temp, image_old = source.read()
    if (temp == False):
        continue

    image = cv2.resize(image_old, dim, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # gray = gray.astype(np.uint8)
    # temp, binary = cv2.threshold(gray, 150, 230, cv2.THRESH_BINARY)

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

        # if u wanna draw boxes and stuff do 

        # drawBoxesAndLabelStuff(r, image)

        # find Pos
        pose = r.pose_t
        updateApriltagStuff(r.tag_id, pose)
    updateCorrectionFactors(curCorrectionFactor)
