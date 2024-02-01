# imports
from dt_apriltags import Detector
import cv2
import numpy as np
import math
import time
from ntcore import NetworkTableInstance, CameraServer


correctionFactor = {
    24 : 6.94467694627
}

params = {678.154, 678.17, 318.135, 228.374}

dim = (640, 480)

videoSource = 0

prev_frame_time = 0

new_frame_time = 0

numOfATags = 0




ninst = NetworkTablesInstance.getDefault()
ninst.startClient(("10.8.30.41", idk))
# inst = CameraServer.getInstance()
# camera = UsbCamera()

tables = ninst.getTable("Shuffleboard")
table = tables.getSubTable('vision')

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)



source = cv2.VideoCapture(videoSource)

def resetApriltagstuff():
    for i in (3, 5, 6, 7):
        table.putString("Apriltag ID{a}".format(a=str(i)))

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
    return image


while True:
    time.sleep(.01)
    temp, image_old = source.read()
    if (temp == False):
        continue

    image = cv2.resize(image_old, dim, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # gray = gray.astype(np.uint8)
    # temp, binary = cv2.threshold(gray, 150, 230, cv2.THRESH_BINARY)

    results = detector.detect(gray, True, params, 0.01524)
    numOfATags = len(results)

    new_frame_time = time.time()
    fps = 1 / (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    
    for r in results:
        if(r.tag_id == 3 or r.tag_id == 5 or r.tag_id == 6 or r.tag_id == 7):
            table.putString("Objective AprilTag Detected?", "Detected!")
            print()
            # new_img = drawBoxesAndLabelStuff(r, image)
        else:
            table.putString("Objective AprilTag Detected?", "Not Detected!")
            resetApriltagstuff()
            continue

        # if u wanna draw boxes and stuff do 

        # drawBoxesAndLabelStuff(r, image)

        # find Pos
        pose = r.pose_t


        table.putString("Apriltag ID:{a}".format(a=str(r.tag_id)), "X: {a},Y: {b},Z: {c}".format(a=str(pose[0][3]), b=str(pose[2][3]), c=str(math.sqrt(pow(x, 2) +  pow(y, 2)))))
