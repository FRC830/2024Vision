# imports
from networktables import NetworkTables
from dt_apriltags import Detector
import cv2
import numpy as np
import math
import time
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink, CvSource, VideoMode

curCorrectionFactor = {
0.3139 : 12
0.5860 : 24
0.8690 : 36
1.1447 : 48
1.4221 : 60
1.7142 : 72
1.9760 :
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


# camera = CameraServer.startAutomaticCapture(0)

#videoInput = CameraServer.getVideo()

camera = CameraServer.startAutomaticCapture()
camera.setResolution(640, 480)

cvsinkVIDEO = CameraServer.getVideo()

emptyNArray = np.zeros((640, 480, 3))



#server = CameraServer.addServer("Camera ServerA", 1182)
#serverb = CameraServer.addServer("Camera ServerB", 1183)

#videoOutput = CvSource("videoOutput", VideoMode.PixelFormat(4), 160, 120, 30)
#visionOutput = CvSource("special_source", VideoMode(VideoMode.PixelFormat(4), 160, 120, 30))
videoOutput = CameraServer.putVideo("I HATE PYTHON", 640, 480)
visionOutput = CameraServer.putVideo("how does someone kill python? ", 160, 120)
#server.setSource(visionOutput)
#serverb.setSource(visionOutput)


detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


#source = cv2.VideoCapture(videoSource)

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


#def drawBoxesAndLabelStuff(r, image):
    #return image

while True:

    curCorrectionFactor = getCorrectionFactors()
    time.sleep(.01)
    delete, image = cvsinkVIDEO.grabFrame(emptyNArray)

    if (image is None):
        continue

#    image = cv2.resize(image_old, dim, interpolation=cv2.INTER_AREA)
    #print(type(image))
    print(image.shape)

    image32 = np.float32(image)
    gray = cv2.cvtColor(image32, cv2.COLOR_BGR2GRAY)
    gray = gray.astype(np.uint8)

    processedImage = image32

#    temp, binary = cv2.threshold(gray, 150, 230, cv2.THRESH_BINARY)


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
#        image = drawBoxesAndLabelStuff(r, image)

    # extract R bounding box (x, y)-coordinates for the AprilTag
    # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners

        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(processedImage, ptA, ptB, (0, 255, 0), 2)
        cv2.line(processedImage, ptB, ptC, (0, 255, 0), 2)
        cv2.line(processedImage, ptC, ptD, (0, 255, 0), 2)
        cv2.line(processedImage, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        #cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
#       tagFamily = r.tag_family
 #       cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # new code
        # find distance
        # print(str(r.tag_id))
        # write a number
        cv2.putText(processedImage, str(r.tag_id), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_4, False)

        print(ptA, ptB, ptC, ptD)
        #print("\n\n\n\n\n\n\n\n" + str(type(image)))





    videoOutput.putFrame(processedImage)
    updateCorrectionFactors(curCorrectionFactor)
