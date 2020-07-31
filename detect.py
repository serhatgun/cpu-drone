"""
@Author Serhat Gun
@File   detect.py
@Brief  Color detection script
"""

import math
import datetime
import socket
import time

import cv2, os
import numpy as np
from logg import logg
from utils import utils

class detect:

    #Constructor of the class takes 4 parameters
    #capture -> Capture frames from camera or a video (string)
    #show -> True/False to show or hide
    #record -> True/False to record or not
    def __init__(self, capture, show, record):

        #Camera parameters
        self.cam = cv2.VideoCapture(capture)
        self.capture = capture
        self.resHorizontal = 640
        self.resVertical = 480


        #Object center points in pixel
        self.objCenterX = 0
        self.objCenterY = 0


        #Video record configurations
        self.show = show
        self.fps = self.cam.get(cv2.CAP_PROP_FPS)
        self.fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
        self.record = record
        self.date = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M')
        self.out = cv2.VideoWriter("Records/" + str(self.date) + "_flight.avi" ,self.fourcc, 10, (640,480))


        #QR detection configurations & parameters
        #self.qrDecoder = cv2.QRCodeDetector()
        self.isQrDetected = False
        self.dataQR = "None"


        #Handle log file
        #self.logg = logg()
        self.utils = utils()

    #This method is intended to send pixelRelativeLocations
    #But the algorithm is turned into return method
    #data -> Data to be sent
    def sendUDP(self, data):
        try:
            while self.isSocketOpen and self.isConnectionValid:
                sck.sendto(str(data), (self.udpIP, self.udpSendPORT))
        except Exception as error:
            print("UDP Send Error: " + str(error))

    #_record method saves given frame if the ret parameter is True
    #_ret -> Parameter to check if the camera output is valid
    #_bgrFrame -> Frame to be recorded
    def _record(self, _ret, _bgrFrame):
        try:
            if _ret == True:
                self.out.write(_bgrFrame)
        except Exception as error:
	        print ("Error in main: " + str(error))

    #setBounds sets different bounding parameters for mask
    #lowerBound -> Lower bound value to be set
    #upperBound -> Upper bound value to be set
    def setBounds(self, lowerBound, upperBound):
        self.lowerBound = lowerBound
        self.upperBound = upperBound

    #readQR detects and returns the data that the QR code contains
    #frame -> Frame where QR to be detected
    def readQR(self, frame):
        data, box, rectifiedImage = self.qrDecoder.detectAndDecode(frame)
        
        if len(data) > 0:
            print("Decoded Data: {}".format(data))

            for index in range(len(box)):
                cv2.line(frame, tuple(box[index][0]), tuple(box[ (index+1) % len(box)][0]), (255,0,0), 3)

            self.isQrDetected = True
            
            return data
        else:
            #print("QR Code not detected")
            pass

    #findObject method detects color according to given HSV color bounds
    #Returns 2D centroid of the biggest contour, width and height of the object
    def findObject(self):
        try:
            #Algorithm time is important for control of the aircraft
            startTime = time.time()


            #Read frames and convert to HSV
            ret, bgrFrame = self.cam.read()
            
            #Find object
            pixelRelativeLocations = self.utils.detectColor(bgrFrame)


            #Unless the QR detected, check constantly. Otherwise, log the returned data
            #if self.isQrDetected is False:
            #    self.dataQR = self.readQR(self.bgrFrame)

            #    if self.dataQR is not None:
            #        self.logg.logFile(0.000, [0,0,0,0], self.dataQR)            



            #We do not need to show if we do not want to
            if self.show:
                cv2.imshow("frame", bgrFrame)

            #If desired, record the output
            if self.record:
                self._record(ret, bgrFrame)

            finishTime = time.time() - startTime

            #nothing detected
            if pixelRelativeLocations is None:
                #Log return value and time the process takes
                #self.logg.logFile(finishTime, [0,0,0,0])
                return 0,0,0,0

            else:
                #Log return value and time the process takes
                #self.logg.logFile(finishTime, pixelRelativeLocations)
                return pixelRelativeLocations
            
        except Exception as error:
            print("Detection Error: " + str(error))

    def main(self):
        ret = self.findObject()     
        return ret

    def __del__(self):
        self.cam.release()
        cv2.destroyAllWindows()
        self.out.release()
