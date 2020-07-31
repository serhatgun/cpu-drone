import math
import time
import cv2
import numpy as np

class utils(object):

    def __init__(self):
        #Pixel parameters
        self.objCenterX = None
        self.objCenterY = None
        self.width = None
        self.height = None

        #Contour parameters
        self.contourSizes = []
        self.contourCentroids = []
        self.biggestContourCentroid = []
        self.minContourArea = 300
        self.biggestContourIndex = 0


        #Different HSV bounding parameters for masks
        self.lowerBoundD = np.array([148,0,235])
        self.upperBoundD = np.array([180,110,235])
        self.lowerBoundM0 = np.array([160,100,52])
        self.upperBoundM0 = np.array([180,189,255])
        self.lowerBoundM1 = np.array([142, 27, 121])
        self.upperBoundM1 = np.array([180, 110, 244])
        self.lowerBoundM2 = np.array([143, 35, 234])
        self.upperBoundM2 = np.array([165, 82, 255])
        self.lowerBoundM3 = np.array([1, 190, 200])
        self.upperBoundM3 = np.array([18, 255, 255])
        self.lowerBoundB = np.array([150,30,203])
        self.upperBoundB = np.array([180,255,255])

	#Set object centroid, width and height
    def setPixelLocations(self, biggestContourCentroid, width, height):
        self.objCenterX = biggestContourCentroid[0]
        self.objCenterY = biggestContourCentroid[1]
        self.width = width
        self.height = height

	#Return object's center, width and height
    def getPXlocs(self, contours, bgrFrame):
        self.contourSizes = []
        self.contourCentroids = []
        self.biggestContourCentroid = []
        self.biggestContourIndex = 0

        #If contour size is smaller than 300, filter them out
        #It is essential to get rid of small contours that look like our interest!
        for contour in contours:
            realArea = cv2.contourArea(contour)

            if realArea >= self.minContourArea:

                #Mark the center of the contour with RED dot
                M = cv2.moments(contour)
                cx, cy = int(M['m10']/M['m00']),int(M['m01']/M['m00'])
                cv2.circle(bgrFrame, (cx,cy), 5, (0,0,255), -1)
                self.contourSizes.append(realArea)
                self.contourCentroids.append((cx,cy))

                #Select the biggset contour 
                for i in range(1, len(self.contourSizes)):
                    if self.contourSizes[i] > self.contourSizes[self.biggestContourIndex]:
                        self.biggestContourIndex = i

                if len(self.contourSizes) > 0:
                    self.biggestContourCentroid = self.contourCentroids[self.biggestContourIndex]

                #Draw blue around the contour detected and draw yellow rectangle around it
                cv2.drawContours(bgrFrame,contour,-1,(255,0,0),3)
                x,y,width,height = cv2.boundingRect(contour)
                cv2.rectangle(bgrFrame, (x,y), (x+width,y+height),(0,255,255), 2)

                self.setPixelLocations(self.biggestContourCentroid, width, height)

                #Return pixel location
                return self.objCenterX, self.objCenterY, self.width, self.height
        
        if contours is None:
            return None

	#Create mask according to given bounds
    def createMask(self, hsvFrame):

        #Create mask using inRange function
        #Refer to https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
        #for both HSV and function info.

        #Mask for different conditions
        maskBright = cv2.inRange(hsvFrame, self.lowerBoundB, self.upperBoundB)
        maskM0 = cv2.inRange(hsvFrame, self.lowerBoundM0, self.upperBoundM0)
        maskM1 = cv2.inRange(hsvFrame, self.lowerBoundM1, self.upperBoundM1)
        maskM2 = cv2.inRange(hsvFrame, self.lowerBoundM2, self.upperBoundM2)
        maskM3 = cv2.inRange(hsvFrame, self.lowerBoundM3, self.upperBoundM3)
        maskDdark = cv2.inRange(hsvFrame,self.lowerBoundD, self.lowerBoundD)

        #Bitwise or these masks to merge them
        maskM = cv2.bitwise_or(maskM0, maskM1, maskM2, maskM3)
        maskF = cv2.bitwise_or(maskBright, maskDdark, maskM)


        #Create 5x5 and 20x20 matrices both consist of one(1)s to use in 
        #Morphological transformations
        kernelOpen = np.ones((5,5))
        kernelClose = np.ones((20,20))


        #MORPH_OPEN is a method that do erosion followed by dilation to removing noise
        maskOpen = cv2.morphologyEx(maskF,cv2.MORPH_OPEN,kernelOpen)
        
        #MORPH_CLOSE is a method that do dialation followed by erosion to
        #Get rid of small holes inside the result of the mask
        maskClose = cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
        maskFinal = maskClose

        return maskFinal

	#Detect contours in the current frame according to given mask
    def detectContours(self, mask):

        #Find contours on the maskFinal frames
        contours,hierarchy = cv2.findContours(mask.copy(),
                                                cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE)

        return contours

	#Detect object according to its color
    def detectColor(self, bgrFrame):
        
        hsvFrame = cv2.cvtColor(bgrFrame, cv2.COLOR_BGR2HSV)

        maskFinal = self.createMask(hsvFrame)
        contours = self.detectContours(maskFinal)

        #Util function to get pixel location of the given contour
        pixelRelativeLocations = self.getPXlocs(contours, bgrFrame)

        return pixelRelativeLocations
    
    pass