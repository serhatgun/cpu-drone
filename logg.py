import time
import datetime

class logg:
    def __init__ (self):
        self.dataTime = ""
        self.dataPixel = ""

        self.errorX = ""
        self.errorY = ""
        self.xCorrection = ""
        self.yCorrection = ""
        self.sendX = ""
        self.sendY = ""

        self.dataQR = ""

        self.date = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        self.fileName = str(self.date)
        self.file = None

    def logFile(self, dataPixel, dataQR, errorX, errorY, xCorrection, yCorrection, sendX, sendY):
        try:
            if self.file is None:
                self.dataPixel = dataPixel
                self.dataQR = dataQR
                self.errorX = errorX
                self.errorY = errorY
                self.xCorrection = xCorrection
                self.yCorrection = yCorrection
                self.sendX = sendX
                self.sendY = sendY

                self.file = open("Logs/" + str(self.fileName) + "_flight.txt", "w+")
                self.date = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
                self.file.write(  " @PL: " + str(self.dataPixel) 
                                + " @EX: " + str(self.errorX) + " @EY: " + str(self.errorY) 
                                + " @EXC: " + str(self.xCorrection) + " @EYC: " + str(self.yCorrection) 
                                + " @XS: " + str(self.sendX) + " @YS: " + str(self.sendY) 
                                + " @QR: " + str(self.dataQR) + "\n")
                self.file.close()

            else:
                self.dataPixel = dataPixel
                self.dataQR = dataQR
                self.errorX = errorX
                self.errorY = errorY
                self.xCorrection = xCorrection
                self.yCorrection = yCorrection
                self.sendX = sendX
                self.sendY = sendY

                self.file = open("Logs/" + str(self.fileName) + "_flight.txt", "a+")
                self.date = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
                self.file.write(  " @PL: " + str(self.dataPixel) 
                                + " @EX: " + str(self.errorX) + " @EY: " + str(self.errorY) 
                                + " @EXC: " + str(self.xCorrection) + " @EYC: " + str(self.xCorrection) 
                                + " @XS: " + str(self.sendX) + " @YS: " + str(self.sendY) 
                                + " @QR: " + str(self.dataQR) + "\n")
                self.file.close()

        except Exception as error:
	           print ("Error in main: " + str(error))
