import math
import time
import argparse
import socket
import numpy as np
import pymavlink
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
from logg import logg
from pid import pid
from detect import detect

class drone:

    #Constructor of the drone class, takes no parameter
    def __init__(self, connectionstring):

        #Connection parameters
        self.isSocketOpen = False
        self.isConnectionValid = False #this will be decided acc. to first data package
        self.isSerialOpen = False


        #Detection parameters
        self.det = detect(0, False, True)
        self.objCenterX = None
        self.objCenterY = None
        self.focalCenterX = 320
        self.focalCenterY = 240
        self.resHorizontal = 640
        self.resVertical = 480
        self.horizontalFOV = 118.2 * math.pi/180
        self.verticalFOV = 69.5 * math.pi/180
        self.actualObjHeight = 21.6
        self.actualObjWidth = 14.6
        self.relativeObjHeight = 0
        self.relativeObjWidth= 0


        #PID parameters
        self.velUpdateRate = 0.5
        self.lastUpdate = time.time()
        self.velSpeedLast = None

        P_xy = 5.0
        I_xy = 0.1
        D_xy = 1
        max_xy = 0.5
        self.PID_xy = pid(P_xy, I_xy, D_xy, max_xy)
        self.speedX = None
        self.speedY = None


        self.udpIP = "127.0.0.1"
        self.udpListenPORT = 5001
        self.udpSendPORT = 5006
        self.sendSock = socket.socket(socket.AF_INET, # Internet
                                  socket.SOCK_DGRAM) # UDP
        self.recvSock = socket.socket(socket.AF_INET, # Internet
                                  socket.SOCK_DGRAM) # UDP

        #Vehicle connection & logging parameters
        self.connectionString = connectionstring
        self.vehicle = None
        self.logg = logg()

    #Bind method binds vehicle to the given connetion string
    def bind(self):
        try:
            #Connect to given connection with 57600 baud rate
            self.vehicle = connect(self.connectionString, baud=57600, wait_ready=True)
            print ("Connected: " + str(self.connectionString))

            self.recvSock.bind((self.udpIP, self.udpListenPORT))
            print ("Listening: " + str(self.udpListenPORT))

            self.setConnection()
            self.setHomeLocation()

            #Wait till the connection fully established
            time.sleep(5)

        except Exception as error:
            print("Bind Error: " + str(error))

    #Prints vehicle attributes
    def printAttributes(self):
        print ('GPS: ' + str(self.vehicle.gps_0) + '\n')

    #Set connection parameters
    def setConnection(self):
        self.isSerialOpen = True
        self.isSocketOpen = True

    #Set home location initially
    def setHomeLocation(self):
        self.vehicle.home_location = self.vehicle.location.global_frame

    #Set ground speed to the targetSpeed
    def setGroundSpeed(self, targetSpeed):
        self.vehicle.airspeed = targetSpeed
        print ('Ground speed set to ' + str(targetSpeed))
        print ('Current speed: ' + str(self.vehicle.airspeed))

    #arm_and_takeoff method takes targetAltitude & makes vehicle take-off to this altitude
    def arm_and_takeoff(self, targetAltitude):

        while not self.vehicle.is_armable:
            print("Waiting for vehicle to initialise")
            time.sleep(1)

        print("Arming motors")

        #Change vehicle mode to GUIDED
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)

        print("Taking off...")
        self.vehicle.simple_takeoff(targetAltitude)

        while True:
            print("Altitue: ", self.vehicle.location.global_relative_frame.alt)

            if self.vehicle.location.global_relative_frame.alt >= targetAltitude*0.9:
                print("Altitude: ", str(targetAltitude), "is reached!")
                break

            time.sleep(1)

    # send_nav_velocity - send nav_velocity command to vehicle to request it fly in specified direction
    def send_nav_velocity(self, velocity_x, velocity_y, velocity_z):
        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                                                     0,       # time_boot_ms (not used)
                                                     0, 0,    # target system, target component
                                                     mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                                                     0b0000111111000111, # type_mask (only speeds enabled)
                                                     0, 0, 0, # x, y, z positions (not used)
                                                     velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                                                     0, 0, 0, # x, y, z acceleration (not used)
                                                     0, 0)    # yaw, yaw_rate (not used)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    #Move vehicle in direction based on specified velocity vectors.
    def send_ned_velocity(self, x, y, velocity_z, duration):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                0b0000111111000111, # type_mask (only speeds enabled)
                0,0, 0, # x, y, z positions (not used)
                x, y, velocity_z, # x, y, z velocity in m/s
                0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            time.sleep(1)

    #...
    def goto_position_target_local_ned(self, north, east, down):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.

        It is important to remember that in this frame, positive altitudes are entered as negative
        "Down" values. So if down is "10", this will be 10 metres below the home altitude.

        Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
        ignored. For more information see:
        http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.
        """

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    # condition_yaw - send condition_yaw mavlink command to vehicle so it points at specified heading (in degrees)
    def condition_yaw(self, heading):
        # create the CONDITION_YAW command
        msg = self.vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
                                                     0,     # sequence
                                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
                                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW,         # command
                                                     2, # current - set to 2 to make it a guided command
                                                     0, # auto continue
                                                     heading, 0, 0, 0, 0, 0, 0) # param 1 ~ 7
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    #Land vehicle while repositioning it over the target
    #x -> Radian input in x axis
    #y -> Radian input in y axis
    def send_land_message(self, x, y):
        msg = self.vehicle.message_factory.landing_target_encode(
            0,       # time_boot_ms (not used)
            0,       # target num
            0,       # frame
            (x-self.resHorizontal/2)*self.horizontalFOV/self.resHorizontal,
            (y-self.resVertical/2)*self.verticalFOV/self.resVertical,
            0,       # altitude.  Not supported.
            0,0)     # size of target in radians

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    #setAltitude method make the vehicle descent or ascent in altitude with a
    #feedbackAltitude coming from the LIDAR sensor
    def setAltitude(self, targetAltitude, feedbackAltitude, descentRange, duration):

        #while not feedbackAltitude <= targetAltitude:
        while not self.vehicle.location.global_relative_frame.alt <= targetAltitude:
            self.send_ned_velocity(0, 0, descentRange, duration)
            self.send_ned_velocity(0,0,0,1)
            print('Descending to altitude: ' + str(feedbackAltitude) + ' (Current: ' + str(self.vehicle.location.global_relative_frame.alt))

        print('Target Altitude ' + str(targetAltitude) + ' is reached!')

    #Move vehicle to a given GPS location to get the first interact with the
    #package current on the ground
    #targetLat -> target lattitude given by main side for now
    #targetLon -> target longtitude given by main side for now
    #altitude -> cruise altitude of the aircraft
    #groundSpeed -> ground speed of the aircraft
    #sleepTime -> sleep time to wait the aircraft going to given location
    def gotoRelativeGlobalLocation(self, targetLat,
                                   targetLon, altitude,
                                   groundSpeed, sleepTime):

        print('Going to location ' +
        str(targetLat) + ', ' +
        str(targetLon) +
        ' with altitude ' +
        str(altitude) +
        ' and ground speed ' +
        str(groundSpeed))

        point = LocationGlobalRelative(targetLat, targetLon, altitude)
        self.vehicle.simple_goto(point, groundspeed = groundSpeed)
        time.sleep(sleepTime)
   
   	#Set pixel ratio
    def setRatio(self, width, height):
        self.relativeObjHeight = float(self.actualObjHeight/height)
        self.relativeObjWidth = float(self.actualObjWidth/width)

	#Return error of distance between the center of the aircraft and object
    def getError(self):

        error_y = float((320-self.objCenterX)*self.relativeObjHeight*0.01)
        error_x = float((240-self.objCenterY)*self.relativeObjWidth*0.01)

        return [error_x, error_y]

	#Set aircraft speed
    def setSpeed(self, x, y):
        self.speedX = x
        self.speedY = y

    #Move vehicle according to given pixel locations with PID control
    def gotoPixelLocation(self):

        now = time.time()

        if (now - self.lastUpdate) < 0.2:
            return

        else:

            errorX, errorY = self.getError()

            self.setSpeed(0.5, 0.5)

            #Get time since last time velocity pid controller was run
            dt = self.PID_xy.get_dt(2.0)

            xCorrection = self.PID_xy.getPID(errorX, dt)
            yCorrection = self.PID_xy.getPID(errorY, dt)

            send_x = self.speedX * xCorrection * 1.0
            send_y = self.speedX * yCorrection * 1.0

            self.send_nav_velocity(send_x, send_y, 0)
            print(str((self.objCenterX,self.objCenterY)) + " " + str(errorX) + " " + str(errorY) + " " + str(xCorrection) + " " + str(yCorrection) + " " + str(send_x) + " " + str(send_y))
            self.logg.logFile((self.objCenterX,self.objCenterY), " ", errorX, errorY, xCorrection, yCorrection, send_x, send_y)

    #Circles around the location while looking for the object until find
    def circle(self):
        x, y, h, w = self.det.main()
        print("circle")
        if x != 0 and y != 0:
            print("circle not zero")
            return True

        print("circle1")
        self.send_ned_velocity(0.5,0.5,0,2)
        time.sleep(2)
        for i in range(0,100):
            x, y, h, w = self.det.main()
            if x != 0 and y != 0:
                print("circle1 not zero")
                return True

        print("circle2")
        self.send_ned_velocity(-0.5,0.5,0,2)
        time.sleep(2)
        for i in range(0,100):
            x, y, h, w = self.det.main()
            if x != 0 and y != 0:
                print("circle2 not zero")
                return True

        print("circle3")
        self.send_ned_velocity(-0.5,-0.5,0,2)
        time.sleep(2)
        for i in range(0,100):
            x, y, h, w = self.det.main()
            if x != 0 and y != 0:
                print("circle3 not zero")
                return True

        print("circle4")
        self.send_ned_velocity(0.5,-0.5,0,2)
        time.sleep(2)
        for i in range(0,100):
            x, y, h, w = self.det.main()
            if x != 0 and y != 0:
                print("circle4 not zero")
                return True
        pass

    #Set servos for given PWM signal value to upload the package or to download
    def setServo(self, ServoNum, PWMval):
        # Set an ArduPilot servo output
        # Argument ServoNum is the ArduPilot servo number
        # Argument PWMval is the PWM value the servo will be set for

        msg = self.vehicle.message_factory.command_long_encode(
             0, 1,           # target system, target component
             mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # frame
             0,              # confirmation
             ServoNum,       # param 1: Desired servo number
             PWMval,         # param 2: Desired PWM value
             0, 0, 0, 0, 0)  # params 3-7 (not used)
        self.vehicle.send_mavlink(msg)

    #Deconstructor of the class (Although Python has its own garbage collector
    #it is gaurantee to call the deconstructor)
    def __del__(self):
        self.vehicle.mode = VehicleMode("LAND")
        print("Closing vehicle...")
        time.sleep(1)
        self.vehicle.close()
        self.sendSock.close()
        self.recvSock.close()
