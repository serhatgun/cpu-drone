from drone import drone
from detect import detect
import pymavlink
from dronekit import Vehicle,connect, VehicleMode, LocationGlobalRelative
from logg import logg
import time
import argparse

"""
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connectionString = args.connect
sitl = None

if not connectionString:
    raise Exception('Connection string is not given!')
"""
connectionString = "127.0.0.1:14551"
print('Connecting to vehicle on: %s' % connectionString)

iDrone = drone(connectionString)

no_alt_change = False
land_time_start = None
takeoff_alt = 5
run_alt = 5

while True:
    try:
        while not iDrone.isSerialOpen:
            startTime = time.time()
            iDrone.bind()
            elapsedTime = time.time()-startTime

            if elapsedTime >= 20:
                raise Exception('Connection time out!')
                break

        if not iDrone.vehicle == None:
            iDrone.setGroundSpeed(3)
            iDrone.arm_and_takeoff(takeoff_alt)

            #start coordinate
            """
            iDrone.gotoRelativeGlobalLocation(39.743357, 30.471389,
                                              run_alt,
                                              3,
                                              30)
            """

            while True:
                iDrone.objCenterX, iDrone.objCenterY, width, height = iDrone.det.main()

                while (iDrone.objCenterX != 0 and iDrone.objCenterY != 0):
                    
                    iDrone.objCenterX, iDrone.objCenterY, width, height = iDrone.det.main()

                    if width != 0 and height != 0:

                        iDrone.setRatio(width, height)
                        iDrone.gotoPixelLocation()
                        time.sleep(0.5)

                    #If the position is proper (320, 240), descent
                    while (iDrone.objCenterX >= 315 and iDrone.objCenterX <= 325) and (iDrone.objCenterY >= 235 and iDrone.objCenterY <= 245):
                        iDrone.objCenterX, iDrone.objCenterY, width, height = iDrone.det.main()

                        #corresponding 40 cm/s descend speed
                        iDrone.send_ned_velocity(0, 0 ,0.4, 0)

                        #If the proportion close to the 0, this means altitude is ?60?cm
                        #switch to LAND mode
                        land_time_start = None

                        while (iDrone.vehicle.location.global_relative_frame.alt <= 1) and ((iDrone.objCenterX >= 315 and iDrone.objCenterX <= 325) and (iDrone.objCenterY >= 235 and iDrone.objCenterY <= 245)):

                            iDrone.objCenterX, iDrone.objCenterY, width, height = iDrone.det.main()

                            if land_time_start is None:
                                land_time_start = time.time()

                            temp = iDrone.vehicle.location.global_relative_frame.alt;

                            iDrone.send_ned_velocity(0, 0, 0.4, 0)
                            land_time = time.time() - land_time_start

                            if (iDrone.vehicle.location.global_relative_frame.alt - temp) < 0.08:
                                no_alt_change = True

                            #Vehicle does not descend anymore
                            if (land_time > 10) and (no_alt_change == True):
                                iDrone.vehicle.mode = VehicleMode("LAND")

                                time.sleep(10)

                                print("Box uplodaing")
                                iDrone.setServo(9, 1900)
                                iDrone.setServo(10, 1900)

                                """
                                iDrone.gotoRelativeGlobalLocation(39.743357, 30.471389,
                                                                  run_alt,
                                                                  3,
                                                                  30)
                                """

                                print("Mission Completed")
                                iDrone.__del__()
                                break
                                break
                                break
                                break

    except Exception as Error:
        print(str(Error))
