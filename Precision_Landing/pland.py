import cv2
import cv2.aruco as aruco
from picamera2 import Picamera2
import numpy as np
import time
import dronekit
import pymavlink

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

id_to_find = 72
takeoff_height = 5

horizontal_res = 640
vertical_res = 480
picam2 = Picamera2()
picam2.preview_configuration.main.size=(horizontal_res,vertical_res)
picam2.preview_configuration.main.format = "RGB888" #8 bits
picam2.start()

horizontal_fov = 62.2 * (np.pi / 180 ) ##Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (np.pi / 180)    ##Pi cam V1: 41.41 V2: 48.8

calib_path="calibrationFiles/"
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

##Counters and script triggers
found_count=0
notfound_count=0

first_run=0 #Used to set initial time of function to determine FPS
start_time=0
end_time=0
script_mode = 1 ##1 for arm and takeoff, 2 for manual LOITER to GUIDED land 
ready_to_land=0 ##1 to trigger landing

def arm_and_takeoff(targetHeight):
    while vehicle.is_armable!=True:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")

    vehicle.mode = dronekit.VehicleMode("GUIDED")
            
    while vehicle.mode!='GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")

    
    vehicle.armed = True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    
    print("Look out! Props are spinning!!")
            
    vehicle.simple_takeoff(targetHeight) ##meters

    while True:
        print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")

def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        pymavlink.mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,
        0,)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def lander():
    global first_run,notfound_count,found_count,marker_size,start_time
    if first_run==0:
        print("First run of lander!!")
        first_run=1
        start_time=time.time()
        
    frame = picam2.capture_array()
    frame = cv2.resize(frame,(horizontal_res,vertical_res))
    #frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ids=''
    corners, ids, rejected = detector.detectMarkers(image=gray_img)
    if vehicle.mode!='LAND':
        vehicle.mode=dronekit.VehicleMode("LAND")
        while vehicle.mode!='LAND':
            print('WAITING FOR DRONE TO ENTER LAND MODE')
            time.sleep(1)
    try:
        if ids is not None and ids[0] == id_to_find:
            
            y_sum = 0
            x_sum = 0
            
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
    
            x_avg = x_sum*.25
            y_avg = y_sum*.25
            
            x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
            y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)
            
            if vehicle.mode!='LAND':
                vehicle.mode = dronekit.VehicleMode('LAND')
                while vehicle.mode!='LAND':
                    time.sleep(1)
                print("------------------------")
                print("Vehicle now in LAND mode")
                print("------------------------")
                send_land_message(x_ang,y_ang)
            else:
                send_land_message(x_ang,y_ang)
                pass
            print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
            print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
            found_count=found_count+1
            print("")
        else:
            notfound_count=notfound_count+1
    except Exception as e:
        print('Target likely not found. Error: '+str(e))
        notfound_count=notfound_count+1



vehicle = dronekit.connect("tcp:127.0.0.1:5762",wait_ready=True)

vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1 ##1 for companion computer
vehicle.parameters['PLND_EST_TYPE'] = 0 ##0 for raw sensor, 1 for kalman filter pos estimation
vehicle.parameters['LAND_SPEED'] = 20 ##Descent speed of 30cm/s

arm_and_takeoff(takeoff_height)

print("Despegue completado, esperando 5 segundos")

time.sleep(5)

while vehicle.armed==True:
    lander()