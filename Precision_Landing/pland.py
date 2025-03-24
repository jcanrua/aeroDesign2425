from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
from picamera2 import Picamera2
import numpy as np
import time

from datetime import datetime
import os
import logging

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

horizontal_res = 640
vertical_res = 480
picam2 = Picamera2()
picam2.preview_configuration.main.size=(horizontal_res,vertical_res)
picam2.preview_configuration.main.format = "RGB888" #8 bits
picam2.start()

horizontal_fov = 62.2 * (np.pi / 180 ) ##Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (np.pi / 180)    ##Pi cam V1: 41.41 V2: 48.8

##Counters and script triggers
found_count=0
notfound_count=0
first_run=0 #Used to set initial time of function to determine FPS
start_time=0
end_time=0

# Set up logging configuration
def setup_logger():
    # Get current time and format it for use in filename
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"logs/log_vuelo_{current_time}.txt"
    
    logging.basicConfig(
        filename=log_filename,
        level=logging.INFO,  # Can also use DEBUG for more detailed logs
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

# Utility function to log and print messages
def log_and_print(message, is_error=False):
    if is_error:
        print(f"ERROR: {message}")
        logging.error(message)
    else:
        print(message)
        logging.info(message)

def move_servos(pwm11, pwm12):
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        11,
        pwm11,
        0,
        0,
        0,
        0,
        0
    )

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        12,
        pwm12,
        0,
        0,
        0,
        0,
        0
    )


def send_land_message(marker_id,corners):
    x_sum = corners[marker_id][0][0][0]+ corners[marker_id][0][1][0]+ corners[marker_id][0][2][0]+ corners[marker_id][0][3][0]
    y_sum = corners[marker_id][0][0][1]+ corners[marker_id][0][1][1]+ corners[marker_id][0][2][1]+ corners[marker_id][0][3][1]
    
    x_avg = x_sum* 0.25
    y_avg = y_sum* 0.25
            
    x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
    y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)
            
    msg = vehicle.mav.landing_target_send(
        0,
        0,
        0,
        x_ang,
        y_ang,
        0,
        0,
        0
    )

    return x_avg, y_avg

def lander():
    global first_run,notfound_count,found_count,start_time
    if first_run==0:
        log_and_print("First run of lander!!")
        first_run=1
        start_time=time.time()
        
    frame = picam2.capture_array()
    frame = cv2.resize(frame,(horizontal_res,vertical_res))
    gray_img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)


    corners, ids, rejected = detector.detectMarkers(image=gray_img)
 

    altitude = vehicle.messages["AHRS2"].altitude
    print(altitude)

    try:
        if ids is not None:
            if altitude > 7:
                x_avg, y_avg = send_land_message(np.where(ids[0] == 50), corners)
            else:
                x_avg, y_avg = send_land_message(np.where(ids[0] == 70), corners)
                              
            log_and_print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
            log_and_print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
            found_count=found_count+1
        else:
            notfound_count=notfound_count+1
    except Exception as e:
        log_and_print('Target likely not found. Error: '+str(e))
        notfound_count=notfound_count+1

######################################################

#######################MAIN###########################

######################################################

setup_logger()

log_and_print("Connecting to vehicle...")

vehicle = mavutil.mavlink_connection("/dev/serial0", baud = 115200)


vehicle.wait_heartbeat()

log_and_print("Connected to vehicle")


"""
while True:
    move_servos(800, 2200) #Open
    time.sleep(4)
    move_servos(1650, 1220) #Closed
    time.sleep(4)
"""


while True:
    msg = vehicle.recv_match(type = 'HEARTBEAT', blocking = False)
    if msg:
        mode = mavutil.mode_string_v10(msg)
        if mode == "QLAND":
            break
        print(mode)


#Esperar a que el avion este en modo VTOL_LANDING

log_and_print("Starting landing...")
while True:
    lander()

log_and_print("Script finished")
