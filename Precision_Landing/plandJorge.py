import pymavlink.mavutil
import cv2
import cv2.aruco as aruco
from picamera2 import Picamera2
import numpy as np
import time

import pymavlink
import logging
import os
from datetime import datetime

# Se inicializa el detector
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

id_to_find_carga = 72
id_to_find_sin_carga = 1
takeoff_height = 5
dlz_height = 0

open_pwm11 = 1100
close_pwm11 = 1800

open_pwm12 = 2000
close_pwm12 = 1450

# Configuración de la cámara
horizontal_res = 640
vertical_res = 480
picam2 = Picamera2()
picam2.preview_configuration.main.size = (horizontal_res, vertical_res)
picam2.preview_configuration.main.format = "RGB888"  # 8 bits
picam2.start()

# Campo de visión de la cámara
horizontal_fov = 62.2 * (np.pi / 180)  # Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (np.pi / 180)  # Pi cam V1: 41.41 V2: 48.8

# Calibración de la cámara
calib_path = "calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

# Counters and script triggers
script_mode = 1  # 1 for arm and takeoff, 2 for manual LOITER to GUIDED land
ready_to_land = 0  # 1 to trigger landing

# Set up logging configuration
def setup_logger():
    # Get current time and format it for use in filename
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"log_vuelo_{current_time}.txt"
    
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

def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        log_and_print("Waiting for vehicle to become armable.")
        time.sleep(1)
    log_and_print("Vehicle is now armable")

    vehicle.mode = dronekit.VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':
        log_and_print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    log_and_print("Vehicle now in GUIDED MODE. Have fun!!")

    vehicle.armed = True

    log_and_print("Look out! Props are spinning!!")

    vehicle.simple_takeoff(targetHeight)  # meters

    while True:
        log_and_print(f"Current Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= .95 * targetHeight:
            break
        time.sleep(1)
    log_and_print("Target altitude reached!!")

def send_land_message(x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        pymavlink.mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        x,
        y,
        0,
        0,
        0,)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def open_servos():
    msg1 = vehicle.message_factory.command_long_encode(
        0,
        0,
        pymavlink.mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        11,
        open_pwm11
    )
    vehicle.send_mavlink(msg1)

    msg2 = vehicle.message_factory.command_long_encode(
        0,
        0,
        pymavlink.mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        12,
        open_pwm12
    )
    vehicle.send_mavlink(msg2)

def close_servos():
    msg1 = vehicle.message_factory.command_long_encode(
        0,
        0,
        pymavlink.mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        11,
        close_pwm11
    )
    vehicle.send_mavlink(msg1)

    msg2 = vehicle.message_factory.command_long_encode(
        0,
        0,
        pymavlink.mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        12,
        close_pwm12
    )
    vehicle.send_mavlink(msg2)

def aruco_position(pos_id, corners):
    x_sum = corners[pos_id][0][0][0] + corners[pos_id][0][1][0] + corners[pos_id][0][2][0] + corners[pos_id][0][3][0]
    y_sum = corners[pos_id][0][0][1] + corners[pos_id][0][1][1] + corners[pos_id][0][2][1] + corners[pos_id][0][3][1]

    x_avg = x_sum * .25
    y_avg = y_sum * .25

    x_ang = (x_avg - horizontal_res * .5) * (horizontal_fov / horizontal_res)
    y_ang = (y_avg - vertical_res * .5) * (vertical_fov / vertical_res)

    if vehicle.mode != 'LAND':
        vehicle.mode = dronekit.VehicleMode('LAND')
        while vehicle.mode != 'LAND':
            time.sleep(1)
        log_and_print("Vehicle now in LAND mode")


    send_land_message(x_ang, y_ang)
    log_and_print(f"X CENTER PIXEL: {x_avg} Y CENTER PIXEL: {y_avg}")




def lander(carga):
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (horizontal_res, vertical_res))
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ids = ''
    corners, ids, rejected = detector.detectMarkers(image=gray_img)
    if ids is not None:
        if vehicle.mode != 'LAND':
            vehicle.mode = dronekit.VehicleMode("LAND")
            while vehicle.mode != 'LAND':
                log_and_print('WAITING FOR DRONE TO ENTER LAND MODE')
                time.sleep(1)
        try:
            if carga:
                for pos_id in range(len(ids)):
                    if ids[pos_id] is id_to_find_carga:
                        log_and_print(f"Ha encontrado el id {pos_id}")
                        aruco_position(pos_id, corners)
            else:
                for pos_id in range(len(ids)):
                    if ids[pos_id] is id_to_find_sin_carga:
                        log_and_print(f"Ha encontrado el id {pos_id}")
                        aruco_position(pos_id, corners)
        except Exception as e:
            log_and_print(f'Target likely not found. Error: {str(e)}', is_error=True)

if __name__ == "__main__":
    vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)

    vehicle.parameters['PLND_ENABLED'] = 1
    vehicle.parameters['PLND_TYPE'] = 1  # 1 for companion computer
    vehicle.parameters['PLND_EST_TYPE'] = 0  # 0 for raw sensor, 1 for kalman filter pos estimation
    vehicle.parameters['LAND_SPEED'] = 20  # Descent speed of 30cm/s

    carga = True
    while vehicle.location.global_relative_frame.alt is not dlz_height:
        lander(carga)
        log_and_print("ID no encontrado")
    log_and_print("Aterrizado")
    open_servos(vehicle)
    carga = False
    arm_and_takeoff(5)
    while vehicle.location.global_relative_frame.alt is not dlz_height:
        lander(carga)
        log_and_print("ID no encontrado")
    close_servos(vehicle)
    carga = True
    # arm_and_takeoff() sube hasta arriba del todo
    # cambia a horizontal y se va a base

# Call setup_logger once to initialize the logger
setup_logger()
