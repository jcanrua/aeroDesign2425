import pymavlink.mavutil
import cv2
import cv2.aruco as aruco
from picamera2 import Picamera2
import numpy as np
import time

import logging
import os
from datetime import datetime

# Se inicializa el detector
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)


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

def send_land_message(x, y):
    msg = vehicle.mav.landing_target_send(
        0,
        0,
        0,
        x,
        y,
        0,
        0,
        0,)

    print("mensaje enviado")

def aruco_position(pos_id, corners):
    x_sum = corners[pos_id][0][0][0] + corners[pos_id][0][1][0] + corners[pos_id][0][2][0] + corners[pos_id][0][3][0]
    y_sum = corners[pos_id][0][0][1] + corners[pos_id][0][1][1] + corners[pos_id][0][2][1] + corners[pos_id][0][3][1]

    x_avg = x_sum * .25
    y_avg = y_sum * .25

    x_ang = (x_avg - horizontal_res * .5) * (horizontal_fov / horizontal_res)
    y_ang = (y_avg - vertical_res * .5) * (vertical_fov / vertical_res)

    send_land_message(x_ang, y_ang)


def lander():
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (horizontal_res, vertical_res))
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ids = ''
    corners, ids, rejected = detector.detectMarkers(image=gray_img)
    if ids is not None:
        print(ids)
        aruco_position(0, corners)
        cv2.imshow("frame", frame)

if __name__ == "__main__":
    vehicle = pymavlink.mavutil.mavlink_connection("/dev/serial0", baud = 115200)

    print("Conectado")

    while True:
        lander()
