import pymavlink.mavutil
import dronekit
import pymavlink
import time

open_pwm11 = 1100
close_pwm11 = 1800

open_pwm12 = 2000
close_pwm12 = 1450

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

if __name__ == "__main__":
    vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)

    vehicle.parameters['PLND_ENABLED'] = 1
    vehicle.parameters['PLND_TYPE'] = 1  # 1 for companion computer
    vehicle.parameters['PLND_EST_TYPE'] = 0  # 0 for raw sensor, 1 for kalman filter pos estimation
    vehicle.parameters['LAND_SPEED'] = 20  # Descent speed of 30cm/s
    close_servos()
    time.sleep(2)
    open_servos()
