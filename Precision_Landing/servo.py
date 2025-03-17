from pymavlink import mavutil
import time


vehicle = mavutil.mavlink_connection("/dev/serial0", baud=115200)

vehicle.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))


def open(a = 1100, b = 1000):    
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        11,  # servo instance, offset by 8 MAIN outputs
        a, # PWM pulse-width
        0,0,0,0,0     # unused parameters
        )

    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        12,  # servo instance, offset by 8 MAIN outputs
        b, # PWM pulse-width
        0,0,0,0,0     # unused parameters
        )

def close(a = 1800, b = 1800):
      
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        11,  # servo instance, offset by 8 MAIN outputs
        a, # PWM pulse-width
        0,0,0,0,0     # unused parameters
        )

    
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        12,  # servo instance, offset by 8 MAIN outputs
        b, # PWM pulse-width
        0,0,0,0,0     # unused parameters
        )

if __name__ == "__main__":
    while(True):
        open()
        time.sleep(2)
        close()
        time.sleep(2)
