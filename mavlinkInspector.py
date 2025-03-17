#Inspector de mensajes Mavlink

from pymavlink import mavutil

connection = mavutil.mavlink_connection("udpin:127.0.0.1:14550", baud = 115200) #mavutil.mavlink_connection(device = "COM7", baud = 57600)

connection.wait_heartbeat()

while True:
	msg = connection.recv_match(blocking = True)
	print(msg)
