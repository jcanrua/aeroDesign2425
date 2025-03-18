#Inspector de mensajes Mavlink

from pymavlink import mavutil

print("Connecting")

connection = mavutil.mavlink_connection("/dev/serial0", baud = 115200)
connection.wait_heartbeat()

print("Vehicle connected")

while True:
	msg = connection.recv_match(blocking = True)
	print(msg)
