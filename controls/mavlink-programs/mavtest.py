import pymavlink
import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
print(vars(master))

while True:
    try:
	print(master.recv_match().to_dict())
    except:
	pass
    time.sleep(0.1)
