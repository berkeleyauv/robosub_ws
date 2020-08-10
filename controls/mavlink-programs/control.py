# Float division
from __future__ import division
 
# Import mavutil
from pymavlink import mavutil
 
# Create the connection
# From topside computer
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550') #
 
def update():
    # Get all messages
    msgs = []
    while True:
        msg = master.recv_match()
        if msg == None:
            print("none")
            break
        msgs.append(msg)
        print("append")
 
    # Create dict
    data = {}
    for msg in msgs:
        data[msg.get_type()] = msg.to_dict()
    # Return dict
    return data
 
while True:
    data = update()
    if 'RAW_IMU' in data:
        print('IMU [x,y,z] [mg]: ',data['RAW_IMU']['xacc'], data['RAW_IMU']['yacc'], data['RAW_IMU']['zacc'])
    if 'ATTITUDE' in data:
        print('ATTITUDE [r,p,y] [rad]: ', data['ATTITUDE']['roll'], data['ATTITUDE']['pitch'], data['ATTITUDE']['yaw'])