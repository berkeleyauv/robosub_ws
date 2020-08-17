#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from getYaw import yawl
import cv2
from datetime import datetime


class IMUListener():
    ''' Subscribes to ROS messages
    '''
    def __init__(self):

        # publishing objects
        self.chatter_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.chatter_callback)
        self.start = rospy.get_time()
        self.accelX = []
        self.accelY = []
        self.accelZ = []
        self.velX = []
        self.velY = []
        self.velZ = []
        self.first = True

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        if self.first:
            self.first = False
            
        self.accelX.append((rospy.get_time()-self.start, msg.linear_acceleration.x))
        self.accelY.append((rospy.get_time()-self.start, msg.linear_acceleration.y))
        self.accelZ.append((rospy.get_time()-self.start, msg.linear_acceleration.z))
        self.velX.append(integrate(self.accelX))
        self.velY.append(integrate(self.accelY))
        self.velZ.append(integrate(self.accelZ))
        #self.position = (integrate(self.velX), integrate(self.velY), integrate(self.velZ))
        #print("Linear vel: {}".format((self.velX[-1], self.velY[-1], self.velZ[-1])))
        # print("Position: {}".format(self.position))
        
        self.linear_acceleration = msg.linear_acceleration
        self.angular_velocity = msg.angular_velocity
        self.orientation = msg.orientation

        # print("Linear accel: {}".format(msg.linear_acceleration))
        # print("Angular vel: {}".format(msg.angular_velocity))
        # print("Orientation: {}".format(msg.orientation))


def integrate(arr):
    total = 0
    prevTime, prevVal = arr[0]
    for i in range(1, len(arr)):
        time, val = arr[i]
        total += (time-prevTime) * (prevVal + val) / 2
        prevTime = time
        prevVal = val
    return total


#rospy.init_node('imu_listener')
imu = IMUListener()
print("IMU Listener node running")

if __name__ == '__main__':
    rospy.spin()
