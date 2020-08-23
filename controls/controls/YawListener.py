#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

'''
A python script to practice receiving ROS messages
'''

class YawListener():
    ''' Subscribes to ROS messages
    '''
    def __init__(self):

        # publishing objects
        self.chatter_sub = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.chatter_callback)

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        self.yaw = msg.data
        #print("Current yaw:", self.yaw)

#rospy.init_node('yaw_listener')
yaw = YawListener()
print("Yaw listener node running")

if __name__ == '__main__':
    rospy.spin()