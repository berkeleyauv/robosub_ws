#!/usr/bin/env python

from __future__ import division, print_function

import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist

'''
A python script to practice sending ROS messages
'''

class VelocityCommander():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, command_frequency=10.0):

        # publishing objects
        self.velocity_command_pub = rospy.Publisher("/velocity_command", Twist, queue_size=1)

        # rate of publishing
        self.command_frequency = rospy.Rate(command_frequency)

    def start_command_stream(self):
        ''' send velocity command messages
        '''
        while (not rospy.is_shutdown()):
            vel_cmd = Twist()
            t = time.time()
            a = 0.1
            '''TODO-START: FILL IN CODE HERE 
            * create a velocity command along the x-axis that oscillates with amplitude 0.1 m/s and period 2 seconds
            * All other velocity command components should be zero
            '''
            vel_cmd.linear.x = a*np.sin(np.pi*t)
            '''raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")'''
            '''TODO-END '''
            self.velocity_command_pub.publish(vel_cmd)
            self.command_frequency.sleep()

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('velocity_commander')
    vc_obj = VelocityCommander()
    print("VelocityCommander node running")

    # start the chatter
    vc_obj.start_command_stream()
    rospy.spin()
