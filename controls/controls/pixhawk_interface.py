#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import OverrideRCIn

"""Sends messages to Pixhawk to move motors"""

MAX_THRUST = 1900
MIN_THRUST = 1100

class PixhawkInterface():
    ''' Generates and publishes ROS messages
    '''

    def __init__(self):
        self.sub = rospy.Subscriber("/sub/cmd_vel", Twist, self.thrust_callback)
        self.pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)


    def scale_thrust(self, num):
        offset = (MAX_THRUST + MIN_THRUST) // 2
        scale = (MAX_THRUST - MIN_THRUST) // 2
        return scale * num + offset
        

    def thrust_callback(self, vel):
        """ Send messages on topic once we receive next controls input.
        A

        Input:
            int[8]:
                Channel	Meaning
                1	    Pitch
                2       Roll
                3	    Throttle
                4	    Yaw
                5	    Forward
                6	    Lateral
                7	    Reserved
                8	    Camera Tilt
                9	    Lights 1 Level
                10	    Lights 2 Level
        """
        msg = OverrideRCIn()
        msg.channels[0] = self.scale_thrust(vel.angular.x)
        msg.channels[1] = self.scale_thrust(vel.angular.y)
        msg.channels[2] = self.scale_thrust(vel.linear.z)
        msg.channels[3] = self.scale_thrust(vel.angular.z)
        msg.channels[4] = self.scale_thrust(vel.linear.y)
        msg.channels[5] = self.scale_thrust(vel.linear.x)
        msg.channels[6] = 1500
        msg.channels[7] = 1500
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('pixhawk_interface')
    interface = PixhawkInterface()
    while not rospy.is_shutdown():
        rospy.spin()
