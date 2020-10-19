#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import ManualControl, OverrideRCIn


class SetMotors():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self):
        self.pub = rospy.Publisher("/sub/cmd_vel", Twist, queue_size=10)

    def send(self, msg):
        """
        Assuming that 
            forward is +y
            lateral right is +x
            depth up is +z
        """
        self.pub.publish(Twist(msg))

    def stop(self):
        self.send(Twist())


if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('RC_override')
    rospy.sleep(.5)

setMotor = SetMotors()