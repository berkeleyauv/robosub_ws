#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, SetMode

from robosub import MotorOutput
from robosub.controllers import *


class ControlMode:
    """Handles switching to different sub control modes."""

    def __init__(self):

        # publishing objects
        self.chatter_sub = rospy.Subscriber("/control/mode", String, self.chatter_callback)
        self.chatter_pub = rospy.Publisher("/control/mode", String, queue_size=10)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.controlMode = 'disarm'
        self.arming(False)
        self.mode(0, "MANUAL")
        self.out = None

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        if self.controlMode == msg.data:
            return
        self.controlMode = msg.data
        if self.out:
            self.out.stop()
        print("Switching to mode: {}".format(self.controlMode))
        if self.controlMode == 'power':
            self.arming(True)
            self.out = MotorOutput.setMotor
           # msg = [1500]*8
        elif self.controlMode == 'velocity':
            self.arming(True)
            self.out = VelocityController.sender
            msg = (0.0,0.0,0.0)
        # elif self.mode == 'position':
        #     self.out = PositionController.sender
        #     msg = (0.0,0.0,0.0)
        elif self.controlMode == 'heading':
            self.arming(True)
            self.out = HeadingController.sender
            msg = 0.0
        elif self.controlMode == 'vision':
            self.arming(True)
        elif self.controlMode == 'disarm':
            self.arming(False)
            return
        elif self.controlMode == 'arm':
            self.arming(True)
            return
        elif self.controlMode == 'stabilize':
            self.mode(0, "STABILIZE")
            return
        elif self.controlMode == 'depth':
            self.mode(0, "ALT_HOLD")
            return
        elif self.controlMode == 'manual':
            self.mode(0, "MANUAL")
            return
        else:
            #print("Invalid control mode:", self.controlMode)
            return
        #self.out.send(msg)  

    def send(self, msg):
        ''' Send messages on chatter topic at regular rate
            String: 
                string data
        '''
        msg = String(msg)
        self.chatter_pub.publish(msg)


mode = ControlMode()
print("ControlMode running")

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('ControlMode')
    msg = String('output')
    mode.send(msg)
    rospy.spin()
