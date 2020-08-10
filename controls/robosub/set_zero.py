#!/usr/bin/env python

from __future__ import division, print_function

import rospy
#import ControlMode
import setRCOutput

'''
A python script to reset motor output to zero.
'''

if __name__ == '__main__':
    #mode = SetControlMode()
    #ControlMode.sender.send('output')
    rospy.init_node('SetZero')
    setMotor = setRCOutput.SetOutput()
    rospy.sleep(.5)
    zero = [1500]*8
    setMotor.send(zero)
    print('Set zero output')
