#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String, Float64
from PIDController import PID
from threading import Thread, Lock
from controls.YawListener import yaw
from controls.MotorOutput import setMotor

'''
A python script to practice receiving ROS messages
'''

class HeadingController():
    ''' Subscribes to ROS messages
    '''
    def __init__(self):
        self.chatter_sub = rospy.Subscriber("/control/heading", Float64, self.chatter_callback)
        self.pid = PID(0.2, 0.0, 0.0)
        self.yaw = lambda: yaw.yaw
        self.rate = rospy.Rate(50)
        self.thread = None

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        if self.thread:
            self.thread.end()
        self.pid.setSetpoint(msg.data)
        self.thread = self.PIDThread()
        self.thread.start(self.pid, self.yaw)
    
    class PIDThread(Thread):

        def start(self, pid, yaw):
            self.stop = False
            self.rate = rospy.Rate(50)
            self.pid = pid
            self.yaw = yaw
            Thread.start(self)
        
        def end(self):
            self.stop = True

        def run(self):
            self.pid.reset()
            start = rospy.get_time()
            while not self.stop:
                output = self.pid.pidLoop(self.yaw(), rospy.get_time()-start)
                msg = [1500]*8
                msg[3] += output
                print("Yaw:",self.yaw())
                print("PID output:", output)
                setMotor.send(msg)
                self.rate.sleep()

class SetHeading():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, chat_frequency=1.0):

        # publishing objects
        self.chatter_pub = rospy.Publisher("/control/heading", Float64, queue_size=1)
        self.chat_frequency = rospy.Rate(chat_frequency)

    def send(self, msg):
        ''' Send messages on chatter topic at regular rate
            Float64: 
                float data
        '''
        msg = Float64(msg)
        self.chatter_pub.publish(msg)

    def stop(self):
        if controller.thread:
            controller.thread.end()

#rospy.init_node('HeadingController')
controller = HeadingController()
sender = SetHeading()
print("Heading controller node running")

if __name__ == '__main__':
    rospy.spin()
