#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String

'''
A python script to practice sending ROS messages
'''

class Talker():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, chat_frequency=1.0):

        # publishing objects
        self.chatter_pub = rospy.Publisher("/chatter", String, queue_size=1)

        # rate of publishing
        self.chat_frequency = rospy.Rate(chat_frequency)

    def start_chatter(self):
        ''' send messages on chatter topic at regular rate
        '''
        i = 0
        while (not rospy.is_shutdown()):
            i = i + 1
            '''TODO-START: FILL IN CODE HERE 
            * create a string message that contains "Hello World" and the iteration number i
            '''
            chat_msg = "Hello World! This is my {}th iteration!".format(i)
            '''raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")'''
            '''TODO-END '''
            self.chatter_pub.publish(chat_msg)
            self.chat_frequency.sleep()

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('talker')
    td = Talker()
    print("Talker node running")

    # start the chatter
    td.start_chatter()
    rospy.spin()
