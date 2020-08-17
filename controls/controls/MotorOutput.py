#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ManualControl, OverrideRCIn

'''
A python script to practice sending ROS messages
'''

class SetMotors():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self):

        # publishing objects
        self.chatter_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        #self.chatter_pub = rospy.Publisher("/mavros/manual_control/send", ManualControl, queue_size=1)
        # rate of publishing
        #self.chat_frequency = rospy.Rate(0.25)
        

    def send(self, msg):
        """ Send messages on chatter topic at regular rate.
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
        # i = 0
        #while (not rospy.is_shutdown()):
        #     i = i + 1
            
        #     #msg.header.seq = i
        #     #msg.header.stamp = rospy.Time.now()
        #     self.chatter_pub.publish(msg)
        #     self.chat_frequency.sleep()
        msg = OverrideRCIn(msg)
        #while not rospy.is_shutdown():
        print(msg)
        self.chatter_pub.publish(msg)
        #self.chat_frequency.sleep()
#        self.chatter_pub.publish(msg)

    def stop(self):
        self.send([1500]*8)


if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('RC_override')
    rospy.sleep(.5)

setMotor = SetMotors()
print("SetMotor node running")

# msg = ManualControl()
# msg.header.frame_id = 'Manual control'
# msg.x = 100
# msg.y = 0
# msg.z = 0
# msg.r = 0
# msg.buttons = 0
