#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from controls.msg import SubThrusts
# from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


class VelocityTransformer:

    def __init__(self):
        self.sub = rospy.Subscriber('/sub/cmd_vel', Twist, self.vel_callback)
        self.pub = rospy.Publisher('/sub/thrusts', SubThrusts, queue_size=10)
        # self.thrusters = [rospy.Publisher(f'/sub/thrusters/{i}/input', FloatStamped, queue_size=10) for i in ]

    def vel_callback(self, twist):
        """
        Assuming that 
            forward is +y
            lateral right is +x
            depth up is +z

        Assuming that thrusts order is

        Horizontal drive
        1. top left
        2. top right
        3. bottom left
        4. bottom right

        Vertical drive
        5. top left
        6. top right
        7. bottom left
        8. bottom right

        Only for Robosub AUV for now

        """

        msg = SubThrusts()
        msg.header.stamp = rospy.Time.now()
        msg.thrusts[0] = twist.linear.x + twist.linear.y - twist.angular.z
        msg.thrusts[1] = -twist.linear.x + twist.linear.y + twist.angular.z
        msg.thrusts[2] = -twist.linear.x + twist.linear.y - twist.angular.z
        msg.thrusts[3] = twist.linear.x + twist.linear.y + twist.angular.z

        for i in range(4):
            msg.thrusts[4 + i] = twist.linear.z

        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('velocity_transformer')
    vel_transform = VelocityTransformer()
    while not rospy.is_shutdown():
        rospy.spin()
