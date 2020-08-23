#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
from threading import Thread


class RazorIMU(Thread):

    def __init__(self, read_frequency=10, *args, **kwargs):
        super(RazorIMU, self).__init__(*args, **kwargs)
        self.serial = serial.Serial('/dev/ttyACM1')
        self.pub = rospy.Publisher('/controls/razor', String, queue_size=10)
        ## TODO: Add more topics that we publish to individually
        self.rate = rospy.Rate(read_frequency) # 10hz
  
    def run(self):
        while not rospy.is_shutdown():
            output = str(self.serial.readline())
            vals = output.replace(' ', '').split(',')
            assert len(vals) == 18
            self.time = vals[0]
            self.accel = vals[1:4]
            self.gyro = vals[4:7]
            self.mag = vals[7:10]
            self.quat = vals[10:14]
            self.angle = vals[14:17]
            self.heading = vals[17]
            self.reading = output
            self.pub.publish(output)
            #rospy.loginfo(output)

if __name__ == '__main__':
    rospy.init_node('razor_imu')

razor = RazorIMU()
razor.daemon = True
razor.start()


if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.spin()
