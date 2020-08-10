#!/usr/bin/env python

from __future__ import print_function
import sys
import time

# Code to handle automatic starting on bootup for competition. Needs refactoring into SystemD
if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'comp':
        import os
        os.spawnl(os.P_NOWAIT, 'roscore')
        time.sleep(3)
        os.spawnl(os.P_NOWAIT, 'roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 gcs_url:=udp://@192.168.137.1:14550')
        time.sleep(3)

import rospy

rospy.init_node("SubControls")
rospy.sleep(0.5)

from robosub import ControlMode, MotorOutput
from robosub.utils import VideoSaver, TelemetrySaver
from robosub.IMUListener import imu
from robosub.YawListener import yaw
from robosub.RazorIMU import razor
from robosub.controllers import *

import traceback
import sys
import time
from datetime import datetime 
import cv2

MODES = ['power', 'velocity', 'heading', 'disarm', 'arm', 'stabilize', 'manual', 'depth']

START_DELAY = 10
RUN_TIME = 3


class Main:

    def __init__(self):
        self.mode = ControlMode.mode
        self.out = MotorOutput.setMotor
        zero = [1500]*8
        self.mode.send('power')
        self.out.send(zero)

    def run(self, option, input):
        if option in MODES:
            code = self.mode.send(option)
        if option in ['power', 'stabilize', 'depth']:
            self.out = MotorOutput.setMotor
            msg = [int(i) for i in input] + [1500, 1500]
        elif option == 'vision':            
           return
        elif option == 'velocity':
            self.out = VelocityController.sender
            msg = input[:3]
        # elif mode == 'position':
        #     self.out = PositionController.sender
        #     msg = output
        elif option == 'heading':
            self.out = HeadingController.sender
            msg = input[0]
        elif option == 'imu':
            print("Linear accel: {}".format(imu.linear_acceleration))
            print("Angular vel: {}".format(imu.angular_velocity))
            print("Orientation: {}".format(imu.orientation))
            print("Linear vel: {}".format((imu.velX[-1], imu.velY[-1], imu.velZ[-1])))
            return
        elif option == 'razor':
            # print("Razor IMU reading: {}".format(razor.reading))
            print("Razor accel: {}".format(razor.accel))
            print("Razor gyro: {}".format(razor.gyro))
            print("Razor mag: {}".format(razor.mag))
            print("Razor quaternion: {}".format(razor.quat))
            print("Razor angles: {}".format(razor.angle))
            print("Razor heading: {}".format(razor.heading))
            return
        elif option == 'yaw':
            print("Current yaw:", yaw.yaw)
            return
        elif option == 'stop':
            self.mode.send('power')
            self.out = MotorOutput.setMotor
            msg = [1500]*8
        else:
            print("Invalid option:", option)
            return
        self.out.send(msg)


def processInput():
    try:
        m = Main()
        print()
        print("Instructions:")
        print("Options are: power, velocity, heading, vision, imu, yaw, razor, arm, disarm, stop, stabilize, depth, manual")
        print("Outputs are needed for: power, velocity, and heading. Need to be in space delimited format")
        print("Keyboard interrupt(Ctrl+C) to exit")
        while not rospy.is_shutdown():
            try:
                inputString = raw_input("Provide [option] [input] \n").split()
                if inputString == 'exit':
                    raise KeyboardInterrupt()
                if inputString:
                    option = inputString[0]
                    input = [float(item) for item in inputString[1:]]
                    m.run(option, input)
                rospy.sleep(0.1)
            except Exception as e:
                print("Error occurred: " + str(e))
                traceback.print_exc()
    except KeyboardInterrupt:
        print("Sub shutting down...")
    finally:
        MotorOutput.setMotor.stop()
        m.mode.send('disarm')


if __name__ == '__main__':
    if len(sys.argv) == 1:
        processInput()
    elif sys.argv[1] == 'auto' or sys.argv[1] == 'comp': # TODO: Rewrite the autonomou competition code
        video = VideoSaver()
        try:
            start_time = time.time()
            # while ((time.time() - start_time) < START_DELAY):
            #     writeVideo()
            main = Main()
            main.mode.send('arm')
            time.sleep(1)
            main.mode.send('power')
            start_time = time.time()
            while ((time.time() - start_time) < RUN_TIME*2.5):
                main.out.send([1500,1500,1500,1500,1600,1500,1500,1500])
                video.writeVideo()
            start_time = time.time()
            while ((time.time() - start_time) < RUN_TIME):
                main.out.send([1500,1500,1500,1500,1500,1600,1500,1500])
                video.writeVideo()
            start_time = time.time()
            while ((time.time() - start_time) < RUN_TIME*2.5):
                main.out.send([1500,1500,1500,1500,1400,1500,1500,1500])
                video.writeVideo()
            while ((time.time() - start_time) < RUN_TIME):
                main.out.send([1500,1500,1500,1500,1500,1400,1500,1500])
                video.writeVideo()
            # Release everything if job is finished
            main.out.send([1500,1500,1500,1500,1500,1500,1500,1500])
        except KeyboardInterrupt as i:
            pass
        finally:
            video.releaseVideo()
