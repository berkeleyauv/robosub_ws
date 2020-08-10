import cv2
import os
from datetime import datetime

from robosub.IMUListener import imu
from robosub.YawListener import yaw


class VideoSaver:

    def __init__(self, video_dir):
        self.zed = cv2.VideoCapture(0)
        self.webcam = cv2.VideoCapture(1)
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.zed_video = cv2.VideoWriter(os.path.join(video_dir, 'zed_video_' + str(datetime.now()) + '.avi'), \
                        fourcc, 30, (int(self.zed.get(3)), int(self.zed.get(4))))
        self.webcam_video = cv2.VideoWriter(os.path.join(video_dir, 'webcam_video_' + str(datetime.now()) + '.avi'), \
                        fourcc, 30, (int(self.webcam.get(3)), int(self.webcam.get(4))))

    def writeVideo(self):
        zed_ret, zed_frame = self.zed.read()
        if zed_ret:
            self.zed_video.write(zed_frame)
        webcam_ret, webcam_frame = self.webcam.read()
        if webcam_ret:
            self.webcam_video.write(webcam_frame)

    def releaseVideo(self):
        self.zed.release()
        self.zed_video.release()
        self.webcam.release()
        self.webcam_video.release()


class TelemetrySaver:

    def __init__(self, file_dir):
        self.file = os.path.join(file_dir, 'telemetry_' + str(datetime.now()) + '.csv')
        self.write_header()

    def write_header(self):
        telemetry_header = 'Time,Linear Acceleration X,Linear Acceleration Y,Linear Acceleration Z,Angular Velocity X,Angular Velocity Y,Angular Velocity Z,Orientation W,Orientation X,Orientation Y,Orientation Z,Yaw\n'
        with open(self.file, 'a+') as file:
            file.write(telemetry_header)

    def write(self):
        with open(self.file, 'a+') as file:
            data = [datetime.now(), imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
                    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z, imu.orientation.w, imu.orientation.x,
                    imu.orientation.y, imu.orientation.z, yaw.yaw]
            data = ','.join(map(str, data))
            file.write(data + '\n')

