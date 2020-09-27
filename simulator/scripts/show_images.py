import rospy
import numpy as np
import cv2
import read_cameras

def display_camera_images(listener, time, window_name="video"):
    """Displays camera images received from an ImageListener object, which
    gets video data for the specified amount of time, in seconds."""
    imgs = listener.get_images(1.0)
    for frame in imgs:
        cv2.imshow(window_name, frame)
         # you need to hold a key to see the video progress. Press q to quit.
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('img_listener', anonymous=True)
    front_listen = read_cameras.FrontCameraListener()
    display_camera_images(front_listen, 1, "front camera")
