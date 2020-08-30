import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageListener():
    """ Subscribes to ROS Image messages. Used for getting camera image data
    from the simulator and converting it for OpenCV. """

    def __init__(self):
        self.sub = rospy.Subscriber("/urab_sub/urab_sub/camerafront/camera_image",
            Image, self.callback)

    def callback(self, img_msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        self.images.append(cv_image)

    def get_images(self, time):
        self.images = []
        rospy.sleep(time) # time is in seconds
        return self.images

if __name__ == '__main__':
    rospy.init_node('img_listener', anonymous=True)
    listener = ImageListener()
    imgs = listener.get_images(1.0)

    print "ImageListener.get_images() has type " + str(type(imgs))
    print "Length of image list: " + str(len(imgs))
    print "The first image has type " + str(type(imgs[0]))
    print "The first image has dimensions " + str(imgs[0].shape)
    print "Print out the first image:"
    print imgs[0]
