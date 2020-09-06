import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageListener:
    """ Subscribes to ROS Image messages. Used for getting camera image data
    from the simulator and converting it for OpenCV. """

    def callback(self, img_msg):
        bridge = CvBridge()
        # Stores as a BGR array for OpenCV.
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        self.images.append(cv_image)

    def get_images(self, time):
        self.images = []
        rospy.sleep(time) # time is in seconds
        return self.images

class FrontCameraListener(ImageListener):
    """ Gets image data from the front camera. """
    def __init__(self):
        self.subscr = rospy.Subscriber("/urab_sub/urab_sub/camerafront/camera_image",
            Image, self.callback)

class UnderCameraListener(ImageListener):
    """ Gets image data from the bottom camera. """
    def __init__(self):
        self.subscr = rospy.Subscriber("/urab_sub/urab_sub/cameraunder/camera_image",
            Image, self.callback)

if __name__ == '__main__':
    rospy.init_node('img_listener', anonymous=True)
    front_listen = FrontCameraListener()
    imgs = front_listen.get_images(1.0)

    print "FRONT CAMERA"
    print "ImageListener.get_images() has type " + str(type(imgs))
    print "Length of image list: " + str(len(imgs))
    print "The first image has type " + str(type(imgs[0]))
    print "The first image has dimensions " + str(imgs[0].shape)
    print "Print out the first image:"
    print imgs[0]

    under_listen = UnderCameraListener()
    imgs = under_listen.get_images(1.0)

    print "\n\nBOTTOM CAMERA"
    print "ImageListener.get_images() has type " + str(type(imgs))
    print "Length of image list: " + str(len(imgs))
    print "The first image has type " + str(type(imgs[0]))
    print "The first image has dimensions " + str(imgs[0].shape)
    print "Print out the first image:"
    print imgs[0]
