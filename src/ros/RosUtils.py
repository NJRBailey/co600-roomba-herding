# Utility module for ROS python modules, used to store commonly used methods

import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

## Creates a geometry twist message used to control drone's direction.
def create_twist(x, y, z, a, b, c):
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = z
    twist.angular.x = a
    twist.angular.y = b
    twist.angular.z = c
    return twist

## Converts an image recieved via ROS into an OpenCV compatible format
#
# @param image A ROS message image.
def RosImageToCv(image):
    bridge = CvBridge()
    try:
        cvImage = bridge.imgmsg_to_cv2(image, "bgr8")
        return cvImage
    except CvBridgeError as e:
        print(e)