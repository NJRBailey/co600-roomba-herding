#!/usr/bin/env python

# Subscribes to the drones camera feed and stores the most recent image

import rospy
import sys
from sensor_msgs.msg import Image
from co600_proj.srv import GetLatestImage

class ImageSub:

    def __init__(self, simulator=True):
        if simulator:
            queue = 'ardrone/bottom/image_raw'
        else:
            queue = 'ardrone/front/image_raw'
        rospy.init_node('image_server', anonymous=False)
        # __NOTE__ : subscribes to 'ardrone/bottom/image_raw' in simulator
        #            and 'ardrone/front/image_raw' in real life
        self.imageSub = rospy.Subscriber(queue, Image, self.callback)
        self.service = rospy.Service('latest_image_srv', GetLatestImage, self.getLatestImage)
        self.latestImage = None
        rospy.spin()

    def callback(self, RosImage):
        self.latestImage = RosImage

    def getLatestImage(self, req):
        return self.latestImage

if __name__ == "__main__":
    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) > 1:
        ImageSub(myargv[1])
    else:
        ImageSub()

