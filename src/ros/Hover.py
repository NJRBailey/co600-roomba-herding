#!/usr/bin/env python

# Hover calls two services, ensures the drone flys at 2meters and will hover
# over any marker it detects through the camera feed.

import rospy
from image_analysis import PatternLocation
import movementApi
from RosUtils import RosImageToCv
from co600_proj.srv import HieghtOffset, GetLatestImage


class Hover:

    def __init__(self):
        rospy.init_node('hover_node', anonymous=False)
        self.movementApi = movementApi.movementApi()
        rospy.wait_for_service('hieght_offset_srv')
        rospy.wait_for_service('latest_image_srv')
        self.HieghtOffsetSrv = rospy.ServiceProxy('hieght_offset_srv', HieghtOffset)
        self.LatestImageSrv = rospy.ServiceProxy('latest_image_srv', GetLatestImage)
        self.hover_loop()

    def hover_loop(self):
        while(True):
            HieghtOffset = self.HieghtOffsetSrv()
            LatestImage = self.LatestImageSrv()
            x = 0
            y = 0
            z = 0
            if HieghtOffset.hieght > 0:
                z = -1
            elif HieghtOffset.hieght < 0:
                z = 1
            try:
                ImageResult = PatternLocation.findPatternLocation(RosImageToCv(LatestImage.image), debug=True)
                if ImageResult == 'c':
                    print("centered")
                elif ImageResult == 'tl':
                    x = 0.5
                    y = 0.5
                elif ImageResult == 'tr':
                    x = 0.5
                    y = -0.5
                elif ImageResult == 't':
                    x = 0.5
                elif ImageResult == 'l':
                    y = 0.5
                elif ImageResult == 'r':
                    y = -0.5
                elif ImageResult == 'bl':
                    x = -0.5
                    y = 0.5
                elif ImageResult == 'br':
                    x = -0.5
                    y = -0.5
                elif ImageResult == 'b':
                    x = -0.5
                print("publishing "+ str(x) +", "+ str(y) +", "+str(z))
                self.movementApi.custom(x, y, z, 0, 0, 0)
            except Exception as e:
                print(e)

if __name__ == "__main__":
    Hover()