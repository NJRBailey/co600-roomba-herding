#!/usr/bin/env python

# Subscribes to the navdata stream and monitors the difference in desired/actual height

import rospy
from ardrone_autonomy.msg import Navdata
from co600_proj.srv import HeightOffset, RotationOffset

# Desired height in mm 1.9m = 1900mm
DESIRED_HEIGHT = 1900
ACCEPTABLE_OFFSET = 100
# 10 degrees of error for rotational drift
ROT_OFFSET = 5

class NavdataCoordinator:

    def __init__(self):
        rospy.init_node('navdata_coordination_server', anonymous=False)
        self.subscriber = rospy.Subscriber('ardrone/navdata', Navdata, self.callback)
        self.heightService = rospy.Service('height_offset_srv', HeightOffset, self.getHeightOffset)
        self.rotationService = rospy.Service('rotation_offset_srv', RotationOffset, self.getRotationOffset)
        self.heightOffset = 0
        self.rotationOffset = 0
        rospy.spin()

    def callback(self, navdata):
        if navdata.altd - DESIRED_HEIGHT > ACCEPTABLE_OFFSET:
            self.heightOffset = -1
        elif navdata.altd - DESIRED_HEIGHT < -ACCEPTABLE_OFFSET:
            self.heightOffset = 1
        else:
            self.heightOffset = 0
        print(navdata.rotZ)
        if navdata.rotZ > ROT_OFFSET:
            self.rotationOffset = -1
        elif navdata.rotZ < -ROT_OFFSET:
            self.rotationOffset = 1
        else:
            self.rotationOffset = 0

    def getHeightOffset(self, req):
        return self.heightOffset

    def getRotationOffset(self, req):
        return self.rotationOffset

if __name__ == "__main__":
    NavdataCoordinator()