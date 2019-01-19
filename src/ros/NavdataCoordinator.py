#!/usr/bin/env python

# Subscribes to the navdata stream and monitors the difference in desired/actual hieght

import rospy
from ardrone_autonomy.msg import Navdata
from co600_proj.srv import HieghtOffset

# Desired hieght in mm 1.9m = 1900mm
DESIRED_HIEGHT = 1900
ACCEPTABLE_OFFSET = 100

class NavdataCoordinator:

    def __init__(self):
        rospy.init_node('navdata_coordination_server', anonymous=False)
        self.subscriber = rospy.Subscriber('ardrone/navdata', Navdata, self.callback)
        self.service = rospy.Service('height_offset_srv', HeightOffset, self.getHeightOffset)
        self.height_offset = 0
        rospy.spin()

    def callback(self, navdata):
        if navdata.altd - DESIRED_HIEGHT > ACCEPTABLE_OFFSET:
            self.height_offset = navdata.altd - DESIRED_HIEGHT
        elif navdata.altd - DESIRED_HIEGHT < -ACCEPTABLE_OFFSET:
            self.height_offset = navdata.altd - DESIRED_HIEGHT
        else:
            self.height_offset = 0
    
    def getHeightOffset(self, req):
        return self.height_offset

if __name__ == "__main__":
    NavdataCoordinator()