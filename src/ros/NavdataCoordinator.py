#!/usr/bin/env python

import rospy
from ardrone_autonomy.msg import Navdata
from co600_proj.srv import HeightOffset, RotationOffset

# Desired height in mm 1.9m = 1900mm
DESIRED_HEIGHT = 1900
ACCEPTABLE_OFFSET = 100
# 10 degrees of error for rotational drift
ROT_OFFSET = 5

## NavdataCoordinator is responsible for interpreting navdata messages sent from the drone.
#
# Service is launched with command: "rosrun co600_proj NavdataCoordinator"
class NavdataCoordinator:

    ## Initialises NavdataCoordinator
    #
    # Launches 1 ROS subscriber and 2 ROS services
    def __init__(self):
        rospy.init_node('navdata_coordination_server', anonymous=False)
        self.subscriber = rospy.Subscriber('ardrone/navdata', Navdata, self.callback)
        self.heightService = rospy.Service('height_offset_srv', HeightOffset, self.getHeightOffset)
        self.rotationService = rospy.Service('rotation_offset_srv', RotationOffset, self.getRotationOffset)
        self.heightOffset = 0
        self.rotationOffset = 0
        rospy.spin()

    ## Callback for ROS subscriber.
    #
    # Stores the value of the drones height and rotation.
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

    ## Response for ROS service
    #
    # Returns the value of height offset.
    # @param req ROS service request
    def getHeightOffset(self, req):
        return self.heightOffset

    ## Response for ROS service
    #
    # Returns the value of rotation offset
    # @param req ROS service request
    def getRotationOffset(self, req):
        return self.rotationOffset

if __name__ == "__main__":
    NavdataCoordinator()