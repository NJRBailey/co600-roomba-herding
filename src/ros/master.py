#!/usr/bin/env python

# Master is responsible for Initiating takeoff and hover at desired hieght 
# -> Searching for roomba -> Hovering and returning.
# It is also responsible for controling the emergency landing (KEY).

import rospy
import movementApi
import TakeOff
import search
import Hover
import guide

class master:

    def __init__(self):
        rospy.init_node('co600_master', anonymous=False)
        self.movementApi = movementApi.movementApi()
        rospy.wait_for_service('rotation_offset_srv')
        rospy.wait_for_service('height_offset_srv')
        rospy.wait_for_service('latest_image_srv')
        self.execute()

    def execute(self):
        # Take off -> Search -> Hover/Return
        rospy.sleep(5)
        TakeOff.TakeOff(self.movementApi)
        search.SearchRoomba(self.movementApi)
        guide.guide(self.movementApi)

if __name__ == "__main__":
    master()