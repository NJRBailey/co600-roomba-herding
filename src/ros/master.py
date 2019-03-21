#!/usr/bin/env python

import rospy
import movementApi
import TakeOff
import search
import Hover
import guide


## Master is responsible for initiating each stage of the project.
# The workflow is as follows: TakeOff -> Search -> Guide. \n
# The command to run this module is: "rosrun co600_proj master.py". \n 
# Note: Requires either simulator or ardrone autonomy to be running
# as well as the services: "rotation_offset_srv", "height_offset_srv"
# and "latest_image_srv".
class master:

    ## Initialises master object
    #
    # Initialises movementApi object and waits for the ROS services to come online
    def __init__(self):
        rospy.init_node('co600_master', anonymous=False)
        self.movementApi = movementApi.movementApi()
        rospy.wait_for_service('rotation_offset_srv')
        rospy.wait_for_service('height_offset_srv')
        rospy.wait_for_service('latest_image_srv')
        self.execute()

    ## Sequentially runs TakeOff, Search and Guide
    #
    # Waits 5 seconds to ensure ROS message subs/pubs have enough time to start
    def execute(self):
        rospy.sleep(5)
        TakeOff.TakeOff(self.movementApi)
        search.SearchRoomba(self.movementApi)
        guide.guide(self.movementApi)

if __name__ == "__main__":
    master()