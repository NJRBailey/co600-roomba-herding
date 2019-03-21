#!/usr/bin/env python

import rospy
import roombaMovementApi
import random
from co600_proj.srv import StopRoomba

## roombaIdle is responsible for publishing random movement commands to the roomba.
#
# Run this command to run the service: "rosrun co600_proj roombaIdle.py".
class roombaIdle:

    ## Initialises roombaIdle, creates an instance of roombaMovementApi and a ROS service.
    def __init__(self):
        rospy.init_node('roomba_movement')
        self.stop = False
        self.service = rospy.Service('stop_roomba_srv', StopRoomba, self.stopRoomba)
        self.movementApi = roombaMovementApi.roombaMovementApi()
        self.roombaLoop()

    ## Publishes random movement commands to the roomba in the simulator.
    def roombaLoop(self):
        commandList = [self.movementApi.left, self.movementApi.right, self.movementApi.forward, self.movementApi.backward]
        while (self.stop == False):
            command  = commandList[random.randint(0, 3)]
            duration = random.randint(1, 4)
            command(0.5)
            rospy.sleep(duration)
        self.movementApi.stop()

    ## ROS service response
    #
    # Returns true and stops sending roomba random movement commands.
    # @param req ROS service request
    def stopRoomba(self, req):
        self.stop = True
        return 1

if __name__ == '__main__':
    roombaIdle()