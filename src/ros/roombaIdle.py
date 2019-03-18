#!/usr/bin/env python

import rospy
import simRoombaMove
import random
from co600_proj.srv import StopRoomba

class roombaIdle:

    def __init__(self):
        rospy.init_node('roomba_movement')
        self.stop = False
        self.service = rospy.Service('stop_roomba_srv', StopRoomba, self.stopRoomba)
        self.movementApi = simRoombaMove.simRoombaMove()
        self.roombaLoop()

    def roombaLoop(self):
        commandList = [self.movementApi.left, self.movementApi.right, self.movementApi.forward, self.movementApi.backward]
        while (self.stop == False):
            command  = commandList[random.randint(0, 3)]
            duration = random.randint(1, 4)
            command(0.5)
            rospy.sleep(duration)
        self.movementApi.stop()

    def stopRoomba(self, req):
        self.stop = True
        return 1

if __name__ == '__main__':
    roombaIdle()