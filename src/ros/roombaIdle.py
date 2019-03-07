import rospy
import simRoombaMove
import random

class roombaIdle:

    def __init__(self):
        rospy.init_node('roomba_movement')
        self.stop = False
        self.movementApi = simRoombaMove.simRoombaMove()
        self.roombaLoop()

    def roombaLoop(self):
        commandList = [self.movementApi.left, self.movementApi.right, self.movementApi.forward, self.movementApi.backward]
        while (self.stop == False):
            command  = commandList[random.randint(0, 3)]
            duration = random.randint(1, 4)
            command(0.5)
            rospy.sleep(duration)

if __name__ == '__main__':
    roombaIdle()