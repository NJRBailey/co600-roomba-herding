import movementApi
import rospy
from co600_proj.srv import HeightOffset, RotationOffset

## TakeOff is responsible for monitoring the drones take off stage.
#
# TakeOff ensures the drone reaches the required height and corrects rotational drift.
class TakeOff:

    ## Initialises the TakeOff object
    #
    # @param movement movementApi object required to send commands to drone.
    def __init__(self, movement=movementApi.movementApi()):
        self.movement = movement
        self.rotationOffsetSrv = rospy.ServiceProxy('rotation_offset_srv', RotationOffset)
        self.heightOffsetSrv = rospy.ServiceProxy('height_offset_srv', HeightOffset)
        self.execute()

    ## Issues a takeoff command and ensures the drone reaches the desired height.
    def execute(self):
        self.movement.takeOff()
        while(not self.rotationOffsetSrv().rotation == 0 or not self.heightOffsetSrv().height == 0):
            if self.rotationOffsetSrv().rotation > 0:
                zRot = 0.5
            elif self.rotationOffsetSrv().rotation == 0:
                zRot = 0
            else:
                zRot = -0.5
            if self.heightOffsetSrv().height > 0:
                z = 0.5
            elif self.heightOffsetSrv().height == 0:
                z = 0
            else:
                z = -0.5
            self.movement.custom(0, 0 , z, 0, 0, zRot)