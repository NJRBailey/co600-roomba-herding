import movementApi
import rospy
from co600_proj.srv import HeightOffset, RotationOffset

class TakeOff:

    def __init__(self, movement=movementApi.movementApi()):
        self.movement = movement
        self.rotationOffsetSrv = rospy.ServiceProxy('rotation_offset_srv', RotationOffset)
        self.heightOffsetSrv = rospy.ServiceProxy('height_offset_srv', HeightOffset)
        self.execute()

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