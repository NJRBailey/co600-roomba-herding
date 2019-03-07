import rospy
from geometry_msgs.msg import Twist
import RosUtils

class simRoombaMove:

    def __init__(self):
        self.dirPub = rospy.Publisher('robot1/mobile_base/commands/velocity', Twist, queue_size=1)

    def forward(self, speed):
        self.dirPub.publish(RosUtils.create_twist(speed, 0, 0, 0, 0, 0))

    def backward(self, speed):
        self.dirPub.publish(RosUtils.create_twist(-speed, 0, 0, 0, 0, 0))
    
    def left(self, speed):
        self.dirPub.publish(RosUtils.create_twist(0, 0, 0, 0, 0, speed))

    def right(self, speed):
        self.dirPub.publish(RosUtils.create_twist(0, 0, 0, 0, 0, -speed))

    def stop(self):
        self.dirPub.publish(RosUtils.create_twist(0, 0, 0, 0, 0, 0))

    def custom(self, x, y, z, a, b, c):
        self.dirPub.publish(RosUtils.create_twist(x, y, z, a, b, c))

simRoombaMove()