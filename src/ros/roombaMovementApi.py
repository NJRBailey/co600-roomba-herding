import rospy
from geometry_msgs.msg import Twist
import RosUtils

## roombaMovementApi is responsible for publishing movement messages to the simulated roomba.
#
# Note: communicating with the roomba in real life is done via serial.
class roombaMovementApi:

    ## Initialises roombaMovementApi
    #
    # Initialises a ROS publisher to send movement commands to the simulated roomba
    def __init__(self):
        self.dirPub = rospy.Publisher('robot1/mobile_base/commands/velocity', Twist, queue_size=1)

    ## Moves the roomba forward
    #
    # @param speed Speed at which the roomba will move.
    def forward(self, speed):
        self.dirPub.publish(RosUtils.create_twist(speed, 0, 0, 0, 0, 0))

    ## Moves the roomba backward
    #
    # @param speed Speed at which the roomba will move.
    def backward(self, speed):
        self.dirPub.publish(RosUtils.create_twist(-speed, 0, 0, 0, 0, 0))
    
    ## Rotates the roomba left
    #
    # @param speed Speed at which the roomba will rotate.
    def left(self, speed):
        self.dirPub.publish(RosUtils.create_twist(0, 0, 0, 0, 0, speed))

    ## Rotates the roomba right
    #
    # @param speed Speed at which the roomba will rotate.
    def right(self, speed):
        self.dirPub.publish(RosUtils.create_twist(0, 0, 0, 0, 0, -speed))

    ## Stops the roomba
    def stop(self):
        self.dirPub.publish(RosUtils.create_twist(0, 0, 0, 0, 0, 0))

    ## Send a custom movement command to the roomba.
    def custom(self, x, y, z, a, b, c):
        self.dirPub.publish(RosUtils.create_twist(x, y, z, a, b, c))

roombaMovementApi()