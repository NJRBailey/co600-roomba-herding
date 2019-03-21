#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import RosUtils


## MovementApi is responsible for publishing movement messages to the drone
#
# Note the drone will continue with any movement command until a stop command is issued.
class movementApi:

    ## Initialises movementApi object.
    #
    # Creates ROS message publishers that allow us to take off, land, emergency land and move.
    def __init__(self):
        self.dirPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.takeOffPub = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
        self.landPub = rospy.Publisher('ardrone/land', Empty, queue_size=1)
        self.emergencyPub = rospy.Publisher('ardrone/reset', Empty, queue_size=1)

    ## Moves the drone forwards.
    #
    # @param speed The speed at which the drone will move in metres per second.
    def forward(self, speed):
        self.dirPub.publish(RosUtils.create_twist(speed, 0, 0, 0, 0, 0))

    ## Moves the drone backwards.
    #
    # @param speed The speed at which the drone will move in metres per second.
    def backward(self, speed):
        self.dirPub.publish(RosUtils.create_twist(-speed, 0, 0, 0, 0, 0))
    
    ## Moves the drone left.
    #
    # @param speed The speed at which the drone will move in metres per second.
    def left(self, speed):
        self.dirPub.publish(RosUtils.create_twist(0, speed, 0, 0, 0, 0))

    ## Moves the drone right.
    #
    # @param speed The speed at which the drone will move in metres per second.
    def right(self, speed):
        self.dirPub.publish(RosUtils.create_twist(0, -speed, 0, 0, 0, 0))

    ## Stops the drone.
    def stop(self):
        self.dirPub.publish(RosUtils.create_twist(0, 0, 0, 0, 0, 0))

    ## Sends a custom movement command to the drone.
    #
    # @param x movement speed in the x axis.
    # @param y movement speed in the y axis.
    # @param z movement speed in the z axis.
    # @param rotX movement speed of rotation around x axis.
    # @param rotY movement speed of rotation around y axis.
    # @param rotZ movement speed of rotation around z axis.
    def custom(self, x, y, z, rotX, rotY, rotZ):
        self.dirPub.publish(RosUtils.create_twist(x, y, z, rotX, rotY, rotZ))
    
    ## Sends a takeoff message to the drone.
    def takeOff(self):
        self.takeOffPub.publish(Empty())

    ## Sends a land message to the drone.
    def land(self):
        self.landPub.publish(Empty())
    
    ## Sends emergency message to the drone.
    #
    # Emergency message causes drone to enter emergency mode, effectively shutting the drone down.
    # Note: emergency mode does not work in the simulator.
    def emergency(self):
        self.emergencyPub.publish(Empty())

movementApi()


