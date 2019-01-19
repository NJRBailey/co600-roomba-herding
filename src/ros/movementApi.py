#!/usr/bin/env python

# Movement API for controlling drone, both in simulator and real world

import rospy
from geometry_msgs.msg import Twist
import RosUtils

# movementApi should be instansiated once and then called when needed
class movementApi:

    def __init__(self):
        self.dirPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def forward(self, speed):
        self.dirPub.publish(RosUtils.create_twist(speed, 0, 0, 0, 0, 0))

    def backward(self, speed):
        self.dirPub.publish(RosUtils.create_twist(-speed, 0, 0, 0, 0, 0))
    
    def left(self, speed):
        self.dirPub.publish(RosUtils.create_twist(0, speed, 0, 0, 0, 0))

    def right(self, speed):
        self.dirPub.publish(RosUtils.create_twist(0, -speed, 0, 0, 0, 0))

    def stop(self):
        self.dirPub.publish(RosUtils.create_twist(0, 0, 0, 0, 0, 0))

    def custom(self, x, y, z, a, b, c):
        self.dirPub.publish(RosUtils.create_twist(x, y, z, a, b, c))

movementApi()


