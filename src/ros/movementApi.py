#!/usr/bin/env python

# Movement API for controlling drone, both in simulator and real world

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import RosUtils

class movementApi:

    def __init__(self):
        self.dirPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.takeOffPub = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
        self.landPub = rospy.Publisher('ardrone/land', Empty, queue_size=1)
        self.emergencyPub = rospy.Publisher('ardrone/reset', Empty, queue_size=1)

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

    def custom(self, x, y, z, rotX, rotY, rotZ):
        self.dirPub.publish(RosUtils.create_twist(x, y, z, rotX, rotY, rotZ))

    def takeOff(self):
        self.takeOffPub.publish(Empty())

    def land(self):
        self.landPub.publish(Empty())
    
    def emergency(self):
        self.emergencyPub.publish(Empty())

movementApi()


