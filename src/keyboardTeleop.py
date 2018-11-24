#!/usr/bin/env python
# keyboardTeleop.py uses keyboard input to control drone
# works for simulator and real drone.

from pynput.keyboard import Key, Listener
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def keyboardTeleop():

    def onPress(key):
        print('{0} pressed'.format(key))
        if key == Key.up:
            print('Going forward!')
            dirPub.publish(createTwist(0.5, 0, 0, 0, 0, 0))
        if key == Key.down:
            print('Going back')
            dirPub.publish(createTwist(-0.5, 0, 0, 0, 0, 0))
        if key == Key.left:
            print('Going left')
            dirPub.publish(createTwist(0, 0.5, 0, 0, 0, 0))
        if key == Key.right:
            print('Going right')
            dirPub.publish(createTwist(0, -0.5, 0, 0, 0, 0))
        if key == Key.backspace or key == Key.esc:
            print('Landing')
            landPub.publish(Empty())
        if key == Key.enter:
            print('Taking off')
            takeOffPub.publish(Empty())
        if key == Key.space:
            print('Emergency landing')
            emergencyPub.publish(Empty())
        try:
            if key.char == "u":
                print('Going up')
                dirPub.publish(createTwist(0, 0, 0.5, 0, 0, 0))
            if key.char == "j":
                print('Going down')
                dirPub.publish(createTwist(0, 0, -0.5, 0, 0, 0))
            if key.char == "h":
                print('Rotating left')
                dirPub.publish(createTwist(0, 0, 0, 0, 0, 1.0))
            if key.char == "k":
                print('Rotating right')
                dirPub.publish(createTwist(0, 0, 0, 0, 0, -1.0))
        except AttributeError:
            pass

    def onRelease(key):
        print('{0} release'.format(key))
        if key == Key.esc:
            # Stop listener
            return False
        if key == Key.enter or key == Key.backspace:
            pass
        else:
            dirPub.publish(createTwist(0, 0, 0, 0, 0, 0))

    def createTwist(x, y, z, a, b, c):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.x = a
        twist.angular.y = b
        twist.angular.z = c
        return twist

    takeOffPub = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
    landPub = rospy.Publisher('ardrone/land', Empty, queue_size=1)
    emergencyPub = rospy.Publisher('ardrone/reset', Empty, queue_size=1)
    dirPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    print('registering')
    rospy.init_node('command_node', anonymous=False)

    with Listener(on_press=onPress,on_release=onRelease) as listener:
        listener.join()

try: 
    keyboardTeleop()
except KeyboardInterrupt:
    pass
            