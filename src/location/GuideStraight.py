#!/usr/bin/env python

import rospy
from image_analysis import PatternLocation
import movementApi
import simRoombaMove
from RosUtils import RosImageToCv
from co600_proj.srv import HeightOffset, GetLatestImage, RotationOffset

class guide:

    def __init__(self, movementApi = movementApi.movementApi(), currentX, currentY):
        self.movementApi = movementApi
        self.roombaMovementApi = simRoombaMove.simRoombaMove()
        self.latestImageSrv = rospy.ServiceProxy('latest_image_srv', GetLatestImage)
        self.rotationOffsetSrv = rospy.ServiceProxy('rotation_offset_srv', RotationOffset)
        self.heightOffsetSrv = rospy.ServiceProxy('height_offset_srv', HeightOffset)
        self.execute(currentX, currentY)

    def execute(self):
        self.roombaMovementApi.stop()
        #step 1: rotate roomba to 270:
        print("Step 1")
        frame = RosImageToCv(self.latestImageSrv().image)
        while not(260 <= PatternLocation.getOrientation(frame) <= 280):
            self.moveDrone(frame)
            self.rotateRoomba(frame, 270)
            frame = RosImageToCv(self.latestImageSrv().image)
        self.roombaMovementApi.stop()
        #step 2: calculate the remaining angle needed to face the pen
        angle = 270 - math.degrees(math.atan(currentX/currentY))
        while not(angle-5<= PatternLocation.getOrientation(frame) <= angle+5):
            self.moveDrone(frame)
            self.rotateRoomba(frame, angle)
            frame = RosImageToCv(self.latestImageSrv().image)
        self.roombaMovementApi.stop()
        #step 3: move forward until pen is reached
        self.roombaMovementApi.forward(0.5)
        frame = RosImageToCv(self.latestImageSrv().image)
        while ('b' not in PatternLocation.getBoundary(frame)):
            self.moveDrone(frame)
            frame = RosImageToCv(self.latestImageSrv().image)
        self.roombaMovementApi.stop()
        while (True):
            self.moveDrone(RosImageToCv(self.latestImageSrv().image))
    
    def moveDrone(self, frame):
        heightOffset = self.heightOffsetSrv()
        rotationOffset = self.rotationOffsetSrv()
        x = 0
        y = 0
        if rotationOffset.rotation > 0:
            zRot = 0.5
        elif rotationOffset.rotation == 0:
            zRot = 0
        else:
            zRot = -0.5
        if heightOffset.height > 0:
            z = 0.5
        elif heightOffset.height == 0:
            z = 0
        else:
            z = -0.5
        try:
            patterns = PatternLocation.getPattern(frame, debug=True)
            roombaPosition = patterns['roombaPosition']
            if roombaPosition == 'c':
                self.movementApi.stop()
            elif roombaPosition == 'tl':
                x = 0.5
                y = 0.5
            elif roombaPosition == 'tr':
                x = 0.5
                y = -0.5
            elif roombaPosition == 't':
                x = 0.5
            elif roombaPosition == 'l':
                y = 0.5
            elif roombaPosition == 'r':
                y = -0.5
            elif roombaPosition == 'bl':
                x = -0.5
                y = 0.5
            elif roombaPosition == 'br':
                x = -0.5
                y = -0.5
            elif roombaPosition == 'b':
                x = -0.5
            if not(x == 0 and y == 0 and z == 0):
                self.movementApi.custom(x, y, z, 0, 0, zRot)
        except Exception as e:
            print(e)

    def rotateRoomba(self, frame, desiredRotation):
        currentRotation = PatternLocation.getOrientation(frame)
        if currentRotation > desiredRotation+5:
            self.roombaMovementApi.left(0.5)
        elif currentRotation < desiredRotation-5:
            self.roombaMovementApi.right(0.5)
        else:
            self.roombaMovementApi.stop()