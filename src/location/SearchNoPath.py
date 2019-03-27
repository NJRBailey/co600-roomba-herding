#!/usr/bin/env python

import rospy
from image_analysis import PatternLocation
import movementApi
from co600_proj.srv import GetLatestImage, HeightOffset, RotationOffset
from RosUtils import RosImageToCv
import time

## SearchNoPath.py automates the drone's movement and whilst doing so checks if the current frame is showing the QR code
## This uses a lawnmowing search pattern with not path coordinates, covering the whole arena

class SearchNoPath:

    ## Initialises the SearchNoPath object
    #
    # @param movementApi movementApi to publish messages to drone
    def __init__(self, movementApi = movementApi.movementApi()):
        self.movementApi = movementApi
        self.latestImageSrv = rospy.ServiceProxy('latest_image_srv', GetLatestImage)
        self.rotationOffsetSrv = rospy.ServiceProxy('rotation_offset_srv', RotationOffset)
        self.heightOffsetSrv = rospy.ServiceProxy('height_offset_srv', HeightOffset)
        self.run = True
        self.movementX = 'r'
        self.movementY = 't'
        self.search()

    ## Moves drone, and only calls to change direction if it sees a boundary
    def search(self):
        while(self.run):
            boundary = self.checkBoundary()
            if self.movementX in boundary:
                self.moveDrone(self.movementY)
            else:
                self.moveDrone(self.movementX)
            self.updateMovementDirections(boundary)

    ## Changes direction in x or y if it meets a boundary
    def updateMovementDirections(self, boundaries):
        if self.movementX in boundaries:
            if self.movementX == 'l':
                self.movementX = 'r'
            elif self.movementX == 'r':
                self.movementX = 'l'
        if self.movementY in boundaries:
            if self.movementY == 't':
                self.movementY = 'b'
            elif self.movementY == 'b':
                self.movementY = 't'

    ## Moves the drone depending on the given direction, and when does so, checks the frame captured by the drone camera
    #
    # @param direction The direction the drone should move towards
    def moveDrone(self, direction):
        if direction == 'r':
            found = self.checkImages(yMove=-0.5)
        elif direction == 'l':
            found = self.checkImages(yMove=0.5)
        elif direction == 't':
            found = self.checkImages(xMove=0.5)
        elif direction == 'b':
            found = self.checkImages(xMove=-0.5)
        self.movementApi.stop()
        found = self.checkFrame()
        self.foundRoomba(found)

    ## Whilst the drone moves, corrects its drift and rotation
    #
    # @param xMove The movement direction in the x coordinate
    # @param yMove The movement direction in the y coordinate
    def checkImages(self, xMove = 0, yMove = 0):
        startTime = time.clock()
        found = False
        while(time.clock() - startTime < 1 and found == False):
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
            self.movementApi.custom(xMove, yMove, z, 0, 0, zRot)
            found = self.checkFrame()
        return found
    
    ## If the Roomba has been returned as found, will stop the drone and check the latest image again. If not found, changes run to True
    #
    # @param found The Boolean result of the latest image
    def foundRoomba(self, found):
        if found:
            self.movementApi.stop()
            recapture = self.checkFrame()
            if not recapture:
                self.run = True

    ## Gets the most recent frame and sends to Image Analysis module to check if Roomba can be found
    def checkFrame(self):
        try:
            frameCheck = PatternLocation.getPattern(RosImageToCv(self.latestImageSrv().image), True)
            roombaPosition = frameCheck['roombaPosition']
            if roombaPosition is not None: 
                print("Found roomba")
                self.run = False
                return True
            return False
        except Exception as e:
            print(e)
            return False

    ## Returns whether or not there is a boundary in the frame, and acts on it
    def checkBoundary(self):
        try:
            boundary = PatternLocation.getBoundary(RosImageToCv(self.latestImageSrv().image))
            return boundary
        except Exception as e:
            print(e)

if __name__ == "__main__":
    SearchNoPath()