#!/usr/bin/env python

# SearchSpiralPattern.py automates the drone's movement and
# whilst doing so checks if the current frame is showing the QR code
#
# This uses a spiral-like pattern, covering the whole arena
# When this finds a boundary, it hugs the boundary but continues the rest as normal

import rospy
from image_analysis import PatternLocation
import movementApi
from co600_proj.srv import GetLatestImage, HeightOffset, RotationOffset
from RosUtils import RosImageToCv
import time
 
class SearchSpiralPattern:

    def __init__(self, movementApi = movementApi.movementApi(), lastKnownLocation = [20,20]):
        self.movementApi = movementApi
        self.latestImageSrv = rospy.ServiceProxy('latest_image_srv', GetLatestImage)
        self.rotationOffsetSrv = rospy.ServiceProxy('rotation_offset_srv', RotationOffset)
        self.heightOffsetSrv = rospy.ServiceProxy('height_offset_srv', HeightOffset)        
        self.run = False
        self.xAxis=0
        self.yAxis=0
        self.lastKnownLocation = lastKnownLocation
        self.counter = 0
        self.length = 1
        self.movement = ['r','f','l','b']
        self.movementDirection = 0
        self.corners = [['l','t'],['r','t'],['l','b'],['r','b']]
        self.cornersSeen = []
        self.currentX = 0
        self.currentY = 0
        self.search()

    # Main function that calls methods to start search pattern
    def search(self):
        # if drone started elsewhere, must return to origin to estimate coordinates
        if not self.lastKnownLocation:
            self.returnToStartingPoint()
            self.spiralMovement()
            self.spiralInward()
        else:
            self.scanXAxis('r')
            self.scanYAxis('f')
            self.moveToLocation()
            self.currentX = self.lastKnownLocation[0]
            self.currentY = self.lastKnownLocation[1]
            self.spiralOutward()
            self.spiralInward()
    
    # Return to the starting point        
    def returnToStartingPoint(self):
        while 'l' not in self.checkBoundaryboundary():
            self.moveDrone('l')
        while ['l','b'] not in self.checkBoundary:
            self.moveDrone('b')

    # Scans the x axis whilst counting
    def scanXAxis(self, dir):
        self.xAxis=0
        while self.checkBoundary() == None or dir not in self.checkBoundary():
            self.moveDrone(dir)
            self.xAxis+=1

    # Scans the y axis and and counts the y axis
    def scanYAxis(self, dir):
        self.yAxis=0
        while self.checkBoundary() == None or 't' not in self.checkBoundary():
            self.moveDrone(dir)
            self.yAxis+=1

    # Moves to the last known location of the Roomba
    def moveToLocation(self):
        for i in range(0, self.xAxis - self.lastKnownLocation[0]):
            self.moveDrone('l')
        for i in range(0, self.yAxis - self.lastKnownLocation[1]):
            self.moveDrone('b')

    # While loop that keeps the drone moving depending on whether it has encountered a boundary, hasn't encountered one, or has search the whole arena
    # Function that calculates how many steps the drone should move and which direction (to make a spiral-like pattern)
    def spiralOutward(self):
        self.run = True
        self.cornersSeen=[]
        spiralIn = False
        while(self.run):
            for i in range(0,self.length):
                if(self.run == False):
                    break
                print(self.movement[self.movementDirection])
                self.moveDrone(self.movement[self.movementDirection])
                self.calculateCurrentPoint(self.movement[self.movementDirection])
                boundary = self.checkBoundary()
                if self.movement[self.movementDirection] in boundary:
                    self.movementDirection+=1
                    # self.spiralMovement()
                    # if it has searched the whole arena and has seen all corners corners
                    if len(boundary)>2:
                        self.cornersSeen.append(boundary)
                        if all(elem in self.cornersSeen for elem in self.corners):
                            run = False
                            spiralIn = True
                    break     
            self.movementDirection+=1
            self.counter+=1
            if self.movementDirection >= 4:
                self.movementDirection=0
            if self.counter==2:
                self.counter=0
                self.length+=10
        if spiralIn:
            self.spiralInward()

    def spiralInward(self):
        self.run = True
        self.cornersSeen=[]
        spiralOut = False
        while(self.run):
            for i in range(0,self.length):
                if(self.run == False):
                    break   
                print(self.movement[self.movementDirection])
                self.moveDrone(self.movement[self.movementDirection])
                self.calculateCurrentPoint(self.movement[self.movementDirection])
                boundary = self.checkBoundary()
                if self.movement[self.movementDirection] in boundary:
                    self.movementDirection-=1
                    self.spiralOutward()
                    # if it has searched the whole arena and has seen all corners corners
                    if len(boundary)>2:
                        self.cornersSeen.append(boundary)
                        if all(elem in self.cornersSeen for elem in self.corners):
                            run = False
                            spiralOut = True
                    break     
            self.movementDirection-=1
            self.counter+=1
            if self.movementDirection<=-1:
                self.movementDirection=3
            if self.counter==2:
                self.counter=0
                self.length-=10
        if spiralOut:
            self.spiralMovement()
    
    def calculateCurrentPoint(self, dir):
        if dir is 'r':
            self.currentX+=1
        elif dir is 'f':
            self.currentY+=1
        elif dir is 'l':
            self.currentX-=1
        elif dir is 'b':
            self.currentY-=1

    # Moves the drone depending on the given direction, and when does so, checks the frame captured by the drone camera
    def moveDrone(self, direction):
        if direction == 'r':
            found = self.checkImages(yMove=-0.5)
        elif direction == 'l':
            found = self.checkImages(yMove=0.5)
        elif direction == 'f':
            found = self.checkImages(xMove=0.5)
        elif direction == 'b':
            found = self.checkImages(xMove=-0.5)
        self.movementApi.stop()
        found = self.checkFrame()
        self.foundRoomba(found)

    def checkImages(self, xMove = 0, yMove = 0):
        endTime = time.time() + 2
        found = False
        while( time.time() < endTime and found == False):
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
            found = self.checkFrame()   #needed?
        return found

    # if Roomba == found -> stop drone, and then re check after a few seconds
    #              else -> found == false, and then go back to process
    def foundRoomba(self, found):
        if found:
            self.movementApi.stop()
            recapture = self.checkFrame()
            if not recapture:
                self.run = True

    # Gets the most recent frame and sends to Image Analysis module to check if Roomba can be found
    def checkFrame(self):
        try:
            frameCheck = PatternLocation.getPattern(RosImageToCv(self.latestImageSrv().image), True)
            roombaPosition = frameCheck['roombaPosition']
            if roombaPosition is not None:
                print("Found roomba")
                self.run = False
                return True
        except Exception as e:
            print(e)
            return False

    # Returns whether or not there is a boundary in the frame, and acts on it
    def checkBoundary(self):
        try:
            boundary = PatternLocation.getBoundary(RosImageToCv(self.latestImageSrv().image))
            return boundary
        except Exception as e:
            print(e)

if __name__ == "__main__":
    SearchSpiralPattern()