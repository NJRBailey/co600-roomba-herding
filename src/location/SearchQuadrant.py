#!/usr/bin/env python

# Quadrant Search splits the arena into four, calculates which quadrant to go to depending on last known location
# then does a lawnmower path within the specified quadrant
#
# Once it searches one quadrant, it goes to the next random one and reiterates the process


import rospy
from image_analysis import PatternLocation
import movementApi
from co600_proj.srv import GetLatestImage, HeightOffset, RotationOffset
from RosUtils import RosImageToCv
import time

from random import seed
from random import shuffle
import itertools
 
class SearchQuadrant:

    def __init__(self, movementApi = movementApi.movementApi(), lastKnownLocation=[20,20]):
        self.movementApi = movementApi
        self.latestImageSrv = rospy.ServiceProxy('latest_image_srv', GetLatestImage)
        self.rotationOffsetSrv = rospy.ServiceProxy('rotation_offset_srv', RotationOffset)
        self.heightOffsetSrv = rospy.ServiceProxy('height_offset_srv', HeightOffset)        
        self.lastKnownLocation = lastKnownLocation
        self.run = False
        self.xAxis=0
        self.yAxis=0
        self.xMidpoint = 0
        self.yMidpoint = 0
        self.startingQuadrant = 0
        self.currentX = self.xMidpoint
        self.currentY = self.yMidpoint
        self.moves = len(self.path)
        self.counter = 0
        self.currentDirection = ''
        self.search()

    # Main function that calls methods to start search pattern
    def search(self):
        self.scanXAxis()
        self.scanYAxis()
        self.moveToLocation()
        self.calcuateQaudrant()
        sequence = self.randomiseSequence()
        for i in itertools.cycle(sequence)
            self.run=True
            if(self.run):
                self.startSearch(i)
            else:
                break

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

    # Moves to the midpoint of the map
    def moveToLocation(self):
        self.xMidpoint = round(xAxis/2)
        self.yMidpoint = round (yAxis/2)
        for i in range(0,self.xMidpoint):
            self.moveDrone('l')
        for i in range(0, 0,self.yMidpoint):
            self.moveDrone('b')

    # Calculates the starting quadrant 
    def calculateQuadrant(self):
        if self.lastKnownLocation[0] < self.xMidpoint and self.lastKnownLocation[1] < self.yMidpoint:
            self.startingQuadrant = 1
        elif self.lastKnownLocation[0] > self.xMidpoint and self.lastKnownLocation[1] < self.yMidpoint:
            self.startingQuadrant = 2
        elif self.lastKnownLocation[0] > self.xMidpoint and self.lastKnownLocation[1] > self.yMidpoint:
            self.startingQuadrant = 3
        elif self.lastKnownLocation[0] < self.xMidpoint and self.lastKnownLocation[1] > self.yMidpoint:
            self.startingQuadrant = 4

    # Will randomise a sequence of quadrants for pattern to through
    def randomiseSequence(self):
        seed(1)
        quadrantPath = []
        quadrantPath.append(self.startingQuadrant)
        if self.startingQuadrant = 1:
            quadrantPath = shuffle([2,3,4])
        elif self.startingQuadrant = 2:
            quadrantPath = shuffle([1,3,4])
        elif self.startingQuadrant = 3:
            quadrantPath = shuffle([1,2,4])
        elif self.startingQuadrant = 4:
            quadrantPath = shuffle([1,2,3])
        return quadrantPath

    # Does the lawnmower pattern for the quadrant
    def startSearch(self, quadrant):
        if quadrant==1 or quadrant==4:
            self.currentDirection=self.directionCounter()
            self.moveDrone(self.currentDirection)
            while(self.run):
                boundary = self.checkBoundary()
                if (boundary==['l'] or self.currentX==self.xMidpoint):
                    if quadrant==1:
                        self.moveDrone('b')
                    elif quadrant==4:
                        self.moveDrone('f')
                    self.counter+=1
                    self.directionCounter()
                    self.moveDrone(self.currentDirection)
                elif len(boundary)>1:
                    while('b' in boundary or 't' in boundary):
                        if self.currentX==self.xMidpoint:
                            run = False
                            break
                        boundary = self.checkBoundary()
                        self.moveDrone('r')
                else:
                    self.moveDrone(self.currentDirection)
            self.reset()
        elif quadrant==2 or quadrant==3:
            self.counter==1
            self.currentDirection = self.directionCounter()
            self.moveDrone(self.currentDirection)
            while(self.run):
                boundary = self.checkBoundary()
                if (boundary==['r'] or self.currentX==self.xMidpoint):
                    if quadrant==2:
                        self.moveDrone('b')
                    elif squadrantt==3:
                        self.moveDrone('f')
                    self.counter+=1
                    self.directionCounter()
                    self.moveDrone(self.currentDirection)
                elif len(boundary)>1:
                    while('b' in boundary or 't' in boundary):
                        if self.currentX==self.xMidpoint:
                            run = False
                            break
                        boundary = self.checkBoundary()
                        self.moveDrone('l')
                else:
                    self.moveDrone(self.currentDirection)
            self.reset(quadrant)
    
    # Determines which direction (left or right) to go 
    def directionCounter(self):     
        directionPattern = ['l','r']   
        if self.counter==2:
            self.counter=0
        self.currentDirection = directionPattern[counter]

    # Moves drone back to the center 
    def reset(self, quadrant):
        while self.currentY != self.yMidpoint:
            if quadrant==1 or quadrant==2:
                self.moveDrone('f')
            elif quadrant==3 or quadrant==4:
                self.moveDrone('b')

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
            self.checkImages(yMove=-0.5)
        elif direction == 'l':
            self.checkImages(yMove=0.5)
        elif direction == 'f':
            self.checkImages(xMove=0.5)
        elif direction == 'b':
            self.checkImages(xMove=-0.5)
        self.calculateCurrentPoint(direction)
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
                self.run==true

    # Gets the most recent frame and sends to Image Analysis module to check if Roomba can be found
    def checkFrame(self):
        try:
            frameCheck = PatternLocation.getPattern(RosImageToCv(self.latestImageSrv().image), True)
            roombaPosition = frameCheck['roombaPosition']
            if roombaPosition is not None:
                print("Found roomba")
                self.run = False
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

if _name_ == "_main_":
    SearchQuadrant()