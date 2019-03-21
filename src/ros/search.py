#!/usr/bin/env python

import rospy
from image_analysis import PatternLocation
import movementApi
from co600_proj.srv import GetLatestImage, RotationOffset, HeightOffset
from RosUtils import RosImageToCv
import time

## SearchRoomba is responsible for finding the roomba.
#
# This version of search roomba creates a lawnmowing path and follows that until it finds the roomba.
class SearchRoomba:

    ## Initialises the SearchRoomba object
    #
    # @param movementApi movementApi to publish movement commands to drone.
    def __init__(self, movementApi=movementApi.movementApi()):
        self.movementApi = movementApi
        rospy.wait_for_service('latest_image_srv')
        self.LatestImageSrv = rospy.ServiceProxy('latest_image_srv', GetLatestImage)
        self.rotationOffsetSrv = rospy.ServiceProxy('rotation_offset_srv', RotationOffset)
        self.heightOffsetSrv = rospy.ServiceProxy('height_offset_srv', HeightOffset)
        self.search()

    ## Checks the most recent image to locate roomba.
    def checkFrame(self):
        try:
            frameCheck = PatternLocation.getPattern(RosImageToCv(self.LatestImageSrv().image), True)
            roombaPosition = frameCheck['roombaPosition']
            if not(roombaPosition == None):
                print("Found roomba")
                return True
            else:
                return False
        except Exception as e:
            print("Image code broken!!!!!")
            print(e)
            return False

    ## Creates a path and then follows it.
    def search(self):
        # backward  left    right   stop
        #check if near a boundary by getting the coordinates
        #and also getting positon of the boundary
        #assuming there are no boundaries and you know the size of arena (not needed if boundaryCheck works)
        self.furthestPoint = [3,3]
        self.createPath(self.furthestPoint)

    ## Generates a lawnmowing bath based on the furthest coordinate.
    #
    # @param endCoord the ending coordinate.
    def createPath(self, endCoord):
        xCoord = 0
        path = []
        for x in range(endCoord[1]+1):
            if xCoord == 0:
                for z in range(endCoord[0]+1):
                    xCoord = z
                    path.append([z,x])
            else:
                for z in range(endCoord[0], -1, -1):
                    xCoord = z
                    path.append([z,x])
        print(path)
        self.moveDrone(path)
    
    ## Moves the drone along the lawnmowing path.
    #
    # @param path The path the drone should follow.
    # @param lastX the last x coordinate the drone travelled along.
    # @param lastY the last y coordinate the drone travelled along.
    def moveDrone(self, path, lastX=0, lastY=0):
        found = False
        for pos in path:
            if pos[0] > lastX:
                found = self.checkImages(yMove=-0.5)
                self.movementApi.stop()
                lastX = pos[0]
            elif pos[0] < lastX:
                found = self.checkImages(yMove=0.5)
                self.movementApi.stop()
                lastX = pos[0]
            elif pos[1] > lastY:
                found = self.checkImages(xMove=0.5)
                self.movementApi.stop()
                lastY = pos[1]
            elif pos[1] < lastY:
                found = self.checkImages(xMove=-0.5)
                self.movementApi.stop()
                lastY = pos[1]
            if found == True:
                return found
        self.moveDrone(list(reversed(path)),lastX,lastY)  # reverses the given path then starts again

    ## Moves the drone for an amount of time whilst checking the images the drone returns.
    # 
    # @param xMove the speed at which the drone should move along the x axis.
    # @param yMove the speed at which the drone should move along the y axis.
    def checkImages(self, xMove = 0, yMove = 0):
        startTime = time.clock()
        found = False
        while(time.clock() - startTime < 2 and found == False):
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

if __name__ == "__main__":
    SearchRoomba()
