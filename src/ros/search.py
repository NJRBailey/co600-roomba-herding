#!/usr/bin/env python

# Search.py automates the drone's movement and
# whilst doing so checks if the current frame is showing
# the roomba marker.
#
# This uses a lawnmowing path, covering the whole arena
#

import rospy
from image_analysis import PatternLocation
import movementApi
from co600_proj.srv import GetLatestImage, RotationOffset, HeightOffset
from RosUtils import RosImageToCv
import time


class SearchRoomba:

    def __init__(self, movementApi=movementApi.movementApi()):
        self.movementApi = movementApi
        rospy.wait_for_service('latest_image_srv')
        self.LatestImageSrv = rospy.ServiceProxy('latest_image_srv', GetLatestImage)
        self.rotationOffsetSrv = rospy.ServiceProxy('rotation_offset_srv', RotationOffset)
        self.heightOffsetSrv = rospy.ServiceProxy('height_offset_srv', HeightOffset)
        self.search()

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

    def search(self):
        # backward  left    right   stop
        #check if near a boundary by getting the coordinates
        #and also getting positon of the boundary
        #assuming there are no boundaries and you know the size of arena (not needed if boundaryCheck works)
        self.furthestPoint = [3,3]
        self.createPath(self.furthestPoint)

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
