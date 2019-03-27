#!/usr/bin/env python

import rospy
from image_analysis import PatternLocation
import movementApi
from co600_proj.srv import GetLatestImage
from RosUtils import RosImageToCv
import time

## findRoomba.py automates the drone's movement and whilst doing so checks if the current frame is showing the QR code
## This uses a lawnmowing path, covering the whole arena

class findRoomba:

    ## Initialises the findRoomba object
    def __init__(self):
        rospy.init_node('search_node', anonymous=False)
        self.movementApi = movementApi.movementApi()
        rospy.wait_for_service('latest_image_srv')
        self.latestImageSrv = rospy.ServiceProxy('latest_image_srv', GetLatestImage)
        self.search()

    ## Gets the most recent frame and sends to Image Analysis module to check if Roomba can be found
    def checkFrame(self):
        try:
            frameCheck = PatternLocation.getPattern(RosImageToCv(self.latestImageSrv().image), True)
            roombaPosition = frameCheck['roombaPosition']
            if roombaPosition is not None:
                while(True):
                    print("Found roomba")
                    # Will return, and exit the process
        except Exception as e:
            print("Image code broken!!!!!")
            print(e)

    ## Returns whether or not there is a boundary in the frame, and acts on it
    def checkBoundary(self):
        None

    ## Calls createPath with the furthestPoint. furthestPoint sets the size of the arena by giving the furthest coordinate
    def search(self):
        self.furthestPoint = [3,3]
        path = self.createPath(self.furthestPoint)
        self.moveDrone(path)

    ## Creates a path for the drone in form of a list of coordinate
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
        return path

    ## Using the list, drone moves according to the change in the set of values, and also calls the checkFrame method to check if the Roomba can be seen. It calls the function again with a reversed list if Rooma is still not found.
    # 
    # @param path A list of coordinates the drone should move to
    # @param lastX The last x coordinates
    # @param lastY The last y coordinates
    def moveDrone(self, path, lastX=0, lastY=0):
        for pos in path:
            if pos[0] > lastX:
                self.movementApi.right(0.5)
                self.checkImages()
                self.movementApi.stop()
                self.checkFrame()
                lastX = pos[0]
            elif pos[0] < lastX:
                self.movementApi.left(0.5)
                self.checkImages()
                self.movementApi.stop()
                self.checkFrame()
                lastX = pos[0]
            elif pos[1] > lastY:
                self.movementApi.forward(0.5)
                self.checkImages()
                self.movementApi.stop()
                self.checkFrame()
                lastY = pos[1]
            elif pos[1] < lastY:
                self.movementApi.backward(0.5)
                self.checkImages()
                self.movementApi.stop()
                self.checkFrame()
                lastY = pos[1]
        self.moveDrone(list(reversed(path)),lastX,lastY)
    
    ## Increases frame rate by refreshing the frame a few times within 2 seconds
    def checkImages(self):
        startTime = time.clock()
        while(time.clock() - startTime < 2):
            self.checkFrame()

if __name__ == "__main__":
    findRoomba()
