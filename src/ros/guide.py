#!/usr/bin/env python

import traceback
import rospy
from image_analysis import PatternLocation
import movementApi
import roombaMovementApi
from RosUtils import RosImageToCv
from co600_proj.srv import HeightOffset, GetLatestImage, RotationOffset, StopRoomba

## guide is responsible for returning the roomba to the pen and ensuring the drone stays above the roomba.
class guide:

    ## Initialises the guide object
    #
    # @param movementApi movementApi to publish messages to drone.
    def __init__(self, movementApi = movementApi.movementApi()):
        self.movementApi = movementApi
        self.roombaMovementApi = roombaMovementApi.roombaMovementApi()
        self.latestImageSrv = rospy.ServiceProxy('latest_image_srv', GetLatestImage)
        self.rotationOffsetSrv = rospy.ServiceProxy('rotation_offset_srv', RotationOffset)
        self.heightOffsetSrv = rospy.ServiceProxy('height_offset_srv', HeightOffset)
        self.stopRoombaSrv = rospy.ServiceProxy('stop_roomba_srv', StopRoomba)
        self.execute()

    ## Guides the roomba back to its pen.
    def execute(self):
        self.stopRoombaSrv()
        self.roombaMovementApi.stop()
        #step 1: rotate roomba to 270:
        print("Step 1")
        self.rotateRoombaMethod(270)
        #step 2: move roomba toward west side of arena
        print("Step 2")        
        self.moveRoombaUntilBoundary('l')
        print("Step 3")
        self.moveRoombaUpToBoundary('l')
        #step 3: rotate roomba to 180:
        print("Step 4")
        self.rotateRoombaMethod(180)
        #step 4: move roomba towards south side of arena
        print("Step 5")
        self.moveRoombaUntilBoundary('b')
        print("Step 6")        
        self.moveRoombaUpToBoundary('b')
        while (True):
            patterns = self.getPatterns(self.latestImageSrv().image)
            self.moveDrone(patterns['roomba']['location'])

    ## Rotates the roomba until it is at the desiredRotation
    #
    # @param desiredRotation The rotation the roomba will turn to.
    def rotateRoombaMethod(self, desiredRotation):
        patterns = self.getPatterns(self.latestImageSrv().image)
        while not(desiredRotation - 5 <= patterns['roomba']['orientation'] <= desiredRotation + 5):
            self.moveDrone(patterns['roomba']['location'])
            self.rotateRoomba(patterns['roomba']['orientation'], desiredRotation)                
            patterns = self.getPatterns(self.latestImageSrv().image)
        self.roombaMovementApi.stop()

    ## Moves the roomba until the drone's camera can see the specified boundary.
    #
    # @param boundary The boundary to move towards.
    def moveRoombaUntilBoundary(self, boundary):
        print("moving to boundary")
        self.roombaMovementApi.forward(0.25)
        frame = RosImageToCv(self.latestImageSrv().image)
        patterns = self.getPatterns(self.latestImageSrv().image)
        print(PatternLocation.getBoundary(frame))
        while (boundary not in PatternLocation.getBoundary(frame)):
            self.roombaMovementApi.forward(0.25)
            print(PatternLocation.getBoundary(frame))
            self.moveDrone(patterns['roomba']['location'])
            frame = RosImageToCv(self.latestImageSrv().image)
            patterns = self.getPatterns(self.latestImageSrv().image)
        self.roombaMovementApi.stop()
        print("I can see boundary")

    ## Moves the drone until it is 300mm from a boundary
    #
    # @param boundary The boundary to move towards
    def moveRoombaUpToBoundary(self, boundary):
        print("moving up to boundary")
        self.roombaMovementApi.forward(0.25)
        frame  = RosImageToCv(self.latestImageSrv().image)
        while(self.getDistance(frame)[boundary] > 300):
            self.roombaMovementApi.forward(0.25)
            self.moveDrone(self.getPatterns(self.latestImageSrv().image)['roomba']['location'])
            frame = RosImageToCv(self.latestImageSrv().image)
        self.roombaMovementApi.stop()

    ## Gets the distance between the boundary and roomba
    #
    # @param frame An OpenCV compatible frame
    def getDistance(self, frame):
        # try:
        result = PatternLocation.getDistanceFromBoundary(frame)
        print(result)
        if result['l'] == None:
            result['l'] = 501
        if result['b'] == None:
            result['b'] = 501
        return result
        # except Exception as e:
        #     print(e)
        #     traceback.print_exc()
        #     return {'l':200}

    ## Gets the patterns found in a frame.
    #
    # @param frame A ROS compatible frame.
    def getPatterns(self, frame):
        try:
            result = PatternLocation.getPatternAndOrientation(RosImageToCv(frame))
            if result['roomba'] == None:
                return self.getPatterns(self.latestImageSrv().image)
            elif result['roomba']['orientation'] == None:
                return self.getPatterns(self.latestImageSrv().image)
            else:
                return result
        except Exception as e:
            print(e)
            return {'roomba':{'location':'c', 'orientation':0}}

    ## Rotates the roomba to the desired rotation
    #
    # @param currentRotation The current rotation of the roomba.
    # @param desiredRotation The desired rotation of the roomba.
    def rotateRoomba(self, currentRotation, desiredRotation):
        try:
            if currentRotation > desiredRotation+5:
                self.roombaMovementApi.left(0.5)
            elif currentRotation < desiredRotation-5:
                self.roombaMovementApi.right(0.5)
            else:
                self.roombaMovementApi.stop()
        except Exception as e:
            print(e)

    ## Moves the drone over the roomba along with correcting rotational and vertical drift.
    # 
    # @param roombaPosition The position of the roomba returned from image code. 
    def moveDrone(self, roombaPosition):
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
            #print("publishing "+ str(x) +", "+ str(y) +", "+str(z))
            if not(x == 0 and y == 0 and z == 0):
                self.movementApi.custom(x, y, z, 0, 0, zRot)
        except Exception as e:
            print(e)