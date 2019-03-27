#!/usr/bin/env python

import serial

## This code  connects to the same port as the Xbee and sends commands to the Roomba
class ConnectRoomba:

    ## Initialises the ConnectRoomba object
    #
    # @param port passing in a port number to know which serial port number to communicate with
    def __init__(self, port='COM6'):
        self.roombarduino = serial.Serial(
            port = port,
            baudrate = 9600
        )
        self.roombarduino.isOpen()

    ## Rotates the Roomba to the left
    #
    # @param degrees takes in the amount of left rotation in degrees
    def left(self, degrees):
        self.roombarduino.write('L' + str(degrees) +'/')

    ## Rotates the Roomba to the right
    #
    # @param degrees takes in the amount of right rotation in degrees
    def right(self, degrees):
        self.roombarduino.write('R' + str(degrees) +'/')

    ## Moves the Roomba forwards
    #
    # @param dist requires the how much it should move forwards
    def forward(self, dist):
        self.roombarduino.write('F' + str(dist) +'/')

    ## Moves the Roomba backwards
    #
    # @param dist requires the how much it should move forwards
    def backward(self, dist):
        self.roombarduino.write('B' + str(dist) +'/')

    ## Stops the Roomba
    def stop(self):
        self.roombarduino.write('S')

    ## Allows the Roomba to roam
    def roam(self):
        self.roombarduino.write('W')

ConnectRoomba()