#!/usr/bin/env python

# Code that connects to the same port as the Xbee and sends commands

import serial

class ConnectRoomba:

    def __init__(self, port='COM6'):
        self.roombarduino = serial.Serial(
            port = port,
            baudrate = 9600
        )
        self.roombarduino.isOpen()

    def left(self, degrees):
        self.roombarduino.write('L' + str(degrees) +'/')

    def right(self, degrees):
        self.roombarduino.write('R' + str(degrees) +'/')

    def forward(self, dist):
        self.roombarduino.write('F' + str(dist) +'/')

    def backward(self, dist):
        self.roombarduino.write('B' + str(dist) +'/')

    def stop(self):
        self.roombarduino.write('S')

    def roam(self):
        self.roombarduino.write('W')

ConnectRoomba()