import cv2
from numpy import zeros

# NoiseReduction
# This file contains functions to reduce the level of irrelevant information in an image.
# There is one for patterns (Pen and Roomba) and one for the arena boundary (hazard tape).


# Should hold its own dominant colour, its neighbours, and should set neighbours to None if on an edge
class Region:
    def __init__(self, ident, edges, pixels, neighbours, lightness, regionsW, regionsH):
        self.id = ident
        self.edges = edges
        self.pixels = pixels
        self.neighbours = []
        for neighbour in neighbours:
            if -1 in neighbour or neighbour[0] == regionsW or neighbour[1] == regionsH:
                neighbour = None
            self.neighbours.append(neighbour)
        self.dominantLightness = lightness


# Finds the dominant lightness category for the supplied group
def findDominantLightness(pxLightGroups):
    dominantLightness = None
    if pxLightGroups['low'] >= 5:
        dominantLightness = 'low'
    elif pxLightGroups['low'] == 4 and pxLightGroups['mid'] < 5 and pxLightGroups['high'] < 5:
        dominantLightness = 'low'
    if pxLightGroups['mid'] >= 5:
        dominantLightness = 'mid'
    elif pxLightGroups['mid'] == 4 and pxLightGroups['low'] < 5 and pxLightGroups['high'] < 5:
        dominantLightness = 'mid'
    if pxLightGroups['high'] >= 5:
        dominantLightness = 'high'
    elif pxLightGroups['high'] == 4 and pxLightGroups['mid'] < 5 and pxLightGroups['low'] < 5:
        dominantLightness = 'high'
    return dominantLightness


# Processes and returns a frame to try and reduce the amount of information which is unrelated to patterns.
def reduceNoiseForPatterns(frame, regionsW=16, regionsH=9):  # TODO make samples a parameter; return small regions
    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Split frame up into regions
    height, width = grayFrame.shape
    normalHeight = height / regionsH
    extraHeight = height % regionsH
    normalWidth = width / regionsW
    extraWidth = width % regionsW
    regions = []
    for column in range(regionsW):
        for row in range(regionsH):
            # Make a region
            ident = (row, column)
            print(ident)
            neighbours = ((row - 1, column), (row, column + 1), (row + 1, column), (row, column - 1))  # N, E, S, W
            regionHeight = normalHeight
            regionWidth = normalWidth
            if row == regionsH - 1:
                regionHeight += extraHeight
            if column == regionsW - 1:
                regionWidth += extraWidth
            leftX = column * normalWidth
            # leftX = 0
            rightX = leftX + regionWidth
            # rightX = regionWidth
            topY = row * normalHeight
            # topY = 0
            bottomY = topY + regionHeight
            # bottomY = regionHeight
            edges = (leftX, rightX, topY, bottomY)  # x1, x2, y1, y2
            pixels = grayFrame[topY:bottomY, leftX:rightX]
            print(pixels)
            print(len(pixels))
            # print(len(pixels))
            # print(len(pixels[0]))
            # Find dominant colour
            pixelLightnessGroups = {'low': 0, 'mid': 0, 'high': 0}
            pxSpaceX = (rightX - leftX) / 4
            pxSpaceY = (bottomY - topY) / 4
            # print(pxSpaceX)
            # print(pxSpaceY)
            # Get 9 pixels in a 3x3 shape around centre
            sampledPixels = [
                pixels[0 + pxSpaceX, 0 + pxSpaceY],
                pixels[0 + (pxSpaceX * 2), 0 + pxSpaceY],
                pixels[regionWidth - pxSpaceX, 0 + pxSpaceY],
                pixels[0 + pxSpaceX, 0 + (pxSpaceY * 2)],
                pixels[0 + (pxSpaceX * 2), 0 + (pxSpaceY * 2)],
                pixels[regionWidth - pxSpaceX, 0 + (pxSpaceY * 2)],
                pixels[0 + pxSpaceX, regionHeight - pxSpaceY],
                pixels[0 + (pxSpaceX * 2), regionHeight - pxSpaceY],
                pixels[regionWidth - pxSpaceX, regionHeight - pxSpaceY]
            ]
            for px in sampledPixels:
                # print(px)
                if px < 85:  # 1/3 of 255
                    pixelLightnessGroups['low'] += 1
                elif px < 170:  # 2/3 of 255
                    pixelLightnessGroups['mid'] += 1
                else:
                    pixelLightnessGroups['high'] += 1
            dominantLightness = findDominantLightness(pixelLightnessGroups)
            # print(dominantLightness)
            regions.append(Region(ident, edges, pixels, neighbours, dominantLightness, regionsW, regionsH))
    darkRegions = []
    for region in regions:
        if region.dominantLightness == 'low':
            darkRegions.append(region)
    processedFrame = zeros((height, width))
    for darkRegion in darkRegions:
        e = darkRegion.edges
        processedFrame[e[2]:e[3], e[0]:e[1]] = darkRegion.pixels
        for neighbour in darkRegion.neighbours:
            if neighbour is not None:
                for region in regions:
                    if region.id == neighbour:
                        nE = region.edges
                        processedFrame[nE[2]:nE[3], nE[0]:nE[1]] = region.pixels
    return processedFrame


# Processes and returns a frame to try and reduce the amount of information which is unrelated to the arena boundary.
def reduceNoiseForBoundary(frame):
    return frame


i1 = cv2.imread('test-images/IndexCrash3.png')
i2 = cv2.imread('test-images/IndexCrash3Shadowed.png')
i3 = cv2.imread('test-images/IndexCrash3PenShadow.png')
i4 = cv2.imread('test-images/IndexCrash3TapeShadow.png')
i5 = cv2.imread('test-images/IndexCrash3PenPartShadow.png')
iA = [i1, i2, i3, i4, i5]

i = -1
while i < 4:
    print('--------------------------------------------')
    i += 1
    # try:
    prImage = reduceNoiseForPatterns(iA[i])
    cv2.imshow('prImage', prImage)
    cv2.waitKey(0)
    # except Exception as e:
    #     print('error with i' + str(i + 1))
    #     print(e)
