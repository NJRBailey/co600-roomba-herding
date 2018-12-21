import cv2
from numpy import zeros
from numpy import uint8

# NoiseReduction
# This file contains functions to reduce the level of irrelevant information in an image.
# There is one for patterns (Pen and Roomba) and one for the arena boundary (hazard tape).


# Should hold its own dominant colour, its neighbours, and should set neighbours to None if on an edge
class Region:
    def __init__(self, ident, edges, pixels, sampledPixels, neighbours, averageValue, regionsW, regionsH):
        self.id = ident
        self.edges = edges
        self.pixels = pixels
        self.sampledPixels = sampledPixels
        self.neighbours = []
        for neighbour in neighbours:
            if -1 in neighbour or neighbour[0] == regionsW or neighbour[1] == regionsH:
                neighbour = None
            self.neighbours.append(neighbour)
        self.averageValue = averageValue

    def setDominantLightness(self, lightness):
        self.dominantLightness = lightness


# Returns two values - the first is the value under which pixels are 'low' lightness, the second is
# the value under which pixels are 'mid' lightness
def findLightnessThresholds(regions):
    lowestValue = 255
    highestValue = 0
    # frameAverage = 0
    for region in regions:
        if region.averageValue < lowestValue:
            lowestValue = region.averageValue
        if region.averageValue > highestValue:
            highestValue = region.averageValue
        # frameAverage += region.averageValue
    # frameAverage = frameAverage / len(regions)
    third = ((highestValue - lowestValue) / 3) + lowestValue
    lowThreshold, midThreshold = third, third * 2
    return lowThreshold, midThreshold


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
    # hslFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    # h, l, s = cv2.split(hslFrame)
    # cv2.imshow('s', s)
    # cv2.imshow('h', h)
    # cv2.imshow('l', l)
    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('gray', grayFrame)
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
            # print(ident)
            neighbours = (
                (row - 1, column), (row, column + 1), (row + 1, column), (row, column - 1),  # N, E, S, W
                (row - 1, column + 1), (row + 1, column + 1), (row + 1, column - 1), (row - 1, column - 1)  # NE, SE, SW, NW
            )
            regionHeight = normalHeight
            regionWidth = normalWidth
            if row == regionsH - 1:
                regionHeight += extraHeight
            if column == regionsW - 1:
                regionWidth += extraWidth
            leftX = column * normalWidth
            rightX = leftX + regionWidth
            topY = row * normalHeight
            bottomY = topY + regionHeight
            edges = (leftX, rightX, topY, bottomY)  # x1, x2, y1, y2
            pixels = grayFrame[topY:bottomY, leftX:rightX]
            # Find dominant colour
            pxSpaceX = (rightX - leftX) / 4
            pxSpaceY = (bottomY - topY) / 4
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
            averageValue = sum(sampledPixels) / len(sampledPixels)
            regions.append(Region(ident, edges, pixels, sampledPixels, neighbours, averageValue, regionsW, regionsH))
    low, mid = findLightnessThresholds(regions)
    print(low)
    print(mid)
    darkRegions = []
    for region in regions:
        pixelLightnessGroups = {'low': 0, 'mid': 0, 'high': 0}
        for px in region.sampledPixels:
            print(px)
            if px < low:
                pixelLightnessGroups['low'] += 1
            elif px < mid:
                pixelLightnessGroups['mid'] += 1
            else:
                pixelLightnessGroups['high'] += 1
        region.dominantLightness = findDominantLightness(pixelLightnessGroups)
        if region.dominantLightness == 'low':
            darkRegions.append(region)
    processedFrame = zeros((height, width), dtype=uint8)
    for darkRegion in darkRegions:
        e = darkRegion.edges
        # cv2.imshow('darkregion', darkRegion.pixels)
        # cv2.waitKey(5000)
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
i6 = cv2.imread('test-images/grass1.png')
i7 = cv2.imread('test-images/grass2.png')
i8 = cv2.imread('test-images/concrete1.png')
iA = [i1, i2, i3, i4, i5, i6, i7, i8]

for i in iA:
    print('--------------------------------------------')
    prImage = reduceNoiseForPatterns(i)
    cv2.imshow('prImage', prImage)
    cv2.waitKey(0)
