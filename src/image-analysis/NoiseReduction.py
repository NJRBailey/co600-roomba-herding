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
    frameAverage = 0
    for region in regions:
        # print(region.averageValue)
        if region.averageValue < lowestValue:
            lowestValue = region.averageValue
        if region.averageValue > highestValue:
            highestValue = region.averageValue
        frameAverage += region.averageValue
        # print(frameAverage)
    frameAverage = frameAverage / len(regions)
    # print(frameAverage)
    thresholdBias = frameAverage / 128.0  # If the avg lightness is lower/higher than 128, we will lower/raise thresholds
    # print(thresholdBias)
    third = ((highestValue - lowestValue) / 3) + lowestValue
    # print(third)
    lowThreshold, midThreshold = third * thresholdBias, (third * 2) * thresholdBias
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
            ident = (column, row)
            # print(ident)
            neighbours = (
                (column, row - 1), (column + 1, row), (column, row + 1), (column - 1, row),  # N, E, S, W
                (column + 1, row - 1), (column + 1, row + 1), (column - 1, row + 1), (column - 1, row - 1)  # NE, SE, SW, NW
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
                pixels[0 + pxSpaceY][0 + pxSpaceX],
                pixels[0 + pxSpaceY][0 + (pxSpaceX * 2)],
                pixels[0 + pxSpaceY][regionWidth - pxSpaceX],
                pixels[0 + (pxSpaceY * 2)][0 + pxSpaceX],
                pixels[0 + (pxSpaceY * 2)][0 + (pxSpaceX * 2)],
                pixels[0 + (pxSpaceY * 2)][regionWidth - pxSpaceX],
                pixels[regionHeight - pxSpaceY][0 + pxSpaceX],
                pixels[regionHeight - pxSpaceY][0 + (pxSpaceX * 2)],
                pixels[regionHeight - pxSpaceY][regionWidth - pxSpaceX]
            ]
            averageValue = sum(sampledPixels) / len(sampledPixels)
            regions.append(Region(ident, edges, pixels, sampledPixels, neighbours, averageValue, regionsW, regionsH))
    low, mid = findLightnessThresholds(regions)
    # print(low)
    # print(mid)
    darkRegions = []
    for region in regions:
        pixelLightnessGroups = {'low': 0, 'mid': 0, 'high': 0}
        for px in region.sampledPixels:
            if px <= low:
                pixelLightnessGroups['low'] += 1
            elif px <= mid:
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
        cv2.imshow('proc', processedFrame)  # TODO show which box is being changed
        cv2.imshow('lows', frame)
        for neighbour in darkRegion.neighbours:
            if neighbour is not None:
                for region in regions:
                    if region.id == neighbour:
                        nE = region.edges
                        processedFrame[nE[2]:nE[3], nE[0]:nE[1]] = region.pixels
                        cv2.imshow('proc', processedFrame)
                        cv2.imshow('lows', frame)
                        # cv2.waitKey(0)
    return processedFrame


# Processes and returns a frame to try and reduce the amount of information which is unrelated to the arena boundary.
def reduceNoiseForBoundary(frame):
    return frame


iA = []
# iA.append(cv2.imread('test-images/IndexCrash3.png'))
# iA.append(cv2.imread('test-images/IndexCrash3Shadowed.png'))
# iA.append(cv2.imread('test-images/IndexCrash3PenShadow.png'))
# iA.append(cv2.imread('test-images/IndexCrash3TapeShadow.png'))
# iA.append(cv2.imread('test-images/IndexCrash3PenPartShadow.png'))
iA.append(cv2.imread('test-images/grass1.png'))
iA.append(cv2.imread('test-images/grass2.png'))
iA.append(cv2.imread('test-images/concrete1.png'))
# iA.append(cv2.imread('test-images/grass1notape.png'))
# iA.append(cv2.imread('test-images/grass2notape.png'))
# iA.append(cv2.imread('test-images/concrete1notape.png'))

# prImage = reduceNoiseForPatterns(i8)
# cv2.imshow('prImage', prImage)
# cv2.waitKey(0)

for i in iA:
    print('--------------------------------------------')
    prImage = reduceNoiseForPatterns(i)
    cv2.imshow('prImage', prImage)
    cv2.waitKey(0)
