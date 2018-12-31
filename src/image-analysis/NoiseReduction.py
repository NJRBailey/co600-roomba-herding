import cv2
from numpy import zeros
from numpy import uint8
from math import sqrt
from ImageAnalysisUtils import calculateLength

# NoiseReduction
# This file contains functions to reduce the level of irrelevant information in an image.
# There is one for patterns (Pen and Roomba) and one for the arena boundary (hazard tape).


# Should hold its own dominant colour, its neighbours, and should set neighbours to None if on an edge
class Region:
    def __init__(self, ident, edges, lightPixels, hPixels, sPixels, neighbours, averageLightness, averageHue, averageSaturation, regionsW, regionsH, regionPixels):
        self.id = ident
        self.edges = edges
        self.lightnessPixels = lightPixels
        self.huePixels = hPixels
        self.saturationPixels = sPixels
        self.neighbours = []
        for neighbour in neighbours:
            if -1 in neighbour or neighbour[0] == regionsW or neighbour[1] == regionsH:
                neighbour = None
            self.neighbours.append(neighbour)
        self.averageLightness = averageLightness
        self.averageHue = averageHue
        self.averageSaturation = averageSaturation
        self.regionPixels = regionPixels
        self.retain = None

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
        if region.averageLightness < lowestValue:
            lowestValue = region.averageLightness
        if region.averageLightness > highestValue:
            highestValue = region.averageLightness
        frameAverage += region.averageLightness
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


# Searches the pixels for nine evenly-spaced pixels. Returns the pixels in a list and the average lightness.
def sampleLightness(grayPixels, pxSpaceX, pxSpaceY, regionWidth, regionHeight):
    sampledPixels = [
        grayPixels[0 + pxSpaceY][0 + pxSpaceX],
        grayPixels[0 + pxSpaceY][0 + (pxSpaceX * 2)],
        grayPixels[0 + pxSpaceY][regionWidth - pxSpaceX],
        grayPixels[0 + (pxSpaceY * 2)][0 + pxSpaceX],
        grayPixels[0 + (pxSpaceY * 2)][0 + (pxSpaceX * 2)],
        grayPixels[0 + (pxSpaceY * 2)][regionWidth - pxSpaceX],
        grayPixels[regionHeight - pxSpaceY][0 + pxSpaceX],
        grayPixels[regionHeight - pxSpaceY][0 + (pxSpaceX * 2)],
        grayPixels[regionHeight - pxSpaceY][regionWidth - pxSpaceX]
    ]
    averageValue = sum(sampledPixels) / len(sampledPixels)
    return sampledPixels, averageValue


#
def sampleHue(hslPixels, pxSpaceX, pxSpaceY, regionWidth, regionHeight):
    hue, _, _ = cv2.split(hslPixels)
    sampledPixels = [
        hue[0 + pxSpaceY][0 + pxSpaceX],
        hue[0 + pxSpaceY][0 + (pxSpaceX * 2)],
        hue[0 + pxSpaceY][regionWidth - pxSpaceX],
        hue[0 + (pxSpaceY * 2)][0 + pxSpaceX],
        hue[0 + (pxSpaceY * 2)][0 + (pxSpaceX * 2)],
        hue[0 + (pxSpaceY * 2)][regionWidth - pxSpaceX],
        hue[regionHeight - pxSpaceY][0 + pxSpaceX],
        hue[regionHeight - pxSpaceY][0 + (pxSpaceX * 2)],
        hue[regionHeight - pxSpaceY][regionWidth - pxSpaceX]
    ]
    averageValue = sum(sampledPixels) / len(sampledPixels)
    varianceSum = 0
    for pixel in sampledPixels:
        varianceSum += (pixel - averageValue) ** 2
    variance = varianceSum / len(sampledPixels)
    stdDeviation = sqrt(variance)
    normalisedAverage = 0
    normLower = averageValue - stdDeviation
    normUpper = averageValue + stdDeviation
    normalLength = 0
    for pixel in sampledPixels:
        if normLower <= pixel <= normUpper:
            normalisedAverage += pixel
            normalLength += 1
    normalisedAverage = normalisedAverage / normalLength
    return sampledPixels, normalisedAverage


#
def sampleSaturation(hslPixels, pxSpaceX, pxSpaceY, regionWidth, regionHeight):
    _, _, saturation = cv2.split(hslPixels)
    sampledPixels = [
        saturation[0 + pxSpaceY][0 + pxSpaceX],
        saturation[0 + pxSpaceY][0 + (pxSpaceX * 2)],
        saturation[0 + pxSpaceY][regionWidth - pxSpaceX],
        saturation[0 + (pxSpaceY * 2)][0 + pxSpaceX],
        saturation[0 + (pxSpaceY * 2)][0 + (pxSpaceX * 2)],
        saturation[0 + (pxSpaceY * 2)][regionWidth - pxSpaceX],
        saturation[regionHeight - pxSpaceY][0 + pxSpaceX],
        saturation[regionHeight - pxSpaceY][0 + (pxSpaceX * 2)],
        saturation[regionHeight - pxSpaceY][regionWidth - pxSpaceX]
    ]
    averageValue = sum(sampledPixels) / len(sampledPixels)
    varianceSum = 0
    for pixel in sampledPixels:
        varianceSum += (pixel - averageValue) ** 2
    variance = varianceSum / len(sampledPixels)
    stdDeviation = sqrt(variance)
    normalisedAverage = 0
    normLower = averageValue - stdDeviation
    normUpper = averageValue + stdDeviation
    normalLength = 0
    for pixel in sampledPixels:
        if normLower <= pixel <= normUpper:
            normalisedAverage += pixel
            normalLength += 1
    normalisedAverage = normalisedAverage / normalLength
    return sampledPixels, normalisedAverage


#
def sampleContrast(region, samplesPerLine, contrastDiff=60):
    e = region.edges
    tl = (e[0], e[2])
    tr = (e[1], e[2])
    bl = (e[0], e[3])
    # Sample top and bottom
    xDist = calculateLength(tl, tr)
    xSpacing = int(round(xDist / samplesPerLine * 1.0))
    topSamples = []
    bottomSamples = []
    pixels = cv2.cvtColor(region.regionPixels, cv2.COLOR_BGR2GRAY)
    topRow = 0
    bottomRow = e[3] - e[2] - 1
    leftColumn = 0
    rightColumn = e[1] - e[0] - 1
    for i in range(samplesPerLine - 1):
        column = i * xSpacing
        topSamples.append(pixels[topRow][column])
        bottomSamples.append(pixels[bottomRow][column])
    topSamples.append(pixels[topRow][rightColumn])
    bottomSamples.append(pixels[bottomRow][rightColumn])
    topAverage = sum(topSamples) / len(topSamples)
    bottomAverage = sum(bottomSamples) / len(bottomSamples)
    if abs(topAverage - bottomAverage) >= contrastDiff:
        return True
    # Sample left and right
    yDist = calculateLength(tl, bl)
    ySpacing = int(round(yDist / samplesPerLine * 1.0))
    leftSamples = []
    rightSamples = []
    for i in range(samplesPerLine - 1):
        row = i * ySpacing
        leftSamples.append(pixels[row][leftColumn])
        rightSamples.append(pixels[row][rightColumn])
    leftSamples.append(pixels[bottomRow][leftColumn])
    rightSamples.append(pixels[bottomRow][rightColumn])
    leftAverage = sum(leftSamples) / len(leftSamples)
    rightAverage = sum(rightSamples) / len(rightSamples)
    if abs(leftAverage - rightAverage) >= contrastDiff:
        return True
    return False


# Processes and returns a frame to try and reduce the amount of information which is unrelated to patterns.
def reduceNoiseForPatterns(frame, regionsW=16, regionsH=9):  # TODO make samples a parameter; return small regions
    # cv2.imshow('frame', frame)
    cloneFrame = frame.copy()
    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    height, width = grayFrame.shape
    # Filter out green and red, leave greyscale and blue
    # Remove things above 20% saturation, within green and red bounds
    # blueBoundaries = (93, 142)
    # saturationThreshold = 51  # 255 * 0.2
    hslFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    h, l, s = cv2.split(hslFrame)
    # cv2.imshow('s', s)
    # cv2.imshow('h', h)
    # cv2.imshow('l', l)
    # for row in range(height):
        # for column in range(width):
            # print
            # print(h[column][row])
            # print(s[column][row])
            # if not (93 <= h[row][column] <= 142 and s[row][column] >= 51):
                # cloneFrame[row][column] = [255, 255, 255]

    # cv2.imshow('colFilter', cloneFrame)
    # cv2.imshow('gray', grayFrame)
    # Split frame up into regions
    # 1. Create regions
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
            regionPixels = frame[topY:bottomY, leftX:rightX]
            grayPixels = grayFrame[topY:bottomY, leftX:rightX]
            hslPixels = hslFrame[topY:bottomY, leftX:rightX]
            # Find dominant colour
            pxSpaceX = (rightX - leftX) / 4
            pxSpaceY = (bottomY - topY) / 4
            # Get 9 pixels in a 3x3 shape around centre
            lightnessPixels, averageLightness = sampleLightness(grayPixels, pxSpaceX, pxSpaceY, regionWidth, regionHeight)
            huePixels, averageHue = sampleHue(hslPixels, pxSpaceX, pxSpaceY, regionWidth, regionHeight)
            saturationPixels, averageSaturation = sampleSaturation(hslPixels, pxSpaceX, pxSpaceY, regionWidth, regionHeight)
            regions.append(Region(ident, edges, lightnessPixels, huePixels, saturationPixels, neighbours, averageLightness, averageHue, averageSaturation, regionsW, regionsH, regionPixels))

    # 2. Filter red and green areas
    processedFrame = zeros((height, width), dtype=uint8)
    for region in regions:
        # print(region.averageHue)
        # print(region.averageSaturation)
        e = region.edges
        # print(e)
        cv2.circle(cloneFrame, ((e[1] + e[0]) / 2, (e[3] + e[2]) / 2), 8, (0, 255, 0))
        # cv2.imshow('csr', cloneFrame)
        # print('waiting1')
        # cv2.waitKey(0)
        blue = 93 <= region.averageHue <= 130
        grayscale = region.averageSaturation <= 38
        if not (blue or grayscale):
            # cv2.circle(cloneFrame, ((e[1] + e[0]) / 2, (e[3] + e[2]) / 2), 8, (0, 0, 255))
            erase = True
            for ident in region.neighbours:  # TODO make a while loop for efficiency
                if ident is not None:
                    for regionCheck in regions:  # TODO adjust region class so that neighbours are actual objects
                        if regionCheck.id == ident:
                            # cv2.imshow('csr', cloneFrame)
                            # print('waiting2')
                            # cv2.waitKey(0)
                            nBlue = 93 <= regionCheck.averageHue <= 142
                            nGrayscale = regionCheck.averageSaturation <= 38
                            if nBlue or nGrayscale:
                                nE = regionCheck.edges
                                # cv2.circle(cloneFrame, ((nE[1] + nE[0]) / 2, (nE[3] + nE[2]) / 2), 8, (0, 255, 0))
                                cv2.circle(cloneFrame, ((e[1] + e[0]) / 2, (e[3] + e[2]) / 2), 8, (255, 0, 0))
                                # cv2.imshow('csr', cloneFrame)
                                # print('waiting3')
                                # cv2.waitKey(0)
                                erase = False
            if erase is True:
                region.retain = False
                frame[e[2]:e[3], e[0]:e[1]].fill(255)
                cloneFrame[e[2]:e[3], e[0]:e[1]].fill(255)
    cv2.imshow('check', cloneFrame)
    cv2.imshow('proc', frame)
    cv2.waitKey(0)

    # 3. Find and retain regions with high contrast
    for region in regions:
        if region.retain is not False:
            if sampleContrast(region, 5) is True:
                region.retain = True
                # cv2.imshow('retain', region.regionPixels)
                e = region.edges
                cv2.circle(cloneFrame, ((e[1] + e[0]) / 2, (e[3] + e[2]) / 2), 8, (255, 0, 255))
                cv2.imshow('retain', cloneFrame)
                cv2.waitKey(0)
    cv2.imshow('retain', cloneFrame)
    cv2.waitKey(0)

    # 4. Filter out regions of low lightness
    low, mid = findLightnessThresholds(regions)
    darkRegions = []
    for region in regions:
        if region.retain is not False:
            pixelLightnessGroups = {'low': 0, 'mid': 0, 'high': 0}
            for px in region.lightnessPixels:
                if px <= low:
                    pixelLightnessGroups['low'] += 1
                elif px <= mid:
                    pixelLightnessGroups['mid'] += 1
                else:
                    pixelLightnessGroups['high'] += 1
            region.dominantLightness = findDominantLightness(pixelLightnessGroups)
            if region.dominantLightness == 'low':
                darkRegions.append(region)

    for darkRegion in darkRegions:
        e = darkRegion.edges
        grayPixels = cv2.cvtColor(darkRegion.regionPixels, cv2.COLOR_BGR2GRAY)
        processedFrame[e[2]:e[3], e[0]:e[1]] = grayPixels
        cv2.imshow('proc', processedFrame)
        for neighbour in darkRegion.neighbours:
            if neighbour is not None:
                for region in regions:
                    if region.id == neighbour:
                        nE = region.edges
                        nGrayPixels = cv2.cvtColor(region.regionPixels, cv2.COLOR_BGR2GRAY)
                        processedFrame[nE[2]:nE[3], nE[0]:nE[1]] = nGrayPixels
                        cv2.imshow('proc', processedFrame)
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
# iA.append(cv2.imread('test-images/grass1.png'))
# iA.append(cv2.imread('test-images/grass2.png'))
# iA.append(cv2.imread('test-images/concrete1.png'))

iA.append(cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/grass1.png'))
iA.append(cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/grass2.png'))
iA.append(cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/concrete1.png'))

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
