import cv2
import HierarchyReader
from operator import itemgetter
from math import asin
from math import degrees
from ImageAnalysisUtils import calculateLength
import NoiseReduction

# RoombaPenReader
# Module used for identifying pattern boxes on a frame. Attempts to identify Roomba and Pen
# patterns, and their orientation relative to the frame's Y-axis. Returns a dict with two keys:
# {identified: [], investigate: []}
#
# 'identified' holds the identified patterns in the form: {id: String, polygon: Tuple[4], orientation: Int}
# 'polygon' is a set of four coordinates (AKA points) which show the corners of the pattern.
# 'investigate' holds the boxes which were found, but could not be identified as part of a pattern.
#
# A box is a dict in the form: {corners: Array[4], id: Int, centre: Tuple(2)}
# The ID of a box is automatically assigned by OpenCV.
#
# Known bugs:
# 1. Does not correctly try all combinations of boxes (should do 1-6 | 1,3-7 | 1,4-8 etc.
REGION_TOLERANCE = 10


# Finds and returns the coordinate of the point between two points
def findMidpoint(point1, point2):
    midpoint = ((point2[0] + point1[0]) / 2, (point1[1] + point2[1]) / 2)
    return midpoint


# Finds the gradient and y-intercept of the line between two points. Swaps the Y values because
# screen has origin in top-left corner, not bottom-left corner
def findLineEquation(point1, point2):
    if point1[0] == point2[0]:
        m = float('inf')  # If the two points are on the same x axis, m is infinity (special case in further code)
    else:
        m = (point2[1] - point1[1]) / (point2[0] - point1[0] * 1.0)
    c = point1[1] - (m * point1[0])
    return {'point1': point1, 'point2': point2, 'm': m, 'c': c}


# Creates and returns a region of coordinates around the centre of the box supplied
def generateBoxRegion(box):
    tolerance = REGION_TOLERANCE
    region = []
    tl = (box['centre'][0] - tolerance, box['centre'][1] - tolerance)
    r = 0
    s = 0
    while r <= tolerance * 2:
        while s <= tolerance * 2:
            region.append((tl[0] + s, tl[1] + r))
            s += 1
        r += 1
        s = 0
    return region


# Checks a line from end to end and checks how many boxes it passes through
def countLineCrossesBoxes(line, boxes):
    crossCount = 0
    for box in boxes:
        crossed = False
        boxX = box['centre'][0]
        boxY = box['centre'][1]
        lineX = line['point1'][0]
        if line['m'] == float('inf'):
            crossed = (boxX - REGION_TOLERANCE) <= lineX <= (boxX + REGION_TOLERANCE)
        else:
            y = (line['m'] * boxX) + line['c']
            crossed = (boxY - REGION_TOLERANCE) <= y <= (boxY + REGION_TOLERANCE)
        if crossed:
            crossCount += 1
    return crossCount


# Checks that a supplied y falls approximately on the line
# def verifyLineEquation(expectedY, x, m, c):
#     y = (m * x) + c
#     return y > expectedY - 2 and y < expectedY + 2


# Finds and returns the coordinates for the corners of a pattern.
def identifyBounds(boxes, frame):
    # Check whether pattern is a perfectly-aligned square
    # Save all the xs and ys with the number of times they appear
    # If two xs and two ys appear twice each, that is a perfect square
    xCount = {}
    yCount = {}
    for box in boxes:
        x = str(box['centre'][0])
        y = str(box['centre'][1])
        if x in xCount:
            xCount[x] += 1
        else:
            xCount[x] = 1
        if y in yCount:
            yCount[y] += 1
        else:
            yCount[y] = 1
    numberOfSameXs = 0
    numberOfSameYs = 0
    for x in xCount:
        if xCount[x] >= 2:
            numberOfSameXs += 1
    for y in yCount:
        if yCount[y] >= 2:
            numberOfSameYs += 1
    perfectSquare = False
    if numberOfSameXs >= 2 and numberOfSameYs >= 2:
        perfectSquare = True
    if perfectSquare:
        tl = boxes[0]['corners'][0]  # Lowest X, highest Y
        tr = boxes[0]['corners'][1]  # Highest X, highest Y
        bl = boxes[0]['corners'][2]  # Lowest X, lowest Y
        br = boxes[0]['corners'][3]  # Highest X, lowest Y
        for box in boxes:
            for corner in box['corners']:
                if corner[0] <= tl[0] and corner[1] >= tl[1]:
                    tl = corner
                if corner[0] >= tr[0] and corner[1] >= tr[1]:
                    tr = corner
                if corner[0] <= bl[0] and corner[1] <= bl[1]:
                    bl = corner
                if corner[0] >= br[0] and corner[1] <= br[1]:
                    br = corner
        return [tl, tr, bl, br]
    else:
        # Construct bounds from boxes coordinates
        hX = boxes[0]['corners'][0]  # Highest X
        lX = boxes[0]['corners'][1]  # Lowest X
        hY = boxes[0]['corners'][2]  # Highest Y
        lY = boxes[0]['corners'][3]  # Lowest Y
        for box in boxes:
            for corner in box['corners']:
                if corner[0] > hX[0]:
                    hX = corner
                if corner[0] < lX[0]:
                    lX = corner
                if corner[1] > hY[1]:
                    hY = corner
                if corner[1] < lY[1]:
                    lY = corner
        return [hX, lX, hY, lY]


# Finds the orientation of a shape relative to the top of the frame, from 0 degrees to 359 degrees (clockwise)
def findOrientation(boxes, outerBoxes, shape, frame):  # TODO frame probably can be replaced by w, h params
    lines = []
    # Find 3 equations from first outer box
    distBC = calculateLength(outerBoxes[1]['centre'], outerBoxes[2]['centre'])
    distBD = calculateLength(outerBoxes[1]['centre'], outerBoxes[3]['centre'])
    distCD = calculateLength(outerBoxes[2]['centre'], outerBoxes[3]['centre'])
    lengthList = [{'name': 'BC', 'length': distBC},
                  {'name': 'BD', 'length': distBD},
                  {'name': 'CD', 'length': distCD}]
    sortedLengthList = sorted(lengthList, key=itemgetter('length'))
    nonDiagonalEnds = []
    lastOuterBox = None
    maxDistance = sortedLengthList[2]
    if maxDistance['name'] == 'BC':
        nonDiagonalEnds = [outerBoxes[1]['centre'], outerBoxes[2]['centre']]
        lastOuterBox = outerBoxes[3]
    elif maxDistance['name'] == 'BD':
        nonDiagonalEnds = [outerBoxes[1]['centre'], outerBoxes[3]['centre']]
        lastOuterBox = outerBoxes[2]
    else:
        nonDiagonalEnds = [outerBoxes[2]['centre'], outerBoxes[3]['centre']]
        lastOuterBox = outerBoxes[1]
    outerEquations = []
    outerEquations.append(findLineEquation(outerBoxes[0]['centre'], nonDiagonalEnds[0]))
    outerEquations.append(findLineEquation(outerBoxes[0]['centre'], nonDiagonalEnds[1]))
    outerEquations.append(findLineEquation(nonDiagonalEnds[0], lastOuterBox['centre']))
    outerEquations.append(findLineEquation(nonDiagonalEnds[1], lastOuterBox['centre']))
    for eqtn in outerEquations:
        lines.append({'endPoints': (eqtn['point1'], eqtn['point2']), 'crosses': countLineCrossesBoxes(eqtn, boxes)})
    topLine = None
    if shape == 'pen':
        # Identify lone corner
        twoCrossLines = []
        for line in lines:
            if line['crosses'] == 2:
                twoCrossLines.append(line)
        loneCorner = None
        for box in outerBoxes:
            if box['centre'] in twoCrossLines[0]['endPoints'] and box['centre'] in twoCrossLines[1]['endPoints']:
                loneCorner = box
        # Identify opposite corner
        oppCorner = None
        for box in outerBoxes:
            if box['centre'] not in twoCrossLines[0]['endPoints'] and box['centre'] not in twoCrossLines[1]['endPoints']:
                oppCorner = box
        # Store remaining two corners
        otherCorners = []
        for box in outerBoxes:
            if box is not loneCorner and box is not oppCorner:
                otherCorners.append(box)
        # Find other topLine corner
        if loneCorner['centre'][1] < oppCorner['centre'][1]:
            # Determine left-most other corner
            if otherCorners[0]['centre'][0] < otherCorners[1]['centre'][0]:
                topLine = {'endPoints': ((otherCorners[0]['centre']), (oppCorner['centre']))}
            else:
                topLine = {'endPoints': ((otherCorners[1]['centre']), (oppCorner['centre']))}
        elif loneCorner['centre'][1] > oppCorner['centre'][1]:
            # Determine right-most other corner
            if otherCorners[0]['centre'][0] > otherCorners[1]['centre'][0]:
                topLine = {'endPoints': ((otherCorners[0]['centre']), (oppCorner['centre']))}
            else:
                topLine = {'endPoints': ((otherCorners[1]['centre']), (oppCorner['centre']))}
        elif loneCorner['centre'][0] < oppCorner['centre'][0]:
            # Determine bottom-most other corner
            if otherCorners[0]['centre'][1] > otherCorners[1]['centre'][1]:
                topLine = {'endPoints': ((otherCorners[0]['centre']), (oppCorner['centre']))}
            else:
                topLine = {'endPoints': ((otherCorners[1]['centre']), (oppCorner['centre']))}
        else:
            # Determine top-most other corner
            if otherCorners[0]['centre'][1] < otherCorners[1]['centre'][1]:
                topLine = {'endPoints': ((otherCorners[0]['centre']), (oppCorner['centre']))}
            else:
                topLine = {'endPoints': ((otherCorners[1]['centre']), (oppCorner['centre']))}
    elif shape == 'roomba':
        for line in lines:
            if line['crosses'] == 3:
                topLine = line
    # Get line eqtn between top line ends
    topLineEquation = findLineEquation(topLine['endPoints'][0], topLine['endPoints'][1])
    centre = findMidpoint(topLine['endPoints'][0], topLine['endPoints'][1])
    perpLineEquation = {}
    if abs(topLineEquation['m']) == 0.0:
        perpLineEquation = {'linePoint': centre, 'm': float('inf'), 'c': -float('inf')}
    elif topLineEquation['m'] == float('inf'):
        perpLineEquation = {'linePoint': centre, 'm': 0.0}
        perpLineEquation['c'] = centre[1] - (perpLineEquation['m'] * centre[0])
    else:
        perpLineEquation = {'linePoint': centre, 'm': (-1 / topLineEquation['m'] * 1.0)}
        perpLineEquation['c'] = centre[1] - (perpLineEquation['m'] * centre[0])
    # Find whether to use y = 0 or y = Y
    otherCorners = []
    for box in outerBoxes:
        if box['centre'] not in topLine['endPoints']:
            otherCorners.append(box)
    otherLineCentre = findMidpoint(otherCorners[0]['centre'], otherCorners[1]['centre'])
    if otherLineCentre[1] < centre[1]:
        if otherLineCentre[0] == centre[0]:
            return 180
        yPlus1 = 0
        try:
            yPlus1, _ = frame.shape
        except ValueError:
            yPlus1, _, _ = frame.shape
        perpLineEquation['edgePoint'] = (None, yPlus1 - 1)
    elif otherLineCentre[1] > centre[1]:
        if otherLineCentre[0] == centre[0]:
            return 0
        perpLineEquation['edgePoint'] = (None, 0)
    elif otherLineCentre[0] < centre[0]:
        return 90
    else:
        return 270
    perpX = int(round((perpLineEquation['edgePoint'][1] - perpLineEquation['c']) / perpLineEquation['m']))
    perpLineEquation['edgePoint'] = (perpX, perpLineEquation['edgePoint'][1])
    # Apply sine rule to find angle
    lengthOpp90 = calculateLength(perpLineEquation['linePoint'], perpLineEquation['edgePoint'])
    primeMeridianPointX = perpLineEquation['linePoint'][0]
    primeMeridianPointY = perpLineEquation['edgePoint'][1]
    lengthOppTarget = calculateLength((primeMeridianPointX, primeMeridianPointY), perpLineEquation['edgePoint'])
    angle = int(round(degrees(asin(lengthOppTarget / lengthOpp90))))
    # Do the necessary addition or subtraction to find the angle from North (0 to 359)
    if perpLineEquation['edgePoint'][1] == 0:
        if perpLineEquation['edgePoint'][0] <= primeMeridianPointX:
            return 360 - angle
        else:
            return angle
    else:
        if perpLineEquation['edgePoint'][0] >= primeMeridianPointX:
            return 180 - angle
        else:
            return 180 + angle


# Takes 6 boxes and attempts to recognise the Roomba or Pen
def identifyPattern(boxes, frame, returnOrientation):
    if len(boxes) is not 6:
        raise ValueError('parameter "boxes" must be of length 6')
    # Find bounds
    bounds = identifyBounds(boxes, frame)
    # Pick corner bound boxes
    outerBoxes = []
    for box in boxes:
        for corner in box['corners']:
            if (corner == bounds[0] or corner == bounds[1] or corner == bounds[2] or corner == bounds[3]) and box not in outerBoxes:
                outerBoxes.append(box)
    if len(outerBoxes) is not 4:
        return {'id': 'unknown', 'polygon': tuple(bounds)}
    # Find line equations between centres of outer corner boxes
    lineEquations = []
    i = 0
    j = 1
    while i < 3:
        while j < 4:
            lineEquations.append(findLineEquation(outerBoxes[i]['centre'], outerBoxes[j]['centre']))
            j += 1
        i += 1
        j = i + 1
    # Go along each line equation and check how many regions the line passes through
    crosses = []
    for line in lineEquations:
        crosses.append(countLineCrossesBoxes(line, boxes))
    # Pen = 2x 3, 4x 2
    # Roomba = 3x 3, 3x 2
    orientation = None
    if crosses.count(3) == 2 and crosses.count(2) == 4:
        if returnOrientation:
            orientation = findOrientation(boxes, outerBoxes, 'pen', frame)
        return {'id': 'pen', 'polygon': tuple(bounds), 'orientation': orientation}
    elif crosses.count(3) == 3 and crosses.count(2) == 3:
        if returnOrientation:
            orientation = findOrientation(boxes, outerBoxes, 'roomba', frame)
        return {'id': 'roomba', 'polygon': tuple(bounds), 'orientation': orientation}
    else:
        return {'id': 'unknown', 'polygon': tuple(bounds)}


# Takes an image and attempts to identify boxes in Roomba and Pen patterns.
# Returns a dict containing two arrays. The array with key 'identified' contains patterns
# which have been identified in the image. The array with key 'investigate' contains
# boxes which could not be linked to any pattern.
def decode(frame, returnOrientation=False):
    cleanFrame = NoiseReduction.reduceNoiseForPatterns(frame)
    cannyEdge = cv2.Canny(cleanFrame, 127, 255)
    _, contours, hierarchy = cv2.findContours(cannyEdge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return {'identified': [], 'investigate': []}
    cannyLinkedLists = HierarchyReader.readHierarchy(hierarchy[0])

    # cloneFrame = cleanFrame.copy()
    # cv2.drawContours(cloneFrame, contours, -1, (0, 255, 0), 2)
    # cv2.imshow('cont', cloneFrame)
    # cv2.waitKey(0)
    # for ll in cannyLinkedLists:
        # if len(ll.list) > 4:
            # ll.printList()

    boxes = []
    # Find the big squares with 6 contours
    for linkedList in cannyLinkedLists:
        if len(linkedList.list) > 5:
            item = linkedList.list[len(linkedList.list) - 1]
            boundingBox = cv2.minAreaRect(contours[item.id])
            corners = cv2.boxPoints(boundingBox).tolist()
            moment = cv2.moments(contours[item.id])
            centreX = int(moment['m10'] / moment['m00'])
            centreY = int(moment['m01'] / moment['m00'])
            boxes.append({'id': item.id, 'corners': corners, 'centre': (centreX, centreY)})
    # Cases:
    length = len(boxes)
    # 1. none - Nothing on screen
    if length == 0:
        return {'identified': [], 'investigate': []}
    # 2. less than six - mark for investigation
    if length < 6:
        return {'identified': [], 'investigate': boxes}
    # 3. six - we should try to identify a pattern
    if length == 6:
        identifiedPattern = identifyPattern(boxes, cleanFrame, returnOrientation)
        if identifiedPattern['id'] in ['roomba', 'pen']:
            return {'identified': [identifiedPattern], 'investigate': []}
        else:
            return {'identified': [], 'investigate': boxes}
    # 4. more than six - We should try to identify a pattern, and mark the others for investigation
    if length > 6:
        # 1. Find closest boxes and check first
        # 2. If that didn't find a pattern, use a complicated loop over a list to check each combination
        # Save each combination of boxes so they can be order-independently checked
        # If a pattern is found, remove those boxes from the list, and restart the complicated loop
        # Each loop should check the current boxes against the previously checked combinations
        identified = []
        whitelist = []
        i = 0
        while i <= length - 6:
            boxesSet = [boxes[i], boxes[i + 1], boxes[i + 2], boxes[i + 3], boxes[i + 4], boxes[i + 5]]
            identifiedPattern = identifyPattern(boxesSet, cleanFrame, returnOrientation)
            if identifiedPattern['id'] in ['roomba', 'pen']:
                identified.append(identifiedPattern)
                for box in boxesSet:
                    whitelist.append(box)
                i += 6
            else:
                i += 1
        # Include the boxes to be investigated
        investigate = []
        for box in boxes:
            if box not in whitelist:
                investigate.append(box)
        return {'identified': identified, 'investigate': investigate}


# i1 = cv2.imread('test-images/IndexCrash3.png')
# i2 = cv2.imread('test-images/IndexCrash3Shadowed.png')
# i3 = cv2.imread('test-images/IndexCrash3PenShadow.png')
# i4 = cv2.imread('test-images/IndexCrash3TapeShadow.png')
# i5 = cv2.imread('test-images/IndexCrash3PenPartShadow.png')
# iA = [i1, i2, i3, i4, i5]
# puregrey = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test/test-images/PureGrey.png')
# perfect = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test/test-images/RoombaBoxesInvertTight.png')
ic2 = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test/test-images/FakeSimDistance2.png')
# print(decode(perfect))
print(decode(ic2))
# decode(puregrey)

# i = -1
# while i < 4:
#     print('--------------------------------------------')
#     i += 1
#     try:
#         print(decode(iA[i]))
#         cv2.waitKey(0)
#     except Exception as e:
#         print('error with i' + str(i + 1))
#         print(e)

# iA = []
# iA.append(cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/grass1.png'))
# iA.append(cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/grass2.png'))
# iA.append(cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/concrete1.png'))
# for im in iA:
#     print('---------')
#     print(decode(im))
