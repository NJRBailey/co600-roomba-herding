import cv2
from RoombaPenReader import decode
from ImageAnalysisUtils import calculateLength

# PatternLocation
# Find the location of any identified patterns in one of nine locations on the screen.


## Checks to see if a coordinate falls within a box.
#
# @param coordinate A tuple (x,y) representing a pixel on screen.
# @param boundingBox A tuple (id, (x,y), (a,b)) where (x,y) and (a,b) are the top-left and bottom-right corners.
# @return A Boolean value, True if the coordinate was within the bounding box.
def coordInBounds(coordinate, boundingBox):
    # Check with a < x < b and c < y < d
    if (
        coordinate[0] > boundingBox[1][0] and coordinate[0] < boundingBox[2][0] and
        coordinate[1] > boundingBox[1][1] and coordinate[1] < boundingBox[2][1]
    ):
        return True
    return False


## Divides the screen into a 3x3 grid and returns the grid each pattern is in.
#
# @param frame An OpenCV-compatible image.
# @param decodedPatterns An array of patterns which have been found in the frame.
# @return A dict containing the position ('t','l','b','r','c') of the Roomba and pen if applicable.
def findPatternLocations(frame, decodedPatterns):
    sections = []
    height, width, _ = frame.shape
    # Sectioning of frame
    thirdWidth = int(round(width / 3.0))
    thirdHeight = int(round(height / 3.0))
    # Define sectioning boxes as Tuple (ID, Top-left coordinate, bottom-right coordinate)
    sections.append(('tl', (0, 0), (thirdWidth, thirdHeight)))
    sections.append(('t', (thirdWidth, 0), (thirdWidth * 2, thirdHeight)))
    sections.append(('tr', (thirdWidth * 2, 0), (width, thirdHeight)))
    sections.append(('l', (0, thirdHeight), (thirdWidth, thirdHeight * 2)))
    sections.append(('c', (thirdWidth, thirdHeight), (thirdWidth * 2, thirdHeight * 2)))
    sections.append(('r', (thirdWidth * 2, thirdHeight), (width, thirdHeight * 2)))
    sections.append(('bl', (0, thirdHeight * 2), (thirdWidth, height)))
    sections.append(('b', (thirdWidth, thirdHeight * 2), (thirdWidth * 2, height)))
    sections.append(('br', (thirdWidth * 2, thirdHeight * 2), (width, height)))
    patterns = {'roombaPosition': None, 'penPosition': None}
    for code in decodedPatterns:
        # if code['id'] in ['roomba', 'pen', 'unknown']:  Will be used if making improvements to search
        if code['id'] in ['roomba', 'pen']:
            points = code['polygon']
            # Find centre
            i = 1
            longestDistance = (0, 0, 0)
            while i < 4:
                length = calculateLength((points[0][0], points[0][1]), (points[i][0], points[i][1]))
                if length > longestDistance[2]:
                    longestDistance = (points[0], points[i], length)
                i += 1
            centreX = (longestDistance[0][0] + longestDistance[1][0]) / 2
            centreY = (longestDistance[0][1] + longestDistance[1][1]) / 2
            # Detect what section centre of QR code is in
            foundSection = False
            j = 0
            while foundSection is False and j < 9:
                foundSection = coordInBounds((centreX, centreY), sections[j])
                if foundSection is True:
                    if code['id'] == 'roomba':
                        patterns['roombaPosition'] = sections[j][0]
                    else:
                        patterns['penPosition'] = sections[j][0]
                j += 1
    return patterns


## Finds the location of a pattern on the screen
#
# @param frame An OpenCV-compatible image.
# @param debug Optional - if True will display the live camera feed in a window.
# @return A dict containing the position ('t','l','b','r','c') of the Roomba and pen if applicable.
def getPattern(frame, debug=False):
    # frame = cv2.flip(camFrame, 0)
    if debug:
        cv2.imshow('frame', frame)
        cv2.waitKey(1)
    decoded = decode(frame)['identified']
    return findPatternLocations(frame, decoded)


## Returns the orientation of the Roomba in degrees relative to the top of the frame.
#
# @param frame An OpenCV-compatible image.
# @return An Integer between 0-359 representing the rotation of the Roomba, or None.
def getOrientation(frame):
    decoded = decode(frame, True)['identified']
    for code in decoded:
        if code['id'] == 'roomba':
            return code['orientation']
    return None


## Returns the location of the patterns on screen, and the orientation of the Roomba.
#
# @param frame An OpenCV-compatible image.
# @return Patterns and rotations e.g.{'roomba': {'location': 'c', 'orientation': 84}, 'pen': {'location: 't'}}
def getPatternAndOrientation(frame):
    decoded = decode(frame, True)['identified']
    output = {'roomba': None, 'pen': None}
    locations = findPatternLocations(frame, decoded)
    if locations['penPosition'] is not None:
        output['pen'] = {'location': locations['penPosition']}
    if locations['roombaPosition'] is not None:
        output['roomba'] = {'location': locations['roombaPosition']}
        for code in decoded:
            if code['id'] == 'roomba':
                output['roomba']['orientation'] = code['orientation']
    return output


## Casts four lines in a + shape, returns the first pixel found for each line and the corresponding letter.
#
# @param frame An OpenCV-compatible image.
# @return A dict containing the pixel location for each direction {'l','r','t','b'}.
def findBoundary(frame):
    height, width, _ = frame.shape
    centre = (width / 2, height / 2)
    boundarySections = {'l': None, 'r': None, 't': None, 'b': None}
    left = centre[0]  # start in middle of frame
    pinkL = False
    while left >= 0 and pinkL is False:
        pixel = frame[centre[1]][left].tolist()
        if pixel == [80, 0, 80] or (103 <= pixel[0] <= 155 and pixel[1] <= 38 and 103 <= pixel[2] <= 155):
            pinkL = True
            boundarySections['l'] = left
        left -= 1
    right = centre[0]
    pinkR = False
    while right < width and pinkR is False:
        pixel = frame[centre[1]][right].tolist()
        if pixel == [80, 0, 80] or (103 <= pixel[0] <= 155 and pixel[1] <= 38 and 103 <= pixel[2] <= 155):
            pinkR = True
            boundarySections['r'] = right
        right += 1
    top = centre[1]
    pinkT = False
    while top >= 0 and pinkT is False:
        pixel = frame[top][centre[0]].tolist()
        if pixel == [80, 0, 80] or (103 <= pixel[0] <= 155 and pixel[1] <= 38 and 103 <= pixel[2] <= 155):
            pinkT = True
            boundarySections['t'] = top
        top -= 1
    bottom = centre[1]
    pinkB = False
    while bottom < height and pinkB is False:
        pixel = frame[bottom][centre[0]].tolist()
        if pixel == [80, 0, 80] or (103 <= pixel[0] <= 155 and pixel[1] <= 38 and 103 <= pixel[2] <= 155):
            pinkB = True
            boundarySections['b'] = bottom
        bottom += 1
    return boundarySections


## Searches for the arena boundary and returns a list containing l, r, t or b.
#
# @param frame An OpenCV-compatible image.
# @return An array which may contain the letters 'l', 'r', 't' or 'b'.
def getBoundary(frame):
    boundaries = findBoundary(frame)
    boundarySections = []
    for key, value in boundaries.iteritems():
        if value:
            boundarySections.append(key)
    if boundarySections:
        return boundarySections
    return []


## Returns an approximate distance in mm between the roomba and each visible boundary.
#
# @param frame An OpenCV-compatible image.
# @param roombaSupplied Optional - the four corners of the Roomba pattern, so decode() is not called.
# @return A dict containing an approximate value in mm for 'l', 'r', 't' and 'b'.
def getDistanceFromBoundary(frame, roombaSupplied=None):
    output = {'l': None, 'r': None, 't': None, 'b': None}
    roombaWidthMm = 240  # millimetres, NOT pixels
    roombaCorners = None
    if roombaSupplied:
        roombaCorners = roombaSupplied
    else:
        decoded = decode(frame)['identified']
        for code in decoded:
            if code['id'] == 'roomba':
                roombaCorners = code['polygon']
        if roombaCorners is None:
            return output
    length1 = calculateLength(roombaCorners[0], roombaCorners[1])
    length2 = calculateLength(roombaCorners[0], roombaCorners[2])
    roombaWidthPx = min([length1, length2])  # Gets the min of two so we don't find the diagonal length
    mmPerPx = roombaWidthMm / roombaWidthPx
    boundaries = findBoundary(frame)
    roombaXs = []
    roombaYs = []
    for corner in roombaCorners:
        roombaXs.append(corner[0])
        roombaYs.append(corner[1])
    for position, pixel in boundaries.iteritems():
        if position == 'l':  # pixel will be an 'x' coordinate
            if pixel:
                leftRoombaX = min(roombaXs)
                pixelDistance = abs(leftRoombaX - pixel)
                mmFromLeft = mmPerPx * pixelDistance
                output['l'] = int(round(mmFromLeft))
        elif position == 'r':  # pixel will be an 'x' coordinate
            if pixel:
                rightRoombaX = max(roombaXs)
                pixelDistance = abs(pixel - rightRoombaX)
                mmFromRight = mmPerPx * pixelDistance
                output['r'] = int(round(mmFromRight))
        elif position == 't':  # pixel will be a 'y' coordinate
            if pixel:
                topRoombaY = min(roombaYs)
                pixelDistance = abs(topRoombaY - pixel)
                mmFromTop = mmPerPx * pixelDistance
                output['t'] = int(round(mmFromTop))
        else:  # pixel will be a 'y' coordinate
            if pixel:
                bottomRoombaY = max(roombaYs)
                pixelDistance = abs(pixel - bottomRoombaY)
                mmFromBottom = mmPerPx * pixelDistance
                output['b'] = int(round(mmFromBottom))
    return output


# ins = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test/test-images/intnotstr.png')
# bound1 = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test/test-images/FakePinkBound1.png')
# bound0 = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test/test-images/IndexCrash3.png')
# bound1shadow = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test/test-images/FakePinkBound1Shadow.png')
# bound2 = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test/test-images/FakePinkBound2.png')
# bound22 = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test/test-images/pinktape2.png')
# intnotstring = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test/test-images/pinktape3shadow.png')
# getPattern(intnotstring)
# print(getPattern(ins))
# print(getBoundary(bound1))
# print(getBoundary(bound1shadow))
# print(getBoundary(bound2))
# print(getPatternAndOrientation(bound0))
# print(getBoundary(bound22))
# ic2 = cv2.imread('test/test-images/IndexCrash2.png')
# ic3 = cv2.imread('test/test-images/IndexCrash3.png')
# print(getPattern(ic2))
# print(getOrientation(ic2))
# print(getPattern(ic3))
# print(getOrientation(ic3))
