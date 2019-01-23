import cv2
from RoombaPenReader import decode
from ImageAnalysisUtils import calculateLength

# PatternLocation
# Find the location of any identified patterns in one of nine locations on the screen.


# Checks to see if a coordinate falls within a box.
def coordInBounds(coordinate, boundingBox):
    # Check with a < x < b and c < y < d
    if (
        coordinate[0] > boundingBox[1][0] and coordinate[0] < boundingBox[2][0] and
        coordinate[1] > boundingBox[1][1] and coordinate[1] < boundingBox[2][1]
    ):
        return True
    return False


# Divides the frame into 9 sections
def createFrameSections(frame):
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
    return sections


# Finds the location of a pattern on the screen
def getPattern(frame, debug=False):
    # frame = cv2.flip(camFrame, 0)
    if debug:
        cv2.imshow('frame', frame)
        cv2.waitKey(1)
    sections = createFrameSections(frame)
    # QR Code detection
    decoded = decode(frame)['identified']
    patterns = {'roombaPosition': None, 'penPosition': None}
    for code in decoded:
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


# Returns the orientation of the Roomba in degrees relative to the top of the frame
def getOrientation(frame):
    decoded = decode(frame)['identified']
    for code in decoded:
        if code['id'] == 'roomba':
            return code['orientation']
    return None


# Searches for the arena boundary and returns a list containing T, L, R or B, or None.
def getBoundary(frame):
    sections = createFrameSections(frame)
    cardinalSections = [sections[1], sections[3], sections[5], sections[7]]
    boundarySections = []
    for section in cardinalSections:
        width = section[2][0]
        height = section[2][1]
        x = section[1][0]
        y = section[1][1]
        pinkPixels = False
        # Sample pixels along a diagonal line
        while y < height and x < width and pinkPixels is False:
            region = frame[y][x].tolist()
            # Gazebo shading means unshadowed pink bound can be between (155, 0, 155) and (103, 38, 103)
            if region == [80, 0, 80] or (103 <= region[0] <= 155 and region[1] <= 38 and 103 <= region[2] <= 155):
                pinkPixels = True
                boundarySections.append(section[0])
            y += 1
            x += 1
    if boundarySections:
        return boundarySections
    return None


# ins = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/intnotstr.png')
# bound1 = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/FakePinkBound1.png')
# bound0 = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/IndexCrash3.png')
# bound1shadow = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/FakePinkBound1Shadow.png')
# bound2 = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/FakePinkBound2.png')
# bound22 = cv2.imread('C:/Users/Nicholas/Desktop/CO600/Git/co600-roomba-herding/src/image-analysis/test-images/pinktape2.png')
# print(getPattern(ins))
# print(getBoundary(bound1))
# print(getBoundary(bound1shadow))
# print(getBoundary(bound2))
# print(getBoundary(bound0))
# print(getBoundary(bound22))
# ic2 = cv2.imread('test-images/IndexCrash2.png')
# ic3 = cv2.imread('test-images/IndexCrash3.png')
# print(getPattern(ic2))
# print(getOrientation(ic2))
# print(getPattern(ic3))
# print(getOrientation(ic3))
