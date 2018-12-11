import cv2
from BoxesChecker import decode
from ImageAnalysisUtils import calculateLength


def coordInBounds(coordinate, boundingBox):
    # Check with a < x < b and c < y < d
    if (
        coordinate[0] > boundingBox[1][0] and coordinate[0] < boundingBox[2][0] and
        coordinate[1] > boundingBox[1][1] and coordinate[1] < boundingBox[2][1]
    ):
        return True

    return False


def findPatternLocation(camFrame, debug=False):
    frame = cv2.flip(camFrame, 0)

    if debug:
        cv2.imshow('frame', frame)
        cv2.waitKey(300)

    sections = []
    height, width, _ = frame.shape

    # Prep
    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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

    # QR Code detection
    decoded = decode(grayFrame)['identified']

    for code in decoded:
        if code['id'] in ['roomba', 'pen', 'unknown']:
            # Highlight four corners
            print(code)
            for point in code['polygon']:
                cv2.circle(frame, (point[0], point[1]), 5, (255, 0, 255), -1)

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
                    return sections[j][0]

                j += 1

    return 'search'
