from math import hypot


## Finds and returns the Euclidean distance between two points.
#
# @param point1,point2 A tuple (x,y) for a pixel coordinate.
# @return A Float for the Euclidean distance between the points.
def calculateLength(point1, point2):
    length = hypot(point2[0] - point1[0], point2[1] - point1[1])
    return length
