from math import hypot


# Finds and returns the distance between two points
def calculateLength(point1, point2):
    length = hypot(point2[0] - point1[0], point2[1] - point1[1])
    return length
