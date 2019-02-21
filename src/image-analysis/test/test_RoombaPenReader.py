from os import getcwd
import RoombaPenReader
from cv2 import imread
import pytest

findMid = RoombaPenReader.findMidpoint
findLE = RoombaPenReader.findLineEquation
genBR = RoombaPenReader.generateBoxRegion
countLCB = RoombaPenReader.countLineCrossesBoxes
identB = RoombaPenReader.identifyBounds
findO = RoombaPenReader.findOrientation
identP = RoombaPenReader.identifyPattern
decode = RoombaPenReader.decode


def test_FindMidpoint_Diagonal():
    assert findMid((5, 5), (9, 9)) == (7, 7)
def test_FindMidpoint_Horizontal():
    assert findMid((5, 5), (9, 5)) == (7, 5)
def test_FindMidpoint_Vertical():
    assert findMid((5, 5), (5, 9)) == (5, 7)
def test_FindMidpoint_Same():
    assert findMid((5, 5), (5, 5)) == (5, 5)
def test_FindMidpoint_Rounding():
    assert findMid((5, 5), (10, 10)) == (7, 7)  # Default Python int division is used for efficiency
def test_FindMidpoint_Invalid():
    with pytest.raises(TypeError) as errorInfo:
        findMid((5, 'a'), (9, 9))


def test_FindLineEquation_Diagonal():
    assert findLE((5, 5), (9, 9)) == {'m': 1, 'c': 0, 'point1': (5, 5), 'point2': (9, 9)}
def test_FindLineEquation_Horizontal():
    assert findLE((5, 5), (9, 5)) == {'m': 0, 'c': 5, 'point1': (5, 5), 'point2': (9, 5)}
def test_FindLineEquation_Vertical():
    assert findLE((5, 5), (5, 9)) == {'m': float('inf'), 'c': -float('inf'), 'point1': (5, 5), 'point2': (5, 9)}
def test_FindLineEquation_Same():
    assert findLE((5, 5), (5, 5)) == {'m': float('inf'), 'c': -float('inf'), 'point1': (5, 5), 'point2': (5, 5)}
def test_FindLineEquation_Invalid():
    with pytest.raises(TypeError) as errorInfo:
        findLE((5, 'a'), (9, 9))


def test_GenerateBoxRegion_Normal():
    tol = RoombaPenReader.REGION_TOLERANCE
    output = genBR({'centre': (100, 100)})
    assert output[0] == (100 - tol, 100 - tol)
    assert output[len(output) - 1] == (100 + tol, 100 + tol)
def test_GenerateBoxRegion_TopLeft():
    tol = RoombaPenReader.REGION_TOLERANCE
    output = genBR({'centre': (0, 0)})
    assert output[0] == (0, 0)
    assert output[len(output) - 1] == (tol, tol)


def test_CountLineCrossesBoxes_Perfect3():
    line = {'point1': (5, 5), 'm': 1, 'c': 0}
    boxes = [{'centre': (5, 5)}, {'centre': (7, 7)}, {'centre': (9, 9)}]
    assert countLCB(line, boxes) == 3
def test_CountLineCrossesBoxes_Perfect2():
    line = {'point1': (5, 5), 'm': 1, 'c': 0}
    boxes = [{'centre': (5, 5)}, {'centre': (9, 9)}]
    assert countLCB(line, boxes) == 2
def test_CountLineCrossesBoxes_Mixed2():
    line = {'point1': (5, 5), 'm': 1, 'c': 0}
    boxes = [{'centre': (5, 5)}, {'centre': (0, 100)}, {'centre': (9, 9)}]
    assert countLCB(line, boxes) == 2
def test_CountLineCrossesBoxes_1():
    line = {'point1': (5, 5), 'm': 1, 'c': 0}
    boxes = [{'centre': (5, 5)}]
    assert countLCB(line, boxes) == 1
def test_CountLineCrossesBoxes_Horizontal():
    line = {'point1': (5, 5), 'm': 0, 'c': 5}
    boxes = [{'centre': (5, 5)}, {'centre': (7, 5)}, {'centre': (9, 5)}]
    assert countLCB(line, boxes) == 3
def test_CountLineCrossesBoxes_Vertical():
    line = {'point1': (5, 5), 'm': float('inf'), 'c': -float('inf')}
    boxes = [{'centre': (5, 5)}, {'centre': (5, 7)}, {'centre': (5, 9)}]
    assert countLCB(line, boxes) == 3
def test_CountLineCrossesBoxes_TolerancePlus():
    tol = RoombaPenReader.REGION_TOLERANCE
    line = {'point1': (5, 5), 'm': 1, 'c': 0}
    boxes = [{'centre': (5, 5)}, {'centre': (9 + tol, 9 + tol)}]
    assert countLCB(line, boxes) == 2
def test_CountLineCrossesBoxes_ToleranceMinus():
    tol = RoombaPenReader.REGION_TOLERANCE
    line = {'point1': (5, 5), 'm': 1, 'c': 0}
    boxes = [{'centre': (5, 5)}, {'centre': (9 - tol, 9 - tol)}]
    assert countLCB(line, boxes) == 2
def test_CountLineCrossesBoxes_ToleranceMixed():
    tol = RoombaPenReader.REGION_TOLERANCE
    line = {'point1': (5, 5), 'm': 1, 'c': 0}
    boxes = [{'centre': (5, 5)}, {'centre': (9 + tol, 9 - tol)}]
    assert countLCB(line, boxes) == 1  # Should not find 2nd box


def test_identifyBounds_RoombaPerfect():
    boxes = [
        {'centre': (10, 10), 'corners': [(7, 7), (7, 13), (13, 7), (13, 13)]},
        {'centre': (30, 10), 'corners': [(27, 7), (27, 13), (33, 7), (33, 13)]},
        {'centre': (20, 20), 'corners': [(17, 17), (17, 23), (23, 17), (23, 23)]},
        {'centre': (10, 30), 'corners': [(7, 27), (7, 33), (13, 27), (13, 33)]},
        {'centre': (20, 30), 'corners': [(17, 27), (17, 33), (23, 27), (23, 33)]},
        {'centre': (30, 30), 'corners': [(27, 27), (27, 33), (33, 27), (33, 33)]},
    ]
    assert set(identB(boxes)) == set([(7, 7), (33, 7), (7, 33), (33, 33)])  # Cast as sets to ignore order
def test_identifyBounds_RoombaDiamond():
    boxes = [
        {'centre': (110, 40), 'corners': [(107, 40), (110, 37), (113, 40), (110, 43)]},
        {'centre': (40, 110), 'corners': [(37, 110), (40, 107), (43, 110), (40, 113)]},
        {'centre': (110, 110), 'corners': [(107, 110), (110, 107), (113, 110), (110, 113)]},
        {'centre': (180, 110), 'corners': [(177, 110), (180, 107), (183, 110), (180, 113)]},
        {'centre': (140, 140), 'corners': [(137, 140), (140, 137), (143, 140), (140, 143)]},
        {'centre': (110, 180), 'corners': [(107, 180), (110, 177), (113, 180), (110, 183)]},
    ]
    assert set(identB(boxes)) == set([(110, 37), (37, 110), (183, 110), (110, 183)])  # Cast as sets to ignore order
# def test_identifyBounds_RoombaSkew():  TODO
#     boxes = [
#         {'centre': (10, 10), 'corners': [(7, 7), (7, 13), (13, 7), (13, 13)]},
#         {'centre': (30, 10), 'corners': [(27, 7), (27, 13), (33, 7), (33, 13)]},
#         {'centre': (20, 20), 'corners': [(17, 17), (17, 23), (23, 17), (23, 23)]},
#         {'centre': (10, 30), 'corners': [(7, 27), (7, 33), (13, 27), (13, 33)]},
#         {'centre': (20, 30), 'corners': [(17, 27), (17, 33), (23, 27), (23, 33)]},
#         {'centre': (30, 30), 'corners': [(27, 27), (27, 33), (33, 27), (33, 33)]},
#     ]
#     assert set(identB(boxes)) == set([(7, 7), (33, 7), (7, 33), (33, 33)])  # Cast as sets to ignore order
def test_identifyBounds_PenPerfect():
    boxes = [
        {'centre': (10, 10), 'corners': [(7, 7), (7, 13), (13, 7), (13, 13)]},
        {'centre': (30, 10), 'corners': [(27, 7), (27, 13), (33, 7), (33, 13)]},
        {'centre': (10, 20), 'corners': [(7, 17), (7, 23), (13, 17), (13, 23)]},
        {'centre': (10, 30), 'corners': [(7, 27), (7, 33), (13, 27), (13, 33)]},
        {'centre': (20, 30), 'corners': [(17, 27), (17, 33), (23, 27), (23, 33)]},
        {'centre': (30, 30), 'corners': [(27, 27), (27, 33), (33, 27), (33, 33)]},
    ]
    assert set(identB(boxes)) == set([(7, 7), (33, 7), (7, 33), (33, 33)])  # Cast as sets to ignore order
def test_identifyBounds_PenDiamond():
    boxes = [
        {'centre': (110, 40), 'corners': [(107, 40), (110, 37), (113, 40), (110, 43)]},
        {'centre': (40, 110), 'corners': [(37, 110), (40, 107), (43, 110), (40, 113)]},
        {'centre': (140, 75), 'corners': [(137, 75), (140, 72), (143, 75), (140, 78)]},
        {'centre': (180, 110), 'corners': [(177, 110), (180, 107), (183, 110), (180, 113)]},
        {'centre': (140, 140), 'corners': [(137, 140), (140, 137), (143, 140), (140, 143)]},
        {'centre': (110, 180), 'corners': [(107, 180), (110, 177), (113, 180), (110, 183)]},
    ]
    assert set(identB(boxes)) == set([(110, 37), (37, 110), (183, 110), (110, 183)])  # Cast as sets to ignore order
# def test_identifyBounds_PenSkew():  TODO
#     boxes = [
#
#     ]
#     assert set(identB(boxes)) == set([(7, 7), (33, 7), (7, 33), (33, 33)])  # Cast as sets to ignore order

def test_findOrientation_RoombaPerfect():
    boxes = [
        {'centre': (10, 10)}, {'centre': (30, 10)}, {'centre': (20, 20)},
        {'centre': (10, 30)}, {'centre': (20, 30)}, {'centre': (30, 30)}
    ]
    outerBoxes = [
        {'centre': (10, 10)}, {'centre': (30, 10)}, {'centre': (10, 30)}, {'centre': (30, 30)}
    ]
    frame = imread(getcwd() + '/test-images/RoombaBoxesInvertTight.png')
    assert findO(boxes, outerBoxes, 'roomba', frame) == 0
def test_findOrientation_RoombaDiamond():
    boxes = [
        {'centre': (110, 40)}, {'centre': (40, 110)}, {'centre': (110, 110)},
        {'centre': (180, 110)}, {'centre': (140, 75)}, {'centre': (110, 180)}
    ]
    outerBoxes = [
        {'centre': (110, 40)}, {'centre': (40, 110)}, {'centre': (180, 110)}, {'centre': (110, 180)}
    ]
    frame = imread(getcwd() + '/test-images/RoombaBoxesInvertTight45.png')
    assert findO(boxes, outerBoxes, 'roomba', frame) == 45
# def test_findOrientation_RoombaSkew():  TODO
#     boxes = [
#         {'centre': (110, 40)}, {'centre': (40, 110)}, {'centre': (110, 110)},
#         {'centre': (180, 110)}, {'centre': (140, 75)}, {'centre': (110, 180)}
#     ]
#     outerBoxes = [
#         {'centre': (110, 40)}, {'centre': (40, 110)}, {'centre': (180, 110)}, {'centre': (110, 180)}
#     ]
#     frame = imread(getcwd() + '/test-images/RoombaBoxesInvertTight45.png')
#     assert findO(boxes, outerBoxes, 'roomba', frame) == 45


def test_identifyPattern():
    assert None is None


def test_decode():
    frame = imread(getcwd() + '/test-images/RoombaBoxesInvertTight45.png')
    assert decode(frame, True) == -1
