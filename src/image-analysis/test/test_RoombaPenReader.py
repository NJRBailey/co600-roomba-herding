from os import getcwd
import RoombaPenReader
from cv2 import imread
import pytest

findMid = RoombaPenReader.findMidpoint
findLE = RoombaPenReader.findLineEquation
genBR = RoombaPenReader.generateBoxRegion
countLCB = RoombaPenReader.countLineCrossesBoxes


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


def test_identifyBounds():
    assert None is None


def test_findLineEquation():
    assert None is None


def test_findOrientation():
    assert None is None


def test_identifyPattern():
    assert None is None


def test_decode():
    assert None is None
