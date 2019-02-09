from os import getcwd
import PatternLocation
from cv2 import imread

getBoundary = PatternLocation.getBoundary
getPattern = PatternLocation.getPattern
getOrientation = PatternLocation.getOrientation


def test_GetBoundaryRight():
    right = imread(getcwd() + '/test-images/FakePinkBound1.png')
    assert getBoundary(right) == ['r']
def test_GetBoundaryTopRight():
    topRight = imread(getcwd() + '/test-images/pinktape2.png')
    assert getBoundary(topRight) == ['r', 't']
def test_GetBoundaryShadowed():
    shadow = imread(getcwd() + '/test-images/FakePinkBound1Shadow.png')
    assert getBoundary(shadow) == ['r']
def test_GetBoundaryNone():
    none = imread(getcwd() + '/test-images/IndexCrash3.png')
    assert getBoundary(none) is None
def test_GetBoundaryBlank():
    pureGrey = imread(getcwd() + '/test-images/PureGrey.png')
    assert getBoundary(pureGrey) is None


def test_GetPatternRoombaPerfect():
    roombaPerfect = imread(getcwd() + '/test-images/RoombaBoxesInvertTight.png')
    assert getPattern(roombaPerfect) == {'roombaPosition': 'c', 'penPosition': None}
def test_GetPatternRoombaSimulator():
    roombaSimLight = imread(getcwd() + '/test-images/IndexCrash2.png')
    assert getPattern(roombaSimLight) == {'roombaPosition': 'c', 'penPosition': None}
def test_GetPatternPenPerfect():
    penPerfect = imread(getcwd() + '/test-images/PenBoxesInvertTight.png')
    assert getPattern(penPerfect) == {'roombaPosition': None, 'penPosition': 'c'}
def test_GetPatternPenSimulator():
    penSimLight = imread(getcwd() + '/test-images/IndexCrash3.png')
    assert getPattern(penSimLight) == {'roombaPosition': None, 'penPosition': 'c'}
def test_GetPatternPenSimulatorShadowed():
    penSimShadow = imread(getcwd() + '/test-images/IndexCrash3Shadowed.png')
    assert getPattern(penSimShadow) == {'roombaPosition': None, 'penPosition': 'c'}
def test_GetPatternPenSimulatorHalfShadowed():
    penSimHalfShadow = imread(getcwd() + '/test-images/IndexCrash3PenPartShadow.png')
    assert getPattern(penSimHalfShadow) == {'roombaPosition': None, 'penPosition': 'c'}
def test_GetPatternNone():
    noPattern = imread(getcwd() + '/test-images/pinktape3shadow.png')
    assert getPattern(noPattern) == {'roombaPosition': None, 'penPosition': None}
def test_GetPatternBlank():
    pureGrey = imread(getcwd() + '/test-images/PureGrey.png')
    assert getPattern(pureGrey) == {'roombaPosition': None, 'penPosition': None}


def test_GetOrientationPerfect():
    roombaPerfect = imread(getcwd() + '/test-images/RoombaBoxesInvertTight.png')
    assert getOrientation(roombaPerfect) == 0
def test_GetOrientationSimulator():
    roombaSimLight = imread(getcwd() + '/test-images/IndexCrash2.png')
    assert getOrientation(roombaSimLight) == 84
def test_GetOrientationNone():
    noPattern = imread(getcwd() + '/test-images/pinktape3shadow.png')
    assert getOrientation(noPattern) is None
def test_GetOrientationBlank():
    pureGrey = imread(getcwd() + '/test-images/PureGrey.png')
    assert getOrientation(pureGrey) is None
