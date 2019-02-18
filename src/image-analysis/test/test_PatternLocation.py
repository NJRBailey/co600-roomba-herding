from os import getcwd
import PatternLocation
from cv2 import imread

getBoundary = PatternLocation.getBoundary
getPattern = PatternLocation.getPattern
getOrientation = PatternLocation.getOrientation
getPAO = PatternLocation.getPatternAndOrientation
getDFB = PatternLocation.getDistanceFromBoundary


def test_GetBoundary_Right():
    right = imread(getcwd() + '/test-images/FakePinkBound1.png')
    assert getBoundary(right) == ['r']
def test_GetBoundary_TopRight():
    topRight = imread(getcwd() + '/test-images/pinktape2.png')
    assert getBoundary(topRight) == ['r', 't']
def test_GetBoundary_Shadowed():
    shadow = imread(getcwd() + '/test-images/FakePinkBound1Shadow.png')
    assert getBoundary(shadow) == ['r']
def test_GetBoundary_None():
    none = imread(getcwd() + '/test-images/IndexCrash3.png')
    assert getBoundary(none) is None
def test_GetBoundary_Blank():
    pureGrey = imread(getcwd() + '/test-images/PureGrey.png')
    assert getBoundary(pureGrey) is None


def test_GetPatternRoomba_Perfect():
    roombaPerfect = imread(getcwd() + '/test-images/RoombaBoxesInvertTight.png')
    assert getPattern(roombaPerfect) == {'roombaPosition': 'c', 'penPosition': None}
def test_GetPatternRoomba_Diamond():
    roombaDiamond = imread(getcwd() + '/test-images/RoombaBoxesInvertTight45.png')
    assert getPattern(roombaDiamond) == {'roombaPosition': 'c', 'penPosition': None}
def test_GetPatternRoomba_Simulator():
    roombaSimLight = imread(getcwd() + '/test-images/IndexCrash2.png')
    assert getPattern(roombaSimLight) == {'roombaPosition': 'c', 'penPosition': None}
def test_GetPatternPen_Perfect():
    penPerfect = imread(getcwd() + '/test-images/PenBoxesInvertTight.png')
    assert getPattern(penPerfect) == {'roombaPosition': None, 'penPosition': 'c'}
def test_GetPatternPen_Simulator():
    penSimLight = imread(getcwd() + '/test-images/IndexCrash3.png')
    assert getPattern(penSimLight) == {'roombaPosition': None, 'penPosition': 'c'}
def test_GetPatternPen_SimulatorShadowed():
    penSimShadow = imread(getcwd() + '/test-images/IndexCrash3Shadowed.png')
    assert getPattern(penSimShadow) == {'roombaPosition': None, 'penPosition': 'c'}
def test_GetPatternPen_SimulatorHalfShadowed():
    penSimHalfShadow = imread(getcwd() + '/test-images/IndexCrash3PenPartShadow.png')
    assert getPattern(penSimHalfShadow) == {'roombaPosition': None, 'penPosition': 'c'}
def test_GetPatternBoth_Simulator():
    bothSim = imread(getcwd() + '/test-images/SimulatorFakeTwoPatterns.png')
    assert getPattern(bothSim) == {'roombaPosition': 't', 'penPosition': 'c'}
def test_GetPattern_None():
    noPattern = imread(getcwd() + '/test-images/pinktape3shadow.png')
    assert getPattern(noPattern) == {'roombaPosition': None, 'penPosition': None}
def test_GetPattern_Blank():
    pureGrey = imread(getcwd() + '/test-images/PureGrey.png')
    assert getPattern(pureGrey) == {'roombaPosition': None, 'penPosition': None}


def test_GetOrientation_Perfect():
    roombaPerfect = imread(getcwd() + '/test-images/RoombaBoxesInvertTight.png')
    assert getOrientation(roombaPerfect) == 0
def test_getOrientation_Perfect90():
    roombaPerfect90 = imread(getcwd() + '/test-images/RoombaBoxesInvertTight90.png')
    assert getOrientation(roombaPerfect90) == 90
def test_getOrientation_Perfect180():
    roombaPerfect180 = imread(getcwd() + '/test-images/RoombaBoxesInvertTight180.png')
    assert getOrientation(roombaPerfect180) == 180
def test_getOrientation_Perfect270():
    roombaPerfect270 = imread(getcwd() + '/test-images/RoombaBoxesInvertTight270.png')
    assert getOrientation(roombaPerfect270) == 270
def test_getOrientation_Perfect45():
    roombaPerfect45 = imread(getcwd() + '/test-images/RoombaBoxesInvertTight45.png')
    assert getOrientation(roombaPerfect45) == 45
def test_getOrientation_Perfect265():
    roombaPerfect265 = imread(getcwd() + '/test-images/RoombaBoxesInvertTight2xx.png')
    assert getOrientation(roombaPerfect265) == 265
def test_GetOrientation_Simulator():
    roombaSimLight = imread(getcwd() + '/test-images/IndexCrash2.png')
    assert getOrientation(roombaSimLight) == 84
def test_GetOrientation_SimulatorTwoPatterns():
    twoPatternsSim = imread(getcwd() + '/test-images/SimulatorFakeTwoPatterns.png')
    assert getOrientation(twoPatternsSim) == 84
def test_GetOrientation_None():
    noPattern = imread(getcwd() + '/test-images/pinktape3shadow.png')
    assert getOrientation(noPattern) is None
def test_GetOrientation_Blank():
    pureGrey = imread(getcwd() + '/test-images/PureGrey.png')
    assert getOrientation(pureGrey) is None


def test_GetPatternAndOrientation_TwoPatterns():
    twoPatterns = imread(getcwd() + '/test-images/SimulatorFakeTwoPatterns.png')
    assert getPAO(twoPatterns) == {'roomba': {'location': 't', 'orientation': 84}, 'pen': {'location': 'c'}}
def test_GetPatternAndOrientation_RoombaSimulator():
    roombaSim = imread(getcwd() + '/test-images/IndexCrash2.png')
    assert getPAO(roombaSim) == {'roomba': {'location': 'c', 'orientation': 84}, 'pen': None}
def test_GetPatternAndOrientation_PenSimulator():
    penSim = imread(getcwd() + '/test-images/IndexCrash3.png')
    assert getPAO(penSim) == {'roomba': None, 'pen': {'location': 'c'}}
def test_GetPatternAndOrientation_None():
    roombaSim = imread(getcwd() + '/test-images/pinktape3shadow.png')
    assert getPAO(roombaSim) == {'roomba': None, 'pen': None}
def test_GetPatternAndOrientation_Blank():
    roombaSim = imread(getcwd() + '/test-images/PureGrey.png')
    assert getPAO(roombaSim) == {'roomba': None, 'pen': None}


def test_GetDistanceFromBoundary_TopLeft():
    topLeftSim = imread(getcwd() + '/test-images/FakeSimDistance.png')
    assert getDFB(topLeftSim) == {'l': 1393, 't': 653, 'r': None, 'b': None}
def test_GetDistanceFromBoundary_OneAxisChange():
    beforeTranslation = imread(getcwd() + '/test-images/FakeSimDistance2.png')
    afterTranslation = imread(getcwd() + '/test-images/FakeSimDistance3.png')
    assert getDFB(beforeTranslation)['l'] > getDFB(afterTranslation)['l']
    assert getDFB(beforeTranslation)['b'] == getDFB(afterTranslation)['b']
def test_GetDistanceFromBoundary_TrueSimulatorBottom():
    trueSimBottom = imread(getcwd() + '/test-images/DistanceSim1.png')
    assert getDFB(trueSimBottom) == {'l': None, 't': None, 'r': None, 'b': 1721}
def test_GetDistanceFromBoundary_TrueSimulatorLeft():
    trueSimLeft = imread(getcwd() + '/test-images/DistanceSim2.png')
    print(getPattern(trueSimLeft))
    assert getDFB(trueSimLeft) == {'l': 2108, 't': None, 'r': None, 'b': None}
