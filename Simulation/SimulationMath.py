import math
from math import degrees

import numpy as np
import quaternion as quat
from scipy.spatial.transform import Rotation

from Simulation.Entity import SimulationEntity

noRotation = Rotation.from_euler('x', 0)
vectorUp = np.array([0, 0, 1])



def vectorLerp(vec1: np.array, vec2: np.array, ratio: float):
    return (1 - ratio) * vec1 + ratio * vec2

def setRotationAngle(rot: Rotation, angle: float, degrees: bool = False) -> Rotation:
    vec = rot.as_rotvec()
    return Rotation.from_rotvec(vecNormalize(vec) * angle, degrees=degrees)

def magnitudeOfProjection(vecToProject: np.array, vecToProjectOn: np.array) -> float:
    len1 = np.linalg.norm(vecToProject)
    len2 = np.linalg.norm(vecToProjectOn)
    cos = np.dot(vecToProject, vecToProjectOn) / (len1 * len2)
    return cos * len1

def angleBetweenVectors(vec1: np.array, vec2: np.array) -> float:
    len1 = np.linalg.norm(vec1)
    len2 = np.linalg.norm(vec2)
    cos = np.dot(vec1, vec2) / (len1 * len2)
    return math.acos(cos)

def distanceBetweenObjects(obj1: SimulationEntity, obj2: SimulationEntity) -> float:
    return np.linalg.norm(obj1.position - obj2.position)

def rotationToVectorFromBase(vecTo: np.array) -> Rotation:
    return rotationToVector(vectorUp, vecTo)

def vecNormalize(vec: np.array) -> np.array:
    vecNorm = np.linalg.norm(vec)
    return np.divide(vec, np.linalg.norm(vec), where=vecNorm!=0)

# returns any perpendicular vector
def getPerpendicularVector(vec: np.array) -> np.array:
    vec2 = np.copy(vec)
    for i in range(np.shape(vec)[0]):
        if vec[i] != 0:
            vec2[0 if i != 0 else 1] += vec[i]
    return np.cross(vec, vec2)

# based on https://stackoverflow.com/questions/45142959/calculate-rotation-matrix-to-align-two-vectors-in-3d-space
def rotationToVector(vecFrom: np.array, vecTo: np.array) -> Rotation:
    a = vecNormalize(vecFrom)
    b = vecNormalize(vecTo)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    if s == 0:
        return noRotation if np.allclose(a, b) else Rotation.from_rotvec(180 * vecNormalize(getPerpendicularVector(a)), degrees=True)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return Rotation.from_matrix(rotation_matrix)

# based on https://ru.wikipedia.org/wiki/%D0%A1%D1%82%D0%B0%D0%BD%D0%B4%D0%B0%D1%80%D1%82%D0%BD%D0%B0%D1%8F_%D0%B0%D1%82%D0%BC%D0%BE%D1%81%D1%84%D0%B5%D1%80%D0%B0
def earthAtmosphereDensityFunc(height: float) -> float:
    if height < 500:
        return 1.1673
    elif height < 1000:
        return 1.1117
    elif height < 1500:
        return 1.0581
    elif height < 2000:
        return 1.0065
    elif height < 2500:
        return 0.9569
    elif height < 3000:
        return 0.9093
    elif height < 4000:
        return 0.8194
    elif height < 5000:
        return 0.7365
    elif height < 6000:
        return 0.6601
    elif height < 7000:
        return 0.59
    elif height < 8000:
        return 0.5258
    elif height < 9000:
        return 0.4671
    elif height < 10000:
        return 0.4135
    elif height < 11000:
        return 0.3648
    elif height < 12000:
        return 0.3119
    elif height < 14000:
        return 0.2279
    elif height < 16000:
        return 0.1665
    elif height < 18000:
        return 0.1216
    elif height < 20000:
        return 0.0889
    elif height < 24000:
        return 0.0469
    elif height < 28000:
        return 0.0251
    elif height < 32000:
        return 0.0136
    elif height < 36000:
        return 7.26e-3
    elif height < 40000:
        return 4.00e-3
    elif height < 50000:
        return 1.03e-3
    elif height < 60000:
        return 3.00e-4
    elif height < 80000:
        return 1.85e-5
    elif height < 100000:
        return 5.55e-7
    else:
        return 0