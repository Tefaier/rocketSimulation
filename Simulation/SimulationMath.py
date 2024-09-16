import numpy as np
import quaternion as quat
from scipy.spatial.transform import Rotation

from Simulation.Entity import SimulationEntity

noRotation = Rotation.from_euler('x', 0)
vectorUp = np.array([0, 0, 1])

def distanceBetweenObjects(obj1: SimulationEntity, obj2: SimulationEntity):
    return np.linalg.norm(obj1.position - obj2.position)

def rotationToVector(vecTo: np.array) -> Rotation:
    return rotationToVector(np.array([0, 0, 1]), vecTo)

def vecNormalize(vec: np.array) -> np.array:
    vecNorm = np.linalg.norm(vec)
    return np.divide(vec, np.linalg.norm(vec), where=vecNorm!=0)

# based on https://stackoverflow.com/questions/45142959/calculate-rotation-matrix-to-align-two-vectors-in-3d-space
def rotationToVector(vecFrom: np.array, vecTo: np.array) -> Rotation:
    a = vecNormalize(vecFrom)
    b = vecNormalize(vecTo)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return Rotation.from_matrix(rotation_matrix)