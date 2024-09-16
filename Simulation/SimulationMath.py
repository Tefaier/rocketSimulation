import numpy as np
import quaternion as quat
from scipy.spatial.transform import Rotation

from Simulation.Entity import SimulationEntity

noRotation = Rotation.from_euler('x', 0)

def distanceBetweenObjects(obj1: SimulationEntity, obj2: SimulationEntity):
    return np.linalg.norm(obj1.position - obj2.position)

def angleBetweenVectors(vec1: Rotation, vec2: Rotation):
    vec1 = vec1.as_rotvec()
    vec2 = vec2.as_rotvec()
    scalarProduct = sum(np.multiply(vec1, vec2))
    
    vec1Norm = np.linalg.norm(vec1)
    vec2Norm = np.linalg.norm(vec2)
    angleRadians = np.acos(scalarProduct / (vec1Norm * vec2Norm))
    return angleRadians
