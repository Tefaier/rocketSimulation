import numpy as np
import quaternion as quat
from scipy.spatial.transform import Rotation

from Simulation.Entity import SimulationEntity

noRotation = Rotation.from_euler('x', 0)

def distanceBetweenObjects(obj1: SimulationEntity, obj2: SimulationEntity):
    return np.linalg.norm(obj1.position - obj2.position)