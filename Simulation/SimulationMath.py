import numpy as np
import quaternion as quat

from Simulation.Entity import SimulationEntity


def distanceBetweenObjects(obj1: SimulationEntity, obj2: SimulationEntity):
    return np.linalg.norm(obj1.position - obj2.position)