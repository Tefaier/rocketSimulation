from enum import Enum
import pandas as pd
import numpy as np

class ForceTypes(Enum):
    GRAVITY = 1,

class SimulationEntity:
    name: str

    position: np.array
    speed: np.array
    force: np.array

    rotation: np.array
    rotationSpeed: np.array
    torque: np.array

    def __init__(self, constraintFunction):
        self.constraint = constraintFunction

    def applyTime(self, time: pd.Timedelta):
        print("none")

    def applyConstraint(self):
        self.constraint(self)

    def getData(self):
        np.array([self.name, self.position, self.rotation])

    def clearForces(self):
        force = np.array([0, 0, 0], dtype=np.float64)
        torque = np.array([0, 0, 0], dtype=np.float64)
