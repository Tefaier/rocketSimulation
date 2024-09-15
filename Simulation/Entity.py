from enum import Enum
import pandas as pd
import numpy as np
from quaternion import quaternion
from scipy.spatial.transform import Rotation


class ForceTypes(Enum):
    gravity = 1,
    buoyancy = 2,
    frictionSliding = 3,
    frictionFluid = 4

class SimulationEntity:
    name: str
    mass: float
    volume: float

    position: np.array
    speed: np.array
    force: np.array

    rotation: np.array  # quaternions
    rotationSpeed: np.array  # quaternions
    torque: np.array  # quaternions

    forcesSupported: list[ForceTypes]
    forcesIgnored: list[ForceTypes]

    def __init__(self, name, mass, volume, position, speed, rotation, rotationSpeed, constraintFunction = None, buoyancyFunction = None):
        self.name = name
        self.mass = mass
        self.volume = volume
        self.position = position
        self.speed = speed
        self.rotation = rotation
        self.rotationSpeed = rotationSpeed
        self.constraint = constraintFunction
        self.buoyancy = buoyancyFunction

    def applyConstraint(self):
        if self.constraint != None:
            self.constraint(self)

    def getData(self):
        np.array([self.name, self.position, self.rotation])

    def clearForces(self):
        force = np.array([0, 0, 0], dtype=np.float64)
        torque = np.array([0, 0, 0, 0], dtype=np.float64)

class Rocket(SimulationEntity):
    thrusterForce: float
    thrusterForceMin: float
    thrusterForceMax: float
    thrusterRotation: np.array  # quaternions
    thrusterRotationMax: float

    def __init__(self, name, mass, volume, position, speed, rotation, rotationSpeed, thrusterForce, thrusterForceMin, thrusterForceMax, thrusterRotation, thrusterRotationMax, constraintFunction = None, buoyancyFunction = None):
        super().__init__(name, mass, volume, position, speed, rotation, rotationSpeed, constraintFunction, buoyancyFunction)
        self.thrusterForce = thrusterForce
        self.thrusterForceMin = thrusterForceMin
        self.thrusterForceMax = thrusterRotationMax
        self.thrusterRotation = thrusterRotation
        self.thrusterRotationMax = thrusterRotationMax

    def changeThrusterConfig(self, thrusterForce: float, thrusterRotation: np.array):
        self.thrusterForce = max(min(thrusterForce, self.thrusterForceMax), self.thrusterForceMin)
        # unfinished
        Rotation.as_rotvec()
        rotationFromVertical = np.array(np.sin(thrusterRotation[0])*np.sin(thrusterRotation[0]))

    def applyConstraint(self):
        super().applyConstraint()
        self.force += self.thrusterForce * (self.rotation * self.thrusterRotation).apply(np.array([0, 0, 1]))

