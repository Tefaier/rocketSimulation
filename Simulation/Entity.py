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
    velocity: np.array
    force: np.array

    rotation: Rotation  # rotation from [0, 0, 1]
    rotationSpeed: Rotation  # rotation from [0, 0, 1]
    torque: Rotation  # rotation from [0, 0, 1]

    forcesApplied: list[ForceTypes]
    forcesIgnored: list[ForceTypes]

    def __init__(self, name, mass, volume, position, velocity, rotation, rotationSpeed, force = np.array([0, 0, 0], dtype='float64'), forcesApplied = [], forcesIgnored = [], constraintFunction = None, buoyancyFunction = None):
        self.name = name
        self.mass = mass
        self.volume = volume
        self.position = position
        self.velocity = velocity
        self.force = force
        self.rotation = rotation
        self.rotationSpeed = rotationSpeed
        self.forcesApplied = forcesApplied
        self.forcesIgnored = forcesIgnored
        self.constraint = constraintFunction
        self.buoyancy = buoyancyFunction

    def applyAction(self):
        pass

    def applyConstraint(self):
        if self.constraint != None:
            self.constraint(self)

    def getData(self) -> np.array:
        return np.array([self.name, self.position, self.rotation], dtype='object')

    def clearForces(self):
        from Simulation.SimulationMath import noRotation
        force = np.array([0, 0, 0], dtype=np.float64)
        torque = noRotation

class Rocket(SimulationEntity):
    thrusterForce: float
    thrusterForceMin: float
    thrusterForceMax: float
    thrusterRotation: np.array  # quaternions
    thrusterRotationMax: float

    def __init__(self, name, mass, volume, position, velocity, rotation, rotationSpeed, thrusterForce, thrusterForceMin, thrusterForceMax, thrusterRotation, thrusterRotationMax, forcesApplied = [], forcesIgnored = [], constraintFunction = None, buoyancyFunction = None):
        super().__init__(name, mass, volume, position, velocity, rotation, rotationSpeed, forcesApplied, forcesIgnored, constraintFunction, buoyancyFunction)
        self.thrusterForce = thrusterForce
        self.thrusterForceMin = thrusterForceMin
        self.thrusterForceMax = thrusterForceMax
        self.thrusterRotation = thrusterRotation
        self.thrusterRotationMax = thrusterRotationMax

    def changeThrusterConfig(self, thrusterForce: float, thrusterRotation: Rotation):
        self.thrusterForce = max(min(thrusterForce, self.thrusterForceMax), self.thrusterForceMin)
        # unfinished
        rotationVec = thrusterRotation.as_rotvec()
        rotationVecLimited = rotationVec * min(np.linalg.norm(rotationVec), self.thrusterRotationMax) / np.linalg.norm(rotationVec)
        self.thrusterRotation = Rotation.from_rotvec(rotationVecLimited)

    def applyAction(self):
        self.force += self.thrusterForce * (self.rotation * self.thrusterRotation).apply(np.array([0, 0, 1]))


