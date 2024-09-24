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
    radius: float

    position: np.array
    velocity: np.array
    force: np.array

    rotation: Rotation  # rotation from [0, 0, 1]
    rotationSpeed: Rotation  # rotation from [0, 0, 1]
    torque: Rotation  # rotation from [0, 0, 1]

    forcesApplied: list[ForceTypes]
    forcesIgnored: list[ForceTypes]

    def __init__(self, name, mass, volume, position, velocity, rotation, rotationSpeed, forcesApplied = [], forcesIgnored = [], constraintFunction = None, densityFunction = None, radius = None):
        self.name = name
        self.mass = mass
        self.volume = volume
        self.position = position.astype(np.float64)
        self.velocity = velocity.astype(np.float64)
        self.rotation = rotation
        self.rotationSpeed = rotationSpeed
        self.forcesApplied = forcesApplied
        self.forcesIgnored = forcesIgnored
        self.constraint = constraintFunction
        self.density = densityFunction
        self.radius = radius

    def applyAction(self):
        pass

    def applyConstraint(self):
        if self.constraint != None:
            self.constraint(self)

    def getData(self) -> list:
        return [self.name, np.copy(self.position), self.rotation]

    def clearForces(self):
        from Simulation.SimulationMath import noRotation
        self.force = np.array([0, 0, 0], dtype=np.float64)
        self.torque = noRotation

    def getDensityFunction(self):
        return self.density


class Rocket(SimulationEntity):
    thrusterForce: float
    thrusterForceMin: float
    thrusterForceMax: float
    thrusterRotation: np.array  # quaternions
    thrusterRotationMax: float

    distanceThrusterToCenterOfMass: float

    def __init__(self, name, mass, volume, position, velocity, rotation, rotationSpeed, thrusterForce, thrusterForceMin, thrusterForceMax, thrusterRotation, thrusterRotationMax, distanceTTCOM, forcesApplied = [], forcesIgnored = [], constraintFunction = None, densityFunction = None, radius = None):
        super().__init__(name, mass, volume, position, velocity, rotation, rotationSpeed, forcesApplied, forcesIgnored, constraintFunction, densityFunction, radius)
        self.thrusterForce = thrusterForce
        self.thrusterForceMin = thrusterForceMin
        self.thrusterForceMax = thrusterForceMax
        self.thrusterRotation = thrusterRotation
        self.thrusterRotationMax = thrusterRotationMax
        self.distanceThrusterToCenterOfMass = distanceTTCOM

    def changeThrusterConfig(self, thrusterForce: float, forceDirection):
        from Simulation.SimulationMath import vectorUp, rotationToVector, vecNormalize

        self.thrusterForce = max(min(thrusterForce, self.thrusterForceMax), self.thrusterForceMin)
        if type(forceDirection) == Rotation:
            #self.thrusterRotation = forceDirection
            return
        rotationVec = rotationToVector(self.rotation.apply(vectorUp), forceDirection).as_rotvec()
        if np.linalg.norm(rotationVec) == 0:
            from Simulation.SimulationMath import noRotation
            self.thrusterRotation = noRotation
        else:
            rotationVecLimited = rotationVec * min(np.linalg.norm(rotationVec), self.thrusterRotationMax) / np.linalg.norm(rotationVec)
            self.thrusterRotation = Rotation.from_rotvec(rotationVecLimited)

    def applyAction(self):
        self.force += self.thrusterForce * self.thrusterRotation.apply(self.rotation.apply(np.array([0, 0, 1])))


        # due to torque being not fully supported yet, it's made this way
        #vectorToCenterOfMass = self.distanceThrusterToCenterOfMass * (self.rotation).apply(np.array([0, 0, 1]))
        #self.torque = Rotation.from_rotvec(np.cross(self.force, vectorToCenterOfMass) / self.mass)

