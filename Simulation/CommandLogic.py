from enum import Enum
from typing import List

import numpy as np
from scipy.spatial.transform import Rotation

from Simulation.Entity import SimulationEntity
from ReferenceValues import rocketName, earthName, sunName, marsName
from Simulation.SimulationMath import angleBetweenVectors, getPerpendicularVector, vecNormalize, rotationToVector, \
    setRotationAngle


class CommandType(Enum):
    gravityTurn = 1

class Command:
    type: CommandType
    properties: dict

    def __init__(self, type: CommandType, properties: dict):
        self.type = type
        self.properties = properties

    def executeCommand(self, entities: dict[str, SimulationEntity]) -> bool:
        if self.type == CommandType.gravityTurn:
            # required fields are referenceObject, targetSpeed, maximumDistance, maximumAngleForceToReferenceObject, attackAngleFunction
            return self.gravityTurnCommand(entities)

    def gravityTurnCommand(self, entities: dict[str, SimulationEntity]) -> bool:
        refObj = entities[self.properties["referenceObject"]]
        rocket = entities[rocketName]
        # angle to have velocity at from radius vector from referenceObject
        targetAttackAngle = self.properties["attackAngleFunction"](np.linalg.norm(refObj.position - rocket.position))
        # choose direction for force
        normVector: np.array
        forceDirection: np.array
        if np.linalg.norm(rocket.velocity) < 0.1:
            normVector = vecNormalize(getPerpendicularVector(refObj.position - rocket.position))
            forceDirection = Rotation.from_rotvec(normVector * targetAttackAngle).apply(rocket.position - refObj.position)
        else:
            normVector = np.cross(refObj.position - rocket.position, rocket.velocity)
            forceDirection = setRotationAngle(
                rotationToVector(
                    rocket.position - refObj.position,
                    rocket.velocity
                ),
                targetAttackAngle
            ).apply(rocket.position - refObj.position)


        return self.gravityTurnExitCondition(entities)

    def gravityTurnExitCondition(self, entities: dict[str, SimulationEntity]) -> bool:
        refObj = entities[self.properties["referenceObject"]]
        rocket = entities[rocketName]
        if np.linalg.norm(rocket.velocity - refObj.velocity) > self.properties["targetSpeed"]\
                or np.linalg.norm(rocket.position - refObj.position) > self.properties["maximumDistance"]\
                or angleBetweenVectors(rocket.force, refObj.position - rocket.position) > self.properties["maximumAngleForceToReferenceObject"]:
            return True
        return False