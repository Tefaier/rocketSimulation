import math
from enum import Enum
from typing import List

import numpy as np
from scipy.spatial.transform import Rotation
import numpy.polynomial.polynomial as poly

from Simulation.Entity import SimulationEntity, Rocket
from Simulation.ReferenceValues import rocketName, earthName, sunName, marsName, gravityConstant
from Simulation.SimulationMath import angleBetweenVectors, getPerpendicularVector, vecNormalize, rotationToVector, \
    setRotationAngle, vectorLerp, noRotation, magnitudeOfProjection


class CommandType(Enum):
    gravityTurn = 1
    simpleMove = 2
    hohmannTransfer = 3

class Command:
    type: CommandType
    properties: dict

    def __init__(self, type: CommandType, properties: dict):
        self.type = type
        self.properties = properties

    def executeCommand(self, entities: dict[str, SimulationEntity]) -> bool:
        if self.type == CommandType.gravityTurn:
            # required fields are referenceObject, targetSpeed, maximumDistance, maximumAngleForceToReferenceObject, attackAngleFunction(distance from referenceObject), enforceDirectionRatio(angle of deviation)
            return self.gravityTurnCommand(entities)
        if self.type == CommandType.simpleMove:
            # required fields are force, targetName - may be None
            return self.simpleMoveCommand(entities)
        if self.type == CommandType.hohmannTransfer:
            # required fields are targetObject, orbitAround
            return self.hohmannTransferCommand(entities)

    def gravityTurnCommand(self, entities: dict[str, SimulationEntity]) -> bool:
        refObj = entities[self.properties["referenceObject"]]
        rocket: Rocket = entities[rocketName]  # just ignore
        relativeVelocity = rocket.velocity - refObj.velocity
        # angle to have velocity at from radius vector from referenceObject
        targetAttackAngle = self.properties["attackAngleFunction"](np.linalg.norm(refObj.position - rocket.position))
        # choose direction for force
        normVector: np.array
        attackVector: np.array  # direction in which rocket wants velocity to be in
        # for now it's made so depending on ignoring effect of torque
        # norm = 1
        speedRotateForceDirection: np.array
        decidedDirection: np.array
        if np.linalg.norm(relativeVelocity) < 0.1:
            normVector = vecNormalize(getPerpendicularVector(refObj.position - rocket.position))
            attackVector = Rotation.from_rotvec(normVector * targetAttackAngle).apply(rocket.position - refObj.position)
            decidedDirection = vecNormalize(attackVector)
        else:
            # norm = 1
            normVector = np.cross(refObj.position - rocket.position, rocket.velocity)
            attackVector = vecNormalize(setRotationAngle(
                rotationToVector(
                    rocket.position - refObj.position,
                    relativeVelocity
                ),
                targetAttackAngle
            ).apply(rocket.position - refObj.position))
            speedRotateForceDirection = setRotationAngle(rotationToVector(relativeVelocity, attackVector), 90,
                                                         True).apply(vecNormalize(relativeVelocity))
            decidedDirection = vecNormalize(vectorLerp(
                attackVector,
                speedRotateForceDirection,
                self.properties["enforceDirectionRatio"](angleBetweenVectors(relativeVelocity, attackVector))))

        # force that is needed to acquire maximum possible resultant force in the direction of decidedDirection
        x = rocket.force[0]
        y = rocket.force[1]
        z = rocket.force[2]
        xt = decidedDirection[0]
        yt = decidedDirection[1]
        zt = decidedDirection[2]
        force = rocket.thrusterForceMax
        # equation for where desiredDirection is intersected by circle of to be applied force radius with center in current force point
        roots = poly.polyroots([x*x + y*y + z*z - force**2, -2*(x*xt + y*yt + z*zt), xt*xt + yt*yt + zt*zt])
        forceToApply = max(roots) * decidedDirection - rocket.force
        rocket.changeThrusterConfig(np.linalg.norm(forceToApply), forceToApply)
        print("Relative velocity ",relativeVelocity, "\nRelative position ", rocket.position - refObj.position, "\nDistance ", np.linalg.norm(refObj.position - rocket.position), "\nAttack angle", targetAttackAngle)

        return self.gravityTurnExitCondition(entities)

    def gravityTurnExitCondition(self, entities: dict[str, SimulationEntity]) -> bool:
        refObj = entities[self.properties["referenceObject"]]
        rocket = entities[rocketName]
        if np.linalg.norm(rocket.velocity - refObj.velocity) > self.properties["targetSpeed"]\
                or np.linalg.norm(rocket.position - refObj.position) > self.properties["maximumDistance"]\
                or angleBetweenVectors(rocket.force, refObj.position - rocket.position) > self.properties["maximumAngleForceToReferenceObject"]:
            return True
        return False

    def simpleMoveCommand(self, entities: dict[str, SimulationEntity]) -> bool:
        rocket: Rocket = entities[rocketName]
        rocket.changeThrusterConfig(self.properties["force"], (entities[self.properties["targetName"]].position - rocket.position) if self.properties.get("targetName") is not None else noRotation)

        return self.simpleMoveExitCondition(entities)

    def simpleMoveExitCondition(self, entities: dict[str, SimulationEntity]) -> bool:
        return False

    def hohmannTransferCommand(self, entities: dict[str, SimulationEntity]) -> bool:
        rocket: Rocket = entities[rocketName]
        orbitObject = entities[self.properties["orbitAround"]]
        targetObject = entities[self.properties["targetObject"]]
        relativeVelocity = rocket.velocity - orbitObject.velocity
        if self.properties.get("state") is None:
            self.properties["state"] = 0
            orbitSpeed1 = math.sqrt(gravityConstant * orbitObject.mass / (np.linalg.norm(rocket.position - orbitObject.position)))
            orbitSpeed2 = np.linalg.norm(targetObject.velocity - orbitObject.velocity)
            speedR = math.sqrt((orbitSpeed1 ** 2 + orbitSpeed2 ** 2) / 2)
            self.properties["ellipseSpeed"] = orbitSpeed1 * orbitSpeed1 / speedR

        if self.properties["state"] == 0:
            desiredDirection = vecNormalize(setRotationAngle(
                rotationToVector(
                    rocket.position - orbitObject.position,
                    relativeVelocity
                ),
                np.pi / 2
            ).apply(rocket.position - orbitObject.position))
            achievedTo = magnitudeOfProjection(relativeVelocity, desiredDirection)
            removeTang = math.sqrt(relativeVelocity ** 2 - achievedTo ** 2)
            speedRotateForceDirection = setRotationAngle(rotationToVector(relativeVelocity, desiredDirection), 90,
                                                         True).apply(vecNormalize(relativeVelocity))
            decidedDirection = vecNormalize(vectorLerp(
                desiredDirection,
                speedRotateForceDirection,
                self.properties["enforceDirectionRatio"](angleBetweenVectors(relativeVelocity, desiredDirection))))
        elif self.properties["state"] == 1:
            pass
        elif self.properties["state"] == 2:
            pass

        return self.hohmannTransferExitCondition(entities)

    def hohmannTransferExitCondition(self, entities: dict[str, SimulationEntity]) -> bool:
        return self.properties["state"] == 3