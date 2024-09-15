from typing import List
import numpy as np
import pandas as pd
from Entity import *
import quaternion as quat
from scipy.spatial.transform import Rotation

from Simulation.SimulationMath import distanceBetweenObjects

gravityConstant = 6.67e-11
earthRadius = 6378000
earthMass = 5.972e24
earthPosition = 149e9 * np.array([1, 0, 0])
earthVelocity = 29766 * np.array([0, 1, 0])
earthAxis = np.linalg.norm(np.cross(earthVelocity, earthPosition))
earthRotation = Rotation.from_rotvec(23.5 * np.linalg.norm(np.array(earthPosition)), degrees=True) * Rotation.align_vectors(np.array([0, 0, 1]), earthAxis)[0]
earthRotationSpeed = Rotation.from_rotvec(earthAxis * 360 / (24 * 60 * 60), degrees=True)
sunRadius = 696340000
sunMass = 1.989e30
sunPosition = np.array([0, 0, 0])
marsRadius = 3390000
marsMass = 6.39e23
marsPosition = 229e9 * np.array([1, 0, 0])
marsVelocity = 24000 * np.array([0, 1, 0])
marsAxis = np.linalg.norm(np.cross(marsVelocity, marsPosition))
marsRotation = Rotation.from_rotvec(25.2 * np.linalg.norm(np.array(earthPosition)), degrees=True) * Rotation.align_vectors(np.array([0, 0, 1]), earthAxis)[0]
marsRotationSpeed = Rotation.from_rotvec(marsAxis * 360 / (24 * 60 * 60) / 1.02569, degrees=True)


def startSimulation(timeUnit = pd.Timedelta(minutes = 1)):
    print("Simulation started")
    entities = getSimulationSetup()
    trackedEntities = entities
    collectedData = []
    simulationTime = pd.Timedelta(seconds = 0)
    while True:
        collectedData.append([simulationTime, collectData(trackedEntities)])
        executeFrame(timeUnit, entities)
        simulationTime += timeUnit
        if checkExitCondition(simulationTime, entities, collectedData): break
    print("Simulation ended")

def executeFrame(frameTime: pd.Timedelta, entities: List[SimulationEntity]):
    def calculateForces():
        for obj in entities:
            obj.clearForces()
        for index1 in range(0, len(entities) - 1):
            for index2 in range(index1 + 1, len(entities)):
                calculateInteraction(entities[index1], entities[index2])

    def applyConstraints():
        for obj in entities:
            obj.applyConstraint()

    def applyChanges():
        for obj in entities:
            acceleration = obj.force / obj.mass
            obj.speed += acceleration * frameTime
            obj.position += obj.speed * frameTime


    calculateForces()
    applyConstraints()
    applyChanges()

def collectData(entities: List[SimulationEntity]) -> list:
    return [entity.getData() for entity in entities]

def checkExitCondition(simulationTime: pd.Timedelta, entities: List[SimulationEntity], data: list) -> bool:
    rocket = next(x for x in entities if x.name == "Rocket")
    earth = next(x for x in entities if x.name == "Earth")
    return len(data) > 1e9 or simulationTime > pd.Timedelta(days=365) or (len(data) > 100 and distanceBetweenObjects(rocket, earth) < earthRadius)


def getSimulationSetup() -> List[SimulationEntity]:
    return [SimulationEntity('Earth', earthMass, None, earthPosition, earthVelocity, earthRotation, earthRotationSpeed)]

def calculateInteraction(obj1: SimulationEntity, obj2: SimulationEntity):
    for force in ForceTypes:
        if (force in obj1.forcesSupported or force in obj2.forcesSupported) and force not in obj1.forcesIgnored and force not in obj2.forcesSupported:
            forceSupplier = obj1 if force in obj1.forcesSupported else obj2
            effect = np.array
            if force is ForceTypes.gravity:
                effect = (obj2.position - obj1.position) * gravityConstant * obj1.mass * obj2.mass / (np.linalg.norm(obj2.position - obj1.position) ** 2)
                obj1.force += effect
                obj2.force -= effect