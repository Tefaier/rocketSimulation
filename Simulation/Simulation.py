from typing import List

import numpy as np
import pandas as pd

from Simulation.CommandLogic import Command, CommandType
from Simulation.Entity import *
import quaternion as quat
from scipy.spatial.transform import Rotation

from Simulation.SimulationMath import distanceBetweenObjects, noRotation, rotationToVector, vectorUp, vecNormalize, rotationToVectorFromBase, earthAtmosphereDensityFunc

gravityConstant = 6.67e-11

earthName = "Earth"
earthRadius = 6378000
earthMass = 5.972e24
earthPosition = 149e9 * np.array([1, 0, 0])
earthVelocity = 29766 * np.array([0, 1, 0])
earthAxis = vecNormalize(np.cross(earthVelocity, earthPosition))
earthRotation = Rotation.from_rotvec(23.5 * vecNormalize(earthPosition), degrees=True) * rotationToVectorFromBase(earthAxis)
earthRotationSpeed = Rotation.from_rotvec(earthAxis * 360 / (24 * 60 * 60), degrees=True)

sunName = "Sun"
sunRadius = 696340000
sunMass = 1.989e30
sunPosition = np.array([0, 0, 0])

marsName = "Mars"
marsRadius = 3390000
marsMass = 6.39e23
marsPosition = 229e9 * np.array([1, 0, 0])
marsVelocity = 24000 * np.array([0, 1, 0])
marsAxis = vecNormalize(np.cross(marsVelocity, marsPosition))
marsRotation = Rotation.from_rotvec(25.2 * vecNormalize(marsPosition), degrees=True) * rotationToVectorFromBase(marsAxis)
marsRotationSpeed = Rotation.from_rotvec(marsAxis * 360 / (24 * 60 * 60) / 1.02569, degrees=True)

# based on https://ru.wikipedia.org/wiki/%D0%9A%D0%BE%D1%81%D0%BC%D0%BE%D1%81_(%D1%81%D0%B5%D0%BC%D0%B5%D0%B9%D1%81%D1%82%D0%B2%D0%BE_%D1%80%D0%B0%D0%BA%D0%B5%D1%82-%D0%BD%D0%BE%D1%81%D0%B8%D1%82%D0%B5%D0%BB%D0%B5%D0%B9)
quaternion = quat.as_float_array(quat.from_spherical_coords(np.deg2rad(90), 0))

rocketName = "Rocket"
rocketMass = 109000
rocketVolume = 146.5
rocketMaxForce = 1486000
rocketVelocity = earthVelocity
rocketRotation = earthRotation * Rotation.from_quat(quaternion)
rocketPosition = earthPosition + (rocketRotation).apply(vectorUp) * earthRadius
rocketRadius = 1.2

# based on https://ru.wikipedia.org/wiki/%D0%9C%D0%B5%D0%B6%D0%B4%D1%83%D0%BD%D0%B0%D1%80%D0%BE%D0%B4%D0%BD%D0%B0%D1%8F_%D0%BA%D0%BE%D1%81%D0%BC%D0%B8%D1%87%D0%B5%D1%81%D0%BA%D0%B0%D1%8F_%D1%81%D1%82%D0%B0%D0%BD%D1%86%D0%B8%D1%8F
mksName = "MKS"
mksMass = 440000
mksFlyHeight = 418000
mksRotationProjected = earthRotation * Rotation.from_quat(quaternion)
mksVelocity = mksRotationProjected.apply(np.array([0, 1, 0])) * 7700 + earthVelocity
mksPosition = earthPosition + (mksRotationProjected).apply(vectorUp) * (earthRadius + mksFlyHeight)


def startSimulation(
        timeUnit = pd.Timedelta(minutes = 1),
        commands: List[Command] = [
            Command(
                CommandType.gravityTurn,
                {"from": earthName}
            )
        ]):
    commands.reverse()

    entities = getSimulationSetup()
    trackedEntities = entities
    entitiesDictionary: dict[str, SimulationEntity] = {}
    for entity in entities:
        entitiesDictionary[entity.name] = entity

    collectedData = []
    simulationTime = pd.Timedelta(seconds = 0)

    print("Simulation started")

    while True:
        collectedData.append([simulationTime, collectData(trackedEntities)])
        executeCommands(commands, entitiesDictionary, simulationTime)
        executeFrame(timeUnit, entities)
        simulationTime += timeUnit
        if checkExitCondition(simulationTime, entities, collectedData): break

    print("Simulation ended")
    return collectedData

def executeCommands(commands: List[Command], entities: dict[str, SimulationEntity], simulationTime: pd.Timedelta):
    if len(commands) == 0: return
    if commands[-1].executeCommand(entities):
        commands.pop()

def executeFrame(frameTime: pd.Timedelta, entities: List[SimulationEntity]):
    def calculateForces():
        for obj in entities:
            obj.clearForces()
        for index1 in range(0, len(entities) - 1):
            for index2 in range(index1 + 1, len(entities)):
                calculateInteraction(entities[index1], entities[index2])

    def applyActions():
        for obj in entities:
            obj.applyAction()

    def applyChanges():
        for obj in entities:
            if obj.mass is not None and obj.mass != 0:
                acceleration = obj.force / obj.mass
                obj.velocity += acceleration * frameTime.seconds
                obj.position += obj.velocity * frameTime.seconds

            obj.rotationSpeed *= Rotation.from_rotvec(obj.torque.as_rotvec() * frameTime.seconds)
            obj.rotation *= Rotation.from_rotvec(obj.rotationSpeed.as_rotvec() * frameTime.seconds)

    def applyConstraints():
        for obj in entities:
            obj.applyConstraint()

    calculateForces()
    applyActions()
    applyChanges()
    applyConstraints()

def collectData(entities: List[SimulationEntity]) -> list:
    return [entity.getData() for entity in entities]

def checkExitCondition(simulationTime: pd.Timedelta, entities: List[SimulationEntity], data: list) -> bool:
    rocket = next(x for x in entities if x.name == rocketName)
    earth = next(x for x in entities if x.name == earthName)
    return len(data) > 1e2 or simulationTime > pd.Timedelta(days=365) or (len(data) > 100 and distanceBetweenObjects(rocket, earth) < earthRadius)

def getSimulationSetup() -> List[SimulationEntity]:
    earth = SimulationEntity(name=earthName, mass=earthMass, volume=None, position=earthPosition, velocity=earthVelocity,
                             rotation=earthRotation, rotationSpeed=earthRotationSpeed, forcesApplied=[ForceTypes.gravity],
                             forcesIgnored=[ForceTypes.buoyancy, ForceTypes.frictionFluid])
    rocket = Rocket(name=rocketName, mass=rocketMass, volume=rocketVolume, position=rocketPosition, velocity=rocketVelocity,
                    rotation=rocketRotation, rotationSpeed=noRotation, thrusterForce=0, thrusterForceMin=0,
                    thrusterForceMax=rocketMaxForce, thrusterRotation=noRotation, thrusterRotationMax=np.deg2rad(10), distanceTTCOM=50,
                    forcesApplied=[ForceTypes.gravity], radius=rocketRadius)
    mks = SimulationEntity(name=mksName, mass=mksMass, volume=None, position=mksPosition, velocity=mksVelocity,
                           rotation=noRotation, rotationSpeed=noRotation, forcesApplied=[ForceTypes.gravity],
                           forcesIgnored=[ForceTypes.frictionFluid])

    def earthAtmosphereConstraint(obj: SimulationEntity):
        obj.position = earth.position
        obj.velocity = earth.velocity

    def earthAtmosphereDensity(obj: SimulationEntity, pos: np.array) -> float:
        height = np.linalg.norm(pos - obj.position) - earthRadius
        return earthAtmosphereDensityFunc(height)

    earthAtmosphere = SimulationEntity(name=earthName+"Atmosphere", mass=None, volume=None, position=earthPosition,
                                       velocity=earthVelocity, rotation=noRotation, rotationSpeed=noRotation,
                                       forcesApplied=[ForceTypes.frictionFluid],
                                       forcesIgnored=[ForceTypes.gravity, ForceTypes.frictionSliding],
                                       constraintFunction=earthAtmosphereConstraint,
                                       densityFunction=earthAtmosphereDensity)
    mars = SimulationEntity(name=marsName, mass=marsMass, volume=None, position=marsPosition, velocity=marsVelocity,
                            rotation=marsRotation, rotationSpeed=marsRotationSpeed, forcesApplied=[ForceTypes.gravity],
                            forcesIgnored=[ForceTypes.buoyancy, ForceTypes.frictionFluid])

    return [earth, rocket, mks, earthAtmosphere, mars]

def calculateInteraction(obj1: SimulationEntity, obj2: SimulationEntity):
    for force in ForceTypes:
        if (force in obj1.forcesApplied or force in obj2.forcesApplied) and force not in obj1.forcesIgnored and force not in obj2.forcesIgnored:
            forceSupplier = obj1 if force in obj1.forcesApplied else obj2
            forceReceiver = obj1 if obj2 == forceSupplier else obj2
            if force is ForceTypes.gravity:
                effect = (obj2.position - obj1.position) * gravityConstant * obj1.mass * obj2.mass / (np.linalg.norm(obj2.position - obj1.position) ** 3)
                obj1.force += effect
                obj2.force -= effect
            if force is ForceTypes.frictionFluid:
                relativeSpeed = np.linalg.norm(obj2.velocity - obj1.velocity)
                density = forceSupplier.getDensityFunction()(forceSupplier, forceReceiver.position)
                if density == 0: continue
                area = np.pi * (forceReceiver.radius ** 2)
                # taken for a sphere
                effect = 0.9 * density * (relativeSpeed ** 2) * 0.5 * area
                forceReceiver.force += effect * vecNormalize(forceSupplier.velocity - forceReceiver.velocity)