from typing import List

import numpy as np

from Simulation.ReferenceValues import *
from Simulation.CommandLogic import Command, CommandType
from Simulation.Entity import *
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d

from Simulation.SimulationMath import distanceBetweenObjects, noRotation, rotationToVector, vectorUp, vecNormalize, rotationToVectorFromBase, earthAtmosphereDensityFunc

def startSimulation(
        timeUnit = pd.Timedelta(seconds = 1),
        commands=None):
    if commands is None:
        commands = [
            Command(
                CommandType.gravityTurn,
                {
                    "referenceObject": earthName,
                    "targetSpeed": 20000,
                    "maximumDistance": earthRadius + 1e7,
                    "maximumAngleForceToReferenceObject": np.pi * 0.2,
                    "attackAngleFunction": interp1d(
                        x=[0, earthRadius, earthRadius + 2e3, earthRadius + 1e4, earthRadius + 2e4, earthRadius + 1e5,
                           earthRadius + 1.1e7],
                        y=[0, 0, np.pi / 18, np.pi / 7, np.pi / 5, np.pi / 3, np.pi / 2.2],
                        kind="linear", assume_sorted=True, fill_value="extrapolate"),
                    "enforceDirectionRatio": (lambda angle: 0)
                }
            ),
            Command(
              CommandType.simpleMove,
                {
                    "force": 0
                }
            ),
            Command(
                CommandType.hohmannTransfer,
                {
                    "targetObject": marsName,
                    "orbitAround": sunName,
                    "acceptedError": 0.001,
                    "timeStep": pd.Timedelta(seconds = 1),
                    "acceptedOffset": 1e4
                }
            )
        ]
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
        executeFrame(timeUnit, entities, commands, entitiesDictionary, simulationTime)
        simulationTime += timeUnit
        if checkExitCondition(simulationTime, entities, collectedData): break

    print("Simulation ended")
    return collectedData

def executeCommands(commands: List[Command], entities: dict[str, SimulationEntity], simulationTime: pd.Timedelta):
    if len(commands) == 0: return
    if commands[-1].executeCommand(entities):
        commands.pop()

def executeFrame(frameTime: pd.Timedelta, entities: List[SimulationEntity], commands: List[Command], entitiesDictionary: dict[str, SimulationEntity], simulationTime: pd.Timedelta):
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
                obj.position += obj.velocity * frameTime.seconds + acceleration * (frameTime.seconds ** 2) * 0.5
                obj.velocity += acceleration * frameTime.seconds

            obj.rotationSpeed = Rotation.from_rotvec(obj.torque.as_rotvec() * frameTime.seconds) * obj.rotationSpeed
            obj.rotation = Rotation.from_rotvec(obj.rotationSpeed.as_rotvec() * frameTime.seconds) * obj.rotation

    def applyConstraints():
        for obj in entities:
            obj.applyConstraint()

    calculateForces()
    executeCommands(commands, entitiesDictionary, simulationTime)  # had to move it here to avoid interference from actions
    applyActions()
    applyChanges()
    applyConstraints()

def collectData(entities: List[SimulationEntity]) -> list:
    return [entity.getData() for entity in entities]

def checkExitCondition(simulationTime: pd.Timedelta, entities: List[SimulationEntity], data: list) -> bool:
    rocket = next(x for x in entities if x.name == rocketName)
    earth = next(x for x in entities if x.name == earthName)
    return len(data) > 1e5 or simulationTime > pd.Timedelta(days=365) or (len(data) > 100 and distanceBetweenObjects(rocket, earth) < earthRadius)

def getSimulationSetup() -> List[SimulationEntity]:
    def rocketConstraint(obj: Rocket):
        obj.torque = noRotation
        obj.rotationSpeed = noRotation
        obj.rotation = noRotation

    earth = SimulationEntity(name=earthName, mass=earthMass, volume=None, position=earthPosition, velocity=earthVelocity,
                             rotation=earthRotation, rotationSpeed=earthRotationSpeed, forcesApplied=[ForceTypes.gravity],
                             forcesIgnored=[ForceTypes.buoyancy, ForceTypes.frictionFluid])
    rocket = Rocket(name=rocketName, mass=rocketMass, volume=rocketVolume, position=rocketPosition, velocity=rocketVelocity,
                    rotation=rocketRotation, rotationSpeed=noRotation, thrusterForce=0, thrusterForceMin=0,
                    thrusterForceMax=rocketMaxForce, thrusterRotation=noRotation, thrusterRotationMax=np.deg2rad(190), distanceTTCOM=50,  #  limit on thruster rotation is effectively removed
                    forcesApplied=[ForceTypes.gravity], radius=rocketRadius, constraintFunction=rocketConstraint)
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
    sun = SimulationEntity(name=sunName, mass=sunMass, volume=None, position=sunPosition, velocity=np.array([0, 0, 0]),
                           rotation=noRotation, rotationSpeed=noRotation, forcesApplied=[ForceTypes.gravity],
                           forcesIgnored=[ForceTypes.buoyancy, ForceTypes.frictionFluid])

    return [earth, rocket, mks, earthAtmosphere, mars, sun]

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
                # taken for a sphere (https://ru.wikipedia.org/wiki/Лобовое_сопротивление)
                effect = 0.9 * density * (relativeSpeed ** 2) * 0.5 * area
                forceReceiver.force += effect * vecNormalize(forceSupplier.velocity - forceReceiver.velocity)