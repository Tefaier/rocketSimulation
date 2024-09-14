from typing import List
import numpy as np
import pandas as pd
from Entity import *

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
        if checkExitCondition(simulationTime, entities): break
    print("Simulation ended")

def executeFrame(frameTime: pd.Timedelta, entities: List[SimulationEntity]):
    def calculateForces():
        for index1 in range(0, len(entities) - 1):
            for index2 in range(index1 + 1, len(entities)):
                calculateInteraction(entities[index1], entities[index2])

    def applyChanges():
        pass

    def applyConstraints():
        pass

    calculateForces()
    applyChanges()
    applyConstraints()

def collectData(entities: List[SimulationEntity]) -> list:
    return [entity.getData() for entity in entities]

def checkExitCondition(simulationTime: pd.Timedelta, entities: List[SimulationEntity]) -> bool:
    pass

def getSimulationSetup() -> List[SimulationEntity]:
    return []

def calculateInteraction(obj1: SimulationEntity, obj2: SimulationEntity):
    pass

def calculateAction(obj: SimulationEntity):
    pass