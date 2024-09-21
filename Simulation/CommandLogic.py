from enum import Enum
from typing import List

from Simulation.Entity import SimulationEntity
from ReferenceValues import rocketName, earthName, sunName, marsName

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
            return self.gravityTurnCommand(entities)

    def gravityTurnCommand(self, entities: dict[str, SimulationEntity]) -> bool:

        return self.gravityTurnExitCondition(entities)

    def gravityTurnExitCondition(self, entities: dict[str, SimulationEntity]) -> bool:
        return False
