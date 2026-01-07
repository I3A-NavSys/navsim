from enum import Enum


from uspace_manager.constants import MissionType
from .uav_operator import UAVOperator


class Status(Enum):
    AVAILABLE = "AVAILABLE"
    OCCUPIED = "OCCUPIED"
    OUT_OF_SERVICE = "OUT_OF_SERVICE"

class UAV:
    def __init__(self):
        self.id: str
        self.type: MissionType
        self.status: Status
        self.operator: UAVOperator
