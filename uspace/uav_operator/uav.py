from enum import Enum


from uspace.uspace_manager.constants import MissionType


class Status(Enum):
    AVAILABLE = "AVAILABLE"
    OCCUPIED = "OCCUPIED"
    OUT_OF_SERVICE = "OUT_OF_SERVICE"

class UAV:
    def __init__(self):
        self.id: str
        self.type: MissionType
        self.status: Status
        self.operator: str
