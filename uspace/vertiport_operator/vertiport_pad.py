from enum import Enum


from uspace_manager.constants import MissionType
from .vertiport_operator import VertiportOperator


class Status(Enum):
    AVAILABLE = "AVAILABLE"
    OCCUPIED = "OCCUPIED"
    OUT_OF_SERVICE = "OUT_OF_SERVICE"

class VertiportPad:
    def __init__(self):
        self.id: str
        self.type: MissionType
        self.status: Status
        self.ownership: VertiportOperator
