from enum import Enum
from uspace.operator.operator import Operator

class VertiportPad:
    def __init__(self):
        self.id: str
        self.type: str
        self.status: str
        self.ownership: Operator

class Type(Enum):
    UAV = "UAV"
    AEROTAXI = "AEROTAXI"

class Status(Enum):
    AVAILABLE = "AVAILABLE"
    OCCUPIED = "OCCUPIED"
    OUT_OF_SERVICE = "OUT_OF_SERVICE"