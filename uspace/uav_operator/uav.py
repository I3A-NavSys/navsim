from uspace.uspace_manager.constants import MissionType, UAVStatus


class UAV:
    def __init__(self):
        self.id: str
        self.operator: str
        self.type: MissionType
        self.status: UAVStatus
        self.battery_level: float
        self.location: tuple[float, float, float]
        self.pad_id: str
