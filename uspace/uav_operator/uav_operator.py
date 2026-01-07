from .uav import UAV


class UAVOperator:
    def __init__(self):
        self.id: str
        self.name: str
        self.uavs = dict[str, UAV]