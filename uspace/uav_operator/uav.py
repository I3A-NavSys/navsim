class UAV:
    def __init__(self, id, operator_id, type, status, battery_level, location, pad_id):
        self.id: str = id
        self.operator_id: str = operator_id
        self.type: str = type
        self.status: str = status
        self.battery_level: float = battery_level
        self.location: tuple[float, float, float] = location
        self.pad_id: str = pad_id
