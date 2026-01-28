class Pad:
    def __init__(self, id, type, status, operator, location):
        self.id: str = id
        self.type: str = type
        self.status: str = status
        self.operator: str = operator
        self.location: tuple[float, float, float] = location
