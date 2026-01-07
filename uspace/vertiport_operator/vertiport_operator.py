from .vertiport_pad import VertiportPad

class VertiportOperator:
    def __init__(self):
        self.id: str
        self.name: str
        self.pads: dict[str, VertiportPad]
        