from uspace.vertiport.vertiport_pad import VertiportPad

class Vertiport:
    def __init__(self):
        self.id: str
        self.pads: dict[str, VertiportPad]
        