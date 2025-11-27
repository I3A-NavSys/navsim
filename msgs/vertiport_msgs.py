from dataclasses import dataclass, field

from uspace.vertiport.vertiport import Vertiport


@dataclass
class VertiportList:
    _vertiports: list[Vertiport] = field(default_factory=list)