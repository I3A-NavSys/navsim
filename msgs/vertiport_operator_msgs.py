from __future__ import annotations
from dataclasses import dataclass, field
from uspace.vertiport_operator.vertiport_operator import VertiportOperator


# Quizá no es necesario esto porque la propia clase del Operador
# ya tenga un método "discover" o similar que envíe él mismo la información.
@dataclass
class USpaceManagerRegistryMsg:
    _operator: VertiportOperator = field(default=None)
