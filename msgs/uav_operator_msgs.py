from __future__ import annotations
from dataclasses import dataclass, field
from uspace.uav_operator.uav_operator import UAVOperator


# Quizá no es necesario esto porque la propia clase del Operador
# ya tenga un método "discover" o similar que envíe él mismo la información.
@dataclass
class USpaceManagerRegistryMsg:
    _operator: UAVOperator = field(default=None)
