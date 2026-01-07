from dataclasses import dataclass, field

from uspace.uav_operator.uav_operator import UAVOperator
from uspace.vertiport_operator.vertiport_operator import VertiportOperator


@dataclass
class VertiportOperatorList:
    _vertiport_operators: list[VertiportOperator] = field(default_factory=list)

@dataclass
class UAVOperatorList:
    _uav_operators: list[UAVOperator] = field(default_factory=list)