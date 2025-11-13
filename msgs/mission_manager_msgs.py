from __future__ import annotations
from dataclasses import dataclass, field
from uspace.operator.operator import Operator


@dataclass
class MissionMsg:
    _mission_type: str
    _stop: StopoverMsg
    _operator: Operator

@dataclass
class StopoverMsg:
    _stop_list: list[str] = field(default_factory=list)
    _landing_time: list[int] = field(default_factory=list)
    _stop_time: list[int] = field(default_factory=list)