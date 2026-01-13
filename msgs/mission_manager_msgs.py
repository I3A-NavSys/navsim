from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum


from uspace.uspace_manager.constants import MissionType


@dataclass
class StopOver:
    _stop_list: list[str] = field(default_factory=list)
    _landing_time: int = field(default_factory=int)
    _stop_time: list[int] = field(default_factory=list)

@dataclass
class MissionMsg:
    _mission_type: MissionType
    _stop_over: StopOver
    _operator_id: str
