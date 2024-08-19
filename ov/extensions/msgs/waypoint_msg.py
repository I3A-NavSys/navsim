from point_msg import PointMsg
from vector3_msg import Vector3Msg
from time_msg import TimeMsg

class WaypointMsg:
    def __init__(self) -> None:
        self.pos: PointMsg
        self.vel: Vector3Msg
        self.accel: Vector3Msg
        self.jerk: Vector3Msg
        self.snap: Vector3Msg
        self.crkl: Vector3Msg
        self.time: TimeMsg