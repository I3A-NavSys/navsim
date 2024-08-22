from .point_msg import PointMsg
from .vector3_msg import Vector3Msg

class WaypointMsg:
    def __init__(self, pos: PointMsg, vel: Vector3Msg, accel: Vector3Msg, jerk: Vector3Msg, snap: Vector3Msg, 
                 crkl: Vector3Msg, time: float):
        self.pos = pos
        self.vel = vel
        self.accel = accel
        self.jerk = jerk
        self.snap = snap
        self.crkl = crkl
        self.time = time