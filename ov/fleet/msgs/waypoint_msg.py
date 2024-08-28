from pxr import Gf

class WaypointMsg:
    def __init__(self, pos: Gf.Vec3d, vel: Gf.Vec3d, accel: Gf.Vec3d, jerk: Gf.Vec3d, snap: Gf.Vec3d, 
                 crkl: Gf.Vec3d, time: float):
        self.pos = pos
        self.vel = vel
        self.accel = accel
        self.jerk = jerk
        self.snap = snap
        self.crkl = crkl
        self.time = time