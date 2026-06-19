import numpy as np


class Waypoint:
    # __slots__ eliminates per-instance __dict__, cutting ~200 B per object
    # and slightly speeding up attribute access.
    __slots__ = ('label', 't', 'pos', 'vel', 'acel', 'jerk', 'snap', 'crakle',
                 'fly_over', 'heading')

    def __init__(
        self, 
        label='', 
        t=0, 
        pos=[0,0,0], 
        vel=[0,0,0], 
        acel=[0,0,0],
        jerk=[0,0,0], 
        snap=[0,0,0], 
        crakle=[0,0,0],
        fly_over=False, 
        heading=[0,0]
    ):
        self.label: str = label            # identifier to refer the waypoint
        self.fly_over = fly_over           # mandatory transit (bool)
        self.t: float = t     # time          (s)
        self.pos  = np.array(pos)          # position      (m)
        self.vel  = np.array(vel)          # velocity      (m/s)
        self.acel = np.array(acel)         # acceleration  (m/s2)
        self.jerk = np.array(jerk)         # jerk          (m/s3)
        self.snap = np.array(snap)         # snap          (m/s4)
        self.crakle = np.array(crakle)     # ckl           (m/s5)
        self.heading = np.array(heading)   # orientation vector [x, y]

    def __lt__(self, other) -> bool:
        return self.t < other.t

    def copy(self):
        waypoint = Waypoint.__new__(Waypoint)
        
        waypoint.label = self.label
        waypoint.fly_over = self.fly_over
        waypoint.t = self.t
        waypoint.heading  = self.heading.copy()
        waypoint.pos    = self.pos.copy()
        waypoint.vel    = self.vel.copy()
        waypoint.acel   = self.acel.copy()
        waypoint.jerk   = self.jerk.copy()
        waypoint.snap   = self.snap.copy()
        waypoint.crakle = self.crakle.copy()

        return waypoint

    def stop(self):
        self.vel  = np.zeros(3)
        self.acel = np.zeros(3)
        self.jerk = np.zeros(3)
        self.snap = np.zeros(3)
        self.crakle = np.zeros(3)

    #-------------------------------------------------------------------
    # TIME MANAGEMENT

    def postpone(self, timeStep):
        self.t += timeStep
        self.t = np.round(self.t, 2)

    def time_to(self, wp):
        # Get the time elapsed from this waypoint to another given
        return wp.t - self.t

    #-------------------------------------------------------------------
    # SPACE MANAGEMENT
   
    def distance_to(self, wp) -> float:
        # Get the distance between two waypoints
        return np.linalg.norm(self.pos - wp.pos)

    def direction_to(self, wp) -> np.array:
        # Get a direction vector from one waypoint to another
        dist = self.distance_to(wp)
        if dist == 0:
            return np.zeros(3)
        else:
            return (wp.pos - self.pos) / dist

    def course_to(self, wp) -> float:
        # Get the course from one waypoint to another
        # -X -> -  90
        # +Y ->     0
        # +X -> +  90
        # -Y -> +-180
        dist = self.distance_to(wp)
        if dist == 0:
            return 0
        else:
            cx = wp.pos[0] - self.pos[0]
            cy = wp.pos[1] - self.pos[1]
            angle_rad = np.arctan2(cx, cy)
            angle_deg = np.degrees(angle_rad)
            return angle_deg    

    def angle_with(self, wp) -> float:
        # Get the angle between the direction of two waypoints
        norm_a_vel = np.linalg.norm(self.vel)
        norm_b_vel = np.linalg.norm(wp.vel)
        
        if norm_a_vel == 0 or norm_b_vel == 0:
            return 0
        else:
            dot_product = np.dot(self.vel,wp.vel)
            cos_angle = dot_product / (norm_a_vel * norm_b_vel)
            angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
            return angle
        
    #-------------------------------------------------------------------
    # DYNAMICS MANAGEMENT

    def set_uniform_velocity(self, wp2):
        # Set uniform straight velocity from wp1 to wp2
        self.stop()

        # Time between wp1 and wp2
        t12 = self.time_to(wp2)
        if t12 != 0:
            self.vel = (wp2.pos - self.pos) / t12
            self.vel = np.round(self.vel, 2)

    def connect_to(self, wp2):
        # Dados dos waypoints con 
        #   t1 pos1 vel1 acel1
        #   t2 pos2 vel2 acel2
        # obtiene jerk1, snap1 y crkl1 para ejecutar dicho movimiento
        r1 = self.pos
        v1 = self.vel
        a1 = self.acel
        r2 = wp2.pos
        v2 = wp2.vel
        a2 = wp2.acel

        if np.linalg.norm(r2 - r1) == 0:
            self.stop()
            return
        
        t12 = self.time_to(wp2)

        A = np.array([
            [t12**3 / 6,   t12**4 / 24,   t12**5 / 120],
            [t12**2 / 2,   t12**3 / 6,    t12**4 / 24],
            [t12,          t12**2 / 2,    t12**3 / 6]
        ])

        B = np.array([
            r2 - r1 - v1 * t12 - 0.5 * a1 * (t12 ** 2),
            v2 - v1 - a1 * t12,
            a2 - a1
        ])

        try:
            X = np.linalg.solve(A,B)
        except np.linalg.LinAlgError:
            raise ValueError('Error. Interpolation not possible')

        self.jerk = X[0]
        self.snap = X[1]
        self.crakle = X[2]

    def set_JSC(self, wp2):
        """
        Calculates the kinematic derivatives (Jerk, Snap, Crackle).
        Uses a robust tolerance to detect straight acceleration lanes and
        assigns them pure constant acceleration, avoiding polynomial wobble.
        """
        t12 = wp2.t - self.t
        if t12 <= 0:
            return

        r1 = self.pos
        v1 = self.vel
        r2 = wp2.pos
        v2 = wp2.vel

        # 1. Theoretical constant acceleration needed to bridge the velocities
        a_ideal = (v2 - v1) / t12

        # 2. Where the drone would be if it followed pure constant acceleration
        r2_ideal = r1 + v1 * t12 + 0.5 * a_ideal * (t12 ** 2)

        # 3. Distance between ideal 1D physics and actual 3D target position
        pos_error = np.linalg.norm(r2 - r2_ideal)

        # 4. HYBRID BYPASS: Relaxed tolerance (1.0 meter) to account for 3D float math.
        # If it's a straight lane, force exact linear velocity and zero higher derivatives.
        if pos_error < 1.0:
            self.acel = a_ideal
            
            # CRITICAL FIX: Prepare the NEXT waypoint with this exact acceleration.
            # If the next segment is a complex curve, it will use this realistic 
            # inertia instead of defaulting to 0, preventing spline whip.
            wp2.acel = a_ideal
            
            self.jerk = np.zeros(3)
            self.snap = np.zeros(3)
            self.crakle = np.zeros(3)
            return

        # 5. COMPLEX CURVE FALLBACK (Corners)
        a1 = self.acel
        a2 = wp2.acel

        A = np.array([
            [t12**3 / 6, t12**4 / 24, t12**5 / 120],
            [t12**2 / 2, t12**3 / 6,  t12**4 / 24 ],
            [t12,        t12**2 / 2,  t12**3 / 6  ]
        ])

        # Corrected B matrix containing the initial acceleration terms
        B = np.array([
            r2 - r1 - v1 * t12 - 0.5 * a1 * (t12 ** 2),
            v2 - v1 - a1 * t12,
            a2 - a1
        ])

        try:
            X = np.linalg.solve(A, B)
            self.jerk = X[0]
            self.snap = X[1]
            self.crakle = X[2]
            
        except np.linalg.LinAlgError:
            raise ValueError('Error. 5th-Order Interpolation not possible.')

    def interpolation(self, t2):
    # Dados dos waypoints con tiempo, posición, velocidad y aceleración nula
    # interpola un tercer waypoint a un tiempo dado
        wp2 = Waypoint()
        wp2.t = t2

        r1 = self.pos
        v1 = self.vel
        a1 = self.acel
        j1 = self.jerk
        s1 = self.snap
        c1 = self.crakle

        t12 = self.time_to(wp2)
        
        r2 = r1 + v1*t12 + 1/2*a1*t12**2 + 1/6*j1*t12**3 + 1/24*s1*t12**4 + 1/120*c1*t12**5
        v2 = v1 + a1*t12 + 1/2*j1*t12**2 + 1/6*s1*t12**3 + 1/24*c1*t12**4
        a2 = a1 + j1*t12 + 1/2*s1*t12**2 + 1/6*c1*t12**3
        j2 = j1 + s1*t12 + 1/2*c1*t12**2
        s2 = s1 + c1*t12
        c2 = c1

        wp2.pos  = r2
        wp2.vel  = v2
        wp2.acel = a2
        wp2.jerk = j2
        wp2.snap = s2
        wp2.crakle = c2

        return wp2




    

