import numpy as np



class Waypoint:


    def __init__(self, label='', t=0, pos=None, vel=None, acel=None, jerk=None, jolt=None, snap=None, mandatory=False):
 
        self.label: str = label  if label  is not None else ''           # label to refer the waypoint
        self.t: float = t                                                # time (s)
        
        self.pos  = np.array(pos)  if pos  is not None else np.zeros(3)  # position      (m)
        self.vel  = np.array(vel)  if vel  is not None else np.zeros(3)  # velocity      (m/s)
        self.acel = np.array(acel) if acel is not None else np.zeros(3)  # acceleration  (m/s2)
        self.jerk = np.array(jerk) if jerk is not None else np.zeros(3)  # jerk          (m/s3)
        self.jolt = np.array(jolt) if jolt is not None else np.zeros(3)  # jolt          (m/s4)
        self.snap = np.array(snap) if snap is not None else np.zeros(3)  # snap          (m/s5)

        self.mandatory = mandatory  # transito obligado



    def Stop(self):
        self.vel  = np.zeros(3)
        self.acel = np.zeros(3)
        self.jerk = np.zeros(3)
        self.jolt = np.zeros(3)
        self.snap = np.zeros(3)



    def Postpone(self, timeStep):
        self.t += timeStep



    @staticmethod
    def TimeTo(a,b):
            # Get the time elapsed from this waypoint to another given
            return b.t - a.t



    @staticmethod
    def DistanceTo(a,b):
        # Get the distance between two waypoints
        return np.linalg.norm(a.pos - b.pos)



    def DirectionTo(a,b):
        # Get a direction vector from one waypoint to another
        dist = a.DistanceTo(b)
        if dist == 0:
            return np.zeros(3)
        else:
            return (b.pos - a.pos) / dist



    @staticmethod
    def CourseTo(a,b):
        # Get the course from one waypoint to another
        # -X -> -  90
        # +Y ->     0
        # +X -> +  90
        # -Y -> +-180
        dist = a.DistanceTo(b)
        if dist == 0:
            return 0
        else:
            cx = b.pos[0] - a.pos[0]
            cy = b.pos[1] - a.pos[1]
            angle_rad = np.arctan2(cx, cy)
            angle_deg = np.degrees(angle_rad)
            return angle_deg    

    
    
    @staticmethod
    def AngleWith(a,b):
        # Get the angle between the direction of two waypoints
        
        norm_a_vel = np.linalg.norm(a.vel)
        norm_b_vel = np.linalg.norm(b.vel)
        
        if norm_a_vel == 0 or norm_b_vel == 0:
            return 0
        else:
            dot_product = np.dot(a.vel,b.vel)
            cos_angle = dot_product / (norm_a_vel * norm_b_vel)
            angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
            return angle
        
        

    @staticmethod
    def SetV0000(wp1,wp2):
        # Set uniform straight velocity from A to B

        wp1.Stop()

        t12 = wp1.TimeTo(wp2)
        if t12 != 0:
            wp1.vel = (wp2.pos - wp1.pos) / t12



    @staticmethod
    def SetJLS(wp1,wp2):
        Waypoint.CheckWaypoint(wp2)
        # Dados dos waypoints con 
        #   t1 pos1 vel1 acel1
        #   t2 pos2 vel2 acel2
        # obtiene jerk1, jolt1 y snap1 para ejecutar dicho movimiento

        r1 = wp1.pos
        v1 = wp1.vel
        a1 = wp1.acel
        r2 = wp2.pos
        v2 = wp2.vel
        a2 = wp2.acel

        if np.linalg.norm(r2 - r1) == 0:
            wp1.Stop()
            return
        
        t12 = wp1.TimeTo(wp2)

        A = np.array([
            [t12**3 / 6,   t12**4 / 24,   t12**5 / 120],
            [t12**2 / 2,   t12**3 / 6,    t12**4 / 24],
            [t12,          t12**2 / 2,    t12**3 / 6]
        ])

        B = np.array([
            r2 - r1 - v1 * t12,
            v2 - v1,
            a2 - a1
        ])

        try:
            X = np.linalg.solve(A,B)
        except np.linalg.LinAlgError:
            raise ValueError('Error. Interpolation not possible')

        wp1.jerk = X[0]
        wp1.jolt = X[1]
        wp1.snap = X[2]



    @staticmethod
    def SetJL0_T(wp1,wp2):

        # Dados dos waypoints con 
        #   t1 pos1 vel1 acel1
        #      pos2 vel2 acel2
        # obtiene 
        #   jerk1 jolt1 snap1=0
        #   t2
        # para ejecutar dicho movimiento


        r1 = wp1.pos
        v1 = wp1.vel
        a1 = wp1.acel
        r2 = wp2.pos
        v2 = wp2.vel
        a2 = wp2.acel

        if np.linalg.norm(r2 - r1) == 0:
            wp1.Stop()
            return
        
        t12 = wp1.TimeTo(wp2)

        A = np.array([
            [t12**3 / 6,   t12**4 / 24,   t12**5 / 120],
            [t12**2 / 2,   t12**3 / 6,    t12**4 / 24],
            [t12,          t12**2 / 2,    t12**3 / 6]
        ])

        B = np.array([
            r2 - r1 - v1 * t12,
            v2 - v1,
            a2 - a1
        ])

        try:
            X = np.linalg.solve(A,B)
        except np.linalg.LinAlgError:
            raise ValueError('Error. Interpolation not possible')

        wp1.jerk = X[0]
        wp1.jolt = X[1]
        wp1.snap = X[2]



    @staticmethod
    def Interpolation(wp1,t2):
    # Dados dos waypoints con tiempo, posición, velocidad y aceleración nula
    # interpola un tercer waypoint a un tiempo dado

        wp2 = Waypoint()
        wp2.t = t2

        r1 = wp1.pos
        v1 = wp1.vel
        a1 = wp1.acel
        j1 = wp1.jerk
        l1 = wp1.jolt
        s1 = wp1.snap

        t12 = wp1.TimeTo(wp2)
        
        r2 = r1 + v1*t12 + 1/2*a1*t12**2 + 1/6*j1*t12**3 + 1/24*l1*t12**4 + 1/120*s1*t12**5
        v2 = v1 + a1*t12 + 1/2*j1*t12**2 + 1/6*l1*t12**3 + 1/24*s1*t12**4
        a2 = a1 + j1*t12 + 1/2*l1*t12**2 + 1/6*s1*t12**3
        j2 = j1 + l1*t12 + 1/2*s1*t12**2
        l2 = l1 + s1*t12
        s2 = s1

        wp2.pos  = r2
        wp2.vel  = v2
        wp2.acel = a2
        wp2.jerk = j2
        wp2.jolt = l2
        wp2.snap = s2

        return wp2




    

