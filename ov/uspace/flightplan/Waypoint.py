import numpy as np



class Waypoint:


    def __init__(self, label='', t=0, 
                 pos=[0,0,0], vel=[0,0,0], 
                 fly_over=False):
 
        self.label: str = label                         # identifier to refer the waypoint
        self.t: float = np.round(t, 2)                  # time          (s)
        self.pos  = np.round(np.array(pos), 2)          # position      (m)
        self.vel  = np.round(np.array(vel), 2)          # velocity      (m/s)
        self.acel = np.array([0,0,0])                   # acceleration  (m/s2)
        self.jerk = np.array([0,0,0])                   # jerk          (m/s3)
        self.snap = np.array([0,0,0])                   # snap          (m/s4)
        self.crkl = np.array([0,0,0])                   # ckl           (m/s5)
        self.fly_over = fly_over                        # transito obligado



    def Stop(self):
        self.vel  = np.zeros(3)
        self.acel = np.zeros(3)
        self.jerk = np.zeros(3)
        self.snap = np.zeros(3)
        self.crkl = np.zeros(3)



#-------------------------------------------------------------------
# TIME MANAGEMENT


    def Postpone(self, timeStep):
        self.t += timeStep
        self.t = np.round(self.t, 2)


    def TimeTo(self,wp):
        # Get the time elapsed from this waypoint to another given
        return wp.t - self.t



#-------------------------------------------------------------------
# SPACE MANAGEMENT

   
    def DistanceTo(self,wp) -> float:
        # Get the distance between two waypoints
        dist = np.linalg.norm(self.pos - wp.pos)
        return float(dist)



    def DirectionTo(self,wp) -> np.array:
        # Get a direction vector from one waypoint to another
        dist = self.DistanceTo(wp)
        if dist == 0:
            return np.zeros(3)
        else:
            return (wp.pos - self.pos) / dist



    def CourseTo(self,wp) -> float:
        # Get the course from one waypoint to another
        # -X -> -  90
        # +Y ->     0
        # +X -> +  90
        # -Y -> +-180
        dist = self.DistanceTo(wp)
        if dist == 0:
            return 0
        else:
            cx = wp.pos[0] - self.pos[0]
            cy = wp.pos[1] - self.pos[1]
            angle_rad = np.arctan2(cx, cy)
            angle_deg = np.degrees(angle_rad)
            return angle_deg    

    
    
    def AngleWith(self,wp) -> float:
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

    def SetUniformVelocity(self, wp2):
        # Set uniform straight velocity from wp1 to wp2

        self.Stop()

        # Time between wp1 and wp2
        t12 = self.TimeTo(wp2)
        if t12 != 0:
            self.vel = (wp2.pos - self.pos) / t12
            self.vel = np.round(self.vel, 2)



    def ConnectTo(self,wp2):
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
            self.Stop()
            return
        
        t12 = self.TimeTo(wp2)

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

        self.jerk = X[0]
        self.snap = X[1]
        self.crkl = X[2]



    def SetJS0_T(self,wp2):

        # Dados dos waypoints con 
        #   t1 pos1 vel1 acel1
        #      pos2 vel2 acel2
        # obtiene 
        #   jerk1 snap1 crkl1=0
        #   t2
        # para ejecutar dicho movimiento
        pass



    def Interpolation(self,t2):
    # Dados dos waypoints con tiempo, posición, velocidad y aceleración nula
    # interpola un tercer waypoint a un tiempo dado

        wp2 = Waypoint()
        wp2.t = t2

        r1 = self.pos
        v1 = self.vel
        a1 = self.acel
        j1 = self.jerk
        s1 = self.snap
        c1 = self.crkl

        t12 = self.TimeTo(wp2)
        
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
        wp2.crkl = c2

        return wp2




    

