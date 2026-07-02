import numpy as np
import math

class Waypoint:
    # __slots__ eliminates per-instance __dict__, cutting ~200 B per object
    # and slightly speeding up attribute access.
    __slots__ = (
        'id', 
        'time', 
        'pos', 
        'vel', 
        'acel', 
        'jerk', 
        'snap', 
        'crakle',
        'fly_over', 
        'heading', 
        'position_decimals', 
        'velocity_decimals', 
        'time_decimals'
    )

    def __init__(
        self, 
        id='', 
        time=0, 
        pos=[0,0,0], 
        vel=[0,0,0], 
        acel=[0,0,0],
        jerk=[0,0,0], 
        snap=[0,0,0], 
        crakle=[0,0,0],
        fly_over=False, 
        heading=[0,0],
        position_decimals=3,
        velocity_decimals=3,
        time_decimals=3
    ):
        self.id: str = id                   # identifier to refer the waypoint
        self.fly_over = fly_over            # mandatory transit (bool)
        self.time: float = time             # time          (s)
        self.pos  = np.array(pos)           # position      (m)
        self.vel  = np.array(vel)           # velocity      (m/s)
        self.acel = np.array(acel)          # acceleration  (m/s2)
        self.jerk = np.array(jerk)          # jerk          (m/s3)
        self.snap = np.array(snap)          # snap          (m/s4)
        self.crakle = np.array(crakle)      # ckl           (m/s5)
        self.heading = np.array(heading)    # orientation vector [x, y]
        self.position_decimals = position_decimals
        self.velocity_decimals = velocity_decimals
        self.time_decimals = time_decimals

    def __lt__(self, other) -> bool:
        return self.time < other.time

    # ----------------------------------
    # -------- AUXILIARY FUNCTIONS -----
    # ----------------------------------
    def copy(self) -> Waypoint:
        """
        Create a copy of the waypoint object.

        Returns:
            Waypoint: A new instance of the Waypoint class with the same attributes 
            as the original.
        """

        waypoint = Waypoint.__new__(Waypoint)
        
        waypoint.id         = self.id
        waypoint.fly_over   = self.fly_over
        waypoint.time       = self.time
        waypoint.heading    = self.heading.copy()
        waypoint.pos        = self.pos.copy()
        waypoint.vel        = self.vel.copy()
        waypoint.acel       = self.acel.copy()
        waypoint.jerk       = self.jerk.copy()
        waypoint.snap       = self.snap.copy()
        waypoint.crakle     = self.crakle.copy()

        return waypoint

    def stop(self) -> None:
        """
        Set the velocity, acceleration, jerk, snap, and crackle of the waypoint to zero.
        This effectively stops the motion at this waypoint.
        """

        self.vel[:] = 0
        self.acel[:] = 0
        self.jerk[:] = 0
        self.snap[:] = 0
        self.crakle[:] = 0

    # ----------------------------------
    # -------- TIME MANAGEMENT ---------
    # ----------------------------------
    def postpone(self, time_step: float) -> None:
        """
        Postpone the waypoint's time by a given time step.

        Args:
            - time_step (float) : The amount of time to add to the waypoint's current time.
        """

        # 1. Check if 'time_step' is smaller than precision time decimals
        if time_step < (1 / (10 ** self.time_decimals)):
            raise ValueError(
                f"Time step must be larger than the precision time decimals"
                f" (1/{10 ** self.time_decimals}), for time_step={time_step}."
            )

        # 2. Postpone the waypoint's time and round it to the specified number of decimals
        self.time += time_step
        self.time = round(self.time, self.time_decimals)

    def time_to(self, other_wp: Waypoint) -> float:
        """
        Calculate the time difference between this waypoint and another waypoint.

        Args:
            - other_wp (Waypoint) : The other waypoint to compare with.

        Returns:
            float: The time difference between the two waypoints.
        """

        return other_wp.time - self.time

    # ----------------------------------
    # -------- GEOMETRY MANAGEMENT -----
    # ----------------------------------
    def distance_to(self, other_wp: Waypoint) -> float:
        """
        Calculate the distance between this waypoint and another waypoint.

        Args:
            - other_wp (Waypoint) : The other waypoint to calculate the distance to.

        Returns:
            float: The Euclidean distance between the two waypoints.
        """

        pos_diff = other_wp.pos - self.pos
        return math.sqrt(pos_diff[0]**2 + pos_diff[1]**2 + pos_diff[2]**2)

    def direction_to(self, other_wp: Waypoint) -> np.ndarray:
        """
        Calculate the direction vector from this waypoint to another waypoint.

        Args:
            - other_wp (Waypoint) : The other waypoint to calculate the direction to.

        Returns:
            np.ndarray: A unit vector representing the direction from this waypoint to 
            the other waypoint.
        """

        # 1. Avoid overhead of calculating 'pos_diff' via calling 'distance_to' function
        #    by calculating it directly here.
        pos_diff = other_wp.pos - self.pos
        distance = math.sqrt(pos_diff[0]**2 + pos_diff[1]**2 + pos_diff[2]**2)

        # 2. Return a zero vector if the distance is zero to avoid division by zero.
        if distance == 0:
            return np.zeros(3)
        else:
            return pos_diff / distance

    def course_to(self, other_wp: Waypoint) -> tuple[float, float]:
        """
        Calculate the course from this waypoint to another waypoint.

        Args:
            - other_wp (Waypoint) : The other waypoint to calculate the course to.

        Returns:
            tuple: A tuple containing the course angle in radians and degrees from this 
            waypoint to the other waypoint.
        """

        # 1. Check if both waypoints are at the same position so that the course is undefined.
        dist = self.distance_to(other_wp)
        if dist == 0:
            return 0, 0
        
        # 2. Calculate the course angle
        course_x = other_wp.pos[0] - self.pos[0]
        course_y = other_wp.pos[1] - self.pos[1]

        angle_rad = math.atan2(course_x, course_y)
        angle_deg = math.degrees(angle_rad)

        return angle_rad, angle_deg

    def angle_with(self, other_wp: Waypoint) -> float:
        """
        Calculate the angle between the velocity vectors of this waypoint and 
        another waypoint.

        Args:
            - other_wp (Waypoint) : The other waypoint to calculate the angle with.

        Returns:
            float: The angle in radians between the velocity vectors of the two waypoints.
        """

        # 1. Compute the norms of both velocity vectors
        norm_wp1_vel = math.sqrt(self.vel[0]**2 + self.vel[1]**2 + self.vel[2]**2)
        norm_wp2_vel = math.sqrt(other_wp.vel[0]**2 + other_wp.vel[1]**2 + other_wp.vel[2]**2)

        # 2. If either velocity vector has a norm of zero, the angle is undefined;
        #    return 0 to avoid division by zero.
        if norm_wp1_vel == 0 or norm_wp2_vel == 0:
            return 0

        # 3. Compute the angle from the dot-product formula.
        cos_angle = np.dot(self.vel, other_wp.vel) / (norm_wp1_vel * norm_wp2_vel)

        angle_rad = math.acos(max(-1.0, min(1.0, cos_angle)))
        angle_deg = math.degrees(angle_rad)

        return (
            round(angle_rad, self.position_decimals), 
            round(angle_deg, self.position_decimals)
        )

    # ----------------------------------
    # -------- KINEMATIC MANAGEMENT -----
    # ----------------------------------
    def set_uniform_velocity(self, other_wp: Waypoint) -> None:
        """
        Set the velocity of this waypoint to achieve uniform motion towards another waypoint.

        Args:
            - other_wp (Waypoint) : The target waypoint to set the velocity towards.
        """

        # 1. Stop current motion so the override is clean and doesn't accumulate previous
        #    velocity.
        self.stop()

        # 2. If the time difference is zero, raise an error to avoid division by zero.
        time_diff = self.time_to(other_wp)
        if time_diff == 0:
            raise RuntimeError(
                f"Time difference is 0, cannot set uniform velocity between waypoints"
                f" {self.id} and {other_wp.id}"
            )

        # 3. Calculate the uniform velocity vector needed to reach other_wp in the given time.
        self.vel = (other_wp.pos - self.pos) / time_diff
        self.vel = np.round(self.vel, self.velocity_decimals)

    def connect_to(self, other_wp: Waypoint) -> None:
        """
        Connects this waypoint to another waypoint by calculating the necessary kinematic 
        derivatives (jerk, snap and crakle).

        Args:
            - other_wp (Waypoint) : The target waypoint to connect to.
        """

        # 1. Extract the position, velocity, and acceleration of both waypoints.
        r1 = self.pos
        v1 = self.vel
        a1 = self.acel
        r2 = other_wp.pos
        v2 = other_wp.vel
        a2 = other_wp.acel

        # 2. If the distance between the two waypoints is zero, stop the motion and return.
        if self.distance_to(other_wp) == 0:
            self.stop()
            return
        
        # 3. Calculate the time difference between the two waypoints.
        t12 = self.time_to(other_wp)

        # 4. Construct the matrix A and vector B for the system of equations to solve for
        #    the kinematic derivatives.
        A = np.array([
            [t12**3 / 6, t12**4 / 24, t12**5 / 120],
            [t12**2 / 2, t12**3 / 6,  t12**4 / 24],
            [t12,        t12**2 / 2,  t12**3 / 6]
        ])

        B = np.array([
            r2 - r1 - v1 * t12 - 0.5 * a1 * (t12 ** 2),
            v2 - v1 - a1 * t12,
            a2 - a1
        ])

        # 5. Solve the system of equations to find the jerk, snap, and crakle values.
        try:
            X = np.linalg.solve(A,B)

            # 6. Assign the calculated jerk, snap, and crakle values to this waypoint.
            self.jerk =   X[0]
            self.snap =   X[1]
            self.crakle = X[2]

        except np.linalg.LinAlgError:
            raise ValueError("Error. 5th-Order Interpolation not possible.")

    def interpolate(self, other_time: float) -> Waypoint:
        """
        Interpolates a new waypoint from current waypoint's data at the specified time.

        Args:
            - other_time (float) : The time at which to interpolate the new waypoint.

        Returns:
            Waypoint: The interpolated waypoint at the specified time.
        """

        # 1. Build a new waypoint object to hold the interpolated data.
        interpolated_wp = Waypoint()
        interpolated_wp.time = other_time

        # 2. Extract the current waypoint's data
        r1 = self.pos
        v1 = self.vel
        a1 = self.acel
        j1 = self.jerk
        s1 = self.snap
        c1 = self.crakle

        # 3. Calculate the time difference between the current waypoint and the specified time.
        t12 = other_time - self.time
        
        # 4. Use the Taylor series expansion to calculate the interpolated data 
        #    at the specified time.
        r2 = r1 + v1*t12 + 1/2*a1*t12**2 + 1/6*j1*t12**3 + 1/24*s1*t12**4 + 1/120*c1*t12**5
        v2 = v1 + a1*t12 + 1/2*j1*t12**2 + 1/6*s1*t12**3 + 1/24*c1*t12**4
        a2 = a1 + j1*t12 + 1/2*s1*t12**2 + 1/6*c1*t12**3
        j2 = j1 + s1*t12 + 1/2*c1*t12**2
        s2 = s1 + c1*t12
        c2 = c1

        # 5. Assign the interpolated data to the new waypoint object.
        interpolated_wp.pos  = r2
        interpolated_wp.vel  = v2
        interpolated_wp.acel = a2
        interpolated_wp.jerk = j2
        interpolated_wp.snap = s2
        interpolated_wp.crakle = c2

        return interpolated_wp




    

