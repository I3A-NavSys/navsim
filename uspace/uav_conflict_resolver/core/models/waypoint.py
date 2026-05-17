import numpy as np


class Waypoint:
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
        heading=None
    ):
        self.label: str = label            # identifier to refer the waypoint
        self.t: float = t     # time          (s)
        self.pos  = np.array(pos)          # position      (m)
        self.vel  = np.array(vel)          # velocity      (m/s)
        self.acel = np.array(acel)         # acceleration  (m/s2)
        self.jerk = np.array(jerk)         # jerk          (m/s3)
        self.snap = np.array(snap)         # snap          (m/s4)
        self.crakle = np.array(crakle)     # ckl           (m/s5)
        self.fly_over = fly_over           # mandatory transit (bool)
        self.heading = heading             # orientation vector [x, y]

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
        dist = np.linalg.norm(self.pos - wp.pos)
        return float(dist)

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
        # Set uniform straight-line velocity from this waypoint to wp2.
        # Zeroes all higher-order derivatives (acceleration, jerk, …) so that
        # the motion is Uniform Rectilinear Motion (MRU) from this point.
        self.stop()

        # Elapsed time between the two waypoints
        t12 = self.time_to(wp2)
        if t12 != 0:
            self.vel = (wp2.pos - self.pos) / t12
            self.vel = np.round(self.vel, 2)

    def check_kinematic_feasibility(
        self,
        wp2,
        v_max: float,
        a_max: float | None = None,
    ) -> tuple[bool, str]:
        """
        Checks whether the drone can physically travel from this waypoint (wp1)
        to *wp2* given the imposed kinematic limits.

        Three progressive guards are applied in order:

        Guard 1 — Positive time window
            The time gap Δt = t2 – t1 must be strictly positive.  A zero or
            negative Δt makes the segment undefined.

        Guard 2 — Average speed vs. maximum speed
            The drone must cover *dist* metres in Δt seconds, so its average
            speed must not exceed the hardware top speed v_max:

                v_avg = dist / Δt  ≤  v_max

        Guard 3 (optional) — Reachable distance under maximum acceleration
            Assuming the drone starts at its current scalar speed v1 and
            accelerates at the maximum rate a_max for the entire duration Δt,
            the furthest it can reach is:

                d_max = v1 · Δt + ½ · a_max · Δt²

            If *dist* > d_max, the segment is infeasible even under this
            optimistic scenario.

            IMPORTANT — This is a *necessary* condition, not sufficient.
            The formula assumes full-throttle acceleration throughout the whole
            segment with no deceleration at the end.  In practice, if wp2
            requires a lower exit speed than v_max, the drone must brake before
            arrival, so the actual reachable distance is smaller.  Guard 3 is
            therefore an upper-bound test: passing it does not guarantee
            feasibility, but failing it guarantees infeasibility.

        Args:
            wp2     (Waypoint):  Destination waypoint.
            v_max   (float):     Maximum admissible drone speed [m/s].
            a_max   (float|None): Maximum linear acceleration [m/s²].
                                  If None, Guard 3 is skipped.

        Returns:
            (True,  "ok")          — segment is kinematically feasible.
            (False, reason_str)    — segment is infeasible; reason_str
                                     describes which guard failed and why.
        """
        dist = self.distance_to(wp2)
        dt   = self.time_to(wp2)          # wp2.t – self.t

        # ------------------------------------------------------------------
        # Guard 1: time window must be strictly positive
        # ------------------------------------------------------------------
        if dt <= 0:
            return False, (
                f"Non-positive time window: dt={dt:.3f} s "
                f"(t1={self.t:.3f}, t2={wp2.t:.3f})"
            )

        # Trivial case: no spatial displacement — always feasible
        if dist == 0:
            return True, "ok"

        # ------------------------------------------------------------------
        # Guard 2: required average speed must not exceed the hardware limit
        # ------------------------------------------------------------------
        v_avg = dist / dt
        if v_avg > v_max:
            return False, (
                f"Required average speed ({v_avg:.2f} m/s) exceeds maximum ({v_max:.2f} m/s) "
                f"(dist={dist:.1f} m, dt={dt:.2f} s)"
            )

        # ------------------------------------------------------------------
        # Guard 3 (optional): reachable distance under maximum acceleration
        # ------------------------------------------------------------------
        if a_max is not None:
            v1    = float(np.linalg.norm(self.vel))   # scalar departure speed
            d_max = v1 * dt + 0.5 * a_max * dt ** 2  # optimistic upper bound
            if dist > d_max:
                return False, (
                f"Segment unreachable under max acceleration: dist={dist:.1f} m > d_max={d_max:.1f} m "
                f"(v1={v1:.2f} m/s, a_max={a_max:.2f} m/s^2, dt={dt:.2f} s)"
            )

        return True, "ok"

    def connect_to(self, wp2):
        """
        Given two waypoints that each specify (t, pos, vel, acel), computes
        the three higher-order derivatives (jerk, snap, crakle) of *this*
        waypoint so that the resulting quintic polynomial exactly satisfies
        the boundary conditions at both ends:

            t1, pos1, vel1, acel1  ←  this waypoint
            t2, pos2, vel2, acel2  ←  wp2

        The polynomial is solved per coordinate axis (x, y, z) via a 3×3
        linear system derived from the Taylor expansion of position:

            r(t) = r1 + v1·τ + ½·a1·τ² + (1/6)·j·τ³ + (1/24)·s·τ⁴ + (1/120)·c·τ⁵

        where τ = t – t1.  Matching position, velocity, and acceleration at
        τ = Δt = t2 – t1 gives the system A·[j, s, c]ᵀ = B.

        NOTE: This method is a pure mathematical interpolator.  It does NOT
        validate whether the resulting trajectory is physically achievable
        (e.g., whether peak speed or acceleration stay within hardware limits).
        Use check_kinematic_feasibility() before calling this method if such
        validation is required.
        """
        r1 = self.pos
        v1 = self.vel
        a1 = self.acel
        r2 = wp2.pos
        v2 = wp2.vel
        a2 = wp2.acel

        # Trivial case: no displacement — keep the waypoint stationary
        if np.linalg.norm(r2 - r1) == 0:
            self.stop()
            return

        t12 = self.time_to(wp2)   # Δt = t2 – t1  [s]

        # ------------------------------------------------------------------
        # Build the 3×3 coefficient matrix A and right-hand side B.
        # Each row enforces one boundary condition at t = t12:
        #   row 0  →  position match:    r1 + integral → r2
        #   row 1  →  velocity match:    v1 + integral → v2
        #   row 2  →  acceleration match: a1 + integral → a2
        # Columns correspond to [jerk, snap, crakle] contributions.
        # ------------------------------------------------------------------
        A = np.array([
            [t12**3 / 6,   t12**4 / 24,   t12**5 / 120],
            [t12**2 / 2,   t12**3 / 6,    t12**4 / 24],
            [t12,          t12**2 / 2,    t12**3 / 6]
        ])

        B = np.array([
            r2 - r1 - v1 * t12,   # residual position
            v2 - v1,               # residual velocity
            a2 - a1                # residual acceleration
        ])

        try:
            X = np.linalg.solve(A, B)
        except np.linalg.LinAlgError as e:
            raise ValueError(
                f"Cannot compute coefficients for segment "
                f"{self.label!r}->{wp2.label!r} (dt={t12:.3f} s). "
                f"LinAlgError: {e}"
            )

        self.jerk   = X[0]
        self.snap   = X[1]
        self.crakle = X[2]

    def set_JS0_T(self, wp2):
        # Dados dos waypoints con 
        #   t1 pos1 vel1 acel1
        #      pos2 vel2 acel2
        # obtiene 
        #   jerk1 snap1 crkl1=0
        #   t2
        # para ejecutar dicho movimiento
        pass

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
        wp2.crkl = c2

        return wp2




    

