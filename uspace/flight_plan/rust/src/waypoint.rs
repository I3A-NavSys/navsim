//! # Waypoint
//!
//! Direct Rust translation of `uspace/flight_plan/waypoint_new.py`.
//!
//! The struct holds the kinematic state of a waypoint up to the 5th
//! derivative (crakle) plus the descriptive `id`, `time`, `fly_over` and
//! `heading` fields. In Rust the three 3D vector fields are plain
//! `[f64; 3]` arrays — fixed size, stack-allocated, copy-cheap, and they
//! behave like the original `numpy.array([...])` shape `(3,)` objects for
//! all the operations used by `FlightPlan`.
//!
//! Methods are kept in the same order as the Python class, with the same
//! section comments, to keep the two implementations easy to compare
//! side-by-side.

use std::f64;

/// 3D vector type used throughout the crate (plain `[f64; 3]`).
pub type Vec3 = [f64; 3];

/// 2D heading vector type used by `Waypoint.heading` (`[f64; 2]`).
pub type Vec2 = [f64; 2];

/// One waypoint of a flight plan.
///
/// Stores position up to the 5th kinematic derivative (crakle) plus
/// descriptive metadata (`id`, `time`, `fly_over`, `heading`, decimal
/// precision knobs). The `Vec3` fields are `[f64; 3]` arrays.
#[derive(Debug, Clone)]
pub struct Waypoint {
    /// Identifier used to refer to the waypoint.
    pub id: String,
    /// Mandatory transit (bool) — if `true` the UAV must fly over it.
    pub fly_over: bool,
    /// Time of the waypoint (s).
    pub time: f64,
    /// Position (m).
    pub pos: Vec3,
    /// Velocity (m/s).
    pub vel: Vec3,
    /// Acceleration (m/s²).
    pub acel: Vec3,
    /// Jerk (m/s³).
    pub jerk: Vec3,
    /// Snap (m/s⁴).
    pub snap: Vec3,
    /// Crakle (m/s⁵).
    pub crakle: Vec3,
    /// Orientation vector `[x, y]`.
    pub heading: Vec2,
    /// Number of decimal digits kept for position values.
    pub position_decimals: usize,
    /// Number of decimal digits kept for velocity values.
    pub velocity_decimals: usize,
    /// Number of decimal digits kept for time values.
    pub time_decimals: usize,
}

impl Default for Waypoint {
    fn default() -> Self {
        Self::new(
            "",
            0.0,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            false,
            [0.0, 0.0],
            3,
            3,
            3,
        )
    }
}

impl Waypoint {
    /// Build a new waypoint with the given kinematic state and metadata.
    /// Defaults match the Python constructor exactly.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: impl Into<String>,
        time: f64,
        pos: Vec3,
        vel: Vec3,
        acel: Vec3,
        jerk: Vec3,
        snap: Vec3,
        crakle: Vec3,
        fly_over: bool,
        heading: Vec2,
        position_decimals: usize,
        velocity_decimals: usize,
        time_decimals: usize,
    ) -> Self {
        Self {
            id: id.into(),
            fly_over,
            time,
            pos,
            vel,
            acel,
            jerk,
            snap,
            crakle,
            heading,
            position_decimals,
            velocity_decimals,
            time_decimals,
        }
    }

    // ----------------------------------
    // -------- AUXILIARY FUNCTIONS -----
    // ----------------------------------

    /// Create a copy of the waypoint object. Mirrors `Waypoint.copy()`.
    ///
    /// Returns:
    ///     Waypoint: A new instance of the Waypoint class with the same
    ///     attributes as the original.
    pub fn copy(&self) -> Self {
        Self {
            id: self.id.clone(),
            fly_over: self.fly_over,
            time: self.time,
            heading: self.heading,
            pos: self.pos,
            vel: self.vel,
            acel: self.acel,
            jerk: self.jerk,
            snap: self.snap,
            crakle: self.crakle,
            position_decimals: self.position_decimals,
            velocity_decimals: self.velocity_decimals,
            time_decimals: self.time_decimals,
        }
    }

    /// Set the velocity, acceleration, jerk, snap, and crackle of the
    /// waypoint to zero. Mirrors `Waypoint.stop()`.
    pub fn stop(&mut self) {
        self.vel = [0.0, 0.0, 0.0];
        self.acel = [0.0, 0.0, 0.0];
        self.jerk = [0.0, 0.0, 0.0];
        self.snap = [0.0, 0.0, 0.0];
        self.crakle = [0.0, 0.0, 0.0];
    }

    // ----------------------------------
    // -------- TIME MANAGEMENT ---------
    // ----------------------------------

    /// Postpone the waypoint's time by a given time step.
    ///
    /// Args:
    ///     - time_step (f64) : The amount of time to add to the
    ///       waypoint's current time.
    pub fn postpone(&mut self, time_step: f64) {
        // 1. Check if 'time_step' is smaller than precision time decimals
        if time_step < 1.0 / 10f64.powi(self.time_decimals as i32) {
            panic!(
                "Time step must be larger than the precision time decimals \
                 (1/{}), for time_step={}.",
                10f64.powi(self.time_decimals as i32),
                time_step
            );
        }

        // 2. Postpone the waypoint's time and round it to the specified
        //    number of decimals
        self.time += time_step;
        let m = 10f64.powi(self.time_decimals as i32);
        self.time = (self.time * m).round() / m;
    }

    /// Calculate the time difference between this waypoint and another
    /// waypoint. Mirrors `Waypoint.time_to()`.
    pub fn time_to(&self, other_wp: &Waypoint) -> f64 {
        other_wp.time - self.time
    }

    // ----------------------------------
    // -------- GEOMETRY MANAGEMENT -----
    // ----------------------------------

    /// Calculate the distance between this waypoint and another waypoint.
    /// Mirrors `Waypoint.distance_to()`.
    pub fn distance_to(&self, other_wp: &Waypoint) -> f64 {
        let dx = other_wp.pos[0] - self.pos[0];
        let dy = other_wp.pos[1] - self.pos[1];
        let dz = other_wp.pos[2] - self.pos[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Calculate the unit direction vector from this waypoint to another
    /// waypoint. Mirrors `Waypoint.direction_to()`.
    pub fn direction_to(&self, other_wp: &Waypoint) -> Vec3 {
        // 1. Avoid overhead of calling 'distance_to' by computing it
        //    directly here.
        let dx = other_wp.pos[0] - self.pos[0];
        let dy = other_wp.pos[1] - self.pos[1];
        let dz = other_wp.pos[2] - self.pos[2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();

        // 2. Return a zero vector if the distance is zero to avoid
        //    division by zero.
        if dist == 0.0 {
            [0.0, 0.0, 0.0]
        } else {
            [dx / dist, dy / dist, dz / dist]
        }
    }

    /// Calculate the course from this waypoint to another waypoint.
    /// Returns `(angle_rad, angle_deg)`.
    pub fn course_to(&self, other_wp: &Waypoint) -> (f64, f64) {
        // 1. Check if both waypoints are at the same position so that the
        //    course is undefined.
        let dist = self.distance_to(other_wp);
        if dist == 0.0 {
            return (0.0, 0.0);
        }

        // 2. Calculate the course angle
        let course_x = other_wp.pos[0] - self.pos[0];
        let course_y = other_wp.pos[1] - self.pos[1];

        let angle_rad = course_x.atan2(course_y);
        let angle_deg = angle_rad.to_degrees();

        (angle_rad, angle_deg)
    }

    /// Calculate the angle between the velocity vectors of this waypoint
    /// and another waypoint. Mirrors `Waypoint.angle_with()`.
    pub fn angle_with(&self, other_wp: &Waypoint) -> f64 {
        // 1. Compute the norms of both velocity vectors
        let norm_wp1_vel = (self.vel[0].powi(2) + self.vel[1].powi(2) + self.vel[2].powi(2)).sqrt();
        let norm_wp2_vel =
            (other_wp.vel[0].powi(2) + other_wp.vel[1].powi(2) + other_wp.vel[2].powi(2)).sqrt();

        // 2. If either velocity vector has a norm of zero, the angle is
        //    undefined; return 0 to avoid division by zero.
        if norm_wp1_vel == 0.0 || norm_wp2_vel == 0.0 {
            return 0.0;
        }

        // 3. Compute the angle from the dot-product formula.
        let cos_angle = (self.vel[0] * other_wp.vel[0]
            + self.vel[1] * other_wp.vel[1]
            + self.vel[2] * other_wp.vel[2])
            / (norm_wp1_vel * norm_wp2_vel);
        let cos_angle = cos_angle.clamp(-1.0, 1.0);

        let angle_rad = cos_angle.acos();

        // Mirror the Python round-to-position_decimals rounding.
        let m = 10f64.powi(self.position_decimals as i32);
        (angle_rad * m).round() / m
    }

    // ----------------------------------
    // -------- KINEMATIC MANAGEMENT -----
    // ----------------------------------

    /// Set the velocity of this waypoint to achieve uniform motion
    /// towards another waypoint. Mirrors `Waypoint.set_uniform_velocity()`.
    pub fn set_uniform_velocity(&mut self, other_wp: &Waypoint) {
        // 1. Stop current motion so the override is clean and doesn't
        //    accumulate previous velocity.
        self.stop();

        // 2. If the time difference is zero, raise an error to avoid
        //    division by zero.
        let time_diff = self.time_to(other_wp);
        if time_diff == 0.0 {
            panic!(
                "Time difference is 0, cannot set uniform velocity between \
                 waypoints {} and {}",
                self.id, other_wp.id
            );
        }

        // 3. Calculate the uniform velocity vector needed to reach
        //    other_wp in the given time.
        self.vel = [
            (other_wp.pos[0] - self.pos[0]) / time_diff,
            (other_wp.pos[1] - self.pos[1]) / time_diff,
            (other_wp.pos[2] - self.pos[2]) / time_diff,
        ];
        let m = 10f64.powi(self.velocity_decimals as i32);
        self.vel = [
            (self.vel[0] * m).round() / m,
            (self.vel[1] * m).round() / m,
            (self.vel[2] * m).round() / m,
        ];
    }

    /// Connect this waypoint to another waypoint by computing the
    /// kinematic derivatives (jerk, snap, crakle) so that a 5th-order
    /// Taylor polynomial matches `(r2, v2, a2)` at `t2`.
    ///
    /// Mirrors `Waypoint.connect_to()`. The system `A·X = B` is solved
    /// using Cramer's rule for 3x3 matrices (avoids the dependency on
    /// `nalgebra` for this hot path).
    pub fn connect_to(&mut self, other_wp: &Waypoint) {
        // 1. Extract the position, velocity, and acceleration of both
        //    waypoints.
        let r1 = self.pos;
        let v1 = self.vel;
        let a1 = self.acel;
        let r2 = other_wp.pos;
        let v2 = other_wp.vel;
        let a2 = other_wp.acel;

        // 2. If the distance between the two waypoints is zero, stop the
        //    motion and return.
        if self.distance_to(other_wp) == 0.0 {
            self.stop();
            return;
        }

        // 3. Calculate the time difference between the two waypoints.
        let t12 = self.time_to(other_wp);

        // 4. Construct the matrix A and vector B for the system of
        //    equations to solve for the kinematic derivatives.
        let t12_2 = t12 * t12;
        let t12_3 = t12_2 * t12;
        let t12_4 = t12_3 * t12;
        let t12_5 = t12_4 * t12;

        let a_mat = [
            [t12_3 / 6.0, t12_4 / 24.0, t12_5 / 120.0],
            [t12_2 / 2.0, t12_3 / 6.0, t12_4 / 24.0],
            [t12, t12_2 / 2.0, t12_3 / 6.0],
        ];
        let b_mat = [
            [
                r2[0] - r1[0] - v1[0] * t12 - 0.5 * a1[0] * t12_2,
                r2[1] - r1[1] - v1[1] * t12 - 0.5 * a1[1] * t12_2,
                r2[2] - r1[2] - v1[2] * t12 - 0.5 * a1[2] * t12_2,
            ],
            [
                v2[0] - v1[0] - a1[0] * t12,
                v2[1] - v1[1] - a1[1] * t12,
                v2[2] - v1[2] - a1[2] * t12,
            ],
            [a2[0] - a1[0], a2[1] - a1[1], a2[2] - a1[2]],
        ];

        // 5. Solve 3x3 systems PER AXIS with Cramer's rule. The 3x3
        //    system is the same for every axis (the matrix `a_mat`
        //    depends only on `t12`); only the right-hand side changes.
        let det_a = det3(a_mat);
        if det_a.abs() < 1e-12 {
            panic!("Error. 5th-Order Interpolation not possible.");
        }

        // x[i] is the solution for unknown i (i=0 -> jerk, 1 -> snap,
        // 2 -> crakle), per axis. We accumulate axis-by-axis.
        let mut jerk = [0.0_f64; 3];
        let mut snap = [0.0_f64; 3];
        let mut crakle = [0.0_f64; 3];

        for axis in 0..3 {
            // Build a_sub by replacing the column we want to solve for.
            // We solve independently for `jerk` (column 0), `snap`
            // (column 1) and `crakle` (column 2).
            for col_to_solve in 0..3 {
                let mut a_sub = a_mat;
                for row in 0..3 {
                    a_sub[row][col_to_solve] = b_mat[row][axis];
                }
                let x = det3(a_sub) / det_a;
                match col_to_solve {
                    0 => jerk[axis] = x,
                    1 => snap[axis] = x,
                    2 => crakle[axis] = x,
                    _ => unreachable!(),
                }
            }
        }

        // 6. Assign the calculated jerk, snap, and crakle values to this
        //    waypoint.
        self.jerk = jerk;
        self.snap = snap;
        self.crakle = crakle;
    }

    /// Interpolates a new waypoint from this waypoint's kinematic data at
    /// the specified time, using a 5th-order Taylor series expansion.
    /// Mirrors `Waypoint.interpolate()`.
    pub fn interpolate(&self, other_time: f64) -> Waypoint {
        // 1. Build a new waypoint object to hold the interpolated data.
        let mut interpolated_wp = Waypoint::default();
        interpolated_wp.time = other_time;

        // 2. Extract the current waypoint's data
        let r1 = self.pos;
        let v1 = self.vel;
        let a1 = self.acel;
        let j1 = self.jerk;
        let s1 = self.snap;
        let c1 = self.crakle;

        // 3. Calculate the time difference between the current waypoint
        //    and the specified time.
        let t12 = other_time - self.time;
        let t12_2 = t12 * t12;
        let t12_3 = t12_2 * t12;
        let t12_4 = t12_3 * t12;
        let t12_5 = t12_4 * t12;

        // 4. Use the Taylor series expansion to calculate the
        //    interpolated data at the specified time.
        let r2 = [
            r1[0]
                + v1[0] * t12
                + 0.5 * a1[0] * t12_2
                + (1.0 / 6.0) * j1[0] * t12_3
                + (1.0 / 24.0) * s1[0] * t12_4
                + (1.0 / 120.0) * c1[0] * t12_5,
            r1[1]
                + v1[1] * t12
                + 0.5 * a1[1] * t12_2
                + (1.0 / 6.0) * j1[1] * t12_3
                + (1.0 / 24.0) * s1[1] * t12_4
                + (1.0 / 120.0) * c1[1] * t12_5,
            r1[2]
                + v1[2] * t12
                + 0.5 * a1[2] * t12_2
                + (1.0 / 6.0) * j1[2] * t12_3
                + (1.0 / 24.0) * s1[2] * t12_4
                + (1.0 / 120.0) * c1[2] * t12_5,
        ];
        let v2 = [
            v1[0]
                + a1[0] * t12
                + 0.5 * j1[0] * t12_2
                + (1.0 / 6.0) * s1[0] * t12_3
                + (1.0 / 24.0) * c1[0] * t12_4,
            v1[1]
                + a1[1] * t12
                + 0.5 * j1[1] * t12_2
                + (1.0 / 6.0) * s1[1] * t12_3
                + (1.0 / 24.0) * c1[1] * t12_4,
            v1[2]
                + a1[2] * t12
                + 0.5 * j1[2] * t12_2
                + (1.0 / 6.0) * s1[2] * t12_3
                + (1.0 / 24.0) * c1[2] * t12_4,
        ];
        let a2 = [
            a1[0] + j1[0] * t12 + 0.5 * s1[0] * t12_2 + (1.0 / 6.0) * c1[0] * t12_3,
            a1[1] + j1[1] * t12 + 0.5 * s1[1] * t12_2 + (1.0 / 6.0) * c1[1] * t12_3,
            a1[2] + j1[2] * t12 + 0.5 * s1[2] * t12_2 + (1.0 / 6.0) * c1[2] * t12_3,
        ];
        let j2 = [j1[0] + s1[0] * t12 + 0.5 * c1[0] * t12_2,
                  j1[1] + s1[1] * t12 + 0.5 * c1[1] * t12_2,
                  j1[2] + s1[2] * t12 + 0.5 * c1[2] * t12_2];
        let s2 = [s1[0] + c1[0] * t12,
                  s1[1] + c1[1] * t12,
                  s1[2] + c1[2] * t12];
        let c2 = c1;

        // 5. Assign the interpolated data to the new waypoint object.
        interpolated_wp.pos = r2;
        interpolated_wp.vel = v2;
        interpolated_wp.acel = a2;
        interpolated_wp.jerk = j2;
        interpolated_wp.snap = s2;
        interpolated_wp.crakle = c2;

        interpolated_wp
    }
}

/// Determinant of a 3x3 matrix.
fn det3(m: [[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}
