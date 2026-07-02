//! # Command
//!
//! Direct Rust translation of `uspace/flight_plan/command.py`.
//!
//! A `Command` represents the instantaneous instruction sent to a UAV
//! (rotor state, linear velocity, angular velocity, and command duration).

/// UAV command: rotors on/off, desired linear and angular velocity, duration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// Whether the rotors should be active.
    pub on: bool,
    /// Desired linear velocity along the X axis (m/s).
    pub vel_x: f64,
    /// Desired linear velocity along the Y axis (m/s).
    pub vel_y: f64,
    /// Desired linear velocity along the Z axis (m/s).
    pub vel_z: f64,
    /// Desired angular velocity around the Z axis (rad/s).
    pub rot_z: f64,
    /// Duration of the command (s).
    pub duration: f64,
}

impl Command {
    /// Build a new `Command` with the given parameters (defaults mirror the
    /// Python constructor).
    pub fn new(
        on: bool,
        vel_x: f64,
        vel_y: f64,
        vel_z: f64,
        rot_z: f64,
        duration: f64,
    ) -> Self {
        Self {
            on,
            vel_x,
            vel_y,
            vel_z,
            rot_z,
            duration,
        }
    }

    /// Set every field of the command at once.
    ///
    /// Args:
    ///     on (bool) : Whether the rotors should be active or not.
    ///     vel_x (f64) : Linear velocity desired in X axis (m/s).
    ///     vel_y (f64) : Linear velocity desired in Y axis (m/s).
    ///     vel_z (f64) : Linear velocity desired in Z axis (m/s).
    ///     rot_z (f64) : Angular velocity desired in Z axis (rad/s).
    ///     duration (f64) : Duration of the command (s).
    pub fn set(
        &mut self,
        on: bool,
        vel_x: f64,
        vel_y: f64,
        vel_z: f64,
        rot_z: f64,
        duration: f64,
    ) {
        self.on = on;
        self.vel_x = vel_x;
        self.vel_y = vel_y;
        self.vel_z = vel_z;
        self.rot_z = rot_z;
        self.duration = duration;
    }

    /// Turns off the command. Mirrors `Command.off()`.
    pub fn off(&mut self, duration: f64) {
        self.set(false, 0.0, 0.0, 0.0, 0.0, duration);
    }

    /// Sets the command to hover (on=True, velocities=0). Mirrors
    /// `Command.hover()`.
    pub fn hover(&mut self, duration: f64) {
        self.set(true, 0.0, 0.0, 0.0, 0.0, duration);
    }
}

impl Default for Command {
    fn default() -> Self {
        Self::new(false, 0.0, 0.0, 0.0, 0.0, 0.0)
    }
}
