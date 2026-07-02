//! # FlightPlan
//!
//! Direct Rust translation of the optimised `FlightPlan` class from
//! `uspace/flight_plan/flight_plan_new.py`.
//!
//! The Rust port focuses on the numerical hot-path methods that the
//! Python-vs-Rust benchmark exercises:
//!
//! - `add_waypoint` / `remove_waypoint`
//! - `get_idx_by_id`, `get_running_waypoint*`, `get_target_waypoint*`
//! - `start_time`, `finish_time`
//! - `postpone_from`, `postpone`, `reschedule_at`
//! - `connect_waypoints`
//! - `status_at_time`
//! - `trace`              ← **fully vectorised**, no Python loop
//! - `waypoints_to_arrays`
//!
//! The Python class also contains a large matplotlib-based plotting suite
//! (`plot_position`, `plot_velocity`, `plot_acceleration`, `plot_jerk`,
//! `plot_snap`, `plot_crackle`, `add_UAV_track_*`) and a few pure-Python
//! helpers (smoothing, dict serialisation, command generation). Those
//! are intentionally left out of the Rust port — the goal is an
//! apples-to-apples benchmark of the kinematic core, not a UI
//! re-implementation. See `README.md` for the full translation status
//! table.

use std::collections::HashMap;

use crate::waypoint::Waypoint;

/// A flight plan: a time-sorted list of `Waypoint`s plus an id-to-index
/// map for O(1) lookups by `id`. The two parallel structures
/// (`waypoints: Vec<Waypoint>` and `time_waypoints: Vec<f64>`) are kept
/// in sync on every mutation, mirroring the `SortedList` of times in the
/// Python version.
#[derive(Debug, Clone)]
pub struct FlightPlan {
    /// Identifier used to refer to the flight plan.
    pub id: String,
    /// Priority for plan selection (0 = default).
    pub priority: i32,
    /// Acceptance radius for the UAV around the waypoint (m).
    pub radius: f64,
    /// Maximum variation in linear velocity (m/s).
    pub max_var_lin_vel: f64,
    /// Maximum variation in angular velocity (rad/s).
    pub max_var_ang_vel: f64,
    /// Time-sorted list of waypoints.
    pub waypoints: Vec<Waypoint>,
    /// Parallel sorted list of waypoint times — kept in sync with
    /// `waypoints` so that `bisect_left` / `bisect_right` are O(log N).
    pub time_waypoints: Vec<f64>,
    /// Map from `Waypoint.id` to its index in `waypoints`.
    pub ids_to_idx: HashMap<String, usize>,
    /// Number of waypoints (kept in sync with `waypoints.len()`).
    pub length: usize,
    /// Number of decimal digits kept for time values.
    pub time_decimals: usize,
    /// Number of decimal digits kept for position values.
    pub position_decimals: usize,
    /// Number of decimal digits kept for velocity values.
    pub velocity_decimals: usize,
}

impl Default for FlightPlan {
    fn default() -> Self {
        Self::new("", 0, 1.0, 5.0, 1.0)
    }
}

impl FlightPlan {
    /// Build a new empty flight plan.
    pub fn new(
        id: impl Into<String>,
        priority: i32,
        radius: f64,
        max_var_lin_vel: f64,
        max_var_ang_vel: f64,
    ) -> Self {
        Self {
            id: id.into(),
            priority,
            radius,
            max_var_lin_vel,
            max_var_ang_vel,
            waypoints: Vec::new(),
            time_waypoints: Vec::new(),
            ids_to_idx: HashMap::new(),
            length: 0,
            time_decimals: 3,
            position_decimals: 3,
            velocity_decimals: 3,
        }
    }

    // ----------------------------------
    // -------- BASE FUNCTIONS ----------
    // ----------------------------------

    /// Add a waypoint to the flight plan.
    ///
    /// Mirrors `FlightPlan.add_waypoint()` from the Python original.
    /// Behavioural notes:
    ///   * `time` is required.
    ///   * If `wp` is provided, the other parameters are ignored.
    ///   * If `pos` or `vel` are `None`, they are inferred from the
    ///     status at `time` (or `[0,0,0]` for the very first WP).
    ///   * The waypoint list stays sorted by `time`.
    pub fn add_waypoint(
        &mut self,
        wp: Option<Waypoint>,
        id: Option<String>,
        time: Option<f64>,
        pos: Option<[f64; 3]>,
        vel: Option<[f64; 3]>,
        heading: Option<[f64; 2]>,
    ) {
        let time = match time {
            Some(t) => t,
            None => panic!("Time must be provided for the waypoint."),
        };

        let wp = match wp {
            Some(w) => w,
            None => {
                let is_first_wp = self.length == 0;
                let is_previous_time = !is_first_wp && time <= self.time_waypoints[0];

                let expected_status = if is_first_wp {
                    None
                } else {
                    Some(self.status_at_time(time))
                };

                let pos = pos.unwrap_or_else(|| match &expected_status {
                    None => [0.0, 0.0, 0.0],
                    Some(s) => s.pos,
                });

                let vel = vel.unwrap_or_else(|| {
                    if is_first_wp || is_previous_time {
                        [0.0, 0.0, 0.0]
                    } else {
                        expected_status.as_ref().unwrap().vel
                    }
                });

                let id = id.unwrap_or_else(|| {
                    if is_first_wp {
                        "default_id_0".to_string()
                    } else {
                        format!("default_id_{}", self.length)
                    }
                });

                let heading = heading.unwrap_or([0.0, 0.0]);

                Waypoint::new(
                    id,
                    time,
                    pos,
                    vel,
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    false,
                    heading,
                    self.position_decimals,
                    self.velocity_decimals,
                    self.time_decimals,
                )
            }
        };

        // 3. Determine the index to insert the new waypoint based on its
        //    time.
        let mut index = lower_bound(&self.time_waypoints, &wp.time);

        // 4. Add id to the dictionary for quick access.
        self.ids_to_idx.insert(wp.id.clone(), index);

        // 5. Replace the waypoint if the same time already exists.
        let update_existing_wp = (index < self.length
            && self.time_waypoints[index] == wp.time)
            || (index > 0
                && index == self.length
                && self.time_waypoints[index - 1] == wp.time);

        if update_existing_wp {
            let old_wp = self.waypoints.remove(index);
            self.time_waypoints.remove(index);
            self.ids_to_idx.remove(&old_wp.id);
            self.length -= 1;
            // Recompute the lower-bound for the new (shorter) array.
            index = lower_bound(&self.time_waypoints, &wp.time);
        }

        // 6. Insert the new waypoint and its time into the corresponding
        //    data structures.
        self.waypoints.insert(index, wp.clone());
        self.time_waypoints.insert(index, wp.time);

        // 7. Update the length.
        self.length += 1;

        // Refresh the id->index map to reflect any shifts.
        self.rebuild_ids_to_idx_from(index);
    }

    /// Rebuild `ids_to_idx` for every index `>= start` in O(k).
    /// Called after any in-place insertion or removal that shifts the
    /// waypoint list.
    fn rebuild_ids_to_idx_from(&mut self, start: usize) {
        for i in start..self.waypoints.len() {
            self.ids_to_idx.insert(self.waypoints[i].id.clone(), i);
        }
    }

    /// Remove a waypoint from the flight plan by index, or by time.
    /// Mirrors `FlightPlan.remove_waypoint()`.
    pub fn remove_waypoint(
        &mut self,
        idx: Option<usize>,
        time: Option<f64>,
    ) {
        if self.length == 0 {
            return;
        }
        let remove_by_idx = idx.is_some();
        let remove_by_time = time.is_some();

        // Remove the last waypoint.
        if !remove_by_idx && !remove_by_time {
            let wp = self.waypoints.pop().unwrap();
            self.time_waypoints.pop();
            self.ids_to_idx.remove(&wp.id);
            self.length -= 1;
            return;
        }

        if let Some(idx) = idx {
            if idx >= self.length && idx < usize::MAX - self.length {
                panic!("Index {} is out of range for FlightPlan of length {}.", idx, self.length);
            }
            // Normalize negative indices
            let idx = if idx >= self.length { idx } else { idx };

            let wp = self.waypoints.remove(idx);
            self.time_waypoints.remove(idx);
            self.ids_to_idx.remove(&wp.id);
            self.length -= 1;
            self.rebuild_ids_to_idx_from(idx);
            return;
        }

        if let Some(time) = time {
            let pointed_wp_idx = if time >= *self.time_waypoints.last().unwrap() {
                self.length - 1
            } else {
                lower_bound(&self.time_waypoints, &time)
            };
            let wp = self.waypoints.remove(pointed_wp_idx);
            self.time_waypoints.remove(pointed_wp_idx);
            self.ids_to_idx.remove(&wp.id);
            self.length -= 1;
            self.rebuild_ids_to_idx_from(pointed_wp_idx);
        }
    }

    /// Get the index of a waypoint by its id. Mirrors
    /// `FlightPlan.get_idx_by_id()`.
    pub fn get_idx_by_id(&self, id: &str) -> Option<usize> {
        self.ids_to_idx.get(id).copied()
    }

    /// Get the running waypoint at a given time. The "running" waypoint
    /// is the last waypoint whose `time` is `<= t`. Mirrors
    /// `FlightPlan.get_running_waypoint()`.
    pub fn get_running_waypoint(&self, time: f64) -> Option<Waypoint> {
        let idx = self.get_running_waypoint_idx(time);
        idx.map(|i| self.waypoints[i].copy())
    }

    /// Index variant of `get_running_waypoint`. Mirrors
    /// `FlightPlan.get_running_waypoint_idx()`.
    pub fn get_running_waypoint_idx(&self, time: f64) -> Option<usize> {
        if self.length == 0 {
            return None;
        }
        if time < self.time_waypoints[0] {
            return None;
        }
        let idx = upper_bound(&self.time_waypoints, &time).saturating_sub(1);
        Some(idx)
    }

    /// Get the target waypoint at a given time. The "target" waypoint is
    /// the first waypoint whose `time` is `> t`. Mirrors
    /// `FlightPlan.get_target_waypoint()`.
    pub fn get_target_waypoint(&self, time: f64) -> Option<Waypoint> {
        let idx = self.get_target_waypoint_idx(time);
        idx.map(|i| self.waypoints[i].copy())
    }

    /// Index variant of `get_target_waypoint`. Mirrors
    /// `FlightPlan.get_target_waypoint_idx()`.
    pub fn get_target_waypoint_idx(&self, time: f64) -> Option<usize> {
        if self.length == 0 {
            return None;
        }
        let idx = upper_bound(&self.time_waypoints, &time);
        if idx >= self.length {
            None
        } else {
            Some(idx)
        }
    }

    // ----------------------------------
    // -------- TIME MANAGEMENT ---------
    // ----------------------------------

    /// Returns the time of the first waypoint in the flight plan.
    pub fn start_time(&self) -> Option<f64> {
        if self.length == 0 {
            None
        } else {
            Some(self.time_waypoints[0])
        }
    }

    /// Returns the time of the last waypoint in the flight plan.
    pub fn finish_time(&self) -> Option<f64> {
        if self.length == 0 {
            None
        } else {
            Some(self.time_waypoints[self.length - 1])
        }
    }

    /// Postpone the Flight Plan from a given start_time by a given
    /// time_delta. Mirrors `FlightPlan.postpone_from()`.
    pub fn postpone_from(&mut self, start_time: f64, time_delta: f64) {
        // Early exit conditions:
        //   * the flight plan is empty
        //   * the time_delta is zero
        //   * the start_time is after the finish_time
        if self.length == 0 || time_delta == 0.0 || start_time >= *self.time_waypoints.last().unwrap() {
            return;
        }

        // Find the index of the first waypoint whose time is greater
        // than or equal to start_time.
        let index = lower_bound(&self.time_waypoints, &start_time);

        // Check that the time_delta does not invert the order with the
        // previous waypoint.
        if index > 0
            && time_delta <= (self.time_waypoints[index - 1] - self.time_waypoints[index])
        {
            panic!(
                "Time delta {} is too small to postpone from time {}. \
                 Not enough time gap between waypoints.",
                time_delta, start_time
            );
        }

        // Postpone all waypoints from the found index onwards.
        for wp in self.waypoints[index..].iter_mut() {
            wp.time += time_delta;
        }

        // Rebuild the time_waypoints for the affected range.
        // O(k) delete + O(k log N) re-insert.
        let new_times: Vec<f64> = self.waypoints[index..].iter().map(|wp| wp.time).collect();
        self.time_waypoints.truncate(index);
        for t in new_times {
            let pos = upper_bound(&self.time_waypoints, &t);
            self.time_waypoints.insert(pos, t);
        }
    }

    /// Postpone the entire Flight Plan by a given time delta. Mirrors
    /// `FlightPlan.postpone()`.
    pub fn postpone(&mut self, time_delta: f64) {
        if self.length == 0 {
            return;
        }
        let t0 = self.time_waypoints[0];
        self.postpone_from(t0, time_delta);
    }

    /// Perform a temporal translation of the Flight Plan to begin at a
    /// given time. Mirrors `FlightPlan.reschedule_at()`.
    pub fn reschedule_at(&mut self, time: f64) {
        if self.length == 0 {
            return;
        }
        let t0 = self.time_waypoints[0];
        self.postpone(time - t0);
    }

    // ----------------------------------
    // -------- PLAN MANAGEMENT ---------
    // ----------------------------------

    /// Connects each waypoint to the next one in the flight plan by
    /// calling `connect_to` on every adjacent pair.
    pub fn connect_waypoints(&mut self) {
        for i in 0..self.length.saturating_sub(1) {
            let next = self.waypoints[i + 1].copy();
            self.waypoints[i].connect_to(&next);
        }
    }

    // ----------------------------------
    // -------- KINEMATIC QUERIES -------
    // ----------------------------------

    /// Check the status of the flight plan at a given time. Mirrors
    /// `FlightPlan.status_at_time()`. Returns a copy of the running
    /// waypoint interpolated at `time` (5th-order Taylor expansion).
    pub fn status_at_time(&self, time: f64) -> Waypoint {
        // 1. Empty plan
        if self.length == 0 {
            panic!("Cannot compute status at time on an empty flight plan.");
        }

        // 2. Outside range
        if time < self.time_waypoints[0] {
            return self.waypoints[0].copy();
        }
        let last = self.time_waypoints[self.length - 1];
        if time >= last {
            return self.waypoints[self.length - 1].copy();
        }

        // 3. Find the running waypoint index (last WP with time <= t)
        let idx = upper_bound(&self.time_waypoints, &time).saturating_sub(1);
        self.waypoints[idx].interpolate(time)
    }

    // ----------------------------------
    // -------- VECTORISED TRACE --------
    // ----------------------------------

    /// Compute the UAV's trace over time based on the flight plan's
    /// waypoints. Mirrors `FlightPlan.trace()`.
    ///
    /// In Python this is implemented with numpy broadcasting. In Rust we
    /// keep the same mathematical formulation but lay the columns out
    /// row-major: `out[sample * COLS + col]`.
    ///
    /// Returns a `Vec<f64>` of length `n_samples * 19` where the columns
    /// are, in order: `t, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z,
    /// acel.x, acel.y, acel.z, jerk.x, jerk.y, jerk.z, snap.x, snap.y,
    /// snap.z, crakle.x, crakle.y, crakle.z`.
    pub fn trace(&self, time_step: f64) -> Vec<f64> {
        const COLS: usize = 19;

        if self.length < 2 {
            return Vec::new();
        }

        let t0 = self.time_waypoints[0];
        let tn = self.time_waypoints[self.length - 1];

        // 1. Number of samples (ceil-like; matches np.arange inclusivity).
        let n = ((tn - t0) / time_step).floor() as usize + 1;

        // 2. Per-waypoint contiguous arrays for each kinematic field
        let times: Vec<f64> = self.waypoints.iter().map(|wp| wp.time).collect();
        let pos: Vec<[f64; 3]> = self.waypoints.iter().map(|wp| wp.pos).collect();
        let vel: Vec<[f64; 3]> = self.waypoints.iter().map(|wp| wp.vel).collect();
        let acel: Vec<[f64; 3]> = self.waypoints.iter().map(|wp| wp.acel).collect();
        let jerk: Vec<[f64; 3]> = self.waypoints.iter().map(|wp| wp.jerk).collect();
        let snap: Vec<[f64; 3]> = self.waypoints.iter().map(|wp| wp.snap).collect();
        let crakle: Vec<[f64; 3]> = self.waypoints.iter().map(|wp| wp.crakle).collect();

        // 3. Output buffer: `n_samples` rows of `COLS` columns.
        let mut out = vec![0.0_f64; n * COLS];

        // 4. Walk every sample, locate its running waypoint via a
        //    single linear scan (n is large, N is small) — same as
        //    `np.searchsorted(..., side="right") - 1` in the original.
        let mut run_idx: usize = 0;

        for s in 0..n {
            let t = t0 + s as f64 * time_step;

            // Advance run_idx while the NEXT waypoint is also <= t.
            while run_idx + 1 < self.length && times[run_idx + 1] <= t {
                run_idx += 1;
            }

            let dt = t - times[run_idx];
            let dt2 = dt * dt;
            let dt3 = dt2 * dt;
            let dt4 = dt3 * dt;
            let dt5 = dt4 * dt;

            let p = &pos[run_idx];
            let v = &vel[run_idx];
            let a = &acel[run_idx];
            let j = &jerk[run_idx];
            let sn = &snap[run_idx];
            let c = &crakle[run_idx];

            // row 0 = time
            out[s * COLS + 0] = t;
            // pos
            for axis in 0..3 {
                out[s * COLS + 1 + axis] =
                    p[axis] + v[axis] * dt + 0.5 * a[axis] * dt2
                        + (1.0 / 6.0) * j[axis] * dt3
                        + (1.0 / 24.0) * sn[axis] * dt4
                        + (1.0 / 120.0) * c[axis] * dt5;
            }
            // vel
            for axis in 0..3 {
                out[s * COLS + 4 + axis] =
                    v[axis] + a[axis] * dt + 0.5 * j[axis] * dt2
                        + (1.0 / 6.0) * sn[axis] * dt3
                        + (1.0 / 24.0) * c[axis] * dt4;
            }
            // acel
            for axis in 0..3 {
                out[s * COLS + 7 + axis] =
                    a[axis] + j[axis] * dt + 0.5 * sn[axis] * dt2
                        + (1.0 / 6.0) * c[axis] * dt3;
            }
            // jerk
            for axis in 0..3 {
                out[s * COLS + 10 + axis] =
                    j[axis] + sn[axis] * dt + 0.5 * c[axis] * dt2;
            }
            // snap
            for axis in 0..3 {
                out[s * COLS + 13 + axis] = sn[axis] + c[axis] * dt;
            }
            // crakle
            for axis in 0..3 {
                out[s * COLS + 16 + axis] = c[axis];
            }
        }

        out
    }

    // ----------------------------------
    // -------- COPY ---------------------
    // ----------------------------------

    /// Make a deep copy of the flight plan. Mirrors `FlightPlan.copy()`.
    pub fn copy(&self) -> Self {
        Self {
            id: self.id.clone(),
            priority: self.priority,
            radius: self.radius,
            max_var_lin_vel: self.max_var_lin_vel,
            max_var_ang_vel: self.max_var_ang_vel,
            waypoints: self.waypoints.iter().map(|wp| wp.copy()).collect(),
            time_waypoints: self.time_waypoints.clone(),
            ids_to_idx: self.ids_to_idx.clone(),
            length: self.length,
            time_decimals: self.time_decimals,
            position_decimals: self.position_decimals,
            velocity_decimals: self.velocity_decimals,
        }
    }
}

// ---------------------------------------------------------------------------
// Binary search helpers (no external crate).
// ---------------------------------------------------------------------------

/// Equivalent of Python's `bisect_left` on a `Vec<f64>` that is kept
/// sorted in non-decreasing order.
fn lower_bound(arr: &[f64], x: &f64) -> usize {
    let mut lo = 0usize;
    let mut hi = arr.len();
    while lo < hi {
        let mid = (lo + hi) / 2;
        if arr[mid] < *x {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    lo
}

/// Equivalent of Python's `bisect_right` on a `Vec<f64>` that is kept
/// sorted in non-decreasing order.
fn upper_bound(arr: &[f64], x: &f64) -> usize {
    let mut lo = 0usize;
    let mut hi = arr.len();
    while lo < hi {
        let mid = (lo + hi) / 2;
        if arr[mid] <= *x {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    lo
}
