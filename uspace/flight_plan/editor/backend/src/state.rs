//! Shared application state for the FlightPlan editor backend.
//!
//! The state holds a registry of named [`FlightPlan`]s and a single
//! simulation clock. All API handlers receive `Arc<AppState>` and
//! operate on the registry under a parking_lot mutex.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use parking_lot::Mutex;
use serde::{Deserialize, Serialize};

use flight_plan::{FlightPlan, Trace};

/// Default sampling step for `GET /api/plans/{id}/trace`.
/// Smaller values give smoother visualisations at the cost of
/// payload size: 0.01 s = 100 samples per simulated second and keeps
/// the returned JSON well under ~2 MB for a 200 s flight plan.
pub const DEFAULT_TRACE_DT: f64 = 0.01;

/// Shared application state.
#[derive(Clone)]
pub struct AppState {
    inner: Arc<Inner>,
}

struct Inner {
    /// Registry of flight plans keyed by plan id.
    plans: Mutex<BTreeMap<String, FlightPlan>>,
    /// UI display toggle: which plans are currently visible (per the
    /// "show / hide" UI). Kept separate from the kinematic state.
    visible: Mutex<BTreeMap<String, bool>>,
    /// Simulation clock in seconds (monotonic since first reset).
    sim_time: Mutex<f64>,
    /// Monotonic counter used to assign short ids.
    next_id: AtomicU64,
    /// Static-files directory served at the URL root (`dist/`).
    /// Kept on the state for future endpoints that need it
    /// (e.g. an `/upload` that writes into `dist/...`).
    #[allow(dead_code)]
    dist_dir: PathBuf,
}

impl AppState {
    /// Build the initial (empty) state. The user has to click
    /// "New plan" to create their first flight plan.
    pub fn new(dist_dir: PathBuf) -> Self {
        Self {
            inner: Arc::new(Inner {
                plans: Mutex::new(BTreeMap::new()),
                visible: Mutex::new(BTreeMap::new()),
                sim_time: Mutex::new(0.0),
                next_id: AtomicU64::new(1),
                #[allow(dead_code)]
                dist_dir,
            }),
        }
    }

    /// Convenience: lock the flight-plan registry.
    pub fn plans(&self) -> &Mutex<BTreeMap<String, FlightPlan>> {
        &self.inner.plans
    }

    /// Convenience: lock the visibility registry.
    pub fn visible(&self) -> &Mutex<BTreeMap<String, bool>> {
        &self.inner.visible
    }

    /// Convenience: lock the simulation clock (seconds).
    pub fn sim_time(&self) -> &Mutex<f64> {
        &self.inner.sim_time
    }

    /// Static-files root (`dist/`) — kept for future endpoints that
    /// need it.
    #[allow(dead_code)]
    pub fn dist_dir(&self) -> &PathBuf {
        &self.inner.dist_dir
    }

    /// Generate a short, unique plan id (e.g. `Plan3`).
    pub fn next_plan_id(&self) -> String {
        let n = self.inner.next_id.fetch_add(1, Ordering::SeqCst);
        format!("Plan{n}")
    }
}

// ---------------------------------------------------------------------------
// Wire types
// ---------------------------------------------------------------------------

/// Serialized waypoint returned to the frontend.
///
/// Mirrors `flight_plan::Waypoint` with one cosmetic difference: every
/// 3-vector is a plain JSON array (the Rust crate stores them as
/// `[f64; 3]` which serde flattens either way, but we annotate the
/// shape for the docs).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaypointDto {
    pub id: String,
    pub fly_over: bool,
    pub time: f64,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub acel: [f64; 3],
    pub jerk: [f64; 3],
    pub snap: [f64; 3],
    pub crakle: [f64; 3],
    pub heading: [f64; 2],
    pub position_decimals: usize,
    pub velocity_decimals: usize,
    pub time_decimals: usize,
}

impl From<flight_plan::Waypoint> for WaypointDto {
    fn from(w: flight_plan::Waypoint) -> Self {
        Self {
            id: w.id,
            fly_over: w.fly_over,
            time: w.time,
            pos: w.pos,
            vel: w.vel,
            acel: w.acel,
            jerk: w.jerk,
            snap: w.snap,
            crakle: w.crakle,
            heading: w.heading,
            position_decimals: w.position_decimals,
            velocity_decimals: w.velocity_decimals,
            time_decimals: w.time_decimals,
        }
    }
}

impl From<WaypointDto> for flight_plan::Waypoint {
    fn from(w: WaypointDto) -> Self {
        Self::new(
            w.id,
            w.time,
            w.pos,
            w.vel,
            w.acel,
            w.jerk,
            w.snap,
            w.crakle,
            w.fly_over,
            w.heading,
            w.position_decimals,
            w.velocity_decimals,
            w.time_decimals,
        )
    }
}

/// Serialized flight plan returned to the frontend.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanDto {
    pub id: String,
    pub priority: i32,
    pub radius: f64,
    pub max_var_lin_vel: f64,
    pub max_var_ang_vel: f64,
    pub waypoints: Vec<WaypointDto>,
    pub length: usize,
    pub start_time: Option<f64>,
    pub finish_time: Option<f64>,
    /// Last computed trace. Attached so the derivatives chart and the
    /// 3D viewer can refresh in a single round-trip.
    pub trace: TraceDto,
    /// UI-only: whether the frontend should display this plan.
    pub visible: bool,
}

impl PlanDto {
    pub fn from_plan(p: &FlightPlan, visible: bool) -> Self {
        let trace = p.trace_struct(DEFAULT_TRACE_DT);
        Self {
            id: p.id.clone(),
            priority: p.priority,
            radius: p.radius,
            max_var_lin_vel: p.max_var_lin_vel,
            max_var_ang_vel: p.max_var_ang_vel,
            waypoints: p.waypoints.iter().cloned().map(WaypointDto::from).collect(),
            length: p.length,
            start_time: p.start_time(),
            finish_time: p.finish_time(),
            trace: TraceDto::from(trace),
            visible,
        }
    }
}

/// Column-split trace returned to the frontend.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TraceDto {
    pub dt: f64,
    pub t: Vec<f64>,
    pub pos: Vec<[f64; 3]>,
    pub vel: Vec<[f64; 3]>,
    pub acel: Vec<[f64; 3]>,
    pub jerk: Vec<[f64; 3]>,
    pub snap: Vec<[f64; 3]>,
    pub crakle: Vec<[f64; 3]>,
}

impl From<Trace> for TraceDto {
    fn from(t: Trace) -> Self {
        Self {
            dt: DEFAULT_TRACE_DT,
            t: t.t,
            pos: t.pos,
            vel: t.vel,
            acel: t.acel,
            jerk: t.jerk,
            snap: t.snap,
            crakle: t.crakle,
        }
    }
}

/// List of all plans with their visibility flags.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanListDto {
    pub plans: Vec<PlanSummaryDto>,
    pub sim_time: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanSummaryDto {
    pub id: String,
    pub priority: i32,
    pub radius: f64,
    pub length: usize,
    pub start_time: Option<f64>,
    pub finish_time: Option<f64>,
    pub visible: bool,
}
