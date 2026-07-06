//! REST handlers for the FlightPlan editor.

use std::sync::Arc;

use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::{IntoResponse, Response},
    Json,
};
use flight_plan::Waypoint;
use serde::Deserialize;
use serde_json::json;

use crate::state::{AppState, PlanDto, PlanListDto, PlanSummaryDto, WaypointDto, DEFAULT_TRACE_DT};

/// HTTP error type used by every handler.
#[derive(Debug, thiserror::Error)]
pub enum ApiError {
    #[error("plan '{0}' not found")]
    PlanNotFound(String),
    #[error("waypoint '{0}' not found in plan '{1}'")]
    WaypointNotFound(String, String),
    #[error("bad request: {0}")]
    BadRequest(String),
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let (status, msg) = match &self {
            ApiError::PlanNotFound(_) => (StatusCode::NOT_FOUND, self.to_string()),
            ApiError::WaypointNotFound(_, _) => (StatusCode::NOT_FOUND, self.to_string()),
            ApiError::BadRequest(_) => (StatusCode::BAD_REQUEST, self.to_string()),
        };
        (status, Json(json!({ "error": msg }))).into_response()
    }
}

pub type ApiResult<T> = Result<T, ApiError>;

/// List every flight plan + current simulation clock.
pub async fn list_plans(State(app): State<Arc<AppState>>) -> ApiResult<Json<PlanListDto>> {
    let plans = app.plans().lock();
    let visible = app.visible().lock();
    let sim_time = *app.sim_time().lock();

    let out = PlanListDto {
        plans: plans
            .values()
            .map(|p| PlanSummaryDto {
                id: p.id.clone(),
                priority: p.priority,
                radius: p.radius,
                length: p.length,
                start_time: p.start_time(),
                finish_time: p.finish_time(),
                visible: *visible.get(&p.id).unwrap_or(&true),
            })
            .collect(),
        sim_time,
    };
    Ok(Json(out))
}

/// Get a single flight plan + its latest trace.
pub async fn get_plan(
    State(app): State<Arc<AppState>>,
    Path(plan_id): Path<String>,
) -> ApiResult<Json<PlanDto>> {
    let visible = app.visible().lock();
    let dto = {
        let plans = app.plans().lock();
        let plan = plans
            .get(&plan_id)
            .ok_or_else(|| ApiError::PlanNotFound(plan_id.clone()))?;
        PlanDto::from_plan(plan, *visible.get(&plan_id).unwrap_or(&true))
    };
    Ok(Json(dto))
}

#[derive(Debug, Deserialize)]
pub struct CreatePlanRequest {
    pub id: Option<String>,
    #[serde(default)]
    pub priority: i32,
    #[serde(default = "default_radius")]
    pub radius: f64,
    #[serde(default = "default_lin_vel")]
    pub max_var_lin_vel: f64,
    #[serde(default = "default_ang_vel")]
    pub max_var_ang_vel: f64,
    /// If true, copy `waypoints` (Vec<WaypointDto>) into the new plan.
    #[serde(default)]
    pub waypoints: Vec<WaypointDto>,
}

fn default_radius() -> f64 { 1.0 }
fn default_lin_vel() -> f64 { 5.0 }
fn default_ang_vel() -> f64 { 1.0 }

/// Create a new flight plan. Returns the plan + its trace.
pub async fn create_plan(
    State(app): State<Arc<AppState>>,
    Json(req): Json<CreatePlanRequest>,
) -> ApiResult<Json<PlanDto>> {
    let id = req.id.unwrap_or_else(|| app.next_plan_id());


    let mut plan = flight_plan::FlightPlan::new(
        id.clone(),
        req.priority,
        req.radius,
        req.max_var_lin_vel,
        req.max_var_ang_vel,
    );
    for w in req.waypoints {
        plan.add_waypoint(
            Some(Waypoint::from(w)),
            None,
            None,
            None,
            None,
            None,
        );
    }
    plan.connect_waypoints();

    let dto = PlanDto::from_plan(&plan, true);
    {
        let mut plans = app.plans().lock();
        plans.insert(id.clone(), plan);
    }
    Ok(Json(dto))
}

/// Delete a flight plan.
pub async fn delete_plan(
    State(app): State<Arc<AppState>>,
    Path(plan_id): Path<String>,
) -> ApiResult<StatusCode> {
    let removed = {
        let mut plans = app.plans().lock();
        plans.remove(&plan_id).is_some()
    };
    if !removed {
        return Err(ApiError::PlanNotFound(plan_id));
    }
    {
        let mut visible = app.visible().lock();
        visible.remove(&plan_id);
    }
    Ok(StatusCode::NO_CONTENT)
}

#[derive(Debug, Deserialize)]
pub struct UpdatePlanAttributesRequest {
    pub priority: Option<i32>,
    pub radius: Option<f64>,
    pub max_var_lin_vel: Option<f64>,
    pub max_var_ang_vel: Option<f64>,
}

/// Modify flight-plan-level attributes (`priority`, `radius`, ...).
pub async fn update_plan(
    State(app): State<Arc<AppState>>,
    Path(plan_id): Path<String>,
    Json(req): Json<UpdatePlanAttributesRequest>,
) -> ApiResult<Json<PlanDto>> {
    let visible = {
        let mut plans = app.plans().lock();
        let visible_map = app.visible().lock();
        let plan = plans
            .get_mut(&plan_id)
            .ok_or_else(|| ApiError::PlanNotFound(plan_id.clone()))?;
        if let Some(p) = req.priority { plan.priority = p; }
        if let Some(r) = req.radius   { plan.radius   = r; }
        if let Some(v) = req.max_var_lin_vel { plan.max_var_lin_vel = v; }
        if let Some(w) = req.max_var_ang_vel { plan.max_var_ang_vel = w; }
        PlanDto::from_plan(plan, *visible_map.get(&plan_id).unwrap_or(&true))
    };
    Ok(Json(visible))
}

#[derive(Debug, Deserialize)]
pub struct AddWaypointRequest {
    pub id: Option<String>,
    pub time: f64,
    pub pos: [f64; 3],
    pub vel: Option<[f64; 3]>,
    pub fly_over: Option<bool>,
    pub heading: Option<[f64; 2]>,
}

/// Add a waypoint to a flight plan.
pub async fn add_waypoint(
    State(app): State<Arc<AppState>>,
    Path(plan_id): Path<String>,
    Json(req): Json<AddWaypointRequest>,
) -> ApiResult<Json<PlanDto>> {
    let visible_map = app.visible().lock().clone();
    let dto = {
        let mut plans = app.plans().lock();
        let plan = plans
            .get_mut(&plan_id)
            .ok_or_else(|| ApiError::PlanNotFound(plan_id.clone()))?;

        let pos = req.pos;
        let vel = req.vel.unwrap_or([0.0, 0.0, 0.0]);
        let id = req.id.unwrap_or_else(|| {
            format!("{}-wp{}", plan_id, plan.length)
        });
        let heading = req.heading.unwrap_or([0.0, 0.0]);

        mutate_and_connect(plan, |p| {
            let wp = Waypoint::new(
                id.clone(),
                req.time,
                pos,
                vel,
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                req.fly_over.unwrap_or(false),
                heading,
                p.position_decimals,
                p.velocity_decimals,
                p.time_decimals,
            );
            p.add_waypoint(Some(wp), None, Some(req.time), Some(pos), Some(vel), Some(heading));
        });
        PlanDto::from_plan(plan, *visible_map.get(&plan_id).unwrap_or(&true))
    };
    Ok(Json(dto))
}

#[derive(Debug, Deserialize)]
pub struct PatchWaypointRequest {
    pub id: String,
    pub time: Option<f64>,
    pub pos: Option<[f64; 3]>,
    pub vel: Option<[f64; 3]>,
    pub fly_over: Option<bool>,
    pub heading: Option<[f64; 2]>,
}

/// Update an existing waypoint (`id` + any combination of fields).
/// When `time` changes, the waypoint is removed and reinserted at the
/// correct sorted position so the flight plan stays time-ordered.
/// After every successful mutation we re-run `connect_waypoints()` so
/// the C⁵ trace stays correct, no matter which waypoint moved.
pub async fn patch_waypoint(
    State(app): State<Arc<AppState>>,
    Path(plan_id): Path<String>,
    Json(req): Json<PatchWaypointRequest>,
) -> ApiResult<Json<PlanDto>> {
    let visible_map = app.visible().lock().clone();
    let dto = {
        let mut plans = app.plans().lock();
        let plan = plans
            .get_mut(&plan_id)
            .ok_or_else(|| ApiError::PlanNotFound(plan_id.clone()))?;

        let new_time = req.time;
        let new_pos = req.pos;
        let new_vel = req.vel;
        let fly_over_change = req.fly_over;
        let heading_change = req.heading;

        mutate_and_connect(plan, |p| {
            let idx = match p.get_idx_by_id(&req.id) {
                Some(i) => i,
                None    => return,
            };
            let cur = p.waypoints[idx].copy();
            let nt = new_time.unwrap_or(cur.time);
            let np = new_pos.unwrap_or(cur.pos);
            let nv = new_vel.unwrap_or(cur.vel);
            p.replace_waypoint(&req.id, nt, np, nv);
            if let Some(fo) = fly_over_change {
                if let Some(j) = p.get_idx_by_id(&req.id) {
                    p.waypoints[j].fly_over = fo;
                }
            }
            if let Some(h) = heading_change {
                if let Some(j) = p.get_idx_by_id(&req.id) {
                    p.waypoints[j].heading = h;
                }
            }
        });
        PlanDto::from_plan(plan, *visible_map.get(&plan_id).unwrap_or(&true))
    };
    Ok(Json(dto))
}

/// Delete a waypoint by id.
pub async fn delete_waypoint(
    State(app): State<Arc<AppState>>,
    Path(plan_id): Path<String>,
    Json(req): Json<PatchWaypointRequest>,
) -> ApiResult<Json<PlanDto>> {
    let visible_map = app.visible().lock().clone();
    let dto = {
        let mut plans = app.plans().lock();
        let plan = plans
            .get_mut(&plan_id)
            .ok_or_else(|| ApiError::PlanNotFound(plan_id.clone()))?;
        if !plan.remove_waypoint_by_id(&req.id) {
            return Err(ApiError::WaypointNotFound(req.id, plan_id));
        }
        PlanDto::from_plan(plan, *visible_map.get(&plan_id).unwrap_or(&true))
    };
    Ok(Json(dto))
}

/// Re-derive jerk/snap/crakle from current pos / vel / time.
pub async fn connect_waypoints(
    State(app): State<Arc<AppState>>,
    Path(plan_id): Path<String>,
) -> ApiResult<Json<PlanDto>> {
    let visible_map = app.visible().lock().clone();
    let dto = {
        let mut plans = app.plans().lock();
        let plan = plans
            .get_mut(&plan_id)
            .ok_or_else(|| ApiError::PlanNotFound(plan_id.clone()))?;
        plan.connect_waypoints();
        PlanDto::from_plan(plan, *visible_map.get(&plan_id).unwrap_or(&true))
    };
    Ok(Json(dto))
}

/// Run a closure that mutates the flight plan, then call
/// `connect_waypoints()` so the trace stays C⁵ at all times.
/// Any UI mutation goes through this helper.
fn mutate_and_connect(
    plan: &mut flight_plan::FlightPlan,
    mutator: impl FnOnce(&mut flight_plan::FlightPlan),
) {
    mutator(plan);
    plan.connect_waypoints();
}

#[derive(Debug, Deserialize)]
pub struct VisibilityRequest {
    pub visible: bool,
}

/// Toggle a plan's visibility in the UI.
pub async fn set_visibility(
    State(app): State<Arc<AppState>>,
    Path(plan_id): Path<String>,
    Json(req): Json<VisibilityRequest>,
) -> ApiResult<Json<PlanDto>> {
    {
        let plans = app.plans().lock();
        if !plans.contains_key(&plan_id) {
            return Err(ApiError::PlanNotFound(plan_id));
        }
    }
    app.visible().lock().insert(plan_id.clone(), req.visible);
    let visible_map = app.visible().lock().clone();
    let dto = {
        let plans = app.plans().lock();
        let plan = plans.get(&plan_id).unwrap();
        PlanDto::from_plan(plan, *visible_map.get(&plan_id).unwrap_or(&true))
    };
    Ok(Json(dto))
}

#[derive(Debug, Deserialize)]
pub struct TraceQuery {
    pub dt: Option<f64>,
}

/// Get the current trace of a flight plan.
pub async fn get_trace(
    State(app): State<Arc<AppState>>,
    Path(plan_id): Path<String>,
    axum::extract::Query(q): axum::extract::Query<TraceQuery>,
) -> ApiResult<Json<crate::state::TraceDto>> {
    let dt = q.dt.unwrap_or(DEFAULT_TRACE_DT);
    if dt <= 0.0 || !dt.is_finite() {
        return Err(ApiError::BadRequest("dt must be positive and finite".into()));
    }
    let trace = {
        let plans = app.plans().lock();
        let plan = plans
            .get(&plan_id)
            .ok_or_else(|| ApiError::PlanNotFound(plan_id))?;
        plan.trace_struct(dt)
    };
    Ok(Json(crate::state::TraceDto { dt, ..trace.into() }))
}

#[derive(Debug, Deserialize, Default)]
pub struct SimRequest {
    pub reset: Option<bool>,
    pub time: Option<f64>,
}

/// Set the simulation clock. With `reset=true` the clock goes back to
/// zero; otherwise it's a hard-set to the requested time.
pub async fn set_sim(
    State(app): State<Arc<AppState>>,
    Json(req): Json<SimRequest>,
) -> ApiResult<Json<serde_json::Value>> {
    {
        let mut st = app.sim_time().lock();
        if req.reset.unwrap_or(false) || req.time.is_none() {
            *st = 0.0;
        } else {
            *st = req.time.unwrap();
        }
    }
    Ok(Json(json!({ "sim_time": *app.sim_time().lock() })))
}

/// Read the current simulation clock.
pub async fn get_sim(State(app): State<Arc<AppState>>) -> Json<serde_json::Value> {
    Json(json!({ "sim_time": *app.sim_time().lock() }))
}

/// Health check endpoint.
pub async fn health(State(app): State<Arc<AppState>>) -> Json<serde_json::Value> {
    let plans = app.plans().lock();
    Json(json!({
        "status":   "ok",
        "length":   plans.len(),
        "sim_time": *app.sim_time().lock(),
    }))
}
