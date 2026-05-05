"""
velocity_bounding.py — Strategy 1: Kinematic Bounding (ConnectURM2)
====================================================================

PURPOSE:
    Attempt to resolve a UAV conflict by modifying ONLY the speed of the plebeian
    UAV along its original straight route — no spatial deviation is introduced.

ALGORITHM:
    1. Kinematic Guard Clause: verify that the conflict segment is rectilinear
       (straight line). ConnectURM2 uses Euclidean distance m = p_{i+1} - p_i,
       which is only valid for straight segments. If the segment is curved
       (non-zero higher-order derivatives), Strategy 1 bails out immediately
       and the cascade falls through to Strategy 2 (SAT MTVs).

    2. Geometric Decision: project both UAVs to t_anchor and determine whether
       the plebeian passes in FRONT of or BEHIND the VIP at the conflict point.
           - Passes in front → ACCELERATE (reach the conflict zone first and clear it)
           - Passes behind   → BRAKE      (let the VIP pass first)

    3. Golden Rule (Inversion): If the required Δv exceeds max_var_lin_vel:
           - Invert the maneuver (switch from ACCELERATE to BRAKE or vice versa)
           - If the inverted maneuver also exceeds the limit → STRATEGY 1 FAILS

    4. Step Search (MAX_ITERATIONS): For each candidate speed:
           a. Copy the PRISTINE original flight plan (never a rejected candidate)
           b. Apply ConnectURM2: straight-line connection at the candidate speed
              with temporal propagation to all future waypoints (Ripple Effect)
           c. Validate the COMPLETE plan (shifted timestamps) against the shadow
              R-Tree to detect Temporal Domino Effect conflicts
           d. If valid → return it. If not → discard, try next speed step.

CONNECTURM2 ALGORITHM (from literature):
    Given a flight plan f, segment index i, and desired speed v:
    1. Extract positions p_i, p_{i+1} and times t_i, t_{i+1}
    2. Distance vector: m = p_{i+1} - p_i,  magnitude: |m|
    3. New segment time: t_seg = |m| / v
    4. Update velocity: v_i = m / t_seg
    5. Temporal displacement: t_des = (t_i + t_seg) - t_{i+1}
    6. Ripple Effect: for k = i+1..n: t_k += t_des

SAFETY INVARIANT:
    Every candidate FlightPlan is validated against the live R-Tree before being
    accepted. No plan is returned unless it is globally conflict-free.
"""

from __future__ import annotations

import numpy as np
from typing import Optional, TYPE_CHECKING

from core.models.flight_plan import FlightPlan
from core.config import (
    MAX_ITERATIONS,
    OBB_INTERVAL,
    NORM_ZERO_THRESHOLD,
    VIP_SPEED_SAMPLE_DELTA,
    CLEARANCE_TIME_BUFFER,
    MIN_SPEED_CHANGE,
    S1_MIN_SPEED,
    RECTILINEAR_THRESHOLD,
    FORWARD_PROGRESS_MARGIN,
)

if TYPE_CHECKING:
    from core.models.waypoint import Waypoint
    from detection.rtree_detector import RTreeDetector


# ---------------------------------------------------------------------------
# Helpers: Segment Geometry
# ---------------------------------------------------------------------------

def _find_segment_index(fp: FlightPlan, t: float) -> Optional[int]:
    """
    Find the index i such that waypoints[i].t <= t < waypoints[i+1].t,
    identifying the active segment at time t.

    Returns None if t falls outside the flight plan or there are fewer
    than 2 waypoints.
    """
    wps = fp.waypoints
    if len(wps) < 2:
        return None

    for i in range(len(wps) - 1):
        if wps[i].t <= t < wps[i + 1].t:
            return i

    # Edge case: t exactly equals the last waypoint time
    if t == wps[-1].t and len(wps) >= 2:
        return len(wps) - 2

    return None


def _is_segment_rectilinear(wp_i: Waypoint, wp_next: Waypoint) -> bool:
    """
    Check whether the segment w_i → w_{i+1} is rectilinear (straight line,
    constant velocity).  ConnectURM2 is only mathematically valid for such
    segments.

    A segment is rectilinear if the higher-order derivatives at w_i are
    all near-zero, meaning the degree-5 polynomial reduces to a linear
    function (Uniform Rectilinear Motion).

    If any derivative (acceleration, jerk, snap, crakle) exceeds
    RECTILINEAR_THRESHOLD, the segment is considered curved and Strategy 1
    must bail out immediately.

    Args:
        wp_i:    Starting waypoint of the segment.
        wp_next: Ending waypoint of the segment.

    Returns:
        True if the segment is straight (safe for ConnectURM2),
        False if the segment is curved (ConnectURM2 would be invalid).
    """
    threshold = RECTILINEAR_THRESHOLD
    if np.linalg.norm(wp_i.acel) > threshold:
        return False
    if np.linalg.norm(wp_i.jerk) > threshold:
        return False
    if np.linalg.norm(wp_i.snap) > threshold:
        return False
    if np.linalg.norm(wp_i.crakle) > threshold:
        return False
    return True


# ---------------------------------------------------------------------------
# Helpers: Position / Speed Queries
# ---------------------------------------------------------------------------

def _project_position(fp: FlightPlan, t: float) -> np.ndarray:
    """Return the 3D position of fp's UAV at time t."""
    return fp.status_at_time(t).pos.copy()


def _current_speed(fp: FlightPlan, t: float) -> float:
    """Return the scalar speed of fp's UAV at time t."""
    return float(np.linalg.norm(fp.status_at_time(t).vel))


# ---------------------------------------------------------------------------
# Helpers: Maneuver Decision
# ---------------------------------------------------------------------------

def _decide_maneuver_direction(
    fp_pleb: FlightPlan,
    fp_vip:  FlightPlan,
    t_anchor: float,
) -> str:
    """
    Determine whether the plebeian should ACCELERATE or BRAKE.

    Geometric logic:
        1. Project both UAVs to t_anchor.
        2. Compute the conflict crossing point: the midpoint between them at t_anchor.
        3. Use the VIP velocity direction as the reference axis.
        4. Project the plebeian relative position onto the VIP forward axis:
               - positive projection → plebeian is AHEAD of the VIP → ACCELERATE
               - negative projection → plebeian is BEHIND the VIP  → BRAKE

    Returns:
        "ACCELERATE" or "BRAKE"
    """
    pos_pleb = _project_position(fp_pleb, t_anchor)
    pos_vip  = _project_position(fp_vip,  t_anchor)
    vel_vip  = fp_vip.status_at_time(t_anchor).vel

    norm_vel_vip = np.linalg.norm(vel_vip)
    if norm_vel_vip < NORM_ZERO_THRESHOLD:
        # VIP is stationary: check relative position along plebeian velocity
        vel_pleb = fp_pleb.status_at_time(t_anchor).vel
        norm_vel_pleb = np.linalg.norm(vel_pleb)
        if norm_vel_pleb < NORM_ZERO_THRESHOLD:
            return "BRAKE"  # Both stationary — default to brake
        vip_forward = vel_pleb / norm_vel_pleb
    else:
        vip_forward = vel_vip / norm_vel_vip

    # Vector from VIP to plebeian
    relative = pos_pleb - pos_vip

    # Project plebeian position onto VIP forward axis
    projection = np.dot(relative, vip_forward)

    return "ACCELERATE" if projection > 0 else "BRAKE"


def _compute_required_delta_v(
    fp_pleb:  FlightPlan,
    fp_vip:   FlightPlan,
    t_anchor: float,
    maneuver: str,
) -> Optional[float]:
    """
    Estimate the required Δv so that the plebeian no longer occupies the conflict zone
    at the same time as the VIP, staying on the original straight route.

    Approach:
        - Find when the VIP exits the conflict zone (t_vip_clear).
        - The plebeian must either arrive BEFORE the VIP enters (ACCELERATE)
          or depart AFTER the VIP exits (BRAKE).
        - Compute the speed needed to shift the plebeian's arrival time appropriately.

    Returns:
        The required |Δv| (always positive), or None if the geometry is degenerate.
    """
    pos_pleb_anchor = _project_position(fp_pleb, t_anchor)
    pos_vip_anchor  = _project_position(fp_vip,  t_anchor)

    # Midpoint = conflict zone center approximation
    conflict_center = (pos_pleb_anchor + pos_vip_anchor) / 2.0

    # Find the next waypoint of the plebeian after t_anchor
    target_idx = fp_pleb.get_target_index_from_time(t_anchor)
    if target_idx >= len(fp_pleb.waypoints):
        return None
    next_wp = fp_pleb.waypoints[target_idx]

    # Distance from anchor to next waypoint
    dist_to_next = float(np.linalg.norm(next_wp.pos - pos_pleb_anchor))
    if dist_to_next < NORM_ZERO_THRESHOLD:
        return None

    # Distance from anchor to conflict center along the original route direction
    route_dir = (next_wp.pos - pos_pleb_anchor) / dist_to_next
    dist_to_conflict = float(np.dot(conflict_center - pos_pleb_anchor, route_dir))
    if dist_to_conflict <= 0:
        dist_to_conflict = dist_to_next / 2.0

    # Current speed of the plebeian
    v_current = _current_speed(fp_pleb, t_anchor)
    if v_current < NORM_ZERO_THRESHOLD:
        v_current = float(np.linalg.norm(fp_pleb.status_at_time(t_anchor + VIP_SPEED_SAMPLE_DELTA).vel))
        if v_current < NORM_ZERO_THRESHOLD:
            return None


    # Estimate when the VIP passes through the same region
    # Use VIP speed and distance to conflict center
    dist_vip_to_conflict = float(np.linalg.norm(conflict_center - pos_vip_anchor))
    v_vip = _current_speed(fp_vip, t_anchor)
    if v_vip < NORM_ZERO_THRESHOLD:
        # VIP is hovering — treat it as a static obstacle; BRAKE strategy
        # plebeian needs to wait until after t_vip_clear (VIP's end time)
        t_vip_clear = fp_vip.finish_time()
    else:
        t_vip_clear = t_anchor + dist_vip_to_conflict / v_vip

    # Safety buffer: one UAV radius on each side → 2× radius time margin
    radius_sum = fp_pleb.radius + fp_vip.radius
    t_margin = radius_sum / v_vip if v_vip > NORM_ZERO_THRESHOLD else radius_sum

    if maneuver == "ACCELERATE":
        # Plebeian must arrive at conflict center BEFORE (t_vip_clear - t_margin)
        t_target_arrival = t_vip_clear - t_margin - CLEARANCE_TIME_BUFFER
        if t_target_arrival <= t_anchor:
            return None  # Cannot accelerate enough
        required_speed = dist_to_conflict / (t_target_arrival - t_anchor)
    else:  # BRAKE
        # Plebeian must depart conflict center AFTER (t_vip_clear + t_margin)
        t_target_arrival = t_vip_clear + t_margin + CLEARANCE_TIME_BUFFER
        required_speed = dist_to_conflict / (t_target_arrival - t_anchor)

    delta_v = abs(required_speed - v_current)
    return delta_v


# ---------------------------------------------------------------------------
# ConnectURM2 Algorithm (Literature)
# ---------------------------------------------------------------------------

def connect_urm2(fp: FlightPlan, i: int, v: float) -> FlightPlan:
    """
    ConnectURM2 algorithm: connect waypoint w_i to w_{i+1} in a straight line
    at constant scalar speed v.  Propagate the temporal displacement (delay
    or advance) to ALL future waypoints — the Ripple Effect.

    Mathematical steps:
        1. Extract positions p_i, p_{i+1} and times t_i, t_{i+1}
        2. Distance vector: m = p_{i+1} - p_i,   magnitude: |m|
        3. New segment time: t_seg = |m| / v
        4. Update velocity of w_i: v_i = m / t_seg  (direction preserved)
        5. Temporal displacement: t_des = (t_i + t_seg) - t_{i+1}
           - t_des > 0 → drone arrives later  (speed was reduced)
           - t_des < 0 → drone arrives earlier (speed was increased)
        6. Ripple Effect: for all k from i+1 to n: t_k += t_des

    Args:
        fp: Flight plan (a deep copy is made internally — original is not mutated).
        i:  Index of the starting waypoint of the segment being modified.
        v:  Desired scalar speed [m/s].  Must be > 0.

    Returns:
        Modified FlightPlan with propagated timestamps, recalculated uniform
        velocities, and reconnected interpolation polynomials.
    """
    new_fp = fp.copy()
    wps = new_fp.waypoints

    w_i    = wps[i]
    w_next = wps[i + 1]

    # Step 1: Extract positions and times
    p_i  = w_i.pos.copy()
    t_i  = w_i.t
    p_next = w_next.pos.copy()
    t_next = w_next.t

    # Step 2: Compute physical distance
    m = p_next - p_i                           # displacement vector
    m_mag = float(np.linalg.norm(m))           # scalar distance |m|
    if m_mag < NORM_ZERO_THRESHOLD:
        return new_fp  # Coincident waypoints — nothing to do

    # Step 3: Compute new segment time
    t_segment = m_mag / v                      # t = |m| / v

    # Step 4: Update velocity of waypoint i (URM: zero acceleration)
    w_i.vel  = m / t_segment                   # v_i = m / t
    w_i.acel = np.zeros(3)                     # pure URM: no acceleration
    w_i.jerk = np.zeros(3)
    w_i.snap = np.zeros(3)
    w_i.crakle = np.zeros(3)

    # Step 5: Compute temporal displacement
    t_des = (t_i + t_segment) - t_next         # t_des = (t_i + t) - t_{i+1}

    # Step 6: Ripple Effect — propagate to all future waypoints
    for k in range(i + 1, len(wps)):
        wps[k].t = round(wps[k].t + t_des, 6)  # algebraic sum

    # Reconnect interpolation polynomials (jerk, snap, crakle) for the
    # affected segments.  set_uniform_velocity() is NOT called because:
    #   - Segment i already has the correct velocity set in Step 4.
    #   - All other segments retain their original Δt (uniform shift), so
    #     their velocities remain correct.
    new_fp.connect_waypoints()

    return new_fp


# ---------------------------------------------------------------------------
# Shadow R-Tree Validation
# ---------------------------------------------------------------------------

def _build_shadow_rtree(
    pleb_id: str,
    manager: "RTreeDetector",
) -> "RTreeDetector":
    """
    Build a shadow R-Tree containing ALL UAVs EXCEPT the plebeian.

    This is created ONCE before the Step Search loop and reused for every
    candidate evaluation.  Only the plebeian's entry is swapped in/out
    via register_uav(), which already removes old boxes before inserting
    new ones — so no state leaks between candidates.

    Returns:
        A RTreeDetector with every UAV except pleb_id registered.
    """
    from detection.rtree_detector import RTreeDetector
    shadow = RTreeDetector()

    for uid, data in manager.uavs.items():
        if uid != pleb_id:
            shadow.register_uav(uid, data["fp"], interval=OBB_INTERVAL)

    return shadow


def _validate_against_shadow(
    candidate_fp: FlightPlan,
    pleb_id: str,
    shadow: "RTreeDetector",
    t_conflict_being_solved: float = 0.0,
) -> bool:
    """
    Swap the plebeian's entry in the pre-built shadow R-Tree with the
    candidate FlightPlan and check for conflicts.

    FORWARD PROGRESS GUARANTEE:
    If the route still contains conflicts, the system accepts it ONLY if 
    the earliest remaining conflict is strictly and comfortably after the 
    one we are actively solving. This allows iterative resolution of multiple
    conflicts without DEADLOCK.
    """
    shadow.register_uav(pleb_id, candidate_fp, interval=OBB_INTERVAL)
    conflicts = shadow.detect_all_conflicts(pleb_id)
    if len(conflicts) == 0:
        return True
        
    # Check for forward progress
    conflicts.sort(key=lambda c: c["time_range"][0])
    earliest_new_conflict = conflicts[0]["time_range"][0]
    
    return earliest_new_conflict >= t_conflict_being_solved + FORWARD_PROGRESS_MARGIN


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def run_strategy1(
    fp_pleb:  FlightPlan,
    fp_vip:   FlightPlan,
    pleb_id:  str,
    t_anchor: float,
    manager:  "RTreeDetector",
    t_conflict: float = 0.0,
) -> Optional[FlightPlan]:
    """
    Execute Strategy 1: Kinematic Bounding via ConnectURM2.

    Implements the full pipeline:
        0. Find the segment containing t_anchor
        1. GUARD CLAUSE: verify the segment is rectilinear (straight line).
           If curved → return None immediately (Strategy 1 is invalid).
        2. Decide maneuver direction (ACCELERATE / BRAKE)
        3. Compute required Δv
        4. Apply Golden Rule (inversion if over physical limit)
        5. Step Search: iterate candidate speeds from target_speed toward
           v_current, applying ConnectURM2 + temporal propagation for each.
        6. For each candidate: validate the COMPLETE plan vs shadow R-Tree.

    Args:
        fp_pleb:  Plebeian's current FlightPlan (not mutated).
        fp_vip:   VIP's FlightPlan (read-only reference).
        pleb_id:  UAV identifier for the plebeian (used in R-Tree queries).
        t_anchor: Temporal anchor point (WCET + margin already applied by caller).
        manager:  Active RTreeDetector with all registered UAVs.

    Returns:
        A conflict-free FlightPlan if Strategy 1 succeeds, or None if it fails
        (caller should proceed to Phase 2: SAT + Strategy 2).
    """
    # =========================================================================
    # Step 0: Find the flight plan segment containing t_anchor
    # =========================================================================
    segment_idx = _find_segment_index(fp_pleb, t_anchor)
    if segment_idx is None:
        return None  # t_anchor outside flight plan bounds

    # =========================================================================
    # Step 1: GUARD CLAUSE — Kinematic Safety Check
    #
    # ConnectURM2 uses m = p_{i+1} - p_i (Euclidean straight line).
    # If the segment is curved (smooth_waypoint_speed / smooth_waypoint_duration),
    # this formula would "cut" through the curve interior, destroying the
    # original mission geometry.  Strategy 1 is declared INFEASIBLE and
    # the cascade falls through to Strategy 2 (SAT MTVs + spatial detour).
    # =========================================================================
    wp_i    = fp_pleb.waypoints[segment_idx]
    wp_next = fp_pleb.waypoints[segment_idx + 1]

    if not _is_segment_rectilinear(wp_i, wp_next):
        return None  # Curved segment — ConnectURM2 is invalid

    v_current = _current_speed(fp_pleb, t_anchor)

    # Maximum speed change physically permitted
    max_dv: float = fp_pleb.max_var_lin_vel

    # =========================================================================
    # Step 2: Geometric Decision
    # =========================================================================
    maneuver = _decide_maneuver_direction(fp_pleb, fp_vip, t_anchor)

    # =========================================================================
    # Step 3: Compute required Δv for the chosen maneuver
    # =========================================================================
    delta_v = _compute_required_delta_v(fp_pleb, fp_vip, t_anchor, maneuver)
    if delta_v is None:
        return None  # Degenerate geometry — Strategy 1 cannot operate

    # =========================================================================
    # Step 4: Golden Rule (Inversion)
    # =========================================================================
    if delta_v > max_dv:
        # Primary maneuver exceeds physical limit — invert
        alt_maneuver = "BRAKE" if maneuver == "ACCELERATE" else "ACCELERATE"
        alt_delta_v = _compute_required_delta_v(fp_pleb, fp_vip, t_anchor, alt_maneuver)

        if alt_delta_v is None or alt_delta_v > max_dv:
            # Inverted maneuver also exceeds limit → Strategy 1 fails immediately
            return None

        # Use the inverted maneuver
        maneuver = alt_maneuver
        delta_v = alt_delta_v

    # =========================================================================
    # Step 5: Step Search — ConnectURM2 with temporal propagation
    # =========================================================================
    # Compute target speed after the maneuver
    if maneuver == "ACCELERATE":
        target_speed = v_current + delta_v
    else:  # BRAKE
        target_speed = max(S1_MIN_SPEED, v_current - delta_v)

    # Generate candidate speeds: interpolate from target_speed toward v_current
    # (we try the most effective fix first and relax if needed)
    candidate_speeds = np.linspace(target_speed, v_current, MAX_ITERATIONS + 1)[:-1]
    # This gives MAX_ITERATIONS candidates between target_speed and v_current (exclusive)

    # Build the shadow R-Tree ONCE: register every UAV except the plebeian.
    # Each candidate only swaps the plebeian's entry via register_uav(),
    # which already removes old boxes before inserting new ones.
    shadow = _build_shadow_rtree(pleb_id, manager)

    for iteration, speed_candidate in enumerate(candidate_speeds):
        # Skip if speed change is negligible
        if abs(speed_candidate - v_current) < MIN_SPEED_CHANGE:
            continue

        # Enforce minimum speed floor
        if speed_candidate < S1_MIN_SPEED:
            continue

        # =================================================================
        # Step 6: Apply ConnectURM2 on a PRISTINE copy and validate
        #
        # CRITICAL: Each iteration starts from fp_pleb (the original,
        # unmodified plan). Never reuse a rejected candidate — that would
        # accumulate temporal errors across iterations.
        # =================================================================
        candidate_fp = connect_urm2(
            fp=fp_pleb,
            i=segment_idx,
            v=speed_candidate,
        )

        if _validate_against_shadow(candidate_fp, pleb_id, shadow, t_conflict):
            # Accepted — return the first valid candidate
            return candidate_fp

    # All iterations exhausted without finding a valid plan
    return None
