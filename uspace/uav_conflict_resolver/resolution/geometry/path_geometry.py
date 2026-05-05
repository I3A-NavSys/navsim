"""
path_geometry.py — Spatial Detour Constructor
==============================================

PURPOSE:
    Build a modified FlightPlan for the plebeian UAV that avoids a conflict zone
    by inserting a spatial displacement (the "detour") at the anchor point.

    This module provides TWO detour builders:

    build_spatial_detour()      — Legacy triangle (anc → det → ret).
                                  Still used as a fallback when no conflict_obb
                                  is available (e.g. degenerate geometry).

    build_trapezoid_detour()    — PRIMARY.  Trapezoid (anc → det1 → det2 → ret).
                                  Uses heuristic MTV scaling to find a conflict-free
                                  route via the Shadow R-Tree.

TRAPEZOID GEOMETRY:
    Original route:
        ──────────[anc]────────────────────────────[next_wp]──

    After build_trapezoid_detour():
        ──────────[anc]──[det1]────────────[det2]──[ret]──[next_wp]──
                       ↗  (flat top, inside safe OBB)   ↘
              ramp-in                                  ramp-out

    The flat top (det1 → det2) is routed PARALLEL to the original path but
    displaced by the MTV — exactly through the centre of the safe OBB.

KINEMATIC VALIDATION:
    Before accepting a detour, we check that the angular velocity at the turn
    points does not exceed max_var_ang_vel (validate_curve_kinematics).
"""

from __future__ import annotations

import numpy as np
from typing import Optional, TYPE_CHECKING

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from core.config import (
    NORM_ZERO_THRESHOLD,
    DETOUR_APEX_TIME_FRACTION,
    DETOUR_RETURN_TIME_FRACTION,
    TRAP_DETOUR_RAMP_IN_FRACTION,
    TRAP_DETOUR_TOP_END_FRACTION,
    TRAP_DETOUR_RAMP_OUT_FRACTION,
    MIN_DETOUR_DURATION,
    POST_CONFLICT_BUFFER,
)

if TYPE_CHECKING:
    from detection.conflictDetection import SweptBox_OBB


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def build_spatial_detour(
    fp: FlightPlan,
    t_anchor: float,
    mtv: np.ndarray,
) -> Optional[FlightPlan]:
    """
    Build a new FlightPlan with a spatial detour at t_anchor displaced by mtv.

    The operation is performed on a deep copy of fp — the original is never mutated.

    Strategy:
        1. Find the anchor position: where the plebeian is at t_anchor.
        2. Find the next waypoint after t_anchor (the "target" the drone was heading to).
        3. Insert a detour waypoint at (anchor_pos + mtv) between t_anchor and t_target.
        4. Insert a return waypoint that smoothly rejoins the original route.
        5. Reconnect all waypoints with set_uniform_velocity + connect_waypoints.

    Args:
        fp:       Original FlightPlan (will NOT be mutated).
        t_anchor: The temporal anchor point. The detour starts here.
        mtv:      The displacement vector to apply at the apex of the detour.

    Returns:
        A new FlightPlan with the detour inserted, or None if the geometry is
        degenerate (e.g., mtv is zero, or anchor is beyond the flight plan end).
    """
    # Guard: anchor must be within the flight plan window
    if t_anchor >= fp.finish_time():
        return None

    mtv = np.array(mtv, dtype=float)
    if np.linalg.norm(mtv) < NORM_ZERO_THRESHOLD:
        return None  # Zero-displacement detour is meaningless

    # Deep copy so we never mutate the original flight plan
    new_fp: FlightPlan = fp.copy()

    # =========================================================================
    # Step 1: Determine key positions
    # =========================================================================
    # Position at the anchor time (where the drone is when the maneuver starts)
    status_anchor = new_fp.status_at_time(t_anchor)
    anchor_pos: np.ndarray = status_anchor.pos.copy()

    # Next scheduled waypoint after t_anchor (where the drone was originally heading)
    target_idx = new_fp.get_target_index_from_time(t_anchor)
    if target_idx >= len(new_fp.waypoints):
        return None  # No waypoint after anchor
    next_wp: Waypoint = new_fp.waypoints[target_idx]

    # =========================================================================
    # Step 2: Compute detour apex and return positions
    # =========================================================================
    # Apex of the detour: anchor position shifted by the MTV
    detour_pos: np.ndarray = anchor_pos + mtv

    # Time budget between anchor and next waypoint
    t_to_next: float = next_wp.t - t_anchor
    if t_to_next <= 0:
        return None

    # Detour apex time: proportionally in the middle of the available window.
    # We allocate 40% of the time budget to reach the apex and 60% to return,
    # giving more time to the return leg (which is longer since it must rejoin
    # the original route at a point ahead of the original target).
    t_detour: float = t_anchor + DETOUR_APEX_TIME_FRACTION * t_to_next
    t_return: float = t_anchor + DETOUR_RETURN_TIME_FRACTION * t_to_next

    if t_detour <= t_anchor or t_return <= t_detour or t_return >= next_wp.t:
        # Not enough time window to place three waypoints
        return None

    # Return position: a point on the original route slightly before next_wp
    # so the drone re-enters the original path smoothly
    return_pos: np.ndarray = new_fp.status_at_time(t_return).pos.copy()

    # =========================================================================
    # Step 3: Build the anchor waypoint (stop & start of maneuver)
    # =========================================================================
    anchor_wp = Waypoint(
        label="anc",
        t=t_anchor,
        pos=anchor_pos,
        vel=[0.0, 0.0, 0.0],  # Velocity will be corrected by set_uniform_velocity
    )

    # =========================================================================
    # Step 4: Build the detour apex waypoint
    # =========================================================================
    detour_wp = Waypoint(
        label="det",
        t=t_detour,
        pos=detour_pos,
        vel=[0.0, 0.0, 0.0],
    )

    # =========================================================================
    # Step 5: Build the return waypoint
    # =========================================================================
    return_wp = Waypoint(
        label="ret",
        t=t_return,
        pos=return_pos,
        vel=[0.0, 0.0, 0.0],
    )

    # =========================================================================
    # Step 6: Insert waypoints into the copied flight plan
    # We use set_waypoint which handles insertion at the correct sorted position.
    # =========================================================================
    new_fp.set_waypoint(anchor_wp)
    new_fp.set_waypoint(detour_wp)
    new_fp.set_waypoint(return_wp)

    # =========================================================================
    # Step 7: Recompute uniform velocities for the modified segment and
    # reconnect all waypoints with their polynomial derivatives.
    # =========================================================================
    new_fp.set_uniform_velocity()
    new_fp.connect_waypoints()

    return new_fp


def validate_curve_kinematics(
    wp_prev: Waypoint,
    wp_turn: Waypoint,
    wp_next: Waypoint,
    max_ang_vel: float,
) -> bool:
    """
    Validate that the angular velocity at a turn waypoint is within physical limits.

    The drone must be able to change heading direction between wp_prev→wp_turn
    and wp_turn→wp_next without exceeding max_ang_vel.

    The required angular velocity is estimated as:
        required_ang_vel = angle_between_legs / time_available_at_turn

    Where:
        - angle_between_legs: the heading change in radians using wp_turn.angle_with(wp_prev/next)
        - time_available_at_turn: half the average of the two leg durations

    Args:
        wp_prev:     The waypoint before the turn.
        wp_turn:     The turn waypoint to validate.
        wp_next:     The waypoint after the turn.
        max_ang_vel: Maximum angular velocity of the UAV [rad/s] (fp.max_var_ang_vel).

    Returns:
        True if the turn is kinematically feasible, False otherwise.
    """
    if max_ang_vel <= 0:
        return False

    # Get the velocity vectors at the incoming and outgoing legs
    vel_in  = wp_turn.pos - wp_prev.pos
    vel_out = wp_next.pos - wp_turn.pos

    norm_in  = np.linalg.norm(vel_in)
    norm_out = np.linalg.norm(vel_out)

    if norm_in < NORM_ZERO_THRESHOLD or norm_out < NORM_ZERO_THRESHOLD:
        # One of the legs has zero length — degenerate, consider it valid
        # (the drone is hovering, no angular constraint applies)
        return True

    # Angle between the two direction vectors (incoming and outgoing legs)
    cos_angle = np.dot(vel_in, vel_out) / (norm_in * norm_out)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)

    # Time available at the turn: use the shorter leg duration as the constraint
    t_in  = wp_turn.t - wp_prev.t
    t_out = wp_next.t - wp_turn.t

    if t_in <= 0 or t_out <= 0:
        return False

    # Conservative estimate: the full turn must happen within the shorter leg time
    time_available = min(t_in, t_out)

    required_ang_vel = angle_rad / time_available

    return required_ang_vel <= max_ang_vel


# ---------------------------------------------------------------------------
# Primary detour builder: Trapezoid + Bézier hull guarantee
# ---------------------------------------------------------------------------

def build_trapezoid_detour(
    fp:           FlightPlan,
    t_anchor:     float,
    mtv:          np.ndarray,
    conflict_obbs: Optional[List["SweptBox_OBB"]] = None,
) -> Optional[FlightPlan]:
    """
    Build a trapezoid detour (anc → det1 → det2 → ret) using a heuristic
    translation based on the MTV.

    ALGORITHM
    ---------
    1. Anchor position and next waypoint are retrieved.
    2. If ``conflict_obb`` is provided, the safe center is computed (conflict_obb
       displaced by ``mtv``) and det1/det2 are placed at the entry/exit corners
       of that safe zone along its forward axis (axes[0]).
       Otherwise, the function degrades gracefully to a simple single-point
       detour (same behaviour as build_spatial_detour).
    3. Time fractions TRAP_DETOUR_{RAMP_IN,TOP_END,RAMP_OUT}_FRACTION allocate
       the available time budget [t_anchor, next_wp.t].
    4. Velocity at det1 and det2 is set to the natural flat-crossing speed
       (L / Δt_flat along axes[0]) and capped at fp.max_var_lin_vel.
    5. The four waypoints are inserted into a copy of fp and
       connect_waypoints() is called to commit the polynomial.

    Args:
        fp:           Original FlightPlan (not mutated).
        t_anchor:     Temporal anchor (manoeuvre start time).
        mtv:          Displacement vector (already SAFETY_MTV_SCALE-scaled).
        conflict_obb: The OBB flagged as colliding (from the SAT result).
                      If None, degrades to build_spatial_detour behaviour.

    Returns:
        A FlightPlan candidate (to be validated by the Shadow R-Tree),
        or None if geometry is degenerate.
    """
    # ------------------------------------------------------------------
    # Guard: anchor must be within the flight plan window
    # ------------------------------------------------------------------
    if t_anchor >= fp.finish_time():
        return None

    mtv = np.array(mtv, dtype=float)
    if np.linalg.norm(mtv) < NORM_ZERO_THRESHOLD:
        return None

    # ------------------------------------------------------------------
    # Step 1: Retrieve anchor and next waypoint
    # ------------------------------------------------------------------
    status_anchor = fp.status_at_time(t_anchor)
    anchor_pos: np.ndarray = status_anchor.pos.copy()
    anchor_vel: np.ndarray = status_anchor.vel.copy()

    if conflict_obbs:
        t_max_conflict = conflict_obbs[-1].t_range[1]
        t_target = max(t_anchor + MIN_DETOUR_DURATION, t_max_conflict + POST_CONFLICT_BUFFER)
        target_idx = 0
        for idx, wp in enumerate(fp.waypoints):
            if wp.t >= t_target:
                target_idx = idx
                break
        else:
            target_idx = len(fp.waypoints) - 1
    else:
        target_idx = fp.get_target_index_from_time(t_anchor)

    if target_idx >= len(fp.waypoints):
        return None
    next_wp: Waypoint = fp.waypoints[target_idx]

    t_to_next = next_wp.t - t_anchor
    if t_to_next <= 0:
        return None

    # ------------------------------------------------------------------
    # Step 2: Extract dynamic constraints & build safe center
    # ------------------------------------------------------------------
    v_nominal = np.linalg.norm(anchor_vel)
    if v_nominal < NORM_ZERO_THRESHOLD:
        v_nominal = 1.0  # Fallback minimum speed for hover states
        
    w_max = float(fp.max_var_ang_vel)
    a_max = float(fp.max_var_lin_vel)

    def calc_dt_turn(dir_in: np.ndarray, dir_out: np.ndarray) -> float:
        """Algoritmo 6 (Casado et al., 2026): Turn time computation."""
        dir_in_norm = np.linalg.norm(dir_in)
        dir_out_norm = np.linalg.norm(dir_out)
        if dir_in_norm < NORM_ZERO_THRESHOLD or dir_out_norm < NORM_ZERO_THRESHOLD:
            return 0.0
        
        u_in = dir_in / dir_in_norm
        u_out = dir_out / dir_out_norm
        dot = np.clip(np.dot(u_in, u_out), -1.0, 1.0)
        alpha = np.arccos(dot)
        
        dt_w = alpha / w_max if w_max > NORM_ZERO_THRESHOLD else 0.0
        dt_a = (2.0 * v_nominal * np.sin(alpha / 2.0)) / a_max if a_max > NORM_ZERO_THRESHOLD else 0.0
        return max(dt_w, dt_a)

    safe_center = None
    vel_anc = anchor_vel.copy()

    if conflict_obbs:
        first_obb = conflict_obbs[0]
        last_obb = conflict_obbs[-1]
        
        fwd_axis: np.ndarray = first_obb.axes[0]  # unit vector
        
        # Aggregate the boxes along the forward axis
        dir_vector = last_obb.center - first_obb.center
        l_proj = np.dot(dir_vector, fwd_axis)
        
        half_len = (abs(l_proj) + first_obb.half_extents[0] + last_obb.half_extents[0]) / 2.0
        composite_center = first_obb.center + fwd_axis * (l_proj / 2.0)

        # Build the safe center (conflict composite center shifted by MTV)
        safe_center = composite_center + mtv

        # det1 and det2 at the entry / exit corners of the safe OBB sequence
        det1_pos: np.ndarray = safe_center - half_len * fwd_axis
        det2_pos: np.ndarray = safe_center + half_len * fwd_axis
        
        # Geometrically define ret_pos to return to original line
        # Advance along the line by the distance of the mtv to make a symmetric ramp-out
        line_dir = next_wp.pos - anchor_pos
        line_len = np.linalg.norm(line_dir)
        
        if line_len > NORM_ZERO_THRESHOLD:
            u_line = line_dir / line_len
            proj_dist = np.dot(det2_pos - anchor_pos, u_line)
            ret_dist = min(proj_dist + np.linalg.norm(mtv), line_len - 0.5)
            ret_pos = anchor_pos + ret_dist * u_line
        else:
            ret_pos = det2_pos - mtv
            
        # ------------------------------------------------------------------
        # Step 3: Accumulate turn & transit times dynamically
        # ------------------------------------------------------------------
        dir_anc_det1 = det1_pos - anchor_pos
        dir_det1_det2 = det2_pos - det1_pos
        dir_det2_ret = ret_pos - det2_pos
        dir_ret_next = next_wp.pos - ret_pos
        
        # Turn 1 (anc)
        v_in_anc = anchor_vel.copy()
        if np.linalg.norm(v_in_anc) <= NORM_ZERO_THRESHOLD:
            v_in_anc = line_dir if line_len > NORM_ZERO_THRESHOLD else mtv
            
        dt_turn_anc = calc_dt_turn(v_in_anc, dir_anc_det1)
        transit_anc_det1 = np.linalg.norm(dir_anc_det1) / v_nominal
        t_det1 = t_anchor + dt_turn_anc + transit_anc_det1
        
        # Turn 2 (det1)
        dt_turn_det1 = calc_dt_turn(dir_anc_det1, dir_det1_det2)
        transit_det1_det2 = np.linalg.norm(dir_det1_det2) / v_nominal
        t_det2 = t_det1 + dt_turn_det1 + transit_det1_det2
        
        # Turn 3 (det2)
        dt_turn_det2 = calc_dt_turn(dir_det1_det2, dir_det2_ret)
        transit_det2_ret = np.linalg.norm(dir_det2_ret) / v_nominal
        t_ret = t_det2 + dt_turn_det2 + transit_det2_ret
        
        # Turn 4 (ret)
        dt_turn_ret = calc_dt_turn(dir_det2_ret, dir_ret_next)
        t_post_ret = t_ret + dt_turn_ret
        
        # ------------------------------------------------------------------
        # Step 4: Kinetic validation (Time budget scaling)
        # ------------------------------------------------------------------
        if t_post_ret >= next_wp.t:
            # We scale the allocated deltas to fit inside the time budget
            available_dt = next_wp.t - t_anchor
            total_dt_needed = t_post_ret - t_anchor
            scale = (available_dt * 0.99) / total_dt_needed
            
            t_det1 = t_anchor + (t_det1 - t_anchor) * scale
            t_det2 = t_anchor + (t_det2 - t_anchor) * scale
            t_ret  = t_anchor + (t_ret - t_anchor) * scale
            
            # Since we scale time, the velocity increases
            v_safe = min(v_nominal / scale, float(fp.max_var_lin_vel))
        else:
            v_safe = min(v_nominal, float(fp.max_var_lin_vel))

        # Velocities directed along the trapezoid segments
        vel_det1 = fwd_axis * v_safe
        vel_det2 = fwd_axis * v_safe

        norm_to_next = np.linalg.norm(dir_ret_next)
        vel_ret = (dir_ret_next / norm_to_next * v_safe
                   if norm_to_next > NORM_ZERO_THRESHOLD else np.zeros(3))

    else:
        # Degraded mode: single displaced point (identical to build_spatial_detour)
        det1_pos = anchor_pos + mtv
        det2_pos = anchor_pos + mtv
        ret_pos = fp.status_at_time(t_anchor + TRAP_DETOUR_RAMP_OUT_FRACTION * t_to_next).pos.copy()
        
        t_det1 = t_anchor + TRAP_DETOUR_RAMP_IN_FRACTION * t_to_next
        t_det2 = t_anchor + TRAP_DETOUR_TOP_END_FRACTION * t_to_next
        t_ret  = t_anchor + TRAP_DETOUR_RAMP_OUT_FRACTION * t_to_next
        
        vel_det1 = np.zeros(3)
        vel_det2 = np.zeros(3)
        vel_ret = np.zeros(3)

    # ------------------------------------------------------------------
    # Step 6: Build the four waypoints
    # ------------------------------------------------------------------
    wp_anc  = Waypoint(label="anc",  t=t_anchor, pos=anchor_pos, vel=vel_anc)
    wp_det1 = Waypoint(label="det1", t=t_det1,   pos=det1_pos,   vel=vel_det1)
    wp_det2 = Waypoint(label="det2", t=t_det2,   pos=det2_pos,   vel=vel_det2)
    wp_ret  = Waypoint(label="ret",  t=t_ret,    pos=ret_pos,     vel=vel_ret)

    # ------------------------------------------------------------------
    # Step 7: Insert waypoints into a copy of fp and commit the polynomial
    # ------------------------------------------------------------------
    new_fp: FlightPlan = fp.copy()

    # Filter out waypoints that fall inside the detour interval
    filtered_wps = []
    for wp in new_fp.waypoints:
        if wp.t <= t_anchor or wp.t >= next_wp.t:
            filtered_wps.append(wp)
    new_fp.waypoints = filtered_wps

    new_fp.set_waypoint(wp_anc)
    new_fp.set_waypoint(wp_det1)
    new_fp.set_waypoint(wp_det2)
    new_fp.set_waypoint(wp_ret)

    # If we didn't set velocities explicitly, let the plan compute them
    if safe_center is None:
        new_fp.set_uniform_velocity()

    # commit the quintic polynomial for every segment (= connect_to per pair)
    new_fp.connect_waypoints()

    return new_fp
