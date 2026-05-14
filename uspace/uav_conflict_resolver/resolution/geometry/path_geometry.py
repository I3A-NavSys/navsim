"""
path_geometry.py — Rigid Shift Detour Constructor
=================================================

PURPOSE:
    Build a modified FlightPlan for the plebeian UAV that avoids a conflict zone
    by inserting a kinematically constrained rigid-shift detour (3-point topology).

    This module exposes a single detour builder:

    build_rigid_shift_detour()  — PRIMARY. Rigid Shift (anc → det → ret).
                                  Uses kinematic boundaries and heuristic MTV
                                  scaling to find a conflict-free route via the
                                  Shadow R-Tree.

RIGID SHIFT GEOMETRY (3-POINT TOPOLOGY):
    Original route:
        ──────────[anc]────────────────────────────[next_wp]──

    After build_rigid_shift_detour():
        ──────────[anc]────────[det]────────[ret]──[next_wp]──
                      ↘                    ↗
                        (displaced by MTV)

    DESIGN CHOICE: A 3-point detour (single apex) is used instead of a 4-point 
    trapezoid to handle detours over curved trajectories robustly. Calculating 
    parallel offset curves in 3D space is computationally expensive and prone 
    to topological collapse (loops/spikes) on tight turns.
    
    THE "TRIANGLE PROBLEM": Because we use a triangle over a bounding box, pushing 
    only the apex by the exact Minimum Translation Vector (MTV) leaves the legs of 
    the triangle inside the obstacle. To compensate, the MTV is multiplied by 
    RIGID_SHIFT_MTV_SCALE (e.g., 1.5 - 2.0). This artificially pushes the apex 
    further away, widening the legs enough to clear the safety volume entirely.
"""

from __future__ import annotations

import math
import numpy as np
from typing import List, Optional, TYPE_CHECKING

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from core.config import (
    NORM_ZERO_THRESHOLD,
    MIN_DETOUR_DURATION,
    UAV_MAX_SPEED,
    UAV_MAX_ACCEL,
    RIGID_SHIFT_MTV_SCALE,
    WAYPOINT_TIME_EPSILON,
)

if TYPE_CHECKING:
    from detection.conflictDetection import SweptBox_OBB

# ---------------------------------------------------------------------------
# Primary detour builder: Rigid Shift strategy
# ---------------------------------------------------------------------------

def build_rigid_shift_detour(
    fp:           FlightPlan,
    t_anchor:     float,
    mtv:          np.ndarray,
    conflict_obbs: Optional[List["SweptBox_OBB"]] = None,
    origin_conflict: Optional[np.ndarray] = None,
    t_anc_override: Optional[float] = None,
    t_ret_override: Optional[float] = None,
) -> Optional[FlightPlan]:
    """
    Build a Rigid Shift detour (anc → det → ret).
    
    Uses the kinematic approach to calculate anchor and return times (if not provided).
    Applies the scaled MTV to the NOMINAL position of the drone at the apex time
    to respect original trajectory curvature, and calculates a natural velocity 
    tangent using central finite differences.
    
    Args:
        fp:                   Flight plan to modify
        t_anchor:             Anchor time for maneuver start
        mtv:                  Minimum translation vector (displacement)
        conflict_obbs:        List of conflicting OBB boxes
        origin_conflict:      Conflict origin point (optional)
        t_anc_override:       If provided, use this anchor time instead of calculating
        t_ret_override:       If provided, use this return time instead of calculating
    """
    # Guard: anchor must be within the flight plan window
    if t_anchor >= fp.finish_time():
        return None

    mtv = np.array(mtv, dtype=float)
    if np.linalg.norm(mtv) < NORM_ZERO_THRESHOLD:
        return None

    if not conflict_obbs and origin_conflict is None:
        return None

    new_fp: FlightPlan = fp.copy()

    # Get state at maneuver start (t_anchor)
    status_anchor = new_fp.status_at_time(t_anchor)
    anchor_vel: np.ndarray = status_anchor.vel.copy()

    # v_cruise = current actual speed of the drone at t_anchor.
    # We use this instead of UAV_MAX_SPEED for two reasons:
    #   1. More realistic: the drone may not be flying at maximum speed.
    #   2. Preserves flight plan intent: maneuver timing adapts to actual velocity,
    #      not an arbitrary constant.
    # If the drone is hovering (v ≈ 0), default to UAV_MAX_SPEED as a fallback.
    v_cruise = np.linalg.norm(anchor_vel)
    if v_cruise < NORM_ZERO_THRESHOLD:
        v_cruise = float(UAV_MAX_SPEED)

    # Determine t_anc and t_ret: either use overrides or calculate from conflict_obbs
    if t_anc_override is not None and t_ret_override is not None:
        t_anc = t_anc_override
        t_ret = t_ret_override
    else:
        # Fallback: calculate from conflict_obbs (original behavior)
        # Calculate kinematic anchor distance
        # Heuristic: the higher the cruise speed, the longer the maneuver preparation.
        d_anc = (v_cruise ** 2) / float(UAV_MAX_SPEED)
        if d_anc < MIN_DETOUR_DURATION:
            d_anc = MIN_DETOUR_DURATION

        # Calculate temporal anchors based on the original route
        t_conflict_start = conflict_obbs[0].t_range[0]
        t_anc = t_conflict_start - (d_anc / v_cruise)
        t_anc = max(t_anc, new_fp.init_time() + WAYPOINT_TIME_EPSILON)
        
        # We must ensure t_anc starts at or after t_anchor
        t_anc = max(t_anc, t_anchor)

        t_ret_base = conflict_obbs[-1].t_range[1]
        t_ret = t_ret_base + (d_anc / v_cruise)
        t_ret = min(t_ret, new_fp.finish_time() - WAYPOINT_TIME_EPSILON)
    
    # Validate t_anc and t_ret bounds
    t_anc = max(t_anc, fp.init_time() + WAYPOINT_TIME_EPSILON)
    t_anc = max(t_anc, t_anchor)
    t_ret = min(t_ret, fp.finish_time() - WAYPOINT_TIME_EPSILON)
    
    if t_anc >= t_ret:
        return None
    
    status_anc = fp.status_at_time(t_anc)  # Use fp (original), not new_fp
    pos_anc = status_anc.pos.copy()
    vel_anc = status_anc.vel.copy()

    status_ret = fp.status_at_time(t_ret)  # Use fp (original), not new_fp
    pos_ret = status_ret.pos.copy()
    vel_ret = status_ret.vel.copy()
    
    # CRITICAL: Store the ORIGINAL pos_ret and vel_ret (based on original t_ret)
    # These represent the fixed return-to-path point and will NOT change
    pos_ret_fixed = pos_ret.copy()
    vel_ret_fixed = vel_ret.copy()

    # Remove interior waypoints in the affected segment
    to_remove = []
    for wp in new_fp.waypoints:
        if t_anc < wp.t < t_ret:
            to_remove.append(wp.t)
            
    for t in to_remove:
        new_fp.remove_waypoint_at_time(t)

    # Store the original t_ret (before any extensions) for postponement calculation
    t_ret_original = t_ret

    # Apex Time Calculation
    t_det = (t_anc + t_ret) / 2.0

    # FIX 2: True Nominal Position from original flight plan
    status_nominal = fp.status_at_time(t_det)
    pos_nominal = status_nominal.pos.copy()

    # Apply the MTV directly to the drone's intended flight path
    # IMPORTANT: The MTV from SAT is already a sufficient separation.
    # Do NOT multiply by RIGID_SHIFT_MTV_SCALE - that causes unrealistic displacements.
    # The MTV magnitude is calibrated by SAT for collision-free separation.
    mtv_scaled = mtv  # Use MTV as-is, no scaling
    pos_det = pos_nominal + mtv_scaled
    
    # print(f"[DEBUG MTV] mtv={mtv}, norm={np.linalg.norm(mtv):.3f}")
    # print(f"[DEBUG MTV] Using MTV directly (no scaling)")
    # print(f"[DEBUG POS] pos_nominal={pos_nominal}")
    # print(f"[DEBUG POS] pos_anc={pos_anc}")
    # print(f"[DEBUG POS] pos_det={pos_det}")
    # print(f"[DEBUG POS] pos_ret_fixed={pos_ret_fixed}")

    # Kinematic feasibility guard: Min time required to cover the spatial distances
    dist1 = np.linalg.norm(pos_det - pos_anc)
    dist2 = np.linalg.norm(pos_ret_fixed - pos_det)
    
    # Required time for segment 1 (anc -> det)
    v1 = np.linalg.norm(vel_anc)
    t1 = (-v1 + math.sqrt(max(0, v1**2 + 2 * UAV_MAX_ACCEL * dist1))) / (UAV_MAX_ACCEL + NORM_ZERO_THRESHOLD)
    
    # Required time for segment 2 (det -> ret)
    v2 = v_cruise 
    t2 = (-v2 + math.sqrt(max(0, v2**2 + 2 * UAV_MAX_ACCEL * dist2))) / (UAV_MAX_ACCEL + NORM_ZERO_THRESHOLD)
    
    # Total minimum time needed (use max to ensure both segments have enough time)
    min_time_needed = 2.0 * max(t1, t2)
    time_available = t_ret - t_anc

    if min_time_needed > time_available:
        # Extension strategy: extend t_ret to give more travel time to the SAME FIXED POINT
        # pos_ret_fixed remains the same - it's the point where we return to the original path
        extra = min_time_needed - time_available + WAYPOINT_TIME_EPSILON
        print(f"[INFO] Extending detour time: {time_available:.2f}s -> {time_available + extra:.2f}s (needed {min_time_needed:.2f}s for dist1={dist1:.1f}m, dist2={dist2:.1f}m)")
        
        # Extend t_ret only - this gives more time to reach pos_ret_fixed
        t_ret = t_ret + extra
        t_ret = min(t_ret, fp.finish_time() - WAYPOINT_TIME_EPSILON)
        
        # Recalculate t_det to maintain approximate symmetry
        t_det = (t_anc + t_ret) / 2.0
        
        # Recalculate nominal position and apex
        status_nominal = fp.status_at_time(t_det)
        pos_nominal = status_nominal.pos.copy()
        pos_det = pos_nominal + mtv_scaled
        
        # pos_ret remains FIXED - it's the return point, not a moving target
        # vel_ret also remains FIXED from the original calculation
    else:
        # No extension needed - use the calculated values
        vel_ret = vel_ret_fixed
        pos_ret = pos_ret_fixed
    
    # Ensure pos_ret and vel_ret use the fixed return point
    pos_ret = pos_ret_fixed
    vel_ret = vel_ret_fixed

    # FIX 3: Central Finite Difference Velocity (Raw Magnitude)
    # We calculate a natural curve tangent by averaging the incoming and outgoing 
    # segment vectors. We DO NOT force this to be `v_cruise`. If the detour is sharp,
    # v_in and v_out will partially cancel out, yielding a lower velocity. 
    # This correctly allows the drone to slow down at the apex, preventing 
    # the polynomial spline from overshooting and creating loops.
    v_in_vec = (pos_det - pos_anc) / (t_det - t_anc)
    v_out_vec = (pos_ret - pos_det) / (t_ret - t_det)
    vel_det = (v_in_vec + v_out_vec) / 2.0

    # Insert maneuver waypoints
    wp_anc = Waypoint(label="anc", t=t_anc, pos=pos_anc, vel=vel_anc)
    wp_det = Waypoint(label="det", t=t_det, pos=pos_det, vel=vel_det)
    wp_ret = Waypoint(label="ret", t=t_ret, pos=pos_ret, vel=vel_ret)

    new_fp.set_waypoint(wp_anc)
    new_fp.set_waypoint(wp_det)
    new_fp.set_waypoint(wp_ret)

    # FIX 4: Removed set_uniform_velocity()
    # We deliberately omit new_fp.set_uniform_velocity() here so it doesn't overwrite 
    # the carefully calculated `vel_det` (which needs to be lower than v_cruise 
    # to prevent Runge's phenomenon / overshoot loops).
    
    # Trust connect_to() to generate smooth quintic polynomials that satisfy
    # all kinematic constraints. If any segment is infeasible, strict=True
    # will raise ValueError, which we propagate back to the caller.
    try:
        new_fp.connect_waypoints(strict=True)
    except ValueError as e:
        err_str = str(e).encode('utf-8', 'ignore').decode('utf-8')
        print(f"[DEBUG] connect_waypoints failed: {err_str}")
        print(f"[DEBUG] Maneuver params:")
        print(f"       t_anc={t_anc:.2f}s, t_det={t_det:.2f}s, t_ret={t_ret:.2f}s")
        print(f"       dist(anc->det)={np.linalg.norm(pos_det - pos_anc):.2f}m")
        print(f"       dist(det->ret)={np.linalg.norm(pos_ret - pos_det):.2f}m")
        print(f"       vel_anc={np.linalg.norm(vel_anc):.2f} m/s")
        print(f"       vel_det={np.linalg.norm(vel_det):.2f} m/s")
        print(f"       vel_ret={np.linalg.norm(vel_ret):.2f} m/s")
        return None

    # CRITICAL: After inserting the detour, postpone all waypoints after t_ret_original
    # This compensates for the extra time the detour takes (if any was added)
    if t_ret > t_ret_original:
        extra_time = t_ret - t_ret_original
        print(f"[INFO] Postponing waypoints after t_ret: extra_time={extra_time:.2f}s")
        new_fp.postpone_from(t_ret_original, extra_time)

    return new_fp