"""
path_geometry.py — Rigid Shift Detour Constructor
=================================================

PURPOSE:
    Build a modified FlightPlan for the plebeian UAV that avoids a conflict zone
    by inserting a kinematically constrained rigid-shift detour (3-point topology).

    This module exposes a single detour builder:

    build_rigid_shift_detour()  — PRIMARY. Rigid Shift (detour_start → det → detour_end).
                                  Uses kinematic boundaries and heuristic MTV
                                  scaling to find a conflict-free route via the
                                  Shadow R-Tree.

RIGID SHIFT GEOMETRY (3-POINT TOPOLOGY):
    Original route:
        ──────────[detour_start]────────────────────────────[next_wp]──

    After build_rigid_shift_detour():
        ──────────[detour_start]────────[det]────────[detour_end]──[next_wp]──
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
    t_detour_starthor:     float,
    mtv:          np.ndarray,
    conflict_obbs: Optional[List["SweptBox_OBB"]] = None,
    origin_conflict: Optional[np.ndarray] = None,
    t_detour_start_override: Optional[float] = None,
    t_detour_end_override: Optional[float] = None,
) -> Optional[FlightPlan]:
    """
    Build a Rigid Shift detour (detour_start → det → detour_end).
    
    Uses the kinematic approach to calculate detour_starthor and detour_endurn times (if not provided).
    Applies the scaled MTV to the NOMINAL position of the drone at the apex time
    to respect original trajectory curvature, and calculates a natural velocity 
    tangent using central finite differences.
    
    Args:
        fp:                   Flight plan to modify
        t_detour_starthor:             detour_starthor time for maneuver start
        mtv:                  Minimum translation vector (displacement)
        conflict_obbs:        List of conflicting OBB boxes
        origin_conflict:      Conflict origin point (optional)
        t_detour_start_override:       If provided, use this detour_starthor time instead of calculating
        t_detour_end_override:       If provided, use this detour_endurn time instead of calculating
    """
    # Guard: detour_starthor must be within the flight plan window
    if t_detour_starthor >= fp.finish_time():
        return None

    mtv = np.array(mtv, dtype=float)
    if np.linalg.norm(mtv) < NORM_ZERO_THRESHOLD:
        return None

    if not conflict_obbs and origin_conflict is None:
        return None

    new_fp: FlightPlan = fp.copy()

    # Get state at maneuver start (t_detour_starthor)
    status_detour_starthor = new_fp.status_at_time(t_detour_starthor)
    detour_starthor_vel: np.ndarray = status_detour_starthor.vel.copy()

    # v_cruise = current actual speed of the drone at t_detour_starthor.
    # We use this instead of UAV_MAX_SPEED for two reasons:
    #   1. More realistic: the drone may not be flying at maximum speed.
    #   2. Preserves flight plan intent: maneuver timing adapts to actual velocity,
    #      not an arbitrary constant.
    # If the drone is hovering (v ≈ 0), default to UAV_MAX_SPEED as a fallback.
    v_cruise = np.linalg.norm(detour_starthor_vel)
    if v_cruise < NORM_ZERO_THRESHOLD:
        v_cruise = float(UAV_MAX_SPEED)

    # Determine t_detour_start and t_detour_end: either use overrides or calculate from conflict_obbs
    if t_detour_start_override is not None and t_detour_end_override is not None:
        t_detour_start = t_detour_start_override
        t_detour_end = t_detour_end_override
    else:
        # Fallback: calculate from conflict_obbs (original behavior)
        # Calculate kinematic detour_starthor distdetour_starte
        # Heuristic: the higher the cruise speed, the longer the maneuver preparation.
        d_detour = (v_cruise ** 2) / float(UAV_MAX_SPEED)
        if d_detour < MIN_DETOUR_DURATION:
            d_detour = MIN_DETOUR_DURATION

        # Calculate temporal detour_starthors based on the original route
        t_conflict_start = conflict_obbs[0].t_range[0]
        t_detour_start = t_conflict_start - (d_detour / v_cruise)
        t_detour_start = max(t_detour_start, new_fp.init_time() + WAYPOINT_TIME_EPSILON)
        
        # We must ensure t_detour_start starts at or after t_detour_starthor
        t_detour_start = max(t_detour_start, t_detour_starthor)

        t_detour_end_base = conflict_obbs[-1].t_range[1]
        t_detour_end = t_detour_end_base + (d_detour / v_cruise)
        t_detour_end = min(t_detour_end, new_fp.finish_time() - WAYPOINT_TIME_EPSILON)
    
    # Validate t_detour_start and t_detour_end bounds
    t_detour_start = max(t_detour_start, fp.init_time() + WAYPOINT_TIME_EPSILON)
    t_detour_start = max(t_detour_start, t_detour_starthor)
    t_detour_end = min(t_detour_end, fp.finish_time() - WAYPOINT_TIME_EPSILON)
    
    if t_detour_start >= t_detour_end:
        return None
    
    status_detour_start = fp.status_at_time(t_detour_start)  # Use fp (original), not new_fp
    pos_detour_start = status_detour_start.pos.copy()
    vel_detour_start = status_detour_start.vel.copy()

    status_detour_end = fp.status_at_time(t_detour_end)  # Use fp (original), not new_fp
    pos_detour_end = status_detour_end.pos.copy()
    vel_detour_end = status_detour_end.vel.copy()
    
    # CRITICAL: Store the ORIGINAL pos_detour_end and vel_detour_end (based on original t_detour_end)
    # These represent the fixed detour_endurn-to-path point and will NOT change
    pos_detour_end_fixed = pos_detour_end.copy()
    vel_detour_end_fixed = vel_detour_end.copy()

    # Remove interior waypoints in the affected segment
    to_remove = []
    for wp in new_fp.waypoints:
        if t_detour_start < wp.t < t_detour_end:
            to_remove.append(wp.t)
            
    for t in to_remove:
        new_fp.remove_waypoint_at_time(t)

    # Store the original t_detour_end (before any extensions) for postponement calculation
    t_detour_end_original = t_detour_end

    # Apex Time Calculation
    t_det = (t_detour_start + t_detour_end) / 2.0

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
    # print(f"[DEBUG POS] pos_detour_start={pos_detour_start}")
    # print(f"[DEBUG POS] pos_det={pos_det}")
    # print(f"[DEBUG POS] pos_detour_end_fixed={pos_detour_end_fixed}")

    # Kinematic feasibility guard: Min time required to cover the spatial distdetour_startes
    dist1 = np.linalg.norm(pos_det - pos_detour_start)
    dist2 = np.linalg.norm(pos_detour_end_fixed - pos_det)
    
    # Required time for segment 1 (detour_start -> det)
    v1 = np.linalg.norm(vel_detour_start)
    t1 = (-v1 + math.sqrt(max(0, v1**2 + 2 * UAV_MAX_ACCEL * dist1))) / (UAV_MAX_ACCEL + NORM_ZERO_THRESHOLD)
    
    # Required time for segment 2 (det -> detour_end)
    v2 = v_cruise 
    t2 = (-v2 + math.sqrt(max(0, v2**2 + 2 * UAV_MAX_ACCEL * dist2))) / (UAV_MAX_ACCEL + NORM_ZERO_THRESHOLD)
    
    # Total minimum time needed (use max to ensure both segments have enough time)
    min_time_needed = 2.0 * max(t1, t2)
    time_available = t_detour_end - t_detour_start

    if min_time_needed > time_available:
        # Extension strategy: extend t_detour_end to give more travel time to the SAME FIXED POINT
        # pos_detour_end_fixed remains the same - it's the point where we detour_endurn to the original path
        extra = min_time_needed - time_available + WAYPOINT_TIME_EPSILON
        print(f"[INFO] Extending detour time: {time_available:.2f}s -> {time_available + extra:.2f}s (needed {min_time_needed:.2f}s for dist1={dist1:.1f}m, dist2={dist2:.1f}m)")
        
        # Extend t_detour_end only - this gives more time to reach pos_detour_end_fixed
        t_detour_end = t_detour_end + extra
        t_detour_end = min(t_detour_end, fp.finish_time() - WAYPOINT_TIME_EPSILON)
        
        # Recalculate t_det to maintain approximate symmetry
        t_det = (t_detour_start + t_detour_end) / 2.0
        
        # Recalculate nominal position and apex
        status_nominal = fp.status_at_time(t_det)
        pos_nominal = status_nominal.pos.copy()
        pos_det = pos_nominal + mtv_scaled
        
        # pos_detour_end remains FIXED - it's the detour_endurn point, not a moving target
        # vel_detour_end also remains FIXED from the original calculation
    else:
        # No extension needed - use the calculated values
        vel_detour_end = vel_detour_end_fixed
        pos_detour_end = pos_detour_end_fixed
    
    # Ensure pos_detour_end and vel_detour_end use the fixed detour_endurn point
    pos_detour_end = pos_detour_end_fixed
    vel_detour_end = vel_detour_end_fixed

    # FIX 3: Central Finite Difference Velocity (Raw Magnitude)
    # We calculate a natural curve tangent by averaging the incoming and outgoing 
    # segment vectors. We DO NOT force this to be `v_cruise`. If the detour is sharp,
    # v_in and v_out will partially cdetour_startel out, yielding a lower velocity. 
    # This correctly allows the drone to slow down at the apex, preventing 
    # the polynomial spline from overshooting and creating loops.
    v_in_vec = (pos_det - pos_detour_start) / (t_det - t_detour_start)
    v_out_vec = (pos_detour_end - pos_det) / (t_detour_end - t_det)
    vel_det = (v_in_vec + v_out_vec) / 2.0

    # Insert maneuver waypoints
    wp_detour_start = Waypoint(label="detour_start", t=t_detour_start, pos=pos_detour_start, vel=vel_detour_start)
    wp_det = Waypoint(label="det", t=t_det, pos=pos_det, vel=vel_det)
    wp_detour_end = Waypoint(label="detour_end", t=t_detour_end, pos=pos_detour_end, vel=vel_detour_end)

    new_fp.set_waypoint(wp_detour_start)
    new_fp.set_waypoint(wp_det)
    new_fp.set_waypoint(wp_detour_end)

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
        print(f"       t_detour_start={t_detour_start:.2f}s, t_det={t_det:.2f}s, t_detour_end={t_detour_end:.2f}s")
        print(f"       dist(detour_start->det)={np.linalg.norm(pos_det - pos_detour_start):.2f}m")
        print(f"       dist(det->detour_end)={np.linalg.norm(pos_detour_end - pos_det):.2f}m")
        print(f"       vel_detour_start={np.linalg.norm(vel_detour_start):.2f} m/s")
        print(f"       vel_det={np.linalg.norm(vel_det):.2f} m/s")
        print(f"       vel_detour_end={np.linalg.norm(vel_detour_end):.2f} m/s")
        return None

    # CRITICAL: After inserting the detour, postpone all waypoints after t_detour_end_original
    # This compensates for the extra time the detour takes (if any was added)
    if t_detour_end > t_detour_end_original:
        extra_time = t_detour_end - t_detour_end_original
        print(f"[INFO] Postponing waypoints after t_detour_end: extra_time={extra_time:.2f}s")
        new_fp.postpone_from(t_detour_end_original, extra_time)

    return new_fp
