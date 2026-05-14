"""
velocity_bounding.py — Strategy 1: Kinematic Bounding
======================================================

Resolves a UAV conflict by shifting the timestamp of the waypoint closest
to the conflict instant by ±delta seconds.  connect_waypoints() recomputes
the full trajectory and validates kinematic feasibility internally.

The time shift delta is dynamically calculated based on the UAV's velocity at
the conflict instant: faster UAVs require smaller deltas, slower UAVs require
larger deltas to achieve the same spatial separation.
"""

from __future__ import annotations

from typing import Optional, Callable
import numpy as np
import logging

from core.models.flight_plan import FlightPlan
from core.config import S1_TIME_SHIFTS, UAV_MAX_SPEED, NORM_ZERO_THRESHOLD

log = logging.getLogger(__name__)


def _find_nearest_waypoint(fp: FlightPlan, t_conflict: float) -> int:
    """
    Index of the waypoint whose timestamp is closest to t_conflict.
    """
    wps = fp.waypoints
    if not wps:
        return 0
    
    best_idx  = 0
    best_dist = abs(wps[0].t - t_conflict)
    for i in range(len(wps)):
        d = abs(wps[i].t - t_conflict)
        if d < best_dist:
            best_dist = d
            best_idx  = i
    return best_idx


def _calculate_variable_delta(
    fp: FlightPlan,
    t_conflict: float,
    base_delta: float,
    reference_speed: float = UAV_MAX_SPEED - 2.0,  # Use a slightly lower reference speed to avoid extreme deltas for extreme cases UAVs
) -> float:
    """
    Calculate a variable time shift delta based on the UAV's velocity at conflict time.
    
    The time shift scales inversely with speed:
    - Fast UAV (20 m/s) → smaller delta needed (less time to escape)
    - Slow UAV (5 m/s) → larger delta needed (more time to escape)
    
    Formula: variable_delta = base_delta * (reference_speed / actual_speed)
    
    Args:
        fp: The flight plan of the plebeian UAV.
        t_conflict: The instant of conflict detection.
        base_delta: Base time shift for reference speed [s].
        reference_speed: Speed used as baseline (default: UAV_MAX_SPEED) [m/s].
    
    Returns:
        Scaled delta time shift [s].
    """
    try:
        status = fp.status_at_time(t_conflict)
        actual_speed = np.linalg.norm(status.vel)
        
        # If UAV is stationary or moving very slowly, use base_delta
        if actual_speed < NORM_ZERO_THRESHOLD:
            log.debug(f"[S1] UAV {fp.id}: stationary, using base_delta={base_delta}s")
            return base_delta
        
        # Scale delta inversely with speed 
        variable_delta = base_delta * (reference_speed / actual_speed)
        
        log.debug(f"[S1] UAV {fp.id}: speed={actual_speed:.2f}m/s, "
                  f"base_delta={base_delta}s → variable_delta={variable_delta:.2f}s")
        
        return variable_delta
    except Exception as e:
        log.debug(f"[S1] UAV {fp.id}: exception in variable_delta: {e}, using base_delta={base_delta}s")
        return base_delta





def run_strategy1(
    fp_pleb:      FlightPlan,
    t_conflict:   float,
    validator_fn: Callable[[FlightPlan], bool],
) -> Optional[FlightPlan]:
    """
    Shift the nearest waypoint timestamp by ±delta and call connect_waypoints().
    
    Uses velocity-adaptive time shifts: the delta is scaled based on the UAV's
    speed at the conflict instant. Faster drones → smaller deltas; slower drones
    → larger deltas.
    
    Tries each base delta in S1_TIME_SHIFTS, computes its variable version based
    on UAV speed, then tries both ± directions.
    
    Returns the first conflict-free plan found, or None.
    """
    if len(fp_pleb.waypoints) < 2:
        return None

    nearest_idx = _find_nearest_waypoint(fp_pleb, t_conflict)
    log.debug(f"[S1] UAV {fp_pleb.id}: nearest WP={nearest_idx} at t={fp_pleb.waypoints[nearest_idx].t:.2f}s")

    for base_delta in S1_TIME_SHIFTS:
        # Calculate velocity-adaptive delta
        variable_delta = _calculate_variable_delta(fp_pleb, t_conflict, base_delta)
        
        # Try both positive and negative shifts
        for sign_val, time_shift in [("+", variable_delta), ("-", -variable_delta)]:
            candidate = fp_pleb.copy()
            old_t = candidate.waypoints[nearest_idx].t
            candidate.waypoints[nearest_idx].t += time_shift
            
            try:
                candidate.connect_waypoints(strict=True)
            except ValueError:
                log.debug(f"[S1] UAV {fp_pleb.id}: {sign_val}{variable_delta:.2f}s failed kinematic check")
                continue
            
            if validator_fn(candidate):
                log.info(f"[S1] ✓ UAV {fp_pleb.id}: WP[{nearest_idx}] shifted {sign_val}{variable_delta:.2f}s "
                         f"(t: {old_t:.2f}s → {candidate.waypoints[nearest_idx].t:.2f}s)")
                return candidate
            else:
                log.debug(f"[S1] UAV {fp_pleb.id}: {sign_val}{variable_delta:.2f}s failed R-Tree validation")

    log.debug(f"[S1] UAV {fp_pleb.id}: all attempts exhausted")
    return None
