"""
viz_08_scenario_data.py
=======================

Reusable data helpers for the Viz 08 cascade scenario.

This module keeps the scenario generation and conflict-resolution logic from
viz_08_full_cascade_scenario.py, but removes all visualization code so other
components, such as the Isaac Sim bridge, can import the prepared flight plans
directly.
"""

import sys
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.waypoint import Waypoint
from core.models.flight_plan import FlightPlan
from central_manager import CentralManager


def _heading_from_velocity(velocity: np.ndarray) -> list[float]:
    velocity_xy = np.array(velocity[:2], dtype=float)
    if np.linalg.norm(velocity_xy) < 1e-9:
        return [0, 0]
    return velocity_xy.tolist()


def _sync_headings_to_velocity(flight_plan: FlightPlan) -> None:
    for waypoint in flight_plan.waypoints:
        waypoint.heading = _heading_from_velocity(waypoint.vel)



def generate_straight_fleet(n_uavs: int = 5) -> list[FlightPlan]:
    """
    Generate the exact Viz 08 fleet used by the full-cascade visualizer.

    The first five UAVs follow the original crossing routes. The next five UAVs
    reuse those same routes but reversed, except the purely vertical route,
    which is intentionally skipped so the fleet contains 9 UAVs.
    """
    space: list[FlightPlan] = []

    routes = [
        ([0, 200, 0],   [200, 200, 0], [400, 200, 0], 10.0),       # W -> E
        ([200, 0, 0],   [200, 200, 0], [200, 400, 0], 10.0),       # S -> N
        ([0, 0, 0],     [200, 200, 0], [400, 400, 0], 10.0),       # SW -> NE
        ([400, 0, 0],   [200, 200, 0], [0, 400, 0],   10.0),       # SE -> NW
        ([200, 200, 50], [200, 200, 0], [200, 200, -30],   5.0),        # Drop down
    ]

    for i in range(min(n_uavs, 5)):
        start_pos, mid_pos, end_pos, speed = routes[i]

        dist_to_mid = np.linalg.norm(np.array(mid_pos) - np.array(start_pos))
        t_start = 20.0 - (dist_to_mid / speed)

        dist_from_mid = np.linalg.norm(np.array(end_pos) - np.array(mid_pos))
        t_end = 20.0 + (dist_from_mid / speed)

        fp = FlightPlan()
        fp.id = i + 1
        fp.priority = max(2 - i, 0)

        fp.set_waypoint(Waypoint("start", round(t_start, 2), start_pos, [0, 0, 0]))
        fp.set_waypoint(Waypoint("mid", 20.0, mid_pos, [0, 0, 0]))
        fp.set_waypoint(Waypoint("end", round(t_end, 2), end_pos, [0, 0, 0]))

        fp.max_var_lin_vel = 10.0
        fp.max_var_ang_vel = 2.0
        fp.set_uniform_velocity()
        fp.connect_waypoints(strict=True)
        _sync_headings_to_velocity(fp)

        space.append(fp)

    num_orig = len(space)
    for i in range(num_orig):
        orig_fp = space[i]

        mid_idx = orig_fp.get_index_from_label("mid")
        if mid_idx is None:
            mid_idx = 1 if len(orig_fp.waypoints) >= 3 else max(0, len(orig_fp.waypoints) // 2)

        wp_start = orig_fp.waypoints[0]
        wp_mid = orig_fp.waypoints[mid_idx]
        wp_end = orig_fp.waypoints[-1]

        d1 = wp_mid.t - wp_start.t
        d2 = wp_end.t - wp_mid.t

        t_start_rev = round(wp_mid.t - d2, 2)
        t_end_rev = round(wp_mid.t + d1, 2)

        try:
            start_pos_arr = np.array(wp_start.pos, dtype=float)
            end_pos_arr = np.array(wp_end.pos, dtype=float)
            if np.allclose(start_pos_arr[:2], end_pos_arr[:2], atol=1e-6) and abs(end_pos_arr[2] - start_pos_arr[2]) > 1e-6:
                continue
        except Exception:
            pass

        new_fp = FlightPlan()
        new_id = num_orig + i + 1
        new_fp.id = new_id
        new_fp.priority = orig_fp.priority

        def _pos(wp):
            try:
                return wp.pos.tolist()
            except Exception:
                return wp.pos

        new_fp.set_waypoint(Waypoint("start", t_start_rev, _pos(wp_end), [0, 0, 0]))
        new_fp.set_waypoint(Waypoint("mid", wp_mid.t, _pos(wp_mid), [0, 0, 0]))
        new_fp.set_waypoint(Waypoint("end", t_end_rev, _pos(wp_start), [0, 0, 0]))

        new_fp.max_var_lin_vel = orig_fp.max_var_lin_vel
        new_fp.max_var_ang_vel = orig_fp.max_var_ang_vel
        new_fp.set_uniform_velocity()
        new_fp.connect_waypoints()
        _sync_headings_to_velocity(new_fp)

        space.append(new_fp)

    return space


def build_viz08_plans(n_uavs: int = 5) -> Tuple[Dict[str, FlightPlan], Dict[str, FlightPlan]]:
    """
    Build the original and resolved Viz 08 flight plans.

    Returns:
        A tuple of two dictionaries:
        - original_plans: flight plans before resolution
        - resolved_plans: flight plans after the central manager cascade
    """
    if n_uavs != 9:
        raise ValueError(
            f"Viz 08 full cascade expects exactly 9 UAVs, got {n_uavs}. "
            "Use the same 9-UAV scene as viz_08_full_cascade_scenario.py."
        )

    # Build the exact fleet used by the full visualizer and resolve it in place.
    space = generate_straight_fleet(n_uavs=n_uavs)
    original_plans = {f"UAV_{i}": fp.copy() for i, fp in enumerate(space)}

    manager = CentralManager()

    for i, fp in enumerate(space):
        uav_id = f"UAV_{i}"
        priority = max(2 - i, 0)
        manager.register_uav(uav_id, fp, priority=priority)

        manager.check_and_resolve(uav_id)

    # Keep resolving until the system is stable or a conservative cap is hit.
    max_sweeps = 300
    for _ in range(max_sweeps):
        conflicts = manager._rtree_detector.detect_all_conflicts_system_wide()
        if not conflicts:
            break

        sweep_results = manager.check_and_resolve_all()
        if not sweep_results:
            break

    resolved_plans = {
        uid: manager.get_flight_plan(uid)
        for uid in original_plans.keys()
    }

    for flight_plan in original_plans.values():
        _sync_headings_to_velocity(flight_plan)

    for flight_plan in resolved_plans.values():
        if flight_plan is not None:
            _sync_headings_to_velocity(flight_plan)

    return original_plans, resolved_plans


def get_viz08_flightplans(case: str = "resolved", n_uavs: int = 9) -> Dict[str, FlightPlan]:
    """
    Return one Viz 08 scenario variant by name.

    Args:
        case: "original" or "resolved".
        n_uavs: Number of UAVs to generate. Viz 08 full cascade requires 9.
    """
    original_plans, resolved_plans = build_viz08_plans(n_uavs=n_uavs)

    normalized_case = case.strip().lower()
    if normalized_case == "original":
        return original_plans
    if normalized_case == "resolved":
        return resolved_plans

    raise ValueError(
        f"Unknown Viz 08 case '{case}'. Use 'original' or 'resolved'."
    )