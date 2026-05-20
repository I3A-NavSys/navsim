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


def _build_generated_route(index: int, total_uavs: int) -> tuple[list[float], list[float], list[float], float]:
    """
    Build an extra straight route for scenarios larger than the original five.

    The first five routes stay identical to the visualizer version so the old
    scenario remains reproducible. Any extra UAVs are distributed around the
    same conflict center so the bridge can scale without needing new hardcoded
    paths every time.
    """
    center = np.array([200.0, 200.0, 100.0])
    radius = 200.0
    extra_uavs = max(1, total_uavs - 5)
    angle = 2.0 * np.pi * (index - 5) / extra_uavs
    horizontal = np.array([np.cos(angle), np.sin(angle), 0.0])

    # Alternate the vertical offset so the extra UAVs are not perfectly coplanar.
    vertical_offset = 40.0 if index % 2 == 0 else -40.0
    vertical = np.array([0.0, 0.0, vertical_offset])

    start_pos = (center + radius * horizontal + vertical).tolist()
    mid_pos = center.tolist()
    end_pos = (center - radius * horizontal - vertical).tolist()

    # Keep the speed constant here; the actual timestamps are derived from the distance.
    speed = 10.0
    return start_pos, mid_pos, end_pos, speed


def generate_straight_fleet(n_uavs: int = 5) -> list[FlightPlan]:
    """
    Generate the original Viz 08 fleet before conflict resolution.
    """
    space: list[FlightPlan] = []

    routes = [
        ([0, 200, 100],   [200, 200, 100], [400, 200, 0], 10.0),       # W -> E
        ([200, 0, 100],   [200, 200, 100], [200, 400, 100], 10.0),       # S -> N
        ([0, 0, 100],     [200, 200, 100], [400, 400, 100], 10.0),      # SW -> NE
        ([200, 0, 0],     [200, 200, 100], [200, 400, 200],   10.0),      # SE -> NW
        ([200, 200, 200], [200, 200, 100], [200, 200, 0],   5.0)         # Drop down
    ]

    for i in range(n_uavs):
        # Preserve the original five routes, then synthesize extra ones when the
        # caller asks for a larger scenario.
        if i < len(routes):
            start_pos, mid_pos, end_pos, speed = routes[i]
        else:
            start_pos, mid_pos, end_pos, speed = _build_generated_route(i, n_uavs)

        dist_to_mid = np.linalg.norm(np.array(mid_pos) - np.array(start_pos))
        t_start = 20.0 - (dist_to_mid / speed)

        dist_from_mid = np.linalg.norm(np.array(end_pos) - np.array(mid_pos))
        t_end = 20.0 + (dist_from_mid / speed)

        fp = FlightPlan()
        fp.id = i + 1
        fp.priority = max(2 - i, 0)

        # UAVControl expects every waypoint to expose a heading vector, even if
        # it is just the neutral [0, 0] placeholder that gets replaced later.
        fp.set_waypoint(Waypoint("start", round(t_start, 2), start_pos, [0, 0, 0], heading=[0, 0]))
        fp.set_waypoint(Waypoint("mid", 20.0, mid_pos, [0, 0, 0], heading=[0, 0]))
        fp.set_waypoint(Waypoint("end", round(t_end, 2), end_pos, [0, 0, 0], heading=[0, 0]))

        fp.max_var_lin_vel = 10.0
        fp.max_var_ang_vel = 2.0
        fp.set_uniform_velocity()
        fp.connect_waypoints()
        _sync_headings_to_velocity(fp)

        space.append(fp)

    return space


def build_viz08_plans(n_uavs: int = 5) -> Tuple[Dict[str, FlightPlan], Dict[str, FlightPlan]]:
    """
    Build the original and resolved Viz 08 flight plans.

    Returns:
        A tuple of two dictionaries:
        - original_plans: flight plans before resolution
        - resolved_plans: flight plans after the central manager cascade
    """
    # Build the requested fleet once, then resolve it in place through the same
    # cascade logic that the original visualizer used.
    space = generate_straight_fleet(n_uavs=n_uavs)
    original_plans = {f"UAV_{i + 1}": fp.copy() for i, fp in enumerate(space)}

    manager = CentralManager()

    for i, fp in enumerate(space):
        uav_id = f"UAV_{i + 1}"
        priority = max(2 - i, 0)
        manager.register_uav(uav_id, fp, priority=priority)
        manager.check_and_resolve(uav_id)

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


def get_viz08_flightplans(case: str = "resolved", n_uavs: int = 5) -> Dict[str, FlightPlan]:
    """
    Return one Viz 08 scenario variant by name.

    Args:
        case: "original" or "resolved".
        n_uavs: Number of UAVs to generate.
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