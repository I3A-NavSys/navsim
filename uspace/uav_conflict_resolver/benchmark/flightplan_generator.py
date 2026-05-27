"""
flightplan_generator.py — Realistic FlightPlan Generator
==========================================================

PURPOSE:
    Generate kinematically-valid UAV flight plans that mimic real mission
    profiles.  Each generated plan is bounded within a configurable rectangular
    prism (default: 1 km cube) and follows the full 7D polynomial trajectory
    model used by the system (pos, vel, acel, jerk, snap, crakle).

TRAJECTORY MODEL (7D):
    The state of a UAV at any time t is described by seven dimensions:
        (x, y, z, vx, vy, vz, t)
    The trajectory between two consecutive waypoints is a degree-5 (quintic)
    polynomial computed by Waypoint.connect_to(), which solves for the three
    free higher-order derivatives (jerk, snap, crakle) to achieve C²-continuous
    motion matching position, velocity and acceleration boundary conditions.

GENERATION WORKFLOW:
    1. Pick a random start point near one face of the prism (takeoff location).
    2. Pick a random end point near the opposite/adjacent face (landing location).
    3. Generate intermediate waypoints using a momentum-biased random walk
       that produces smooth, realistic flight corridors — not jagged zigzags.
    4. Distribute timestamps proportionally to inter-waypoint distances,
       yielding approximately uniform cruise speed across segments.
    5. Set takeoff/landing velocity to zero, cruise velocities via URM.
    6. Call connect_waypoints() to compute the full quintic polynomial
       (jerk, snap, crakle) for every segment — the "7D paper" step.

USAGE:
    >>> from trials.flightplan_generator import generate_flight_plan
    >>> fp = generate_flight_plan(seed=42)
    >>> fp.print_waypoints()
    >>> fp.position_figure("Generated Plan", 0.1)

    >>> from trials.flightplan_generator import generate_crossing_pair
    >>> fp1, fp2 = generate_crossing_pair(seed=42)

    >>> from trials.flightplan_generator import generate_random_fleet
    >>> fleet = generate_random_fleet(n_uavs=5, seed=42)
"""

from __future__ import annotations

import sys
from pathlib import Path
import numpy as np
from typing import List, Tuple, Optional

# ---------------------------------------------------------------------------
# Path setup: allow running this script from the trials/ folder directly
# ---------------------------------------------------------------------------
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from core.config import UAV_MAX_SPEED, UAV_MAX_ACCEL


# ============================================================================
# Default Constants
# ============================================================================

DEFAULT_PRISM = (
    (0.0, 1000.0),   # x range [m]
    (0.0, 1000.0),   # y range [m]
    (20.0, 200.0),   # z range [m] — realistic flight altitude band
)

DEFAULT_SPEED_RANGE = (5.0, UAV_MAX_SPEED)  # min/max cruise speed [m/s]
DEFAULT_RADIUS = 2.0                        # safety radius [m]
DEFAULT_MAX_LIN_VEL = UAV_MAX_SPEED         # maximum linear velocity [m/s]
DEFAULT_MAX_ANG_VEL = 1.0                   # maximum angular velocity [rad/s]

# Takeoff / landing altitude offset from the prism floor
_TAKEOFF_ALTITUDE = 30.0             # [m] above prism z_min
_LANDING_ALTITUDE = 30.0             # [m] above prism z_min

# Lateral perturbation scale for intermediate waypoints (fraction of prism size)
# MUCH lower value (0.03) for realistic, professional-looking trajectories
_LATERAL_NOISE_FRACTION = 0.03


def _pick_perimeter_anchor(
    rng: np.random.Generator,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    z_range: Tuple[float, float],
    uav_id: int,
    phase: str,
) -> np.ndarray:
    """Pick a start/end anchor on the prism perimeter to reduce endpoint clustering."""
    x_span = x_range[1] - x_range[0]
    y_span = y_range[1] - y_range[0]
    x_margin = max(25.0, 0.08 * x_span)
    y_margin = max(25.0, 0.08 * y_span)

    lane_count = 12
    x_lanes = np.linspace(x_range[0] + x_margin, x_range[1] - x_margin, lane_count)
    y_lanes = np.linspace(y_range[0] + y_margin, y_range[1] - y_margin, lane_count)

    # Use a stable side pattern so different UAVs do not all start/end on the same face.
    base = uav_id if phase == "start" else uav_id + 3
    side = base % 4
    lane_idx = base % lane_count

    z_anchor = z_range[0] + (_TAKEOFF_ALTITUDE if phase == "start" else _LANDING_ALTITUDE)
    z_anchor = float(np.clip(z_anchor, z_range[0] + 1.0, z_range[1] - 1.0))

    if side == 0:
        point = np.array([x_range[0] + x_margin, y_lanes[lane_idx], z_anchor])
    elif side == 1:
        point = np.array([x_range[1] - x_margin, y_lanes[lane_idx], z_anchor])
    elif side == 2:
        point = np.array([x_lanes[lane_idx], y_range[0] + y_margin, z_anchor])
    else:
        point = np.array([x_lanes[lane_idx], y_range[1] - y_margin, z_anchor])

    # Small jitter keeps different plans from being perfectly stacked on the lane.
    point[0] = float(np.clip(point[0] + rng.uniform(-0.15, 0.15) * x_margin, x_range[0], x_range[1]))
    point[1] = float(np.clip(point[1] + rng.uniform(-0.15, 0.15) * y_margin, y_range[0], y_range[1]))
    return point


# ============================================================================
# Core Generator
# ============================================================================
def generate_flight_plan(
    prism: Tuple[Tuple[float, float], ...] = DEFAULT_PRISM,
    t_start: float = 0.0,
    t_end: float = 120.0,
    num_waypoints: Optional[int] = None,
    speed_range: Tuple[float, float] = DEFAULT_SPEED_RANGE,
    radius: float = DEFAULT_RADIUS,
    max_lin_vel: float = UAV_MAX_SPEED,
    max_ang_vel: float = DEFAULT_MAX_ANG_VEL,
    v_max: float = UAV_MAX_SPEED,
    a_max: Optional[float] = UAV_MAX_ACCEL,
    uav_id: int = 0,
    priority: int = 0,
    seed: Optional[int] = None,
    takeoff_landing: bool = True,
) -> FlightPlan:
    """
    Generate a single, kinematically-valid FlightPlan within a rectangular prism.

    The generated trajectory mimics a realistic UAV mission:
        Takeoff (v_min) → Cruise (URM segments, capped at max_lin_vel) → Landing (v=0)

    Velocity behavior:
        - First waypoint: minimum cruise speed toward next waypoint (directional)
        - Intermediate waypoints: URM velocity capped at max_lin_vel
        - Last waypoint: zero velocity (landing/hover)
        - All velocities are directed toward the next waypoint for smooth transitions

    Args:
        prism:           Bounding volume as ((x_min, x_max), (y_min, y_max), (z_min, z_max)).
        t_start:         Mission start time [s].
        t_end:           Mission end time [s].
        num_waypoints:   Total number of waypoints (including start/end).
                         If None, randomly chosen between 4 and 7.
        speed_range:     (min_speed, max_speed) for cruise segments [m/s].
        radius:          UAV safety radius [m].
        max_lin_vel:     Maximum linear velocity [m/s]. Cruise velocities capped to this value.
        max_ang_vel:     Maximum angular velocity [rad/s].
        v_max:           Maximum speed for kinematic feasibility check [m/s].
        a_max:           Maximum acceleration for kinematic feasibility check [m/s²]. Pass None to skip.
        uav_id:          FlightPlan ID.
        priority:        FlightPlan priority (higher = VIP).
        seed:            Random seed for reproducibility.
        takeoff_landing: If True, start with minimum cruise speed and end with zero velocity.

    Returns:
        A fully-connected FlightPlan with 7D polynomial interpolation.
    """
    rng = np.random.default_rng(seed)

    x_range, y_range, z_range = prism

    # ------------------------------------------------------------------
    # Step 1: Decide number of waypoints (5-6 for consistency/realism)
    # ------------------------------------------------------------------
    if num_waypoints is None:
        num_waypoints = rng.integers(5, 7)  # 5 to 6 waypoints for realistic missions
    num_waypoints = max(3, num_waypoints)   # minimum 3 (start, mid, end)

    # ------------------------------------------------------------------
    # Step 2: Generate start and end positions
    # ------------------------------------------------------------------
    # For takeoff/landing missions, anchor the endpoints on separated
    # perimeter lanes so the first and last segments are less likely to
    # collide across different UAVs.
    if takeoff_landing:
        start_pos = _pick_perimeter_anchor(rng, x_range, y_range, z_range, uav_id, "start")
        end_pos = _pick_perimeter_anchor(rng, x_range, y_range, z_range, uav_id, "end")
    else:
        start_pos = _random_point_in_prism(rng, x_range, y_range, z_range)
        end_pos = _random_point_in_prism(rng, x_range, y_range, z_range)

    # Ensure minimum separation between start and end (at least 20% of prism diagonal)
    prism_diagonal = np.sqrt(
        (x_range[1] - x_range[0])**2 +
        (y_range[1] - y_range[0])**2 +
        (z_range[1] - z_range[0])**2
    )
    min_separation = 0.2 * prism_diagonal
    attempts = 0
    while np.linalg.norm(end_pos - start_pos) < min_separation and attempts < 50:
        end_pos = _random_point_in_prism(rng, x_range, y_range, z_range)
        attempts += 1

    # ------------------------------------------------------------------
    # Step 3: Generate intermediate waypoints (momentum-biased random walk)
    # ------------------------------------------------------------------
    positions = _generate_smooth_path(
        rng, start_pos, end_pos, num_waypoints, prism, _LATERAL_NOISE_FRACTION
    )

    # ------------------------------------------------------------------
    # Step 4: Distribute timestamps proportionally to inter-waypoint distances
    # ------------------------------------------------------------------
    total_time = t_end - t_start
    distances = [np.linalg.norm(positions[i+1] - positions[i])
                 for i in range(len(positions) - 1)]
    total_distance = sum(distances)

    if total_distance < 1e-6:
        # Degenerate case: uniform spacing
        times = np.linspace(t_start, t_end, len(positions))
    else:
        times = [t_start]
        for i, d in enumerate(distances):
            fraction = d / total_distance
            times.append(times[-1] + fraction * total_time)
        times[-1] = t_end  # Ensure exact end time

    # ------------------------------------------------------------------
    # Step 5: Check and cap cruise speed within speed_range
    # ------------------------------------------------------------------
    # Enforce speed limits with smooth time scaling
    max_speed = speed_range[1]
    min_speed = speed_range[0]
    
    for i in range(len(positions) - 1):
        dt = times[i+1] - times[i]
        if dt <= 0:
            continue
        seg_speed = distances[i] / dt
        if seg_speed > max_speed:
            # Stretch total time proportionally to cap speed
            scale = seg_speed / max_speed
            # Rescale all remaining time intervals from this point forward
            for j in range(i+1, len(times)):
                times[j] = times[i] + (times[j] - times[i]) * scale
        elif seg_speed < min_speed and distances[i] > 1e-6:
            # Compress time if speed is too slow (minimum cruising speed)
            scale = min_speed / seg_speed
            for j in range(i+1, len(times)):
                times[j] = times[i] + (times[j] - times[i]) / scale

    # ------------------------------------------------------------------
    # Step 6: Build the FlightPlan
    # ------------------------------------------------------------------
    fp = FlightPlan()
    fp.id = uav_id
    fp.priority = priority
    fp.radius = radius
    fp.max_var_lin_vel = max_lin_vel
    fp.max_var_ang_vel = max_ang_vel

    for i, (pos, t) in enumerate(zip(positions, times)):
        label = f"WP_{i}"
        if i == 0:
            label = "START"
        elif i == len(positions) - 1:
            label = "END"

        # Compute velocity: direction toward next waypoint
        # Último wp: velocidad 0 (landing)
        # Primero wp: velocidad mínima hacia el siguiente
        if i == len(positions) - 1:
            # Last waypoint: zero velocity (landing)
            vel = np.zeros(3)
        elif i == 0 and takeoff_landing:
            # First waypoint: minimum cruise speed toward next waypoint
            dt = times[i+1] - times[i]
            direction = (positions[i+1] - positions[i])
            direction_norm = np.linalg.norm(direction)
            
            if direction_norm > 1e-6 and dt > 0:
                # Normalized direction scaled by minimum cruise speed
                min_speed = speed_range[0]
                vel = (direction / direction_norm) * min_speed
            else:
                vel = np.zeros(3)
        else:
            # Intermediate waypoints: URM velocity toward next waypoint
            dt = times[i+1] - times[i]
            if dt > 0:
                vel = (positions[i+1] - positions[i]) / dt
                # Cap to max linear velocity if needed
                vel_magnitude = np.linalg.norm(vel)
                if vel_magnitude > max_lin_vel and vel_magnitude > 1e-6:
                    vel = vel / vel_magnitude * max_lin_vel
            else:
                vel = np.zeros(3)

        wp = Waypoint(
            label=label,
            t=round(t, 3),
            pos=pos.tolist(),
            vel=vel.tolist(),
        )
        fp.set_waypoint(wp)

    # ------------------------------------------------------------------
    # Step 7: Connect waypoints — computes jerk, snap, crakle (7D model)
    # ------------------------------------------------------------------
    fp.connect_waypoints(v_max=v_max, a_max=a_max)

    return fp


# ============================================================================
# Convenience Generators
# ============================================================================

def generate_crossing_pair(
    prism: Tuple[Tuple[float, float], ...] = DEFAULT_PRISM,
    t_start: float = 0.0,
    t_end: float = 100.0,
    radius: float = DEFAULT_RADIUS,
    v_max: float = UAV_MAX_SPEED,
    a_max: Optional[float] = UAV_MAX_ACCEL,
    seed: Optional[int] = None,
) -> Tuple[FlightPlan, FlightPlan]:
    """
    Generate two FlightPlans guaranteed to cross in space and time.

    UAV 1 flies roughly West→East, UAV 2 flies roughly South→North,
    both passing through the center of the prism at approximately the
    same time. This creates a natural crossing conflict.

    Args:
        prism:   Bounding volume.
        t_start: Mission start time.
        t_end:   Mission end time.
        radius:  UAV safety radius.
        v_max:   Maximum speed for kinematic feasibility check [m/s]. Defaults to UAV_MAX_SPEED from config.
        a_max:   Maximum acceleration for kinematic feasibility check [m/s²]. Defaults to UAV_MAX_ACCEL from config. Pass None to skip.
        seed:    Random seed.

    Returns:
        (fp1, fp2): Two crossing FlightPlans.
    """
    rng = np.random.default_rng(seed)
    x_range, y_range, z_range = prism

    # Center of the prism
    cx = (x_range[0] + x_range[1]) / 2.0
    cy = (y_range[0] + y_range[1]) / 2.0
    cz = (z_range[0] + z_range[1]) / 2.0

    # Small random offsets to avoid exact intersection
    offset = rng.uniform(-20, 20, 3)

    # UAV 1: West → East (X-axis dominant)
    margin_x = (x_range[1] - x_range[0]) * 0.1
    margin_y = (y_range[1] - y_range[0]) * 0.1
    fp1_start = np.array([x_range[0] + margin_x,
                          cy + rng.uniform(-50, 50),
                          cz + rng.uniform(-20, 20)])
    fp1_mid   = np.array([cx + offset[0], cy + offset[1], cz + offset[2]])
    fp1_end   = np.array([x_range[1] - margin_x,
                          cy + rng.uniform(-50, 50),
                          cz + rng.uniform(-20, 20)])

    # Clamp to prism
    fp1_start = _clamp_to_prism(fp1_start, prism)
    fp1_mid   = _clamp_to_prism(fp1_mid, prism)
    fp1_end   = _clamp_to_prism(fp1_end, prism)

    # UAV 2: South → North (Y-axis dominant)
    fp2_start = np.array([cx + rng.uniform(-50, 50),
                          y_range[0] + margin_y,
                          cz + rng.uniform(-20, 20)])
    fp2_mid   = np.array([cx + offset[0], cy + offset[1], cz + offset[2]])
    fp2_end   = np.array([cx + rng.uniform(-50, 50),
                          y_range[1] - margin_y,
                          cz + rng.uniform(-20, 20)])

    fp2_start = _clamp_to_prism(fp2_start, prism)
    fp2_mid   = _clamp_to_prism(fp2_mid, prism)
    fp2_end   = _clamp_to_prism(fp2_end, prism)

    # Build FlightPlan 1
    t_mid = (t_start + t_end) / 2.0
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = radius
    fp1.max_var_lin_vel = DEFAULT_MAX_LIN_VEL
    fp1.max_var_ang_vel = DEFAULT_MAX_ANG_VEL

    fp1.set_waypoint(Waypoint(label="START", t=t_start, pos=fp1_start.tolist(), vel=[0,0,0]))

    dt1 = t_mid - t_start
    vel1_cruise = ((fp1_mid - fp1_start) / dt1).tolist() if dt1 > 0 else [0,0,0]
    fp1.set_waypoint(Waypoint(label="WP_1", t=round(t_start + dt1 * 0.3, 3),
                              pos=_lerp(fp1_start, fp1_mid, 0.3).tolist(),
                              vel=vel1_cruise))

    fp1.set_waypoint(Waypoint(label="CROSS", t=round(t_mid, 3),
                              pos=fp1_mid.tolist(), vel=vel1_cruise))

    dt2 = t_end - t_mid
    vel2_cruise = ((fp1_end - fp1_mid) / dt2).tolist() if dt2 > 0 else [0,0,0]
    fp1.set_waypoint(Waypoint(label="WP_3", t=round(t_mid + dt2 * 0.7, 3),
                              pos=_lerp(fp1_mid, fp1_end, 0.7).tolist(),
                              vel=vel2_cruise))

    fp1.set_waypoint(Waypoint(label="END", t=t_end, pos=fp1_end.tolist(), vel=[0,0,0]))
    fp1.connect_waypoints(v_max=v_max, a_max=a_max)

    # Build FlightPlan 2
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = radius
    fp2.max_var_lin_vel = DEFAULT_MAX_LIN_VEL
    fp2.max_var_ang_vel = DEFAULT_MAX_ANG_VEL

    fp2.set_waypoint(Waypoint(label="START", t=t_start, pos=fp2_start.tolist(), vel=[0,0,0]))

    vel2a_cruise = ((fp2_mid - fp2_start) / dt1).tolist() if dt1 > 0 else [0,0,0]
    fp2.set_waypoint(Waypoint(label="WP_1", t=round(t_start + dt1 * 0.3, 3),
                              pos=_lerp(fp2_start, fp2_mid, 0.3).tolist(),
                              vel=vel2a_cruise))

    fp2.set_waypoint(Waypoint(label="CROSS", t=round(t_mid, 3),
                              pos=fp2_mid.tolist(), vel=vel2a_cruise))

    vel2b_cruise = ((fp2_end - fp2_mid) / dt2).tolist() if dt2 > 0 else [0,0,0]
    fp2.set_waypoint(Waypoint(label="WP_3", t=round(t_mid + dt2 * 0.7, 3),
                              pos=_lerp(fp2_mid, fp2_end, 0.7).tolist(),
                              vel=vel2b_cruise))

    fp2.set_waypoint(Waypoint(label="END", t=t_end, pos=fp2_end.tolist(), vel=[0,0,0]))
    fp2.connect_waypoints(v_max=v_max, a_max=a_max)

    return fp1, fp2


def generate_straight_layered_flight_plan(
    prism: Tuple[Tuple[float, float], ...] = DEFAULT_PRISM,
    t_start: float = 0.0,
    t_end: float = 120.0,
    num_waypoints: Optional[int] = None,
    speed_range: Tuple[float, float] = DEFAULT_SPEED_RANGE,
    radius: float = DEFAULT_RADIUS,
    max_lin_vel: float = UAV_MAX_SPEED,
    max_ang_vel: float = DEFAULT_MAX_ANG_VEL,
    v_max: float = UAV_MAX_SPEED,
    a_max: Optional[float] = UAV_MAX_ACCEL,
    uav_id: int = 0,
    priority: int = 0,
    seed: Optional[int] = None,
    z_level: Optional[float] = None,
    lane_y: Optional[float] = None,
    parallel_axis: str = "x",
    reverse: bool = False,
    takeoff_landing: bool = True,
    lateral_noise_fraction: float = 0.0,
) -> FlightPlan:
    """
    Generate a mostly straight FlightPlan constrained to a cruise Z level.

    The route is deliberately less erratic than generate_flight_plan():
    - XY path is a straight corridor, either along X or along Y
    - cruise happens at one fixed Z level
    - waypoints are ALWAYS connected with strict=True
    """
    rng = np.random.default_rng(seed)

    x_range, y_range, z_range = prism
    if num_waypoints is None:
        num_waypoints = 4
    num_waypoints = max(3, num_waypoints)

    total_time = max(t_end - t_start, 1.0)
    max_route_distance = max_lin_vel * total_time * 0.65
    xy_diag = np.sqrt((x_range[1] - x_range[0])**2 + (y_range[1] - y_range[0])**2)

    # Pick a cruise level inside the prism bounds (with a small safety margin).
    z_margin = 5.0
    z_lo = z_range[0] + z_margin
    z_hi = z_range[1] - z_margin
    if z_level is None:
        z_cruise = rng.uniform(z_lo, z_hi)
    else:
        z_cruise = float(np.clip(z_level, z_lo, z_hi))

    if parallel_axis not in {"x", "y"}:
        parallel_axis = "x"

    if parallel_axis == "x":
        start_xy = np.array([x_range[0] + 250.0, lane_y if lane_y is not None else rng.uniform(y_range[0] + 250.0, y_range[1] - 250.0)])
        end_xy = np.array([x_range[1] - 250.0, start_xy[1]])
    else:
        start_xy = np.array([lane_y if lane_y is not None else rng.uniform(x_range[0] + 250.0, x_range[1] - 250.0), y_range[0] + 250.0])
        end_xy = np.array([start_xy[0], y_range[1] - 250.0])

    if reverse:
        # Swap start and end to reverse direction along the corridor
        start_xy, end_xy = end_xy.copy(), start_xy.copy()

    # Keep the route inside the prism while preserving a straight corridor.
    start_xy[0] = float(np.clip(start_xy[0], x_range[0] + 10.0, x_range[1] - 10.0))
    start_xy[1] = float(np.clip(start_xy[1], y_range[0] + 10.0, y_range[1] - 10.0))
    end_xy[0] = float(np.clip(end_xy[0], x_range[0] + 10.0, x_range[1] - 10.0))
    end_xy[1] = float(np.clip(end_xy[1], y_range[0] + 10.0, y_range[1] - 10.0))

    z_start = z_range[0] + _TAKEOFF_ALTITUDE if takeoff_landing else z_cruise
    z_end = z_range[0] + _LANDING_ALTITUDE if takeoff_landing else z_cruise
    z_start = float(np.clip(z_start, z_range[0], z_range[1]))
    z_end = float(np.clip(z_end, z_range[0], z_range[1]))

    start = np.array([start_xy[0], start_xy[1], z_start])
    end = np.array([end_xy[0], end_xy[1], z_end])

    # Build near-straight positions. Intermediate WPs stay at z_cruise.
    positions: List[np.ndarray] = [start.copy()]
    route_vec_xy = end_xy - start_xy
    route_norm_xy = np.linalg.norm(route_vec_xy)

    if route_norm_xy > 1e-6:
        perp_xy = np.array([-route_vec_xy[1], route_vec_xy[0]]) / route_norm_xy
    else:
        perp_xy = np.array([1.0, 0.0])

    for i in range(1, num_waypoints - 1):
        progress = i / (num_waypoints - 1)
        base_xy = start_xy + route_vec_xy * progress
        envelope = np.sin(np.pi * progress)
        offset_xy = perp_xy * (lateral_noise_fraction * xy_diag * envelope)
        pos_xy = base_xy + offset_xy
        wp_pos = np.array([
            np.clip(pos_xy[0], x_range[0], x_range[1]),
            np.clip(pos_xy[1], y_range[0], y_range[1]),
            z_cruise,
        ])
        positions.append(wp_pos)

    positions.append(end.copy())

    # Time allocation by segment distance for near-constant cruise speed.
    distances = [
        np.linalg.norm(positions[i + 1] - positions[i])
        for i in range(len(positions) - 1)
    ]
    total_distance = sum(distances)
    min_feasible_time = total_distance / max_lin_vel if max_lin_vel > 1e-6 else total_time
    if total_time < min_feasible_time:
        total_time = min_feasible_time * 1.05

    if total_distance < 1e-6:
        times = np.linspace(t_start, t_end, len(positions)).tolist()
    else:
        times = [t_start]
        for d in distances:
            times.append(times[-1] + (d / total_distance) * total_time)
        times[-1] = t_start + total_time

    fp = FlightPlan()
    fp.id = uav_id
    fp.priority = priority
    fp.radius = radius
    fp.max_var_lin_vel = max_lin_vel
    fp.max_var_ang_vel = max_ang_vel

    for i, (pos, t) in enumerate(zip(positions, times)):
        label = f"WP_{i}"
        if i == 0:
            label = "START"
        elif i == len(positions) - 1:
            label = "END"

        if i == len(positions) - 1:
            vel = np.zeros(3)
        else:
            dt = times[i + 1] - times[i]
            direction = positions[i + 1] - positions[i]
            norm = np.linalg.norm(direction)

            if dt <= 0 or norm < 1e-6:
                vel = np.zeros(3)
            else:
                if i == 0 and takeoff_landing:
                    target_speed = min(speed_range[0], norm / dt, max_lin_vel)
                else:
                    target_speed = float(np.clip(norm / dt, speed_range[0], speed_range[1]))
                vel = direction / norm * min(target_speed, max_lin_vel)

        fp.set_waypoint(
            Waypoint(
                label=label,
                t=round(t, 3),
                pos=pos.tolist(),
                vel=vel.tolist(),
            )
        )

    # Required by caller: always enforce strict kinematic feasibility.
    fp.connect_waypoints(v_max=v_max, a_max=a_max, strict=True)
    return fp


def generate_layered_straight_fleet(
    n_uavs: int = 4,
    prism: Tuple[Tuple[float, float], ...] = DEFAULT_PRISM,
    t_start: float = 0.0,
    t_end: float = 120.0,
    radius: float = DEFAULT_RADIUS,
    v_max: float = UAV_MAX_SPEED,
    a_max: Optional[float] = UAV_MAX_ACCEL,
    seed: Optional[int] = None,
    levels: Optional[List[float]] = None,
    num_waypoints: int = 4,
    conflict_mode: bool = False,
) -> List[FlightPlan]:
    """
    Generate a fleet with straight-ish routes distributed by Z levels.

    Each UAV is assigned a cruise altitude in round-robin order from `levels`.
    The routes are laid out as parallel corridors along X with one fixed Y lane
    per UAV so the resulting fleet is organized, straight, and easy to read.
    """
    rng = np.random.default_rng(seed)
    x_range, y_range, z_range = prism

    if levels is None:
        z_lo = z_range[0] + 15.0
        z_hi = z_range[1] - 10.0
        levels = [float(z) for z in np.arange(z_lo, z_hi + 1e-6, 20.0)]
        if not levels:
            levels = [float((z_range[0] + z_range[1]) * 0.5)]

    # In conflict_mode we want fewer lanes and stronger temporal overlap
    if conflict_mode:
        # Collapse the fleet into a single altitude layer so XY crossings stay in 4D conflict space.
        levels = [float((z_range[0] + z_range[1]) * 0.5)]
        lane_count = max(2, int(np.ceil(n_uavs / 8)))
        lane_positions = np.linspace(y_range[0] + 120.0, y_range[1] - 120.0, lane_count)
        cross_positions = np.linspace(x_range[0] + 120.0, x_range[1] - 120.0, lane_count)
    else:
        lane_count = max(1, int(np.ceil(n_uavs / len(levels))))
        lane_positions = np.linspace(y_range[0] + 300.0, y_range[1] - 300.0, lane_count)
        cross_positions = np.linspace(x_range[0] + 300.0, x_range[1] - 300.0, lane_count)

    fleet: List[FlightPlan] = []
    mission_span = max(1.0, t_end - t_start)

    for i in range(n_uavs):
        if conflict_mode:
            # Force the same mission window so temporal overlap is guaranteed.
            uav_t_start = t_start
            uav_t_end = t_end
        else:
            stagger = rng.uniform(0, 0.2) * mission_span
            uav_t_start = t_start + stagger
            uav_t_end = t_end - rng.uniform(0, 0.1) * mission_span

        if uav_t_end <= uav_t_start + 10.0:
            uav_t_end = uav_t_start + 60.0

        level = float(levels[i % len(levels)])
        lane_index = i // len(levels)
        if conflict_mode:
            lane_index = i % len(lane_positions)
        lane_y = float(lane_positions[lane_index])
        lane_x = float(cross_positions[lane_index])
        parallel_axis = "x" if (i % 2 == 0) else "y"
        reverse = bool(rng.random() < 0.5)
        fp = generate_straight_layered_flight_plan(
            prism=prism,
            t_start=round(uav_t_start, 1),
            t_end=round(uav_t_end, 1),
            num_waypoints=num_waypoints,
            radius=radius,
            v_max=v_max,
            a_max=a_max,
            uav_id=i + 1,
            priority=i,
            seed=int(rng.integers(0, 2**31 - 1)),
            z_level=level,
            lane_y=lane_y if parallel_axis == "x" else lane_x,
            parallel_axis=parallel_axis,
            reverse=reverse,
            takeoff_landing=False,
            lateral_noise_fraction=0.0,
        )
        fleet.append(fp)

    return fleet


def generate_random_fleet(
    n_uavs: int = 4,
    prism: Tuple[Tuple[float, float], ...] = DEFAULT_PRISM,
    t_start: float = 0.0,
    t_end: float = 120.0,
    radius: float = DEFAULT_RADIUS,
    v_max: float = UAV_MAX_SPEED,
    a_max: Optional[float] = UAV_MAX_ACCEL,
    seed: Optional[int] = None,
) -> List[FlightPlan]:
    """
    Generate a fleet of N UAVs with overlapping time windows (conflict-prone).

    Each UAV gets a slightly staggered start time and a random route through
    the prism. The time windows overlap to increase collision probability
    for testing purposes.

    Args:
        n_uavs:  Number of UAVs to generate.
        prism:   Bounding volume.
        t_start: Earliest mission start time.
        t_end:   Latest mission end time.
        radius:  UAV safety radius.
        v_max:   Maximum speed for kinematic feasibility check [m/s].
        a_max:   Maximum acceleration for kinematic feasibility check [m/s²]. Pass None to skip.
        seed:    Random seed.

    Returns:
        List of FlightPlans.
    """
    rng = np.random.default_rng(seed)
    fleet = []

    for i in range(n_uavs):
        # Stagger start times slightly (0 to 20% of total time)
        stagger = rng.uniform(0, 0.2) * (t_end - t_start)
        uav_t_start = t_start + stagger
        uav_t_end   = t_end - rng.uniform(0, 0.1) * (t_end - t_start)

        if uav_t_end <= uav_t_start + 10:
            uav_t_end = uav_t_start + 60  # Minimum 60s flight

        fp = generate_flight_plan(
            prism=prism,
            t_start=round(uav_t_start, 1),
            t_end=round(uav_t_end, 1),
            radius=radius,
            v_max=v_max,
            a_max=a_max,
            uav_id=i + 1,
            priority=i,
            seed=rng.integers(0, 2**31) if seed is not None else None,
        )
        fleet.append(fp)

    return fleet


# ============================================================================
# Internal Helpers
# ============================================================================

def _random_point_in_prism(
    rng: np.random.Generator,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    z_range: Tuple[float, float],
) -> np.ndarray:
    """Generate a uniformly random 3D point within the prism."""
    return np.array([
        rng.uniform(x_range[0], x_range[1]),
        rng.uniform(y_range[0], y_range[1]),
        rng.uniform(z_range[0], z_range[1]),
    ])


def _clamp_to_prism(
    pos: np.ndarray,
    prism: Tuple[Tuple[float, float], ...],
) -> np.ndarray:
    """Clamp a 3D point to stay within the prism boundaries."""
    clamped = pos.copy()
    for i, (lo, hi) in enumerate(prism):
        clamped[i] = np.clip(clamped[i], lo, hi)
    return clamped


def _lerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    """Linear interpolation between two points."""
    return a + t * (b - a)


def _generate_smooth_path(
    rng: np.random.Generator,
    start: np.ndarray,
    end: np.ndarray,
    num_waypoints: int,
    prism: Tuple[Tuple[float, float], ...],
    noise_fraction: float,
) -> List[np.ndarray]:
    """
    Generate a smooth path from start to end with minimal perturbations.

    Creates a mostly linear path with subtle, smooth lateral variations.
    Each waypoint is mostly along the direct line (start→end) with minimal
    deviation for realism without erratic behavior.

    Args:
        rng:            Random number generator.
        start:          Start position [x, y, z].
        end:            End position [x, y, z].
        num_waypoints:  Total number of waypoints including start and end.
        prism:          Bounding volume for clamping.
        noise_fraction: Maximum lateral noise as a fraction of prism size.

    Returns:
        List of num_waypoints 3D positions, mostly linear with smooth curves.
    """
    if num_waypoints <= 2:
        return [start.copy(), end.copy()]

    positions = [start.copy()]

    # Direction vector from start to end
    direction = end - start
    total_dist = np.linalg.norm(direction)
    
    if total_dist < 1e-6:
        # Degenerate case: start ≈ end, distribute uniformly
        return [start + (end - start) * (i / (num_waypoints - 1)) for i in range(num_waypoints)]

    direction_normalized = direction / total_dist

    # Generate ONE consistent random direction for all lateral perturbations
    # This creates smooth, coherent curves instead of random zigzags
    lateral_dir = rng.normal(0, 1, 3)
    # Make orthogonal to main direction
    lateral_dir = lateral_dir - np.dot(lateral_dir, direction_normalized) * direction_normalized
    lateral_mag = np.linalg.norm(lateral_dir)
    if lateral_mag > 1e-6:
        lateral_dir = lateral_dir / lateral_mag
    else:
        # Fallback if orthogonalization fails
        lateral_dir = np.array([1, 0, 0]) if abs(direction_normalized[0]) < 0.9 else np.array([0, 1, 0])

    # Compute prism diagonal for noise scaling
    prism_diag = np.sqrt(sum((hi - lo)**2 for (lo, hi) in prism))

    for i in range(1, num_waypoints - 1):
        # Progress fraction along the start→end line (0 to 1)
        progress = i / (num_waypoints - 1)

        # Main position: simple linear interpolation (STRAIGHT PATH)
        base_pos = start + direction * progress

        # Smooth bell-shaped envelope: peaks at 0.5, zero at 0 and 1
        # This ensures waypoints at start and end have zero perturbation
        envelope = np.sin(progress * np.pi)
        
        # Single smooth sine wave for very subtle lateral motion
        # amplitude decreases: high at middle, low at ends
        amplitude = noise_fraction * prism_diag * envelope
        
        # Apply perturbation: coherent smooth curve in one lateral direction
        perturbation = lateral_dir * amplitude

        # Compute the waypoint position
        wp_pos = base_pos + perturbation

        # Clamp to prism boundaries
        wp_pos = _clamp_to_prism(wp_pos, prism)

        positions.append(wp_pos)

    positions.append(end.copy())
    return positions


# ============================================================================
# Standalone Demo
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("FLIGHT PLAN GENERATOR — Standalone Demo")
    print("=" * 70)

    # --- Demo 1: Single random flight plan ---
    print("\n[1] Generating a single random flight plan...")
    fp = generate_flight_plan(seed=42)
    print(f"    ID: {fp.id}, Priority: {fp.priority}, Radius: {fp.radius}m")
    print(f"    Waypoints: {len(fp.waypoints)}")
    print(f"    Time: {fp.init_time():.1f}s -> {fp.finish_time():.1f}s")
    print(f"    Duration: {fp.finish_time() - fp.init_time():.1f}s")
    print("\n    Waypoint details:")
    fp.print_waypoints()

    # Verify 7D: check that jerk/snap/crakle are non-zero
    has_higher_order = False
    for wp in fp.waypoints[:-1]:
        if np.linalg.norm(wp.jerk) > 1e-10 or np.linalg.norm(wp.snap) > 1e-10:
            has_higher_order = True
            break
    print(f"\n    7D polynomial active (jerk/snap/crakle != 0): {'OK' if has_higher_order else 'FAIL'}")

    # --- Demo 2: Crossing pair ---
    print("\n" + "-" * 70)
    print("[2] Generating a crossing pair...")
    fp1, fp2 = generate_crossing_pair(seed=123)
    print(f"    UAV 1: {len(fp1.waypoints)} waypoints, "
          f"t=[{fp1.init_time():.1f}, {fp1.finish_time():.1f}]s")
    print(f"    UAV 2: {len(fp2.waypoints)} waypoints, "
          f"t=[{fp2.init_time():.1f}, {fp2.finish_time():.1f}]s")

    # Check if they actually cross near the center
    t_mid = (fp1.init_time() + fp1.finish_time()) / 2
    pos1_mid = fp1.status_at_time(t_mid).pos
    pos2_mid = fp2.status_at_time(t_mid).pos
    dist_at_cross = np.linalg.norm(pos1_mid - pos2_mid)
    print(f"    Distance at t_mid={t_mid:.1f}s: {dist_at_cross:.1f}m")

    # --- Demo 3: Fleet ---
    print("\n" + "-" * 70)
    print("[3] Generating a fleet of 4 UAVs...")
    fleet = generate_random_fleet(n_uavs=4, seed=456)
    for i, fp in enumerate(fleet):
        print(f"    UAV {fp.id}: {len(fp.waypoints)} WPs, "
              f"t=[{fp.init_time():.1f}, {fp.finish_time():.1f}]s, "
              f"priority={fp.priority}")

    print("\n" + "=" * 70)
    print("All demos completed successfully.")
    print("=" * 70)
