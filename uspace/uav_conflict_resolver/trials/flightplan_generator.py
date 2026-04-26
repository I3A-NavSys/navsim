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


# ============================================================================
# Default Constants
# ============================================================================

DEFAULT_PRISM = (
    (0.0, 1000.0),   # x range [m]
    (0.0, 1000.0),   # y range [m]
    (20.0, 200.0),   # z range [m] — realistic flight altitude band
)

DEFAULT_SPEED_RANGE = (5.0, 20.0)    # min/max cruise speed [m/s]
DEFAULT_RADIUS = 2.0                 # safety radius [m]
DEFAULT_MAX_LIN_VEL = 20.0           # maximum linear velocity [m/s]
DEFAULT_MAX_ANG_VEL = 1.0            # maximum angular velocity [rad/s]

# Takeoff / landing altitude offset from the prism floor
_TAKEOFF_ALTITUDE = 30.0             # [m] above prism z_min
_LANDING_ALTITUDE = 30.0             # [m] above prism z_min

# Lateral perturbation scale for intermediate waypoints (fraction of prism size)
_LATERAL_NOISE_FRACTION = 0.15


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
    max_lin_vel: float = DEFAULT_MAX_LIN_VEL,
    max_ang_vel: float = DEFAULT_MAX_ANG_VEL,
    uav_id: int = 0,
    priority: int = 0,
    seed: Optional[int] = None,
    takeoff_landing: bool = True,
) -> FlightPlan:
    """
    Generate a single, kinematically-valid FlightPlan within a rectangular prism.

    The generated trajectory mimics a realistic UAV mission:
        Takeoff (v=0) → Cruise (URM segments) → Landing (v=0)

    Args:
        prism:           Bounding volume as ((x_min, x_max), (y_min, y_max), (z_min, z_max)).
        t_start:         Mission start time [s].
        t_end:           Mission end time [s].
        num_waypoints:   Total number of waypoints (including start/end).
                         If None, randomly chosen between 4 and 7.
        speed_range:     (min_speed, max_speed) for cruise segments [m/s].
        radius:          UAV safety radius [m].
        max_lin_vel:     Maximum linear velocity [m/s].
        max_ang_vel:     Maximum angular velocity [rad/s].
        uav_id:          FlightPlan ID.
        priority:        FlightPlan priority (higher = VIP).
        seed:            Random seed for reproducibility.
        takeoff_landing: If True, start and end with zero velocity (hover).

    Returns:
        A fully-connected FlightPlan with 7D polynomial interpolation.
    """
    rng = np.random.default_rng(seed)

    x_range, y_range, z_range = prism

    # ------------------------------------------------------------------
    # Step 1: Decide number of waypoints
    # ------------------------------------------------------------------
    if num_waypoints is None:
        num_waypoints = rng.integers(4, 8)  # 4 to 7 waypoints
    num_waypoints = max(3, num_waypoints)   # minimum 3 (start, mid, end)

    # ------------------------------------------------------------------
    # Step 2: Generate start and end positions
    # ------------------------------------------------------------------
    start_pos = _random_point_in_prism(rng, x_range, y_range, z_range)
    end_pos   = _random_point_in_prism(rng, x_range, y_range, z_range)

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
    max_speed = speed_range[1]
    for i in range(len(positions) - 1):
        dt = times[i+1] - times[i]
        if dt <= 0:
            continue
        seg_speed = distances[i] / dt
        if seg_speed > max_speed:
            # Stretch total time proportionally to cap speed
            scale = seg_speed / max_speed
            # Rescale all remaining time intervals
            for j in range(i+1, len(times)):
                times[j] = times[i] + (times[j] - times[i]) * scale

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

        # Compute velocity: URM between consecutive waypoints
        if i < len(positions) - 1:
            dt = times[i+1] - times[i]
            if dt > 0:
                vel = (positions[i+1] - positions[i]) / dt
            else:
                vel = np.zeros(3)
        else:
            vel = np.zeros(3)  # Last waypoint: zero velocity (landing)

        # Takeoff/landing: zero velocity at start and end
        if takeoff_landing and (i == 0 or i == len(positions) - 1):
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
    fp.connect_waypoints()

    return fp


# ============================================================================
# Convenience Generators
# ============================================================================

def generate_crossing_pair(
    prism: Tuple[Tuple[float, float], ...] = DEFAULT_PRISM,
    t_start: float = 0.0,
    t_end: float = 100.0,
    radius: float = DEFAULT_RADIUS,
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
    fp1.connect_waypoints()

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
    fp2.connect_waypoints()

    return fp1, fp2


def generate_random_fleet(
    n_uavs: int = 4,
    prism: Tuple[Tuple[float, float], ...] = DEFAULT_PRISM,
    t_start: float = 0.0,
    t_end: float = 120.0,
    radius: float = DEFAULT_RADIUS,
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
    Generate a smooth path from start to end with intermediate waypoints.

    Uses a momentum-biased random walk: each intermediate point is placed
    along the line from start to end (at uniform progress fractions) with
    a random lateral perturbation.  The perturbation magnitude decreases
    as we approach the destination, producing natural-looking flight corridors.

    Args:
        rng:            Random number generator.
        start:          Start position [x, y, z].
        end:            End position [x, y, z].
        num_waypoints:  Total number of waypoints including start and end.
        prism:          Bounding volume for clamping.
        noise_fraction: Maximum lateral noise as a fraction of prism size.

    Returns:
        List of num_waypoints 3D positions.
    """
    if num_waypoints <= 2:
        return [start.copy(), end.copy()]

    positions = [start.copy()]

    # Direction vector from start to end
    direction = end - start
    total_dist = np.linalg.norm(direction)

    # Compute prism dimensions for noise scaling
    prism_size = np.array([hi - lo for (lo, hi) in prism])

    for i in range(1, num_waypoints - 1):
        # Progress fraction along the start→end line
        progress = i / (num_waypoints - 1)

        # Base position: linear interpolation
        base_pos = _lerp(start, end, progress)

        # Lateral perturbation: perpendicular to the main direction
        # Magnitude decreases near start and end (bell-shaped envelope)
        envelope = np.sin(progress * np.pi)  # peaks at 0.5, zero at 0 and 1
        max_noise = noise_fraction * prism_size * envelope

        perturbation = rng.uniform(-1, 1, 3) * max_noise

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
    print(f"    Time: {fp.init_time():.1f}s → {fp.finish_time():.1f}s")
    print(f"    Duration: {fp.finish_time() - fp.init_time():.1f}s")
    print("\n    Waypoint details:")
    fp.print_waypoints()

    # Verify 7D: check that jerk/snap/crakle are non-zero
    has_higher_order = False
    for wp in fp.waypoints[:-1]:
        if np.linalg.norm(wp.jerk) > 1e-10 or np.linalg.norm(wp.snap) > 1e-10:
            has_higher_order = True
            break
    print(f"\n    7D polynomial active (jerk/snap/crakle ≠ 0): {'✓' if has_higher_order else '✗'}")

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
