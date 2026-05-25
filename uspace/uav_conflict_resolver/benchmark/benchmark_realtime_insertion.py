"""
benchmark_realtime_insertion.py - Incremental real-time resolver benchmark
=========================================================================

PURPOSE:
    Simulate the resolver as it is intended to run in production:

    1. Start with an empty 4D airspace (x, y, z, t).
    2. Insert one new flight plan.
    3. Check for conflicts through CentralManager.
    4. Resolve them immediately.
    5. Repeat until the per-insertion detection + resolution time exceeds
       a configured real-time budget, or until the resolver reaches deadlock.

AIRSPACE:
    20 km x 20 km x 90 m volume.
    Between 30 m and 120 m altitude (volable band).

NOTES:
    The benchmark uses dense, corridor-like flight plans so that conflicts
    appear naturally as the fleet grows. This is closer to a live system than
    a pre-generated batch benchmark with all aircraft present from the start.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import psutil

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from core.config import UAV_MAX_SPEED
from central_manager import CentralManager


# ---------------------------------------------------------------------------
# Benchmark configuration
# ---------------------------------------------------------------------------
REALISTIC_PRISM: Tuple[Tuple[float, float], ...] = (
    (0.0, 20000.0),   # X: 20 km
    (0.0, 20000.0),   # Y: 20 km
    (30.0, 120.0),    # Z: volable band
)

# If a single insertion takes longer than this, the benchmark stops because
# the resolver would no longer be usable in a real-time setting. 1 s is a
# sane starting point for a full detect + resolve cycle on a development box.
DEFAULT_TIME_BUDGET_MS: float = 10000.0
# New UAVs are inserted with a time offset so the scenario grows gradually.
DEFAULT_INSERTION_GAP_S: float = 3.0
# R-Tree sampling interval used when the new plan is registered.
DEFAULT_INTERVAL_S: float = 0.5
DEFAULT_MAX_UAVS: Optional[int] = None


def _pick_altitude_level(index: int, prism: Tuple[Tuple[float, float], ...], rng: np.random.Generator) -> float:
    """Distribute UAVs across a finite set of altitude layers."""
    z_min, z_max = prism[2]
    # Keep the altitude layers inside the flying band and leave a small
    # margin so plans do not touch the hard bounds.
    usable_height = max(20.0, z_max - z_min - 10.0)
    layer_count = 8
    layer_height = usable_height / layer_count
    layer_index = index % layer_count
    base_z = z_min + 5.0 + layer_index * layer_height
    # Add a small random perturbation so different UAVs are not perfectly
    # stacked on the same altitude.
    altitude = base_z + rng.uniform(-0.25 * layer_height, 0.25 * layer_height)
    return float(min(z_max - 1.0, max(z_min + 1.0, altitude)))


def _build_collinear_plan(
    uav_index: int,
    prism: Tuple[Tuple[float, float], ...] = REALISTIC_PRISM,
    seed: int = 42,
) -> FlightPlan:
    """Create a long, corridor-like flight plan that can interact with others."""
    rng = np.random.default_rng(seed + (uav_index * 1009))

    x_min, x_max = prism[0]
    y_min, y_max = prism[1]

    x_span = x_max - x_min
    y_span = y_max - y_min
    x_margin = 400.0
    y_margin = 400.0

    mode = uav_index % 4
    # Each UAV alternates between horizontal and vertical corridors so the
    # fleet gradually fills the airspace from different directions.
    lane_count = 10
    lanes_y = np.linspace(y_min + 1500.0, y_max - 1500.0, lane_count)
    lanes_x = np.linspace(x_min + 1500.0, x_max - 1500.0, lane_count)

    z_level = _pick_altitude_level(uav_index, prism, rng)
    cruise_speed = float(rng.uniform(12.0, min(17.0, UAV_MAX_SPEED)))

    if mode == 0:
        y_line = float(rng.choice(lanes_y) + rng.uniform(-180.0, 180.0))
        start_pos = np.array([x_min + x_margin, y_line, z_level])
        end_pos = np.array([x_max - x_margin, y_line, z_level])
    elif mode == 1:
        y_line = float(rng.choice(lanes_y) + rng.uniform(-180.0, 180.0))
        start_pos = np.array([x_max - x_margin, y_line, z_level])
        end_pos = np.array([x_min + x_margin, y_line, z_level])
    elif mode == 2:
        x_line = float(rng.choice(lanes_x) + rng.uniform(-180.0, 180.0))
        start_pos = np.array([x_line, y_min + y_margin, z_level])
        end_pos = np.array([x_line, y_max - y_margin, z_level])
    else:
        x_line = float(rng.choice(lanes_x) + rng.uniform(-180.0, 180.0))
        start_pos = np.array([x_line, y_max - y_margin, z_level])
        end_pos = np.array([x_line, y_min + y_margin, z_level])

    # A shared offset around the center keeps traffic concentrated in the
    # same region, which increases the chance of conflicts as the fleet grows.
    center = np.array([0.5 * x_span, 0.5 * y_span, z_level])
    lateral_offset = np.array([
        rng.uniform(-800.0, 800.0),
        rng.uniform(-800.0, 800.0),
        rng.uniform(-20.0, 20.0),
    ])

    mid_1 = start_pos * 0.67 + center * 0.33 + 0.5 * lateral_offset
    mid_2 = start_pos * 0.33 + center * 0.67 + 0.5 * lateral_offset

    path_length = (
        np.linalg.norm(mid_1 - start_pos)
        + np.linalg.norm(mid_2 - mid_1)
        + np.linalg.norm(end_pos - mid_2)
    )
    # Give each UAV a long enough mission so its swept volume overlaps in time
    # with later insertions, making the benchmark progressively harder.
    # The total duration is derived from the actual route length so each segment
    # gets a fair time allocation instead of using fixed percentage splits.
    mission_duration = max(900.0, 1.05 * path_length / cruise_speed)

    # The flight plan starts later and later as the benchmark advances.
    t_start = uav_index * DEFAULT_INSERTION_GAP_S + rng.uniform(0.0, 0.75)
    t_end = t_start + mission_duration

    fp = FlightPlan()
    fp.id = uav_index + 1
    fp.priority = 2 if uav_index % 23 == 0 else (1 if uav_index % 7 == 0 else 0)
    fp.radius = 5.0
    fp.max_var_lin_vel = UAV_MAX_SPEED
    fp.max_var_ang_vel = 1.0

    positions = [start_pos, mid_1, mid_2, end_pos]
    segment_distances = [
        np.linalg.norm(positions[i + 1] - positions[i])
        for i in range(len(positions) - 1)
    ]
    total_distance = sum(segment_distances)
    cumulative_time = t_start
    times = [t_start]
    for distance in segment_distances:
        if total_distance > 0:
            cumulative_time += mission_duration * (distance / total_distance)
        times.append(cumulative_time)
    times[-1] = t_end

    for idx, (pos, t) in enumerate(zip(positions, times)):
        if idx == 0:
            # Start with a velocity aligned to the first segment so the first
            # interpolation step is physically coherent.
            first_segment = positions[1] - positions[0]
            first_norm = np.linalg.norm(first_segment)
            vel = (first_segment / first_norm) * cruise_speed if first_norm > 1e-9 else np.zeros(3)
            label = "START"
        elif idx == len(positions) - 1:
            # End waypoint stops the aircraft.
            vel = np.zeros(3)
            label = "END"
        else:
            # Intermediate waypoints point toward the next segment so the
            # plan stays smooth and easy to interpret.
            direction = positions[idx + 1] - pos
            norm = np.linalg.norm(direction)
            vel = (direction / norm) * cruise_speed if norm > 1e-9 else np.zeros(3)
            label = f"WP_{idx}"

        fp.set_waypoint(
            Waypoint(
                label=label,
                t=float(t),
                pos=pos.tolist(),
                vel=vel.tolist(),
            )
        )

    fp.connect_waypoints(strict=False)
    return fp


class IncrementalRealtimeBenchmark:
    """Insert UAVs one by one and stop when the real-time budget is exceeded."""

    def __init__(self, prism: Tuple[Tuple[float, float], ...] = REALISTIC_PRISM, seed: int = 42, verbose: bool = True):
        self.prism = prism
        self.seed = seed
        self.verbose = verbose
        self.manager = CentralManager()
        self.records: List[Dict[str, object]] = []

    def _log(self, message: str) -> None:
        if self.verbose:
            print(message)

    def run(
        self,
        max_uavs: Optional[int] = DEFAULT_MAX_UAVS,
        time_budget_ms: float = DEFAULT_TIME_BUDGET_MS,
        interval: float = DEFAULT_INTERVAL_S,
    ) -> Dict[str, object]:
        """Run the incremental benchmark until the per-UAV budget is broken."""
        process = psutil.Process(os.getpid())
        mem_start_mb = process.memory_info().rss / (1024 * 1024)

        self._log("\n" + "█" * 96)
        self._log("█" + " " * 94 + "█")
        self._log("█" + "  INCREMENTAL UAV RESOLVER BENCHMARK".center(94) + "█")
        self._log("█" + "  20 km x 20 km x 90 m airspace, insert-one-by-one workload".center(94) + "█")
        self._log("█" + " " * 94 + "█")
        self._log("█" * 96)

        inserted = 0
        resolved = 0
        deadlocks = 0
        clear_insertions = 0
        first_budget_fail_uav: Optional[int] = None
        total_elapsed_ms = 0.0

        while max_uavs is None or inserted < max_uavs:
            # Build the next candidate flight plan and immediately push it
            # through the same public path that production should use.
            fp = _build_collinear_plan(inserted, prism=self.prism, seed=self.seed)
            uav_id = str(fp.id)

            # Measure the real cost of registration + detection + resolution
            # for a single incoming UAV.
            t0 = time.perf_counter()
            self.manager.register_uav(uav_id, fp, priority=fp.priority, interval=interval)
            result = self.manager.check_and_resolve(uav_id)
            t1 = time.perf_counter()

            elapsed_ms = (t1 - t0) * 1000.0
            total_elapsed_ms += elapsed_ms

            if result is None:
                clear_insertions += 1
                status = "CLEAR"
                strategy = "NONE"
                iterations = 0
            else:
                status = "RESOLVED" if result.success else "DEADLOCK"
                strategy = result.strategy_used
                iterations = result.iterations
                if result.success:
                    resolved += 1
                else:
                    deadlocks += 1

            record = {
                "uav_id": fp.id,
                "status": status,
                "strategy": strategy,
                "iterations": iterations,
                "elapsed_ms": elapsed_ms,
            }
            self.records.append(record)

            self._log(
                f"UAV {fp.id:04d} | {status:<8s} | strategy={strategy:<8s} | "
                f"iters={iterations:>3d} | time={elapsed_ms:8.2f} ms"
            )

            inserted += 1

            # If the resolver takes too long on one insertion, the system would
            # not be safe for real-time operation, so the benchmark stops.
            if elapsed_ms > time_budget_ms and first_budget_fail_uav is None:
                first_budget_fail_uav = fp.id
                self._log(
                    f"Budget exceeded at UAV {fp.id} ({elapsed_ms:.2f} ms > {time_budget_ms:.2f} ms)."
                )
                break

            # Stop early if the resolver reaches an unsolved conflict.
            if result is not None and not result.success:
                self._log(
                    f"Resolver reached DEADLOCK at UAV {fp.id}. Stopping benchmark."
                )
                break

        mem_end_mb = process.memory_info().rss / (1024 * 1024)
        mem_delta_mb = mem_end_mb - mem_start_mb

        elapsed_samples = [float(item["elapsed_ms"]) for item in self.records]
        # Basic statistics help compare the benchmark across runs and changes.
        average_ms = float(np.mean(elapsed_samples)) if elapsed_samples else 0.0
        median_ms = float(np.median(elapsed_samples)) if elapsed_samples else 0.0
        p95_ms = float(np.percentile(elapsed_samples, 95)) if elapsed_samples else 0.0
        worst_ms = float(np.max(elapsed_samples)) if elapsed_samples else 0.0

        summary = {
            "inserted": inserted,
            "resolved": resolved,
            "deadlocks": deadlocks,
            "clear_insertions": clear_insertions,
            "average_ms": average_ms,
            "median_ms": median_ms,
            "p95_ms": p95_ms,
            "worst_ms": worst_ms,
            "total_elapsed_ms": total_elapsed_ms,
            "memory_delta_mb": mem_delta_mb,
            "first_budget_fail_uav": first_budget_fail_uav,
            "time_budget_ms": time_budget_ms,
        }

        self._log("\n" + "=" * 96)
        self._log("SUMMARY")
        self._log("=" * 96)
        self._log(f"Inserted UAVs:           {inserted}")
        self._log(f"Clear insertions:        {clear_insertions}")
        self._log(f"Resolved conflicts:      {resolved}")
        self._log(f"Deadlocks:               {deadlocks}")
        self._log(f"Average per insertion:   {average_ms:.2f} ms")
        self._log(f"Median per insertion:    {median_ms:.2f} ms")
        self._log(f"95th percentile:         {p95_ms:.2f} ms")
        self._log(f"Worst case:              {worst_ms:.2f} ms")
        self._log(f"Total elapsed:           {total_elapsed_ms:.2f} ms")
        self._log(f"Memory delta:            {mem_delta_mb:.2f} MB")
        if first_budget_fail_uav is not None:
            self._log(f"Budget first exceeded at UAV: {first_budget_fail_uav}")
        self._log("=" * 96 + "\n")

        self.summary = summary
        return summary


if __name__ == "__main__":
    benchmark = IncrementalRealtimeBenchmark(verbose=True)
    benchmark.run(max_uavs=2000, time_budget_ms=DEFAULT_TIME_BUDGET_MS, interval=DEFAULT_INTERVAL_S)