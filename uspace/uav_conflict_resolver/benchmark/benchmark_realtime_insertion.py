"""
benchmark_realtime_insertion.py - Incremental real-time resolver benchmark
=========================================================================

PURPOSE:
    Simulate the resolver as it is intended to run in production:

    1. Start with an empty 4D airspace (x, y, z, t).
    2. Insert one new flight plan.
    3. Check for conflicts through CentralManager.
    4. Resolve them immediately.
     5. Repeat until the resolver reaches HOVER/FB2 failure, deadlock,
         or the requested number of UAVs has been inserted.

AIRSPACE:
    20 km x 20 km x 90 m volume.
    Between 30 m and 120 m altitude (volable band).

NOTES:
    The benchmark uses dense, corridor-like flight plans so that conflicts
    appear naturally as the fleet grows. This is closer to a live system than
    a pre-generated batch benchmark with all aircraft present from the start.

STATUS MEANINGS:
    CLEAR:
        The newly inserted UAV had no detected conflicts.
RESOLVED:
        A conflict was detected and the resolver found a safe new plan.
    HOVER:
        The resolver reached FB2 (hover). In this benchmark it counts as a
        failure and stops the run.
DEADLOCK:
        A conflict was detected but no safe resolution strategy succeeded.

FLIGHT PLAN CREATION:
    Each inserted UAV gets a new FlightPlan built by the generator in
    benchmark/flightplan_generator.py. The generator picks random start and
    end points, adds smooth intermediate waypoints, spreads the timestamps
    across the route, and then connects the waypoints into a full trajectory.
    Each new flight plan starts after the previous one by a small insertion
    gap, but the missions are long enough to overlap in time.
    These flightplans are about 1800 s long and have a cruise speed around
    15 m/s, so they fill the airspace.

TIMING BREAKDOWN:
    create:
        Time spent building the FlightPlan object before registration.
    insert:
        Time spent registering the plan in the CentralManager / R-Tree (indexing all its OBBs).
    detect:
        Time spent finding conflicts for the inserted UAV.
    resolve:
        Time spent running the conflict-resolution cascade.
    total:
        Sum of create + insert + detect + resolve for the insertion.
"""

from __future__ import annotations

import os
import sys
import time
import io
import contextlib
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import psutil
import matplotlib.pyplot as plt

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
from benchmark.flightplan_generator import generate_flight_plan


# ---------------------------------------------------------------------------
# Benchmark configuration
# ---------------------------------------------------------------------------
REALISTIC_PRISM: Tuple[Tuple[float, float], ...] = (
    (0.0, 20000.0),   # X: 20 km
    (0.0, 20000.0),   # Y: 20 km
    (30.0, 120.0),    # Z: volable band
)


# New UAVs are inserted with a time offset so the scenario grows gradually.
DEFAULT_INSERTION_GAP_S: float = 3.0
# R-Tree sampling interval used when the new plan is registered.
DEFAULT_INTERVAL_S: float = 0.5
# The realistic generator needs enough mission time to stay below the max speed
# cap even when it produces longer corridors.
DEFAULT_GENERATED_MISSION_DURATION_S: float = 1800.0
DEFAULT_GENERATED_SPEED_FRACTION: float = 0.90
DEFAULT_GENERATED_V_MAX_MARGIN: float = 0.01
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
    """Insert UAVs one by one and stop on HOVER/FB2 or DEADLOCK."""

    def __init__(self, prism: Tuple[Tuple[float, float], ...] = REALISTIC_PRISM, seed: int = 42, verbose: bool = True):
        self.prism = prism
        self.seed = seed
        self.verbose = verbose
        self.manager = CentralManager()
        self.records: List[Dict[str, object]] = []
        # Keep a snapshot of the original generated flight plans for
        # a before/after visualization at the end of the benchmark.
        self.original_plans: Dict[int, FlightPlan] = {}

    def _log(self, message: str) -> None:
        if self.verbose:
            print(message)

    def write_output_txt(self, output_text: str, output_path: Optional[Path] = None) -> Path:
        if output_path is None:
            output_path = Path(__file__).with_name("benchmark_realtime_insertion_output.txt")
        output_path.write_text(output_text.rstrip("\n") + "\n", encoding="utf-8")
        return output_path

    def run(
        self,
        max_uavs: Optional[int] = DEFAULT_MAX_UAVS,
        interval: float = DEFAULT_INTERVAL_S,
    ) -> Dict[str, object]:
        """Run the incremental benchmark until HOVER/FB2, DEADLOCK, or max_uavs."""
        process = psutil.Process(os.getpid())
        mem_start_mb = process.memory_info().rss / (1024 * 1024)

        self._log("\n" + "█" * 96)
        self._log("█" + " " * 94 + "█")
        self._log("█" + "  INCREMENTAL UAV RESOLVER BENCHMARK".center(94) + "█")
        self._log("█" + "  20 km x 20 km x 90 m airspace, insert-one-by-one workload".center(94) + "█")
        self._log("█" + " " * 94 + "█")
        self._log("█" * 96)
        self._log(f"Run config: max_uavs={max_uavs}, interval={interval:.3f}s, seed={self.seed}")

        inserted = 0
        resolved = 0
        deadlocks = 0
        hover_failures = 0
        clear_insertions = 0
        total_elapsed_ms = 0.0
        stop_reason = "max_uavs"

        while max_uavs is None or inserted < max_uavs:
            # Build the next candidate flight plan using the realistic
            # generator instead of the collinear benchmark helper.
            # Use a per-UAV derived seed so plans vary with the index.
            creation_t0 = time.perf_counter()
            seed_fp = (self.seed + (inserted * 1009)) if self.seed is not None else None
            rng_fp = np.random.default_rng(seed_fp)
            t_start = inserted * DEFAULT_INSERTION_GAP_S + rng_fp.uniform(0.0, 0.75)
            # Keep missions long so they overlap and stay comfortably below the
            # kinematic limit enforced by connect_waypoints().
            t_end = t_start + DEFAULT_GENERATED_MISSION_DURATION_S
            priority = 2 if inserted % 23 == 0 else (1 if inserted % 7 == 0 else 0)

            fp = None
            generation_error: Optional[Exception] = None
            for attempt in range(4):
                try:
                    fp = generate_flight_plan(
                        prism=self.prism,
                        t_start=float(t_start),
                        t_end=float(t_end),
                        speed_range=(5.0, float(UAV_MAX_SPEED) * DEFAULT_GENERATED_SPEED_FRACTION),
                        radius=5.0,
                        max_lin_vel=float(UAV_MAX_SPEED) * DEFAULT_GENERATED_SPEED_FRACTION,
                        max_ang_vel=1.0,
                        v_max=float(UAV_MAX_SPEED) + DEFAULT_GENERATED_V_MAX_MARGIN,
                        uav_id=inserted + 1,
                        priority=priority,
                        seed=int(seed_fp) if seed_fp is not None else None,
                        takeoff_landing=True,
                    )
                    break
                except ValueError as exc:
                    generation_error = exc
                    t_end = t_start + (DEFAULT_GENERATED_MISSION_DURATION_S * (1.35 ** (attempt + 1)))

            if fp is None:
                raise generation_error  # type: ignore[misc]

            creation_ms = (time.perf_counter() - creation_t0) * 1000.0
            uav_id = str(fp.id)

            # Snapshot original plan for later visualization
            try:
                self.original_plans[fp.id] = fp.copy()
            except Exception:
                # If copying fails, still continue the benchmark.
                self.original_plans[fp.id] = fp

            # Measure the real cost of registration + detection + resolution
            # for a single incoming UAV.
            insert_t0 = time.perf_counter()
            self.manager.register_uav(uav_id, fp, priority=fp.priority, interval=interval)
            insert_ms = (time.perf_counter() - insert_t0) * 1000.0

            detect_t0 = time.perf_counter()
            conflicts = self.manager._rtree_detector.detect_all_conflicts(uav_id)
            detect_ms = (time.perf_counter() - detect_t0) * 1000.0

            resolve_ms = 0.0
            phase_times: Dict[str, float] = {}
            result = None
            if conflicts:
                conflicts.sort(key=lambda c: c["time_range"][0])
                conflict = conflicts[0]

                uav_a_id = conflict["uav_a"]
                uav_b_id = conflict["uav_b"]
                priority_a = self.manager._priorities.get(uav_a_id, 0)
                priority_b = self.manager._priorities.get(uav_b_id, 0)

                if priority_a >= priority_b:
                    if priority_a == priority_b and uav_a_id > uav_b_id:
                        vip_id, pleb_id = uav_b_id, uav_a_id
                    else:
                        vip_id, pleb_id = uav_a_id, uav_b_id
                else:
                    vip_id, pleb_id = uav_b_id, uav_a_id

                fp_vip = self.manager._flight_plans[vip_id]
                fp_pleb = self.manager._flight_plans[pleb_id]

                resolve_t0 = time.perf_counter()
                result = self.manager._resolver.resolve(
                    conflict=conflict,
                    fp_pleb=fp_pleb,
                    fp_vip=fp_vip,
                    pleb_id=pleb_id,
                    vip_id=vip_id,
                )
                resolve_ms = (time.perf_counter() - resolve_t0) * 1000.0
                phase_times = dict(result.phase_times)

                if result.success:
                    if result.new_fp_pleb is not None:
                        self.manager.register_uav(
                            pleb_id,
                            result.new_fp_pleb,
                            priority=self.manager._priorities.get(pleb_id, 0),
                            interval=interval,
                        )
                    if result.new_fp_vip is not None:
                        self.manager.register_uav(
                            vip_id,
                            result.new_fp_vip,
                            priority=self.manager._priorities.get(vip_id, 0),
                            interval=interval,
                        )

            elapsed_ms = creation_ms + insert_ms + detect_ms + resolve_ms
            total_elapsed_ms += elapsed_ms

            if result is None:
                clear_insertions += 1
                status = "CLEAR"
                strategy = "NONE"
                iterations = 0
                phase_iterations = {}
            else:
                strategy = result.strategy_used
                iterations = result.iterations
                phase_iterations = dict(getattr(result, "phase_iterations", {}))
                if result.strategy_used == "FB2":
                    status = "HOVER"
                    hover_failures += 1
                elif result.success:
                    status = "RESOLVED"
                    resolved += 1
                else:
                    status = "DEADLOCK"
                    deadlocks += 1

            record = {
                "uav_id": fp.id,
                "status": status,
                "strategy": strategy,
                "iterations": iterations,
                "s1_iterations": int(phase_iterations.get("s1", 0)),
                "s2_iterations": int(phase_iterations.get("s2", 0)),
                "fb1_iterations": int(phase_iterations.get("fb1", 0)),
                "fb2_iterations": int(phase_iterations.get("fb2", 0)),
                "creation_ms": creation_ms,
                "insert_ms": insert_ms,
                "detect_ms": detect_ms,
                "resolve_ms": resolve_ms,
                "resolve_total_ms": phase_times.get("resolve_total_ms", resolve_ms),
                "shadow_build_ms": phase_times.get("shadow_build_ms", 0.0),
                "s1_ms": phase_times.get("s1_ms", 0.0),
                "sat_ms": phase_times.get("sat_ms", 0.0),
                "s2_ms": phase_times.get("s2_ms", 0.0),
                "fb1_ms": phase_times.get("fb1_ms", 0.0),
                "fb2_ms": phase_times.get("fb2_ms", 0.0),
                # Validation metrics (temporary-swap)
                "validation_method": phase_times.get("validation_method", "none"),
                "candidate_boxes": int(phase_times.get("candidate_boxes", 0)),
                "original_boxes": int(phase_times.get("original_boxes", 0)),
                "candidate_gen_ms": float(phase_times.get("candidate_gen_ms", 0.0)),
                "candidate_register_ms": float(phase_times.get("candidate_register_ms", phase_times.get("candidate_insert_ms", 0.0))),
                "remove_original_ms": float(phase_times.get("remove_original_ms", phase_times.get("rm_orig", 0.0))),
                "detect_validation_ms": float(phase_times.get("detect_ms", 0.0)),
                "restore_remove_ms": float(phase_times.get("restore_remove_ms", 0.0)),
                "restore_insert_ms": float(phase_times.get("restore_insert_ms", 0.0)),
                "elapsed_ms": elapsed_ms,
            }
            self.records.append(record)

            if phase_times:
                breakdown = (
                    f"shadow={phase_times.get('shadow_build_ms', 0.0):8.2f} ms | "
                    f"S1={phase_times.get('s1_ms', 0.0):8.2f} ms | "
                    f"SAT={phase_times.get('sat_ms', 0.0):8.2f} ms | "
                    f"S2={phase_times.get('s2_ms', 0.0):8.2f} ms | "
                    f"FB1={phase_times.get('fb1_ms', 0.0):8.2f} ms | "
                    f"FB2={phase_times.get('fb2_ms', 0.0):8.2f} ms"
                )
                iterations_brief = (
                    f"iters_total={iterations:>3d} | "
                    f"S1={phase_iterations.get('s1', 0):>3d} | "
                    f"S2={phase_iterations.get('s2', 0):>3d} | "
                    f"FB1={phase_iterations.get('fb1', 0):>3d} | "
                    f"FB2={phase_iterations.get('fb2', 0):>3d}"
                )
                # Validation brief
                vmethod = phase_times.get('validation_method', 'none')
                vboxes_c = int(phase_times.get('candidate_boxes', 0))
                vboxes_o = int(phase_times.get('original_boxes', 0))
                vdetect = float(phase_times.get('detect_ms', 0.0))
                vgen = float(phase_times.get('candidate_gen_ms', 0.0))
                vins = float(phase_times.get('candidate_insert_ms', phase_times.get('candidate_register_ms', 0.0)))
                validation_brief = f"val_method={vmethod} boxes_c={vboxes_c} boxes_o={vboxes_o} gen={vgen:.1f}ms ins={vins:.1f}ms det={vdetect:.1f}ms"
            else:
                breakdown = "shadow=    0.00 ms | S1=    0.00 ms | SAT=    0.00 ms | S2=    0.00 ms | FB1=    0.00 ms | FB2=    0.00 ms"
                validation_brief = "val_method=none"
                iterations_brief = f"iters_total={iterations:>3d} | S1=  0 | S2=  0 | FB1=  0 | FB2=  0"

            self._log(
                f"UAV {fp.id:04d} | {status:<8s} | strategy={strategy:<8s} | "
                f"{iterations_brief} | create={creation_ms:8.2f} ms | "
                f"insert={insert_ms:8.2f} ms | detect={detect_ms:8.2f} ms | "
                f"resolve={resolve_ms:8.2f} ms | "
                f"total={elapsed_ms:8.2f} ms"
            )
            if phase_times:
                self._log(f"    resolve breakdown: {breakdown}")
                self._log(f"    validation: {validation_brief}")

            inserted += 1

            # Stop early if the resolver reaches HOVER/FB2, which counts as
            # a failure for this benchmark, or if it reaches DEADLOCK.
            if result is not None and result.strategy_used == "FB2":
                self._log(f"Resolver reached HOVER/FB2 at UAV {fp.id}. Stopping benchmark.")
                stop_reason = "FB2"
                break

            if result is not None and not result.success:
                self._log(f"Resolver reached DEADLOCK at UAV {fp.id}. Stopping benchmark.")
                stop_reason = "DEADLOCK"
                break

        mem_end_mb = process.memory_info().rss / (1024 * 1024)
        mem_delta_mb = mem_end_mb - mem_start_mb

        elapsed_samples = [float(item["elapsed_ms"]) for item in self.records]
        creation_samples = [float(item["creation_ms"]) for item in self.records]
        insert_samples = [float(item["insert_ms"]) for item in self.records]
        detect_samples = [float(item["detect_ms"]) for item in self.records]
        resolve_samples = [float(item["resolve_ms"]) for item in self.records]
        resolve_total_samples = [float(item["resolve_total_ms"]) for item in self.records]
        resolve_conflict_samples = [value for value in resolve_total_samples if value > 0.0]
        shadow_samples = [float(item["shadow_build_ms"]) for item in self.records]
        s1_samples = [float(item["s1_ms"]) for item in self.records]
        sat_samples = [float(item["sat_ms"]) for item in self.records]
        s2_samples = [float(item["s2_ms"]) for item in self.records]
        fb1_samples = [float(item["fb1_ms"]) for item in self.records]
        fb2_samples = [float(item["fb2_ms"]) for item in self.records]
        s1_iter_samples = [float(item["s1_iterations"]) for item in self.records]
        s2_iter_samples = [float(item["s2_iterations"]) for item in self.records]
        fb1_iter_samples = [float(item["fb1_iterations"]) for item in self.records]
        fb2_iter_samples = [float(item["fb2_iterations"]) for item in self.records]
        # Basic statistics help compare the benchmark across runs and changes.
        average_ms = float(np.mean(elapsed_samples)) if elapsed_samples else 0.0
        median_ms = float(np.median(elapsed_samples)) if elapsed_samples else 0.0
        p95_ms = float(np.percentile(elapsed_samples, 95)) if elapsed_samples else 0.0
        worst_ms = float(np.max(elapsed_samples)) if elapsed_samples else 0.0
        avg_creation_ms = float(np.mean(creation_samples)) if creation_samples else 0.0
        avg_insert_ms = float(np.mean(insert_samples)) if insert_samples else 0.0
        avg_detect_ms = float(np.mean(detect_samples)) if detect_samples else 0.0
        avg_resolve_ms = float(np.mean(resolve_samples)) if resolve_samples else 0.0
        avg_resolve_total_ms = float(np.mean(resolve_total_samples)) if resolve_total_samples else 0.0
        avg_resolve_conflict_ms = float(np.mean(resolve_conflict_samples)) if resolve_conflict_samples else 0.0
        avg_shadow_ms = float(np.mean(shadow_samples)) if shadow_samples else 0.0
        avg_s1_ms = float(np.mean(s1_samples)) if s1_samples else 0.0
        avg_sat_ms = float(np.mean(sat_samples)) if sat_samples else 0.0
        avg_s2_ms = float(np.mean(s2_samples)) if s2_samples else 0.0
        avg_fb1_ms = float(np.mean(fb1_samples)) if fb1_samples else 0.0
        avg_fb2_ms = float(np.mean(fb2_samples)) if fb2_samples else 0.0
        avg_s1_iters = float(np.mean(s1_iter_samples)) if s1_iter_samples else 0.0
        avg_s2_iters = float(np.mean(s2_iter_samples)) if s2_iter_samples else 0.0
        avg_fb1_iters = float(np.mean(fb1_iter_samples)) if fb1_iter_samples else 0.0
        avg_fb2_iters = float(np.mean(fb2_iter_samples)) if fb2_iter_samples else 0.0

        summary = {
            "inserted": inserted,
            "resolved": resolved,
            "deadlocks": deadlocks,
            "hover_failures": hover_failures,
            "clear_insertions": clear_insertions,
            "average_ms": average_ms,
            "median_ms": median_ms,
            "p95_ms": p95_ms,
            "worst_ms": worst_ms,
            "avg_creation_ms": avg_creation_ms,
            "avg_insert_ms": avg_insert_ms,
            "avg_detect_ms": avg_detect_ms,
            "avg_resolve_ms": avg_resolve_ms,
            "avg_resolve_total_ms": avg_resolve_total_ms,
            "avg_resolve_conflict_ms": avg_resolve_conflict_ms,
            "avg_shadow_ms": avg_shadow_ms,
            "avg_s1_ms": avg_s1_ms,
            "avg_sat_ms": avg_sat_ms,
            "avg_s2_ms": avg_s2_ms,
            "avg_fb1_ms": avg_fb1_ms,
            "avg_fb2_ms": avg_fb2_ms,
            "avg_s1_iters": avg_s1_iters,
            "avg_s2_iters": avg_s2_iters,
            "avg_fb1_iters": avg_fb1_iters,
            "avg_fb2_iters": avg_fb2_iters,
            "total_elapsed_ms": total_elapsed_ms,
            "memory_delta_mb": mem_delta_mb,
        }

        self._log("\n" + "=" * 96)
        self._log("SUMMARY")
        self._log("=" * 96)
        self._log(f"Inserted UAVs:           {inserted}")
        self._log(f"Clear insertions:        {clear_insertions}")
        self._log(f"Resolved conflicts:      {resolved}")
        self._log(f"Hover failures:          {hover_failures}")
        self._log(f"Deadlocks:               {deadlocks}")
        self._log(f"Average per insertion:   {average_ms:.2f} ms")
        self._log(f"Median per insertion:    {median_ms:.2f} ms")
        self._log(f"95th percentile:         {p95_ms:.2f} ms")
        self._log(f"Worst case:              {worst_ms:.2f} ms")
        self._log(f"Avg creation time:       {avg_creation_ms:.2f} ms")
        self._log(f"Avg insertion time:      {avg_insert_ms:.2f} ms")
        self._log(f"Avg detection time:      {avg_detect_ms:.2f} ms")
        self._log(f"Avg resolution time (all insertions): {avg_resolve_ms:.2f} ms")
        self._log(f"Avg resolve total (all insertions):    {avg_resolve_total_ms:.2f} ms")
        self._log(f"Avg resolve total (conflicts only):    {avg_resolve_conflict_ms:.2f} ms")
        self._log(f"Avg shadow build:        {avg_shadow_ms:.2f} ms")
        self._log(f"Avg S1 time:             {avg_s1_ms:.2f} ms")
        self._log(f"Avg S1 iterations:       {avg_s1_iters:.2f}")
        self._log(f"Avg SAT time:            {avg_sat_ms:.2f} ms")
        self._log(f"Avg S2 time:             {avg_s2_ms:.2f} ms")
        self._log(f"Avg S2 iterations:       {avg_s2_iters:.2f}")
        self._log(f"Avg FB1 time:            {avg_fb1_ms:.2f} ms")
        self._log(f"Avg FB1 iterations:      {avg_fb1_iters:.2f}")
        self._log(f"Avg FB2 time:            {avg_fb2_ms:.2f} ms")
        self._log(f"Avg FB2 iterations:      {avg_fb2_iters:.2f}")
        self._log(f"Total elapsed:           {total_elapsed_ms:.2f} ms")
        self._log(f"Memory delta:            {mem_delta_mb:.2f} MB")
        self._log(f"Run ended because:       {stop_reason}")
        self._log(f"Inserted / target:       {inserted} / {max_uavs if max_uavs is not None else 'unbounded'}")
        self._log("=" * 96 + "\n")

        # --------------- 3D Visualization (Before / After) ---------------
        try:
            BG = "#0f1117"
            fig = plt.figure(figsize=(14, 7), facecolor=BG)
            fig.canvas.manager.set_window_title("UAV Conflict Resolver — Benchmark Routes")
            fig.text(0.5, 0.95, "Benchmark: Flight Routes — Before and After Resolution",
                     ha="center", va="center", fontsize=14, fontweight="bold", color="white")

            # Prepare plan dictionaries keyed by integer id
            before_plans = {pid: fp for pid, fp in self.original_plans.items()}
            after_plans = {pid: self.manager.get_flight_plan(str(pid)) for pid in before_plans.keys()}

            colors_list = [
                "#2ecc71", "#e74c3c", "#3498db", "#f1c40f", "#9b59b6",
                "#1abc9c", "#e67e22", "#34495e", "#7f8c8d", "#16a085"
            ]

            for col, title, plan_dict in [
                (0, "BEFORE (Original Generated Fleet)", before_plans),
                (1, "AFTER (Committed Plans in CentralManager)", after_plans),
            ]:
                ax = fig.add_subplot(1, 2, col + 1, projection="3d")
                ax.set_facecolor(BG)
                ax.set_title(title, color="white", fontsize=11, pad=10)

                for i, (pid, fp) in enumerate(sorted(plan_dict.items())):
                    if fp is None:
                        continue

                    try:
                        trace = fp.trace(0.1)
                    except Exception:
                        # If trace sampling fails, skip this plan
                        continue

                    color = colors_list[i % len(colors_list)]
                    z_offset = i * 0.05
                    z_plot = trace[:, 3] + z_offset
                    ax.plot(trace[:, 1], trace[:, 2], z_plot,
                            color=color, linewidth=1.2, linestyle='-', alpha=0.9)
                    ax.scatter(trace[0, 1], trace[0, 2], trace[0, 3] + z_offset, color=color, s=18, marker="o", zorder=5)
                    ax.scatter(trace[-1, 1], trace[-1, 2], trace[-1, 3] + z_offset, color=color, s=18, marker="^", zorder=5)

                ax.set_xlabel("X (m)", color="white", labelpad=4)
                ax.set_ylabel("Y (m)", color="white", labelpad=4)
                ax.set_zlabel("Z (m)", color="white", labelpad=4)
                ax.tick_params(colors="white")
                for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
                    try:
                        pane.fill = False
                        pane.set_edgecolor("#2a2d3a")
                    except Exception:
                        pass

            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            plt.show()
        except Exception:
            # Visualization must not break the benchmark result flow.
            pass

        self.summary = summary
        return summary


if __name__ == "__main__":
    benchmark = IncrementalRealtimeBenchmark(verbose=True)
    capture_buffer = io.StringIO()

    class _Tee:
        def __init__(self, *streams):
            self.streams = streams

        def write(self, text):
            for stream in self.streams:
                stream.write(text)
            return len(text)

        def flush(self):
            for stream in self.streams:
                stream.flush()

    with contextlib.redirect_stdout(_Tee(sys.stdout, capture_buffer)):
        try:
            benchmark.run(max_uavs=2000, interval=DEFAULT_INTERVAL_S)
        finally:
            output_file = benchmark.write_output_txt(capture_buffer.getvalue())
            print(f"Benchmark output written to: {output_file}")