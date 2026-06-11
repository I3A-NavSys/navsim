"""
benchmark_realtime_sequential_conflicts.py — Incremental inserter that forces
sequential conflicts between successive UAV routes.
==============================================================================

PURPOSE:
    Insert UAVs one by one so that every new plan is geometrically routed to
    cross the midpoint of the previously inserted plan at approximately the
    same time. This guarantees a conflict at virtually every insertion and
    creates a progressively harder workload as the airspace fills up.

ROUTES:
    Each plan has N_WAYPOINTS waypoints (including START and END), all at a
    fixed cruise altitude. The route is essentially straight across the
    airspace (edge to edge, ~15–18 km) with very small lateral jitter at
    intermediate waypoints to avoid perfectly collinear plans.
    connect_waypoints(strict=True) is always used.

STOP CONDITION:
    The benchmark runs until max_uavs is reached. FB2 (Hover) and DEADLOCK
    outcomes are counted and logged but do NOT stop the run.

SUMMARY:
    Total UAVs inserted, clear insertions, resolved, FB2 count, DEADLOCK
    count, timing statistics, and memory delta.
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

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from core.config import UAV_MAX_SPEED
from central_manager import CentralManager


# ---------------------------------------------------------------------------
# Airspace
# ---------------------------------------------------------------------------
VERTIPORT_PRISM: Tuple[Tuple[float, float], ...] = (
    (0.0, 20_000.0),   # X: 20 km
    (0.0, 20_000.0),   # Y: 20 km
    (30.0, 120.0),     # Z: UTM low-altitude band
)

CRUISE_LEVELS = [55.0, 70.0, 90.0]  # available altitude layers (m)
N_WAYPOINTS       = 6      # waypoints per plan (START + intermediates + END)
INSERTION_GAP_S   = 3.0    # seconds between successive t_start values
DEFAULT_INTERVAL_S = 0.5   # OBB sampling interval for the R-Tree

# Toggle to enable/disable 3D visualization at the end of the benchmark
ENABLE_VISUALIZATION: bool = True


# ---------------------------------------------------------------------------
# Flight-plan generator
# ---------------------------------------------------------------------------

def _build_straight_plan(
    uav_index: int,
    prism: Tuple[Tuple[float, float], ...] = VERTIPORT_PRISM,
    seed: int = 42,
    crossing_point: Optional[Tuple[float, float]] = None,
    crossing_time: Optional[float] = None,
    n_waypoints: int = N_WAYPOINTS,
) -> FlightPlan:
    """Build a long, straight-ish horizontal flight plan.

    All waypoints share the same cruise altitude (z_cruise).
    Intermediate waypoints have a tiny perpendicular jitter (≤ 200 m) to
    prevent perfectly collinear trajectories while keeping routes visually
    straight.

    If *crossing_point* and *crossing_time* are given the plan is centred on
    that XY point so it passes through it at approximately that time,
    guaranteeing a spatiotemporal conflict with the previous UAV.
    """
    rng = np.random.default_rng(seed + uav_index * 881)
    x_rng, y_rng, z_rng = prism

    z_cruise    = float(rng.choice(CRUISE_LEVELS))
    cruise_speed = float(rng.uniform(10.0, min(15.0, UAV_MAX_SPEED)))
    margin       = 500.0

    # --- Route geometry (edge-to-edge, ~15–18 km long) ---
    if crossing_point is not None:
        cx, cy   = crossing_point
        bearing  = rng.uniform(0.0, 2.0 * np.pi)
        half_dist = rng.uniform(5_000.0, 9_000.0)
        x_start  = cx - half_dist * np.cos(bearing)
        y_start  = cy - half_dist * np.sin(bearing)
        x_end    = cx + half_dist * np.cos(bearing)
        y_end    = cy + half_dist * np.sin(bearing)
    else:
        edge = uav_index % 4
        if edge == 0:        # west → east
            x_start = x_rng[0] + rng.uniform(margin, margin * 2)
            y_start = rng.uniform(y_rng[0] + margin, y_rng[1] - margin)
            x_end   = x_rng[1] - rng.uniform(margin, margin * 2)
            y_end   = rng.uniform(y_rng[0] + margin, y_rng[1] - margin)
        elif edge == 1:      # east → west
            x_start = x_rng[1] - rng.uniform(margin, margin * 2)
            y_start = rng.uniform(y_rng[0] + margin, y_rng[1] - margin)
            x_end   = x_rng[0] + rng.uniform(margin, margin * 2)
            y_end   = rng.uniform(y_rng[0] + margin, y_rng[1] - margin)
        elif edge == 2:      # south → north
            x_start = rng.uniform(x_rng[0] + margin, x_rng[1] - margin)
            y_start = y_rng[0] + rng.uniform(margin, margin * 2)
            x_end   = rng.uniform(x_rng[0] + margin, x_rng[1] - margin)
            y_end   = y_rng[1] - rng.uniform(margin, margin * 2)
        else:                # north → south
            x_start = rng.uniform(x_rng[0] + margin, x_rng[1] - margin)
            y_start = y_rng[1] - rng.uniform(margin, margin * 2)
            x_end   = rng.uniform(x_rng[0] + margin, x_rng[1] - margin)
            y_end   = y_rng[0] + rng.uniform(margin, margin * 2)

    # Clamp inside airspace
    x_start = float(np.clip(x_start, x_rng[0] + 10, x_rng[1] - 10))
    y_start = float(np.clip(y_start, y_rng[0] + 10, y_rng[1] - 10))
    x_end   = float(np.clip(x_end,   x_rng[0] + 10, x_rng[1] - 10))
    y_end   = float(np.clip(y_end,   y_rng[0] + 10, y_rng[1] - 10))

    # --- Waypoint positions ---
    # Evenly spaced along the straight line; intermediate WPs have small
    # perpendicular jitter so the quintic interpolation is non-degenerate.
    positions: List[np.ndarray] = []
    dx = x_end - x_start
    dy = y_end - y_start
    length = np.hypot(dx, dy)
    # unit perpendicular vector
    perp_x = -dy / length if length > 1e-6 else 0.0
    perp_y =  dx / length if length > 1e-6 else 0.0

    for i in range(n_waypoints):
        alpha = i / (n_waypoints - 1)
        px = x_start + alpha * dx
        py = y_start + alpha * dy
        if 0 < i < n_waypoints - 1:
            jitter = rng.uniform(-200.0, 200.0)
            px += jitter * perp_x
            py += jitter * perp_y
        px = float(np.clip(px, x_rng[0] + 10, x_rng[1] - 10))
        py = float(np.clip(py, y_rng[0] + 10, y_rng[1] - 10))
        positions.append(np.array([px, py, z_cruise]))

    # --- Segment distances and total ---
    seg_dists = [
        float(np.linalg.norm(positions[i + 1] - positions[i]))
        for i in range(n_waypoints - 1)
    ]
    total_dist = sum(seg_dists)
    mission_duration = max(800.0, total_dist / cruise_speed)

    # --- Timing ---
    mid_idx = n_waypoints // 2
    if crossing_time is not None:
        dist_to_mid = sum(seg_dists[:mid_idx])
        frac_to_mid = dist_to_mid / total_dist if total_dist > 1e-6 else 0.5
        t_start = crossing_time - frac_to_mid * mission_duration
        t_start = max(0.0, t_start) + rng.uniform(-1.0, 1.0)
    else:
        t_start = uav_index * INSERTION_GAP_S + rng.uniform(0.0, 0.75)

    # Distribute timestamps proportionally to segment distances
    cumulative = [0.0]
    for d in seg_dists:
        cumulative.append(cumulative[-1] + d)
    times = [t_start + (c / total_dist) * mission_duration for c in cumulative]

    # --- Build FlightPlan ---
    fp = FlightPlan()
    fp.id            = uav_index + 1
    fp.priority      = 2 if uav_index % 23 == 0 else (1 if uav_index % 7 == 0 else 0)
    fp.radius        = 5.0
    fp.max_var_lin_vel = UAV_MAX_SPEED
    fp.max_var_ang_vel = 1.0

    for i, (pos, t) in enumerate(zip(positions, times)):
        if i == 0:
            label = "START"
            nxt   = positions[1] - positions[0]
            norm  = np.linalg.norm(nxt)
            vel   = (nxt / norm * cruise_speed).tolist() if norm > 1e-9 else [0.0, 0.0, 0.0]
        elif i == n_waypoints - 1:
            label = "END"
            vel   = [0.0, 0.0, 0.0]
        else:
            label = f"WP_{i}"
            nxt   = positions[i + 1] - positions[i]
            norm  = np.linalg.norm(nxt)
            vel   = (nxt / norm * cruise_speed).tolist() if norm > 1e-9 else [0.0, 0.0, 0.0]
        fp.set_waypoint(Waypoint(label, float(t), pos.tolist(), vel))

    fp.connect_waypoints(strict=True)

    # Store midpoint metadata for the next plan's crossing constraint
    fp._benchmark_mid      = (float(positions[mid_idx][0]), float(positions[mid_idx][1]))
    fp._benchmark_mid_time = float(times[mid_idx])
    return fp


# ---------------------------------------------------------------------------
# Benchmark runner
# ---------------------------------------------------------------------------

class SequentialConflictRealtimeBenchmark:
    """Insert UAVs sequentially, each guaranteed to conflict with the previous one.

    FB2 and DEADLOCK are counted but never stop the run.
    The benchmark stops when max_uavs is reached.
    """

    def __init__(
        self,
        prism: Tuple[Tuple[float, float], ...] = VERTIPORT_PRISM,
        seed: int = 42,
        verbose: bool = True,
    ) -> None:
        self.prism         = prism
        self.seed          = seed
        self.verbose       = verbose
        self.manager       = CentralManager()
        self.records: List[Dict[str, object]] = []
        self.original_plans: Dict[int, FlightPlan] = {}

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(msg)

    def run(
        self,
        max_uavs: int = 200,
        interval: float = DEFAULT_INTERVAL_S,
    ) -> Dict[str, object]:
        process       = psutil.Process(os.getpid())
        mem_start_mb  = process.memory_info().rss / (1024 * 1024)

        prev_mid:      Optional[Tuple[float, float]] = None
        prev_mid_time: Optional[float]               = None

        inserted        = 0
        clear_insertions = 0
        resolved        = 0
        deadlocks       = 0
        anchor_deadlocks = 0
        hover_deadlocks  = 0
        hover_failures  = 0
        total_elapsed_ms = 0.0

        self._log("\n" + "█" * 96)
        self._log("█" + " " * 94 + "█")
        self._log("█" + "  SEQUENTIAL CONFLICT REAL-TIME BENCHMARK".center(94) + "█")
        self._log("█" + "  Forced conflict at every insertion — 20 km × 20 km × 30–120 m".center(94) + "█")
        self._log("█" + " " * 94 + "█")
        self._log("█" * 96)
        self._log(f"Run config: max_uavs={max_uavs}, interval={interval:.3f}s, seed={self.seed}")

        while inserted < max_uavs:
            creation_t0 = time.perf_counter()
            rng_j = np.random.default_rng(self.seed + inserted)

            if prev_mid is None:
                fp = _build_straight_plan(inserted, prism=self.prism, seed=self.seed)
            else:
                jitter_xy = (rng_j.uniform(-80.0, 80.0), rng_j.uniform(-80.0, 80.0))
                cross_pt  = (prev_mid[0] + jitter_xy[0], prev_mid[1] + jitter_xy[1])
                cross_t   = prev_mid_time + rng_j.uniform(-3.0, 3.0)
                fp = _build_straight_plan(
                    inserted,
                    prism=self.prism,
                    seed=self.seed,
                    crossing_point=cross_pt,
                    crossing_time=cross_t,
                )
            creation_ms = (time.perf_counter() - creation_t0) * 1000.0

            try:
                self.original_plans[fp.id] = fp.copy()
            except Exception:
                self.original_plans[fp.id] = fp

            uav_id = str(fp.id)

            insert_t0 = time.perf_counter()
            self.manager.register_uav(uav_id, fp, priority=fp.priority, interval=interval)
            insert_ms = (time.perf_counter() - insert_t0) * 1000.0

            detect_t0 = time.perf_counter()
            conflicts = self.manager._rtree_detector.detect_all_conflicts(uav_id)
            detect_ms = (time.perf_counter() - detect_t0) * 1000.0

            resolve_ms = 0.0
            phase_times: Dict[str, float] = {}
            last_pass_ms = 0.0
            pass_history: List[Dict[str, object]] = []
            result     = None
            status     = "CLEAR"
            strategy   = "NONE"
            iterations = 0
            phase_iterations: Dict[str, int] = {}

            if conflicts:
                resolve_t0 = time.perf_counter()
                result = self.manager.resolve_until_clear(uav_id, interval=interval)
                resolve_ms = (time.perf_counter() - resolve_t0) * 1000.0

                if result is not None:
                    strategy = result.strategy_used
                    iterations = int(result.iterations)
                    phase_iterations = dict(getattr(result, "phase_iterations", {}))
                    phase_times = dict(getattr(result, "phase_times", {}))
                    last_pass_ms = float(phase_times.get("resolve_total_ms", 0.0))
                    pass_history = list(getattr(result, "pass_history", []))

                if result is not None and result.strategy_used == "FB2":
                    status = "HOVER"
                    hover_failures += 1
                elif result is not None and result.success:
                    status = "RESOLVED"
                    resolved += 1
                elif result is not None and not result.success:
                    status = "DEADLOCK"
                    deadlocks += 1
                    deadlock_type = str(getattr(result, "deadlock_type", "unknown"))
                    if deadlock_type in {"anchor_before_conflict", "anchor_after_finish"}:
                        anchor_deadlocks += 1
                    elif deadlock_type == "hover_timeout":
                        hover_deadlocks += 1
                    self.manager.remove_uav(uav_id)
            else:
                clear_insertions += 1

            elapsed_ms       = creation_ms + insert_ms + detect_ms + resolve_ms
            total_elapsed_ms += elapsed_ms

            resolve_phase_sum_ms = float(
                phase_times.get("s1_ms", 0.0)
                + phase_times.get("sat_ms", 0.0)
                + phase_times.get("s2_ms", 0.0)
                + phase_times.get("fb1_ms", 0.0)
                + phase_times.get("fb2_ms", 0.0)
            )
            validation_gen_ms = float(phase_times.get("candidate_gen_ms", 0.0))
            validation_detect_ms = float(phase_times.get("detect_ms", 0.0))
            validation_total_ms = float(validation_gen_ms + validation_detect_ms)

            self.records.append({
                "uav_id":      fp.id,
                "status":      status,
                "strategy":    strategy,
                "conflicts":   len(conflicts),
                "iterations":  iterations,
                "s1_iterations": int(phase_iterations.get("s1", 0)),
                "s2_iterations": int(phase_iterations.get("s2", 0)),
                "fb1_iterations": int(phase_iterations.get("fb1", 0)),
                "fb2_iterations": int(phase_iterations.get("fb2", 0)),
                "creation_ms": creation_ms,
                "insert_ms":   insert_ms,
                "detect_ms":   detect_ms,
                "resolve_ms":  resolve_ms,
                "resolve_loop_ms": resolve_ms,
                "resolve_last_pass_ms": last_pass_ms,
                "resolve_pass_count": len(pass_history),
                "resolve_pass_history": pass_history,
                "resolve_total_ms": float(phase_times.get("resolve_total_ms", resolve_ms)),
                "shadow_build_ms": float(phase_times.get("shadow_build_ms", 0.0)),
                "s1_ms": float(phase_times.get("s1_ms", 0.0)),
                "sat_ms": float(phase_times.get("sat_ms", 0.0)),
                "s2_ms": float(phase_times.get("s2_ms", 0.0)),
                "fb1_ms": float(phase_times.get("fb1_ms", 0.0)),
                "fb2_ms": float(phase_times.get("fb2_ms", 0.0)),
                "resolve_phase_sum_ms": resolve_phase_sum_ms,
                "validation_method": phase_times.get("validation_method", "none"),
                "candidate_boxes": int(phase_times.get("candidate_boxes", 0)),
                "original_boxes": int(phase_times.get("original_boxes", 0)),
                "candidate_gen_ms": validation_gen_ms,
                "candidate_register_ms": float(phase_times.get("candidate_register_ms", phase_times.get("candidate_insert_ms", 0.0))),
                "remove_original_ms": float(phase_times.get("remove_original_ms", phase_times.get("rm_orig", 0.0))),
                "detect_validation_ms": validation_detect_ms,
                "restore_remove_ms": float(phase_times.get("restore_remove_ms", 0.0)),
                "restore_insert_ms": float(phase_times.get("restore_insert_ms", 0.0)),
                "validation_total_ms": validation_total_ms,
                "elapsed_ms":  elapsed_ms,
            })

            if phase_times:
                breakdown = (
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
                validation_brief = (
                    f"val_method={phase_times.get('validation_method', 'none')} "
                    f"boxes_c={int(phase_times.get('candidate_boxes', 0))} "
                    f"boxes_o={int(phase_times.get('original_boxes', 0))} "
                    f"gen={validation_gen_ms:.1f}ms "
                    f"ins={float(phase_times.get('candidate_insert_ms', phase_times.get('candidate_register_ms', 0.0))):.1f}ms "
                    f"det={validation_detect_ms:.1f}ms"
                )
            else:
                breakdown = "S1=    0.00 ms | SAT=    0.00 ms | S2=    0.00 ms | FB1=    0.00 ms | FB2=    0.00 ms"
                iterations_brief = "iters_total=  0 | S1=  0 | S2=  0 | FB1=  0 | FB2=  0"
                validation_brief = "val_method=none"

            self._log(
                f"UAV {fp.id:04d} | {status:<8s} | strategy={strategy:<8s} | "
                f"{iterations_brief} | conflicts={len(conflicts):3d} | "
                f"create={creation_ms:8.2f} ms | insert={insert_ms:8.2f} ms | "
                f"detect={detect_ms:8.2f} ms | resolve_loop={resolve_ms:8.2f} ms | "
                f"last_pass={last_pass_ms:8.2f} ms | total={elapsed_ms:8.2f} ms"
            )
            if phase_times:
                self._log(
                    f"    resolve items: loop={resolve_ms:8.2f} ms | "
                    f"last_pass={last_pass_ms:8.2f} ms | phase_sum={resolve_phase_sum_ms:8.2f} ms"
                )
                self._log(f"    resolve breakdown (last pass): {breakdown}")
                self._log(
                    f"    validation items: total={validation_total_ms:8.2f} ms | {validation_brief}"
                )
            if pass_history:
                self._log(f"    passes: {len(pass_history)}")
                for pass_info in pass_history:
                    pass_times = dict(pass_info.get("phase_times", {}))
                    pass_breakdown = (
                        f"S1={pass_times.get('s1_ms', 0.0):7.2f} | "
                        f"SAT={pass_times.get('sat_ms', 0.0):7.2f} | "
                        f"S2={pass_times.get('s2_ms', 0.0):7.2f} | "
                        f"FB1={pass_times.get('fb1_ms', 0.0):7.2f} | "
                        f"FB2={pass_times.get('fb2_ms', 0.0):7.2f}"
                    )
                    pass_iterations = dict(pass_info.get("phase_iterations", {}))
                    self._log(
                        f"      pass {int(pass_info.get('pass_index', 0)):02d} | "
                        f"conf_before={int(pass_info.get('conflicts_before', 0)):>3d} | "
                        f"conf_after={int(pass_info.get('remaining_conflicts_after', 0)):>3d} | "
                        f"strategy={str(pass_info.get('strategy_used', 'NONE')):<8s} | "
                        f"iters={int(pass_info.get('iterations', 0)):>3d} | "
                        f"total={float(pass_info.get('resolve_total_ms', 0.0)):8.2f} ms | "
                        f"phase_sum={float(pass_info.get('resolve_phase_sum_ms', 0.0)):8.2f} ms | "
                        f"validation={float(pass_info.get('validation_total_ms', 0.0)):8.2f} ms"
                    )
                    self._log(
                        f"        phases: {pass_breakdown} | "
                        f"S1i={int(pass_iterations.get('s1', 0)):>3d} | "
                        f"S2i={int(pass_iterations.get('s2', 0)):>3d} | "
                        f"FB1i={int(pass_iterations.get('fb1', 0)):>3d} | "
                        f"FB2i={int(pass_iterations.get('fb2', 0)):>3d}"
                    )

            if status == "HOVER":
                self._log(f"Resolver reached HOVER/FB2 at UAV {fp.id}. Counting it and continuing.")
            if status == "DEADLOCK":
                self._log(f"Resolver reached DEADLOCK at UAV {fp.id}. The new route is rejected and removed.")
                self._log(f"    deadlock_type={str(getattr(result, 'deadlock_type', 'unknown'))}")

            # Update crossing reference for the next plan
            prev_mid      = getattr(fp, "_benchmark_mid", None)
            prev_mid_time = getattr(fp, "_benchmark_mid_time", None)
            inserted += 1

        # -------------------------------------------------------------------
        # Summary
        # -------------------------------------------------------------------
        mem_end_mb   = process.memory_info().rss / (1024 * 1024)
        mem_delta_mb = mem_end_mb - mem_start_mb

        elapsed_samples = [float(item["elapsed_ms"]) for item in self.records]
        creation_samples = [float(item["creation_ms"]) for item in self.records]
        insert_samples = [float(item["insert_ms"]) for item in self.records]
        detect_samples = [float(item["detect_ms"]) for item in self.records]
        resolve_samples = [float(item["resolve_ms"]) for item in self.records]
        resolve_total_samples = [float(item["resolve_total_ms"]) for item in self.records]

        average_ms = float(np.mean(elapsed_samples)) if elapsed_samples else 0.0
        median_ms = float(np.median(elapsed_samples)) if elapsed_samples else 0.0
        p95_ms = float(np.percentile(elapsed_samples, 95)) if elapsed_samples else 0.0
        worst_ms = float(np.max(elapsed_samples)) if elapsed_samples else 0.0
        avg_creation_ms = float(np.mean(creation_samples)) if creation_samples else 0.0
        avg_insert_ms = float(np.mean(insert_samples)) if insert_samples else 0.0
        avg_detect_ms = float(np.mean(detect_samples)) if detect_samples else 0.0

        conflict_records = [item for item in self.records if float(item["resolve_ms"]) > 0.0]
        conflict_resolve_samples = [float(item["resolve_ms"]) for item in conflict_records]
        conflict_resolve_total_samples = [float(item["resolve_total_ms"]) for item in conflict_records]
        conflict_pass_counts = [int(item.get("resolve_pass_count", 0)) for item in conflict_records]
        conflict_shadow_samples = [float(item["shadow_build_ms"]) for item in conflict_records]
        conflict_s1_samples = [float(item["s1_ms"]) for item in conflict_records]
        conflict_sat_samples = [float(item["sat_ms"]) for item in conflict_records]
        conflict_s2_samples = [float(item["s2_ms"]) for item in conflict_records]
        conflict_fb1_samples = [float(item["fb1_ms"]) for item in conflict_records]
        conflict_fb2_samples = [float(item["fb2_ms"]) for item in conflict_records]
        conflict_s1_iter_samples = [float(item["s1_iterations"]) for item in conflict_records]
        conflict_s2_iter_samples = [float(item["s2_iterations"]) for item in conflict_records]
        conflict_fb1_iter_samples = [float(item["fb1_iterations"]) for item in conflict_records]
        conflict_fb2_iter_samples = [float(item["fb2_iterations"]) for item in conflict_records]

        avg_resolve_ms = float(np.mean(conflict_resolve_samples)) if conflict_resolve_samples else 0.0
        avg_resolve_total_ms = float(np.mean(conflict_resolve_total_samples)) if conflict_resolve_total_samples else 0.0
        avg_resolve_conflict_ms = avg_resolve_total_ms
        avg_resolve_pass_count = float(np.mean(conflict_pass_counts)) if conflict_pass_counts else 0.0
        avg_shadow_ms = float(np.mean(conflict_shadow_samples)) if conflict_shadow_samples else 0.0
        avg_s1_ms = float(np.mean(conflict_s1_samples)) if conflict_s1_samples else 0.0
        avg_sat_ms = float(np.mean(conflict_sat_samples)) if conflict_sat_samples else 0.0
        avg_s2_ms = float(np.mean(conflict_s2_samples)) if conflict_s2_samples else 0.0
        avg_fb1_ms = float(np.mean(conflict_fb1_samples)) if conflict_fb1_samples else 0.0
        avg_fb2_ms = float(np.mean(conflict_fb2_samples)) if conflict_fb2_samples else 0.0
        avg_s1_iters = float(np.mean(conflict_s1_iter_samples)) if conflict_s1_iter_samples else 0.0
        avg_s2_iters = float(np.mean(conflict_s2_iter_samples)) if conflict_s2_iter_samples else 0.0
        avg_fb1_iters = float(np.mean(conflict_fb1_iter_samples)) if conflict_fb1_iter_samples else 0.0
        avg_fb2_iters = float(np.mean(conflict_fb2_iter_samples)) if conflict_fb2_iter_samples else 0.0

        summary = {
            "inserted":          inserted,
            "resolved":          resolved,
            "deadlocks":         deadlocks,
            "anchor_deadlocks":  anchor_deadlocks,
            "hover_deadlocks":   hover_deadlocks,
            "hover_failures":    hover_failures,
            "clear_insertions":  clear_insertions,
            "average_ms":        average_ms,
            "median_ms":         median_ms,
            "p95_ms":            p95_ms,
            "worst_ms":          worst_ms,
            "avg_creation_ms":   avg_creation_ms,
            "avg_insert_ms":     avg_insert_ms,
            "avg_detect_ms":     avg_detect_ms,
            "avg_resolve_ms":    avg_resolve_ms,
            "avg_resolve_total_ms": avg_resolve_total_ms,
            "avg_resolve_conflict_ms": avg_resolve_conflict_ms,
            "avg_resolve_pass_count": avg_resolve_pass_count,
            "avg_shadow_ms":     avg_shadow_ms,
            "avg_s1_ms":         avg_s1_ms,
            "avg_sat_ms":        avg_sat_ms,
            "avg_s2_ms":         avg_s2_ms,
            "avg_fb1_ms":        avg_fb1_ms,
            "avg_fb2_ms":        avg_fb2_ms,
            "avg_s1_iters":      avg_s1_iters,
            "avg_s2_iters":      avg_s2_iters,
            "avg_fb1_iters":     avg_fb1_iters,
            "avg_fb2_iters":     avg_fb2_iters,
            "total_elapsed_ms":  total_elapsed_ms,
            "memory_delta_mb":   mem_delta_mb,
        }

        self._log("\n" + "=" * 96)
        self._log("SUMMARY")
        self._log("=" * 96)
        self._log(f"Inserted UAVs:      {inserted}")
        self._log(f"Clear:              {clear_insertions}")
        self._log(f"Resolved:           {resolved}")
        self._log(f"FB2 (Hover):        {hover_failures}")
        self._log(f"Deadlocks:          {deadlocks}")
        self._log(f"  anchor:           {anchor_deadlocks}")
        self._log(f"  hover timeout:    {hover_deadlocks}")
        self._log(f"Average:            {average_ms:.2f} ms / insertion")
        self._log(f"Median:             {median_ms:.2f} ms / insertion")
        self._log(f"95th percentile:    {p95_ms:.2f} ms / insertion")
        self._log(f"Worst case:         {worst_ms:.2f} ms / insertion")
        self._log(f"Avg creation time:  {avg_creation_ms:.2f} ms")
        self._log(f"Avg insertion time: {avg_insert_ms:.2f} ms")
        self._log(f"Avg detection time: {avg_detect_ms:.2f} ms")
        self._log(f"Avg last-pass resolve time (conflicts only): {avg_resolve_conflict_ms:.2f} ms")
        self._log(f"Avg resolution loop time (conflicts only):   {avg_resolve_ms:.2f} ms")
        self._log(f"Avg passes per conflict insertion:           {avg_resolve_pass_count:.2f}")
        self._log(f"Avg S1 time (conflicts only):  {avg_s1_ms:.2f} ms")
        self._log(f"Avg S1 iterations (conflicts only): {avg_s1_iters:.2f}")
        self._log(f"Avg SAT time (conflicts only): {avg_sat_ms:.2f} ms")
        self._log(f"Avg S2 time (conflicts only):  {avg_s2_ms:.2f} ms")
        self._log(f"Avg S2 iterations (conflicts only): {avg_s2_iters:.2f}")
        self._log(f"Avg FB1 time (conflicts only): {avg_fb1_ms:.2f} ms")
        self._log(f"Avg FB1 iterations (conflicts only): {avg_fb1_iters:.2f}")
        self._log(f"Avg FB2 time (conflicts only): {avg_fb2_ms:.2f} ms")
        self._log(f"Avg FB2 iterations (conflicts only): {avg_fb2_iters:.2f}")
        self._log(f"Total elapsed:      {total_elapsed_ms:.2f} ms")
        self._log(f"Memory delta:       {mem_delta_mb:.2f} MB")
        self._log("=" * 96 + "\n")

        # --------------- 3D Visualization (Before / After) ---------------
        # Visualization disabled by default. To re-enable, set
        # ENABLE_VISUALIZATION = True at the top of this file.
        if ENABLE_VISUALIZATION:
            try:
                BG = "white"
                FG = "#111111"
                fig = plt.figure(figsize=(14, 7), facecolor=BG)
                fig.canvas.manager.set_window_title("UAV Conflict Resolver — Benchmark Routes")
                fig.text(0.5, 0.95, "Benchmark: Sequential Conflict Routes — Before and After Resolution",
                         ha="center", va="center", fontsize=14, fontweight="bold", color=FG)

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
                    ax.set_title(title, color=FG, fontsize=11, pad=10)

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

                    ax.set_xlabel("X (m)", color=FG, labelpad=4)
                    ax.set_ylabel("Y (m)", color=FG, labelpad=4)
                    ax.set_zlabel("Z (m)", color=FG, labelpad=4)
                    ax.tick_params(colors=FG)
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


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    bench          = SequentialConflictRealtimeBenchmark(verbose=True)
    capture_buffer = io.StringIO()

    class _Tee:
        def __init__(self, *streams):
            self.streams = streams

        def write(self, text):
            for s in self.streams:
                s.write(text)
            return len(text)

        def flush(self):
            for s in self.streams:
                s.flush()

    with contextlib.redirect_stdout(_Tee(sys.stdout, capture_buffer)):
        try:
            bench.run(max_uavs=500, interval=0.5)
        finally:
            output_file = Path(__file__).with_name(
                "benchmark_realtime_sequential_conflicts_output.txt"
            )
            output_file.write_text(
                capture_buffer.getvalue().rstrip("\n") + "\n", encoding="utf-8"
            )
            print(f"Benchmark output written to: {output_file}")
