"""
benchmark_realtime_sequential_conflicts.py - Incremental inserter that forces
sequential conflicts between successive vertiport routes.
-------------------------------------------------------------------------

This benchmark inserts flight plans one-by-one (real-time insertion style)
but intentionally constructs each new plan so it will intersect (in space
and time) the previously inserted plan. The crossing point is chosen near
the previous plan's cruise midpoint with a small random jitter so conflicts
do not always occur at the exact same spot.

Usage: run the script; it will insert up to `max_uavs` and report per-insert
conflict counts and a final summary. It uses the CentralManager and R-Tree
detector from the project.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import psutil
import io
import contextlib
import matplotlib.pyplot as plt

# Path setup (project root)
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from core.config import UAV_MAX_SPEED
from central_manager import CentralManager


# Airspace matching the vertiport request: 20 km × 20 km × 30–120 m
VERTIORT_PRISM: Tuple[Tuple[float, float], ...] = (
    (0.0, 20000.0),
    (0.0, 20000.0),
    (30.0, 120.0),
)


def _build_vertiport_plan(
    uav_index: int,
    prism: Tuple[Tuple[float, float], ...] = VERTIORT_PRISM,
    seed: int = 42,
    crossing_point: Optional[Tuple[float, float]] = None,
    crossing_time: Optional[float] = None,
) -> FlightPlan:
    """Build a near-straight vertiport-style FlightPlan.

    If `crossing_point` and `crossing_time` are provided the plan will pass
    through that 2D point (X,Y) at the requested time (Z is chosen as the
    cruise level). This allows forcing temporal-spatial intersections with
    previously inserted plans.
    """
    rng = np.random.default_rng(seed + uav_index * 881)
    x_rng, y_rng, z_rng = prism

    # Choose a vertiport origin somewhere in the area (clustered pads)
    pad_x = x_rng[0] + rng.uniform(1000.0, x_rng[1] - 1000.0)
    pad_y = y_rng[0] + rng.uniform(1000.0, y_rng[1] - 1000.0)
    x_start = pad_x + rng.uniform(-80.0, 80.0)
    y_start = pad_y + rng.uniform(-80.0, 80.0)
    z_start = z_rng[0]

    # Cruise level selection across a few levels
    levels = [40.0, 60.0, 80.0, 100.0]
    z_cruise = float(rng.choice(levels))

    # If a crossing point is requested, use it as the mid point. Otherwise
    # pick a distant endpoint so line is long.
    if crossing_point is not None:
        mid_x, mid_y = crossing_point
        # place end point beyond the crossing so path passes through it
        bearing = rng.uniform(0, 2 * np.pi)
        end_dist = rng.uniform(2000.0, 10000.0)
        x_end = mid_x + end_dist * np.cos(bearing)
        y_end = mid_y + end_dist * np.sin(bearing)
    else:
        bearing = rng.uniform(0, 2 * np.pi)
        end_dist = rng.uniform(3000.0, 12000.0)
        x_end = x_start + end_dist * np.cos(bearing)
        y_end = y_start + end_dist * np.sin(bearing)

    # Clamp inside prism
    x_end = max(x_rng[0] + 10.0, min(x_end, x_rng[1] - 10.0))
    y_end = max(y_rng[0] + 10.0, min(y_end, y_rng[1] - 10.0))

    # Speed and duration
    cruise_speed = float(rng.uniform(12.0, min(17.0, UAV_MAX_SPEED)))

    # If crossing_time is provided align timings so midpoint happens near it
    if crossing_time is None:
        t_start = uav_index * 3.0 + rng.uniform(0.0, 0.75)
    else:
        # choose start so the mid point occurs near crossing_time
        t_start = max(0.0, crossing_time - 0.5 * (end_dist / cruise_speed)) + rng.uniform(-1.0, 1.0)

    # total distance approximated as start->mid->end or start->end if no mid
    if crossing_point is not None:
        d1 = np.hypot(mid_x - x_start, mid_y - y_start)
        d2 = np.hypot(x_end - mid_x, y_end - mid_y)
        total_distance = d1 + d2
    else:
        total_distance = np.hypot(x_end - x_start, y_end - y_start)

    mission_duration = max(120.0, total_distance / cruise_speed)
    t_end = t_start + mission_duration

    # Build flight plan
    fp = FlightPlan()
    fp.id = uav_index + 1
    fp.priority = 1 if uav_index % 7 == 0 else 0
    fp.radius = 5.0
    fp.max_var_lin_vel = UAV_MAX_SPEED
    fp.max_var_ang_vel = 1.0

    # Waypoints: START, CLIMB to cruise (short), CRUISE(mid or computed), END
    wp0 = Waypoint("START", float(t_start), [x_start, y_start, z_start], [0.0, 0.0, 0.0])

    # quick climb to cruise in 5s
    climb_t = float(t_start + 5.0)
    climb_x = x_start + 5.0 * np.cos(bearing)
    climb_y = y_start + 5.0 * np.sin(bearing)
    wp1 = Waypoint("CLIMB", climb_t, [climb_x, climb_y, z_cruise], [cruise_speed * np.cos(bearing), cruise_speed * np.sin(bearing), (z_cruise - z_start) / 5.0])

    if crossing_point is not None:
        mid_t = float(crossing_time if crossing_time is not None else (t_start + mission_duration * 0.5))
        wp2 = Waypoint("CRUISE", mid_t, [mid_x, mid_y, z_cruise], [cruise_speed * np.cos(bearing), cruise_speed * np.sin(bearing), 0.0])
    else:
        mid_x = x_start + 0.5 * (x_end - x_start)
        mid_y = y_start + 0.5 * (y_end - y_start)
        mid_t = float(t_start + mission_duration * 0.5)
        wp2 = Waypoint("CRUISE", mid_t, [mid_x, mid_y, z_cruise], [cruise_speed * np.cos(bearing), cruise_speed * np.sin(bearing), 0.0])

    wp3 = Waypoint("END", float(t_end), [x_end, y_end, z_rng[0]], [0.0, 0.0, 0.0])

    for w in (wp0, wp1, wp2, wp3):
        fp.set_waypoint(wp=w)

    try:
        fp.connect_waypoints(strict=False)
    except Exception:
        pass

    # attach metadata for caller convenience
    fp._benchmark_mid = (float(mid_x), float(mid_y))
    fp._benchmark_mid_time = float(mid_t)
    return fp


class SequentialConflictRealtimeBenchmark:
    def __init__(self, prism: Tuple[Tuple[float, float], ...] = VERTIORT_PRISM, seed: int = 42, verbose: bool = True):
        self.prism = prism
        self.seed = seed
        self.verbose = verbose
        self.manager = CentralManager()
        self.inserted = 0
        self.records: List[Dict[str, object]] = []
        self.original_plans: Dict[int, FlightPlan] = {}

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(msg)

    def run(self, max_uavs: int = 200, interval: float = 0.5) -> Dict[str, object]:
        process = psutil.Process(os.getpid())
        mem_start_mb = process.memory_info().rss / (1024 * 1024)

        prev_mid = None
        prev_mid_time = None

        inserted = 0
        clear_insertions = 0
        resolved = 0
        deadlocks = 0
        hover_failures = 0
        total_elapsed_ms = 0.0

        self._log("\n" + "█" * 96)
        self._log("█" + " " * 94 + "█")
        self._log("█" + "  SEQUENTIAL CONFLICT REAL-TIME BENCHMARK".center(94) + "█")
        self._log("█" + "  Each insertion is crafted to cross the previous plan (20km x 20km x 90m)".center(94) + "█")
        self._log("█" + " " * 94 + "█")
        self._log("█" * 96)

        while inserted < max_uavs:
            creation_t0 = time.perf_counter()
            # Build plan aimed to cross previous plan
            if prev_mid is None:
                fp = _build_vertiport_plan(inserted, prism=self.prism, seed=self.seed)
            else:
                rng_j = np.random.default_rng(self.seed + inserted)
                jitter_xy = (rng_j.uniform(-80.0, 80.0), rng_j.uniform(-80.0, 80.0))
                cross_pt = (prev_mid[0] + jitter_xy[0], prev_mid[1] + jitter_xy[1])
                cross_t = prev_mid_time + rng_j.uniform(-3.0, 3.0)
                fp = _build_vertiport_plan(inserted, prism=self.prism, seed=self.seed, crossing_point=cross_pt, crossing_time=cross_t)

            creation_ms = (time.perf_counter() - creation_t0) * 1000.0

            try:
                self.original_plans[fp.id] = fp.copy()
            except Exception:
                self.original_plans[fp.id] = fp

            uav_id = str(fp.id)

            # Insert
            insert_t0 = time.perf_counter()
            self.manager.register_uav(uav_id, fp, priority=fp.priority, interval=interval)
            insert_ms = (time.perf_counter() - insert_t0) * 1000.0

            # Detect
            detect_t0 = time.perf_counter()
            conflicts = self.manager._rtree_detector.detect_all_conflicts(uav_id)
            detect_ms = (time.perf_counter() - detect_t0) * 1000.0

            resolve_ms = 0.0
            result = None

            if conflicts:
                conflicts.sort(key=lambda c: c.get("time_range", (0.0, 0.0))[0])
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
                try:
                    result = self.manager._resolver.resolve(
                        conflict=conflict,
                        fp_pleb=fp_pleb,
                        fp_vip=fp_vip,
                        pleb_id=pleb_id,
                        vip_id=vip_id,
                    )
                except Exception:
                    result = None
                resolve_ms = (time.perf_counter() - resolve_t0) * 1000.0

                if result is not None and result.success:
                    resolved += 1
                    if result.new_fp_pleb is not None:
                        self.manager.register_uav(pleb_id, result.new_fp_pleb, priority=self.manager._priorities.get(pleb_id, 0), interval=interval)
                    if result.new_fp_vip is not None:
                        self.manager.register_uav(vip_id, result.new_fp_vip, priority=self.manager._priorities.get(vip_id, 0), interval=interval)
                elif result is not None and result.strategy_used == "FB2":
                    hover_failures += 1
                elif result is not None and not result.success:
                    deadlocks += 1

            else:
                clear_insertions += 1

            elapsed_ms = creation_ms + insert_ms + detect_ms + resolve_ms
            total_elapsed_ms += elapsed_ms

            # Record
            record = {
                "uav_id": fp.id,
                "creation_ms": creation_ms,
                "insert_ms": insert_ms,
                "detect_ms": detect_ms,
                "resolve_ms": resolve_ms,
                "conflicts": len(conflicts),
                "status": ("CLEAR" if not conflicts else ("RESOLVED" if result is not None and result.success else ("HOVER" if result is not None and getattr(result, 'strategy_used', '') == "FB2" else ("DEADLOCK" if result is not None and not result.success else "UNKNOWN")))),
            }
            self.records.append(record)

            self._log(f"UAV {fp.id:04d} | conflicts={len(conflicts):3d} | create={creation_ms:8.2f} ms | insert={insert_ms:8.2f} ms | detect={detect_ms:8.2f} ms | resolve={resolve_ms:8.2f} ms")

            # stop conditions similar to realtime benchmark
            if result is not None and getattr(result, "strategy_used", "") == "FB2":
                self._log(f"Resolver reached HOVER/FB2 at UAV {fp.id}. Stopping benchmark.")
                break
            if result is not None and not getattr(result, "success", True):
                self._log(f"Resolver reached DEADLOCK at UAV {fp.id}. Stopping benchmark.")
                break

            # update previous midpoint for next insertion
            prev_mid = getattr(fp, "_benchmark_mid", None)
            prev_mid_time = getattr(fp, "_benchmark_mid_time", None)

            inserted += 1

        mem_end_mb = process.memory_info().rss / (1024 * 1024)
        mem_delta_mb = mem_end_mb - mem_start_mb

        # Summarize
        elapsed_samples = [float(item["creation_ms"] + item["insert_ms"] + item["detect_ms"] + item["resolve_ms"]) for item in self.records]
        average_ms = float(np.mean(elapsed_samples)) if elapsed_samples else 0.0
        median_ms = float(np.median(elapsed_samples)) if elapsed_samples else 0.0
        p95_ms = float(np.percentile(elapsed_samples, 95)) if elapsed_samples else 0.0

        summary = {
            "inserted": inserted,
            "resolved": resolved,
            "deadlocks": deadlocks,
            "hover_failures": hover_failures,
            "clear_insertions": clear_insertions,
            "average_ms": average_ms,
            "median_ms": median_ms,
            "p95_ms": p95_ms,
            "total_elapsed_ms": total_elapsed_ms,
            "memory_delta_mb": mem_delta_mb,
        }

        # Print summary
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
        self._log(f"Total elapsed:           {total_elapsed_ms:.2f} ms")
        self._log(f"Memory delta:            {mem_delta_mb:.2f} MB")

        # Optional visualization (before/after)
        try:
            BG = "#0f1117"
            fig = plt.figure(figsize=(14, 7), facecolor=BG)
            fig.canvas.manager.set_window_title("Sequential Conflict Benchmark — Routes")
            fig.text(0.5, 0.95, "Sequential Conflict Benchmark — Before and After",
                     ha="center", va="center", fontsize=14, fontweight="bold", color="white")

            before_plans = {pid: fp for pid, fp in self.original_plans.items()}
            after_plans = {pid: self.manager.get_flight_plan(str(pid)) for pid in before_plans.keys()}

            colors_list = ["#2ecc71", "#e74c3c", "#3498db", "#f1c40f", "#9b59b6", "#1abc9c"]

            for col, title, plan_dict in [
                (0, "BEFORE (Generated)", before_plans),
                (1, "AFTER (Committed)", after_plans),
            ]:
                ax = fig.add_subplot(1, 2, col + 1, projection="3d")
                ax.set_facecolor(BG)
                ax.set_title(title, color="white", fontsize=11, pad=10)

                for i, (pid, fp) in enumerate(sorted(plan_dict.items())):
                    if fp is None:
                        continue
                    try:
                        trace = fp.trace(0.5)
                    except Exception:
                        continue
                    color = colors_list[i % len(colors_list)]
                    z_offset = i * 0.02
                    z_plot = trace[:, 3] + z_offset
                    ax.plot(trace[:, 1], trace[:, 2], z_plot, color=color, linewidth=1.0, alpha=0.9)

                ax.set_xlabel("X (m)", color="white", labelpad=4)
                ax.set_ylabel("Y (m)", color="white", labelpad=4)
                ax.set_zlabel("Z (m)", color="white", labelpad=4)
                ax.tick_params(colors="white")

            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            plt.show()
        except Exception:
            pass

        self.summary = summary
        return summary


if __name__ == "__main__":
    bench = SequentialConflictRealtimeBenchmark(verbose=True)
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
            bench.run(max_uavs=200, interval=0.5)
        finally:
            output_file = Path(__file__).with_name("benchmark_realtime_sequential_conflicts_output.txt")
            output_file.write_text(capture_buffer.getvalue().rstrip("\n") + "\n", encoding="utf-8")
            print(f"Benchmark output written to: {output_file}")


if __name__ == "__main__":
    bench = SequentialConflictRealtimeBenchmark(verbose=True)
    bench.run(max_uavs=200, interval=0.5)
