"""
benchmark_large_scale.py — Large-Scale Fleet Benchmark (500 UAVs in 5 km³)
===========================================================================

LARGE-SCALE PERFORMANCE TEST WITH CONFLICT RESOLUTION

PURPOSE:
    Test the conflict detection and RESOLUTION pipeline on a massive fleet
    of 500 UAVs in a 5 km³ airspace. This benchmark validates:
    
    • Detection accuracy and performance at scale
    • Conflict resolution via CentralManager
    • System stability under high density
    • Verbose logging of resolution strategies

Airspace: 5000m × 1000m × 1000m = 5 km³
Fleet Size: 500 UAVs
Expected: Some conflicts will be detected and resolved by manager
"""

import sys
from pathlib import Path
import numpy as np
import time
import psutil
import os
from typing import Dict, List, Optional, Tuple
import matplotlib.pyplot as plt
import argparse

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from detection.rtree_detector import RTreeDetector
from central_manager import CentralManager
from flightplan_generator import generate_layered_straight_fleet

# Large-scale airspace: default changed to 10 km × 10 km × 90 m (30m-120m flyable band)
LARGE_PRISM = (
    (0, 10000),      # X: 10 km
    (0, 10000),      # Y: 10 km
    (30, 120)        # Z: 90 m span (30m to 120m altitude)
)

# Vertiport-style airspace: 20 km × 20 km, flyable 30m - 120m
VERTIPORT_PRISM = (
    (0, 20000),     # X: 20 km
    (0, 20000),     # Y: 20 km
    (30, 120)       # Z: 30m - 120m
)

# Main benchmark default fleet size. Change this value to scale the run.
DEFAULT_N_UAVS = 100


def generate_large_fleet(
    n_uavs: int = DEFAULT_N_UAVS,
    prism: Tuple[Tuple[float, float], ...] = LARGE_PRISM,
    t_start: float = 0.0,
    t_end: float = 500.0,
    seed: int = 42,
    verbose: bool = True,
    conflict_mode: bool = True,
) -> List[FlightPlan]:
    """
    Generate a large fleet of UAVs using mostly straight routes stratified by
    altitude levels in Z. If `conflict_mode` is True, the routes are packed
    into fewer altitude layers to increase interaction density.
    """
    if verbose:
        print(f"\n[FLEET GENERATION] Creating {n_uavs} UAVs...")
        print(
            f"  • Airspace: {(prism[0][1]-prism[0][0])/1000:.1f}km × "
            f"{(prism[1][1]-prism[1][0])/1000:.1f}km × {(prism[2][1]-prism[2][0])/1000:.1f}km cube"
        )
        print(f"  • Time window: {t_start:.1f}s - {t_end:.1f}s\n")
    if conflict_mode:
        levels = [40.0, 55.0, 70.0, 85.0]
        fleet = generate_layered_straight_fleet(
            n_uavs=n_uavs,
            prism=prism,
            t_start=t_start,
            t_end=t_end,
            radius=5.0,
            seed=seed,
            levels=levels,
            conflict_mode=True,
            num_waypoints=4,
        )
    else:
        levels = [35.0, 50.0, 65.0, 80.0, 95.0, 110.0]
        fleet = generate_layered_straight_fleet(
            n_uavs=n_uavs,
            prism=prism,
            t_start=t_start,
            t_end=t_end,
            radius=5.0,
            seed=seed,
            levels=levels,
            conflict_mode=False,
            num_waypoints=4,
        )

    if verbose:
        print(f"  ✓ Fleet generation complete: {len(fleet)} UAVs ready\n")

    return fleet


def generate_vertiport_fleet(
    n_uavs: int = 100,
    prism: Tuple[Tuple[float, float], ...] = VERTIPORT_PRISM,
    t_start: float = 0.0,
    t_end: float = 1200.0,
    seed: int = 42,
    levels: List[float] = None,
    orientation: str = "random",
    vertiport_count: int = 6,
    verbose: bool = True,
) -> List[FlightPlan]:
    """
    Generate a fleet of `FlightPlan` objects that emulate vertiport departures:
    - Each plan: START at low altitude (30 m), ascend to a chosen cruise level,
      fly in a near-straight line (with a small persistent lateral offset to the
      left or right), then descend to 30 m at the end point.

    Args:
        n_uavs: number of flight plans to generate.
        prism: bounding box for X, Y, Z (meters).
        t_start, t_end: mission time window.
        seed: RNG seed.
        levels: list of cruise altitudes (meters). If None, a few levels are created.
        orientation: 'left', 'right', or 'random' lateral bias for the route.
        vertiport_count: number of vertiport origin centers to scatter departures from.
        verbose: print progress.

    Returns:
        List of `FlightPlan` objects.
    """
    rng = np.random.default_rng(seed)
    fleet: List[FlightPlan] = []
    x_range, y_range, z_range = prism

    if levels is None:
        # create several stacked cruise levels within the allowed altitude band
        min_z, max_z = int(z_range[0] + 10), int(z_range[1] - 10)
        levels = list(range(min_z + 10, max_z - 5, 20))  # e.g. 40,60,80,100

    # Create a few vertiport centers across the area
    vertiports = []
    margin = 500  # keep centers away from exact borders
    for i in range(vertiport_count):
        cx = x_range[0] + margin + rng.uniform(0, x_range[1] - x_range[0] - 2 * margin)
        cy = y_range[0] + margin + rng.uniform(0, y_range[1] - y_range[0] - 2 * margin)
        vertiports.append((cx, cy))

    if verbose:
        print(f"\n[VERTIPORT GENERATION] Creating {n_uavs} near-linear routes (prism {prism})")

    for uav_idx in range(n_uavs):
        fp = FlightPlan()
        fp.id = uav_idx + 1
        fp.priority = int(rng.integers(0, 3))
        fp.radius = 5.0
        fp.max_var_lin_vel = 20.0
        fp.max_var_ang_vel = 1.0

        # Choose vertiport origin and small scatter radius (takeoff pad)
        vp_x, vp_y = vertiports[rng.integers(0, len(vertiports))]
        pad_radius = rng.uniform(0, 100)  # up to 100m scatter around pad
        theta_pad = rng.uniform(0, 2 * np.pi)
        x_start = vp_x + pad_radius * np.cos(theta_pad)
        y_start = vp_y + pad_radius * np.sin(theta_pad)
        z_start = z_range[0]  # 30 m takeoff altitude

        # Choose cruise altitude level
        z_cruise = float(rng.choice(levels))

        # Choose a ground track bearing and travel distance (straight-ish)
        bearing = rng.uniform(0, 2 * np.pi)
        distance = rng.uniform(3000, 15000)  # 3 km - 15 km

        # Lateral bias (left/right) perpendicular offset magnitude
        lateral_mag = rng.uniform(0, 100)  # small persistent lateral offset
        if orientation == "random":
            sign = 1 if rng.random() < 0.5 else -1
        else:
            sign = 1 if orientation == "right" else -1

        # Compute straight-line end point, add perpendicular offset
        dx = distance * np.cos(bearing)
        dy = distance * np.sin(bearing)
        # perp vector (normalized)
        perp_x = -np.sin(bearing)
        perp_y = np.cos(bearing)
        x_end = x_start + dx + sign * perp_x * lateral_mag
        y_end = y_start + dy + sign * perp_y * lateral_mag
        # Clamp end inside prism
        x_end = max(x_range[0] + 10, min(x_end, x_range[1] - 10))
        y_end = max(y_range[0] + 10, min(y_end, y_range[1] - 10))
        z_end = z_range[0]

        # Timing: start time and estimated duration based on cruise speed
        uav_t_start = t_start + rng.uniform(0, min(120.0, (t_end - t_start) * 0.5))
        cruise_speed = rng.uniform(12, 18)  # m/s
        travel_time = max(10.0, distance / cruise_speed)
        uav_t_end = min(uav_t_start + travel_time + 20.0, t_end)

        # Build waypoints: START (low), ASCEND to cruise, CRUISE mid, DESCEND END
        wp0 = Waypoint("START", uav_t_start, [x_start, y_start, z_start], [0.0, 0.0, 0.0])
        wp1 = Waypoint("CLIMB", uav_t_start + 5.0, [x_start + 5.0 * np.cos(bearing), y_start + 5.0 * np.sin(bearing), z_cruise],
                       [cruise_speed * np.cos(bearing), cruise_speed * np.sin(bearing), (z_cruise - z_start) / 5.0])

        mid_x = x_start + 0.5 * (x_end - x_start)
        mid_y = y_start + 0.5 * (y_end - y_start)
        mid_t = uav_t_start + travel_time * 0.5
        wp2 = Waypoint("CRUISE", mid_t, [mid_x, mid_y, z_cruise], [cruise_speed * np.cos(bearing), cruise_speed * np.sin(bearing), 0.0])

        wp3 = Waypoint("END", uav_t_end, [x_end, y_end, z_end], [0.0, 0.0, 0.0])

        for wp in (wp0, wp1, wp2, wp3):
            fp.set_waypoint(wp=wp)

        fleet.append(fp)

        if verbose and (uav_idx + 1) % 50 == 0:
            print(f"  ✓ Generated {uav_idx + 1}/{n_uavs} vertiport routes...")

    if verbose:
        print(f"  ✓ Vertiport fleet generation complete: {len(fleet)} routes\n")

    return fleet


class LargeScaleBenchmark:
    """Orchestrates large-scale benchmark with conflict resolution."""
    
    def __init__(self, verbose: bool = True):
        self.fleet = []
        self.detector = None
        self.manager = None
        self.metrics = {}
        self.original_plans: Dict[int, FlightPlan] = {}
        self.verbose = verbose
    
    def _log(self, msg: str, level: str = "INFO"):
        """Print verbose log message."""
        if not self.verbose:
            return
        
        prefix_map = {
            "INFO": "ℹ ",
            "OK": "✓ ",
            "WARN": "⚠ ",
            "ERROR": "✗ ",
        }
        prefix = prefix_map.get(level, "→ ")
        print(f"  {prefix} {msg}")
    
    def run(self, n_uavs: int = DEFAULT_N_UAVS, conflict_mode: bool = True) -> dict:
        """
        Execute the full large-scale benchmark pipeline.
        
        Args:
            n_uavs: Number of UAVs (500+)
        
        Returns:
            Dictionary with all performance metrics
        """
        print("\n" + "█" * 100)
        print("█" + " " * 98 + "█")
        print("█" + "  LARGE-SCALE UAV FLEET BENCHMARK — 500 UAVs in 5 km³".center(98) + "█")
        print("█" + "  Conflict Detection & Resolution Performance Analysis".center(98) + "█")
        print("█" + " " * 98 + "█")
        print("█" * 100)
        
        # ─────────────────────────────────────────────────────────────────
        # PHASE 1: FLEET GENERATION
        # ─────────────────────────────────────────────────────────────────
        print(f"\n┌─ [PHASE 1] FLEET GENERATION ──────────────────────────────────────────────────────────┐")
        print(f"└─────────────────────────────────────────────────────────────────────────────────────┘")
        
        t0_gen = time.time()
        mem0_gen = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024
        
        self.fleet = generate_large_fleet(n_uavs=n_uavs, verbose=self.verbose, conflict_mode=conflict_mode)
        self.original_plans = {}
        for fp in self.fleet:
            try:
                self.original_plans[fp.id] = fp.copy()
            except Exception:
                # If copy() is unavailable for any edge case, keep original ref.
                self.original_plans[fp.id] = fp
        
        t1_gen = time.time()
        mem1_gen = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024
        
        time_gen = t1_gen - t0_gen
        mem_gen = mem1_gen - mem0_gen
        
        print(f"\n✓ Fleet Generation Complete:")
        print(f"  │ UAVs: {len(self.fleet)}")
        print(f"  │ Time: {time_gen*1000:.2f} ms")
        print(f"  │ Memory: {mem_gen:.2f} MB ({mem_gen/len(self.fleet):.4f} MB/UAV)")
        
        # ─────────────────────────────────────────────────────────────────
        # PHASE 2: DETECTOR INITIALIZATION & CONFLICT DETECTION
        # ─────────────────────────────────────────────────────────────────
        print(f"\n┌─ [PHASE 2] CONFLICT DETECTION ────────────────────────────────────────────────────────┐")
        print(f"└─────────────────────────────────────────────────────────────────────────────────────┘")
        
        self._log("Initializing RTree detector...")
        t0_init = time.time()
        mem0_init = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024
        
        self.detector = RTreeDetector()
        for fp in self.fleet:
            self.detector.register_uav(fp.id, fp, interval=0.5)
        
        t1_init = time.time()
        mem1_init = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024
        time_init = t1_init - t0_init
        mem_init = mem1_init - mem0_init
        
        total_boxes = sum(len(self.detector.uavs[uid]["boxes"]) for uid in self.detector.uavs)
        self._log(f"RTree initialized with {len(self.fleet)} UAVs (total boxes: {total_boxes})", "OK")
        
        self._log("Running conflict detection on all UAVs...")
        t0_detect = time.time()
        mem0_detect = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024
        
        all_conflicts = []
        detection_times = []
        
        for idx, fp in enumerate(self.fleet):
            t_before = time.time()
            conflicts = self.detector.detect_all_conflicts(fp.id)
            t_after = time.time()
            
            detection_times.append(t_after - t_before)
            all_conflicts.extend(conflicts)
            
            if self.verbose and (idx + 1) % 100 == 0:
                self._log(f"Detection progress: {idx + 1}/{len(self.fleet)} UAVs processed")
        
        t1_detect = time.time()
        mem1_detect = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024
        
        time_detect = t1_detect - t0_detect
        mem_detect = mem1_detect - mem0_detect
        
        # Deduplicate conflicts (each conflict counted twice)
        # Convert to canonical form: (uav_a, uav_b, box_a_idx, box_b_idx) for deduplication
        seen = {}
        unique_conflicts = []
        for conflict in all_conflicts:
            # Create a canonical key with sorted UAV IDs to treat (A,B) and (B,A) as same
            uav_a, uav_b = conflict["uav_a"], conflict["uav_b"]
            box_a, box_b = conflict["box_a_idx"], conflict["box_b_idx"]
            # Normalize the pair so smaller ID comes first
            if uav_a < uav_b:
                key = (uav_a, uav_b, box_a, box_b)
            else:
                key = (uav_b, uav_a, box_b, box_a)
            
            if key not in seen:
                seen[key] = conflict
                unique_conflicts.append(conflict)
        
        print(f"\n✓ Conflict Detection Complete:")
        print(f"  │ Conflicts found: {len(unique_conflicts)}")
        print(f"  │ Detection time: {time_detect*1000:.2f} ms")
        print(f"  │ Memory used: {mem_detect:.2f} MB ({mem_detect/len(self.fleet):.4f} MB/UAV)")
        print(f"  │ Avg detection/UAV: {(time_detect/len(self.fleet))*1000:.4f} ms")
        
        # ─────────────────────────────────────────────────────────────────
        # PHASE 3: CONFLICT RESOLUTION WITH CENTRAL MANAGER
        # ─────────────────────────────────────────────────────────────────
        print(f"\n┌─ [PHASE 3] CONFLICT RESOLUTION ───────────────────────────────────────────────────────┐")
        print(f"└─────────────────────────────────────────────────────────────────────────────────────┘")
        
        # Initialize CentralManager and register all flight plans
        self._log("Initializing CentralManager for conflict resolution...")
        t0_manager = time.time()
        mem0_manager = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024

        self.manager = CentralManager()
        self._log("Adding all flight plans to manager...")

        for fp in self.fleet:
            self.manager.register_uav(str(fp.id), fp)

        self._log(f"Manager has {len(self.manager._flight_plans)} flight plans registered", "OK")

        # Resolve globally until the airspace stabilizes.
        resolve_stats = self.manager.solve_all(max_sweeps=300, interval=0.5)

        t1_manager = time.time()
        mem1_manager = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024

        time_manager = t1_manager - t0_manager
        mem_manager = mem1_manager - mem0_manager

        print(f"\n✓ Conflict Resolution Complete:")
        print(f"  │ Manager overhead: {time_manager*1000:.2f} ms")
        print(f"  │ Memory overhead: {mem_manager:.2f} MB")
        print(f"  │ Sweeps: {resolve_stats['sweeps']}")
        print(f"  │ Attempts: {resolve_stats['attempted']}")
        print(f"  │ Resolved: {resolve_stats['resolved']}")
        print(f"  │ Deadlocks: {resolve_stats['deadlocks']}")
        print(f"  │ Remaining conflicts: {resolve_stats['remaining_conflicts']}")
        
        # ─────────────────────────────────────────────────────────────────
        # SUMMARY REPORT
        # ─────────────────────────────────────────────────────────────────
        total_time = time_gen + time_init + time_detect + time_manager
        total_mem = mem_gen + mem_init + mem_detect + mem_manager
        
        print(f"\n" + "█" * 100)
        print("█" + "  PERFORMANCE SUMMARY".center(98) + "█")
        print("█" * 100)
        
        print(f"\n┌─ EXECUTION TIME ───────────────────────────────────────────────────────────────────┐")
        print(f"│ Total:                          {total_time*1000:>12.2f} ms  ({total_time:>8.3f} s)          │")
        print(f"│   ├─ Fleet generation:          {time_gen*1000:>12.2f} ms  ({time_gen/total_time*100:>5.1f}%)               │")
        print(f"│   ├─ Detector initialization:   {time_init*1000:>12.2f} ms  ({time_init/total_time*100:>5.1f}%)               │")
        print(f"│   ├─ Conflict detection:        {time_detect*1000:>12.2f} ms  ({time_detect/total_time*100:>5.1f}%)               │")
        print(f"│   └─ Manager overhead:          {time_manager*1000:>12.2f} ms  ({time_manager/total_time*100:>5.1f}%)               │")
        print(f"└────────────────────────────────────────────────────────────────────────────────────┘")
        
        print(f"\n┌─ MEMORY USAGE ─────────────────────────────────────────────────────────────────────┐")
        print(f"│ Total delta:                    {total_mem:>12.2f} MB                              │")
        print(f"│   ├─ Fleet generation:          {mem_gen:>12.2f} MB                              │")
        print(f"│   ├─ Detector initialization:   {mem_init:>12.2f} MB                              │")
        print(f"│   ├─ Conflict detection:        {mem_detect:>12.2f} MB                              │")
        print(f"│   └─ Manager overhead:          {mem_manager:>12.2f} MB                              │")
        print(f"└────────────────────────────────────────────────────────────────────────────────────┘")
        
        print(f"\n┌─ CONFLICT ANALYSIS ────────────────────────────────────────────────────────────────┐")
        print(f"│ Total conflicts detected:       {len(unique_conflicts):>12} pairs                  │")
        print(f"│ Density per UAV:                {len(unique_conflicts)/len(self.fleet):>12.2f} conflicts/UAV        │")
        print(f"│ System density:                 {len(unique_conflicts)/(len(self.fleet)*(len(self.fleet)-1)/2)*100:>11.2f}% (theoretical max)     │")
        print(f"└────────────────────────────────────────────────────────────────────────────────────┘")
        
        print(f"\n┌─ SCALABILITY METRICS ──────────────────────────────────────────────────────────────┐")
        print(f"│ Fleet size:                     {len(self.fleet):>12} UAVs                      │")
        print(f"│ Total swept boxes:              {total_boxes:>12} boxes                   │")
        print(f"│ Time per UAV (overall):         {(total_time/len(self.fleet))*1000:>12.4f} ms/UAV               │")
        print(f"│ Memory per UAV (total):         {(total_mem/len(self.fleet)):>12.4f} MB/UAV               │")
        print(f"│ Boxes per UAV:                  {(total_boxes/len(self.fleet)):>12.1f} boxes/UAV            │")
        print(f"└────────────────────────────────────────────────────────────────────────────────────┘")
        
        print("\n" + "█" * 100 + "\n")
        
        # Store metrics
        self.metrics = {
            "n_uavs": len(self.fleet),
            "total_conflicts": len(unique_conflicts),
            "total_boxes": total_boxes,
            "time_gen_ms": time_gen * 1000,
            "time_init_ms": time_init * 1000,
            "time_detect_ms": time_detect * 1000,
            "time_manager_ms": time_manager * 1000,
            "time_total_ms": total_time * 1000,
            "mem_gen_mb": mem_gen,
            "mem_init_mb": mem_init,
            "mem_detect_mb": mem_detect,
            "mem_manager_mb": mem_manager,
            "mem_total_mb": total_mem,
            "detection_times_ms": [t * 1000 for t in detection_times],
            "conflict_density": len(unique_conflicts) / len(self.fleet),
            "resolved_conflicts": resolve_stats.get("resolved", 0),
            "unresolved_conflicts": resolve_stats.get("deadlocks", 0),
            "remaining_conflicts": resolve_stats.get("remaining_conflicts", 0),
            "solve_sweeps": resolve_stats.get("sweeps", 0),
            "solve_attempts": resolve_stats.get("attempted", 0),
        }

        # Visualization: first show realtime-like BEFORE/AFTER 3D routes,
        # then summary plots with timing and resolution metrics.
        try:
            self.visualize_before_after_routes()
        except Exception as e:
            self._log(f"Before/After visualization failed: {e}", "WARN")

        try:
            self.visualize_results(self.metrics, unique_conflicts)
        except Exception as e:
            self._log(f"Visualization failed: {e}", "WARN")

        return self.metrics

    def visualize_before_after_routes(self):
        """Render a realtime-style 3D BEFORE/AFTER route comparison."""
        if not self.verbose:
            return

        BG = "#0f1117"
        fig = plt.figure(figsize=(14, 7), facecolor=BG)
        try:
            fig.canvas.manager.set_window_title("UAV Conflict Resolver - Large Scale Routes")
        except Exception:
            pass

        fig.text(
            0.5,
            0.95,
            "Large-Scale Benchmark: Flight Routes - Before and After Resolution",
            ha="center",
            va="center",
            fontsize=14,
            fontweight="bold",
            color="white",
        )

        before_plans = {pid: fp for pid, fp in self.original_plans.items()}
        after_plans: Dict[int, Optional[FlightPlan]] = {
            pid: self.manager.get_flight_plan(str(pid)) for pid in before_plans.keys()
        }

        colors_list = [
            "#2ecc71", "#e74c3c", "#3498db", "#f1c40f", "#9b59b6",
            "#1abc9c", "#e67e22", "#34495e", "#7f8c8d", "#16a085",
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
                    continue

                if trace is None or len(trace) == 0:
                    continue

                color = colors_list[i % len(colors_list)]
                z_offset = i * 0.05
                z_plot = trace[:, 3] + z_offset
                ax.plot(
                    trace[:, 1],
                    trace[:, 2],
                    z_plot,
                    color=color,
                    linewidth=1.2,
                    linestyle="-",
                    alpha=0.9,
                )
                ax.scatter(trace[0, 1], trace[0, 2], trace[0, 3] + z_offset, color=color, s=16, marker="o", zorder=5)
                ax.scatter(trace[-1, 1], trace[-1, 2], trace[-1, 3] + z_offset, color=color, s=16, marker="^", zorder=5)

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

    def visualize_results(self, metrics: dict, unique_conflicts: List[dict]):
        """Create plots summarizing fleet, conflicts and resolution results."""
        if not self.verbose:
            return

        fig, axs = plt.subplots(2, 2, figsize=(14, 10))
        ax_map = axs[0, 0]
        ax_hist = axs[0, 1]
        ax_bar = axs[1, 0]
        ax_text = axs[1, 1]

        # Map: plot each flight's straight-line route (start -> end)
        xs_start, ys_start = [], []
        xs_end, ys_end = [], []
        for fp in self.fleet:
            if not fp.waypoints:
                continue
            p0 = fp.waypoints[0].pos
            pN = fp.waypoints[-1].pos
            xs_start.append(p0[0]); ys_start.append(p0[1])
            xs_end.append(pN[0]); ys_end.append(pN[1])
            ax_map.plot([p0[0], pN[0]], [p0[1], pN[1]], color="C0", alpha=0.25)

        ax_map.scatter(xs_start, ys_start, s=8, c="green", label="start")
        ax_map.scatter(xs_end, ys_end, s=8, c="blue", label="end")

        # Plot conflict midpoints (xy) in red
        conf_x = []
        conf_y = []
        for c in unique_conflicts:
            try:
                a = str(c["uav_a"]) ; ai = c["box_a_idx"]
                b = str(c["uav_b"]) ; bi = c["box_b_idx"]
                box_a = self.detector.uavs[a]["boxes"][ai]
                box_b = self.detector.uavs[b]["boxes"][bi]
                mid = 0.5 * (box_a.center + box_b.center)
                conf_x.append(mid[0]); conf_y.append(mid[1])
            except Exception:
                continue

        if conf_x:
            ax_map.scatter(conf_x, conf_y, s=24, c="red", marker="x", label="conflicts")

        ax_map.set_title("Flight routes (XY projection) and detected conflicts")
        ax_map.set_xlabel("X (m)")
        ax_map.set_ylabel("Y (m)")
        ax_map.legend(loc="upper right")

        # Histogram: detection times
        detection_times = metrics.get("detection_times_ms", [])
        if detection_times:
            ax_hist.hist(detection_times, bins=40, color="C2", alpha=0.8)
            ax_hist.set_title("Detection time per UAV (ms)")
            ax_hist.set_xlabel("ms")
            ax_hist.set_ylabel("count")
        else:
            ax_hist.text(0.5, 0.5, "No detection timing data", ha="center", va="center")

        # Bar: resolved vs unresolved
        resolved = metrics.get("resolved_conflicts", 0)
        unresolved = metrics.get("unresolved_conflicts", 0)
        ax_bar.bar(["resolved", "unresolved"], [resolved, unresolved], color=["C3", "C1"])
        ax_bar.set_title("Conflict resolution outcomes")
        ax_bar.set_ylabel("count")

        # Text box with key metrics
        text_lines = [
            f"UAVs: {metrics.get('n_uavs', 0)}",
            f"Conflicts: {metrics.get('total_conflicts', 0)}",
            f"Resolved: {resolved}",
            f"Unresolved: {unresolved}",
            f"Total time (s): {metrics.get('time_total_ms', 0)/1000:.2f}",
            f"Memory delta (MB): {metrics.get('mem_total_mb', 0):.2f}",
        ]
        ax_text.axis('off')
        ax_text.text(0.02, 0.98, "\n".join(text_lines), va='top', fontsize=12, family='monospace')

        plt.tight_layout()
        plt.show()


# ============================================================================
# Main Entry Point
# ============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-uavs", type=int, default=DEFAULT_N_UAVS, help="Number of UAVs to generate")
    conflict_group = parser.add_mutually_exclusive_group()
    conflict_group.add_argument("--conflict-mode", dest="conflict_mode", action="store_true", help="Enable aggressive conflict generation")
    conflict_group.add_argument("--no-conflict-mode", dest="conflict_mode", action="store_false", help="Disable aggressive conflict generation")
    parser.set_defaults(conflict_mode=True)
    args = parser.parse_args()

    benchmark = LargeScaleBenchmark(verbose=True)
    metrics = benchmark.run(n_uavs=args.n_uavs, conflict_mode=args.conflict_mode)
    
    print("\n" + "="*100)
    print("Benchmark completed successfully!")
    print(f"Total conflicts detected: {metrics['total_conflicts']}")
    print(f"Conflict density: {metrics['conflict_density']:.4f} conflicts/UAV")
    print("="*100)
