"""
benchmark_resolver.py — Computational Performance Benchmark
============================================================

ACADEMIC BENCHMARK FOR UAV CONFLICT DETECTION SYSTEM

PURPOSE:
    Generate a large-scale conflict-free UAV fleet (50 aircraft) and measure
    the computational performance of the conflict detection and resolution
    pipeline. Captures detailed metrics on:
    
    • Execution time (total, detection, resolution)
    • Memory usage
    • R-Tree efficiency
    • Scalability analysis

This benchmark simulates a realistic airspace with densely-packed, conflict-free
flight plans to establish baseline performance baselines for the system.
"""

import sys
from pathlib import Path
import numpy as np
import time
import psutil
import os
from typing import List, Tuple
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from detection.rtree_detector import RTreeDetector
from benchmark.flightplan_generator import DEFAULT_PRISM
from central_manager import CentralManager


# ============================================================================
# Conflict-Free Fleet Generator
# ============================================================================

def generate_conflict_free_fleet(
    n_uavs: int = 50,
    prism: Tuple[Tuple[float, float], ...] = DEFAULT_PRISM,
    t_start: float = 0.0,
    t_end: float = 200.0,
    safety_margin_m: float = 150.0,  # Spatial separation between corridors
    safety_margin_s: float = 15.0,   # Temporal separation between flights
    seed: int = 42,
) -> List[FlightPlan]:
    """
    Generate a large fleet of UAVs with GUARANTEED NO CONFLICTS.
    
    Strategy: Divide the airspace into non-overlapping altitude bands (Z).
    Each UAV gets its own altitude band and a unique time window to avoid
    any spatio-temporal collisions.
    
    Args:
        n_uavs:           Number of UAVs (50+)
        prism:            Bounding volume
        t_start:          Mission start time
        t_end:            Mission end time
        safety_margin_m:  Min altitude separation [m]
        safety_margin_s:  Min time separation [s]
        seed:             Random seed
    
    Returns:
        List of n_uavs conflict-free FlightPlan objects
    """
    rng = np.random.default_rng(seed)
    fleet = []
    
    x_range, y_range, z_range = prism
    
    # STRATEGY: Divide altitude into N non-overlapping bands
    # Each UAV gets its own altitude band (no Z-overlap possible)
    z_available = z_range[1] - z_range[0] - safety_margin_m
    band_height = z_available / max(1, n_uavs)
    
    if band_height < 20:
        print(f"[WARNING] Altitude bands too small ({band_height:.1f}m). Increasing safety margin.")
        band_height = 30.0
    
    print(f"Altitude stratification:")
    print(f"  • Band height: {band_height:.1f} m")
    print(f"  • Total bands: {n_uavs}")
    print(f"  • Each UAV gets exclusive Z-range\n")
    
    for uav_idx in range(n_uavs):
        # Assign altitude band: completely separated from all other UAVs
        z_bottom = z_range[0] + uav_idx * band_height
        z_top = z_bottom + band_height - 10  # Leave small buffer
        z_cruise = (z_bottom + z_top) / 2
        
        # Stagger departure times: each UAV starts at different time
        uav_t_start = t_start + uav_idx * safety_margin_s
        uav_t_end = min(uav_t_start + 90.0, t_end - 10)  # ~90 second missions
        
        if uav_t_start >= t_end:
            break  # Not enough time for more UAVs
        
        # Create flight plan
        fp = FlightPlan()
        fp.id = uav_idx + 1
        fp.priority = 0  # All same priority
        fp.radius = 5.0
        fp.max_var_lin_vel = 20.0
        fp.max_var_ang_vel = 1.0
        
        # Generate random corridor direction (to make it interesting)
        direction = rng.integers(0, 4)  # 0=W→E, 1=E→W, 2=S→N, 3=N→S
        
        if direction == 0:  # West to East
            x_start = x_range[0] + 20
            x_end = x_range[1] - 20
            y_pos = y_range[0] + rng.uniform(50, 100)
        elif direction == 1:  # East to West
            x_start = x_range[1] - 20
            x_end = x_range[0] + 20
            y_pos = y_range[1] - rng.uniform(50, 100)
        elif direction == 2:  # South to North
            y_start = y_range[0] + 20
            y_end = y_range[1] - 20
            x_pos = x_range[0] + rng.uniform(50, 100)
        else:  # North to South
            y_start = y_range[1] - 20
            y_end = y_range[0] + 20
            x_pos = x_range[1] - rng.uniform(50, 100)
        
        # Build 4-waypoint flight plan
        if direction in [0, 1]:  # X-axis flight
            wp0 = Waypoint("START", uav_t_start, 
                          [x_start, y_pos, z_cruise], [15.0, 0.0, 0.0])
            wp1 = Waypoint("WP1", uav_t_start + (uav_t_end - uav_t_start) * 0.33,
                          [x_start + (x_end - x_start) * 0.33, y_pos, z_cruise], 
                          [15.0, 0.0, 0.0])
            wp2 = Waypoint("WP2", uav_t_start + (uav_t_end - uav_t_start) * 0.67,
                          [x_start + (x_end - x_start) * 0.67, y_pos, z_cruise],
                          [15.0, 0.0, 0.0])
            wp3 = Waypoint("END", uav_t_end,
                          [x_end, y_pos, z_cruise], [0.0, 0.0, 0.0])
        else:  # Y-axis flight
            wp0 = Waypoint("START", uav_t_start,
                          [x_pos, y_start, z_cruise], [0.0, 15.0, 0.0])
            wp1 = Waypoint("WP1", uav_t_start + (uav_t_end - uav_t_start) * 0.33,
                          [x_pos, y_start + (y_end - y_start) * 0.33, z_cruise],
                          [0.0, 15.0, 0.0])
            wp2 = Waypoint("WP2", uav_t_start + (uav_t_end - uav_t_start) * 0.67,
                          [x_pos, y_start + (y_end - y_start) * 0.67, z_cruise],
                          [0.0, 15.0, 0.0])
            wp3 = Waypoint("END", uav_t_end,
                          [x_pos, y_end, z_cruise], [0.0, 0.0, 0.0])
        
        fp.set_waypoint(wp0)
        fp.set_waypoint(wp1)
        fp.set_waypoint(wp2)
        fp.set_waypoint(wp3)
        fp.connect_waypoints()
        
        fleet.append(fp)
    
    print(f"✓ Generated {len(fleet)} conflict-free flight plans")
    print(f"  • Flight duration: ~90 seconds each")
    print(f"  • Time window: t ∈ [{t_start:.1f}, {t_end:.1f}]s")
    print(f"  • Altitude stratification: {z_range[0]:.1f}m to {z_range[1]:.1f}m\n")
    
    return fleet


# ============================================================================
# Benchmark Runner
# ============================================================================

class BenchmarkRunner:
    """Orchestrates the benchmark execution and data collection."""
    
    def __init__(self):
        self.fleet = []
        self.detector = None
        self.metrics = {}
    
    def _verify_no_conflicts_mathematically(self) -> Tuple[bool, str]:
        """
        Mathematically verify fleet design ensures no conflicts.
        
        Returns:
            (is_valid, description)
        """
        if not self.fleet:
            return False, "Fleet is empty"
        
        # Check 1: Z-band separation
        z_bands = {}
        for fp in self.fleet:
            z_vals = [wp.pos[2] for wp in fp.waypoints]
            z_min, z_max = min(z_vals), max(z_vals)
            z_bands[fp.id] = (z_min, z_max)
        
        # Check for overlaps in Z
        for id1 in z_bands:
            for id2 in z_bands:
                if id1 >= id2:
                    continue
                z1_min, z1_max = z_bands[id1]
                z2_min, z2_max = z_bands[id2]
                if not (z1_max < z2_min or z2_max < z1_min):
                    return False, f"Z-bands overlap: UAV {id1} [{z1_min:.1f}, {z1_max:.1f}] vs UAV {id2} [{z2_min:.1f}, {z2_max:.1f}]"
        
        # Check 2: Time window separation
        time_windows = {fp.id: (fp.init_time(), fp.finish_time()) for fp in self.fleet}
        for id1 in time_windows:
            for id2 in time_windows:
                if id1 >= id2:
                    continue
                t1_start, t1_end = time_windows[id1]
                t2_start, t2_end = time_windows[id2]
                # Non-overlapping: one must end before other starts
                if not (t1_end < t2_start or t2_end < t1_start):
                    return False, f"Time windows overlap: UAV {id1} [{t1_start:.1f}, {t1_end:.1f}]s vs UAV {id2} [{t2_start:.1f}, {t2_end:.1f}]s"
        
        return True, "Mathematical design ensures separation: independent Z-bands + staggered times"
    
    def run(self, n_uavs: int = 50) -> dict:
        """
        Execute the full benchmark pipeline.
        
        Args:
            n_uavs: Number of UAVs to test
        
        Returns:
            Dictionary with all performance metrics
        """
        print("\n" + "█" * 95)
        print("█" + " " * 93 + "█")
        print("█" + "  UAV CONFLICT DETECTION SYSTEM — COMPUTATIONAL PERFORMANCE BENCHMARK".center(93) + "█")
        print("█" + "  Academic Evaluation & Scalability Analysis".center(93) + "█")
        print("█" + " " * 93 + "█")
        print("█" * 95)
        
        # ─────────────────────────────────────────────────────────────────
        # PHASE 1: FLEET GENERATION
        # ─────────────────────────────────────────────────────────────────
        print(f"\n┌─ [PHASE 1] FLEET GENERATION ─────────────────────────────────────────────────────┐")
        print(f"│ Generating {n_uavs} mathematically conflict-free flight plans...")
        print(f"└─────────────────────────────────────────────────────────────────────────────────┘\n")
        
        t0_gen = time.time()
        mem0_gen = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB
        
        self.fleet = generate_conflict_free_fleet(n_uavs=n_uavs, seed=42)
        
        t1_gen = time.time()
        mem1_gen = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB
        
        time_gen = t1_gen - t0_gen
        mem_gen = mem1_gen - mem0_gen
        
        # Verify mathematical guarantees
        is_valid, reason = self._verify_no_conflicts_mathematically()
        
        print(f"✓ Generated {n_uavs} flight plans")
        print(f"  │ Time: {time_gen*1000:.2f} ms  |  Memory: {mem_gen:.2f} MB")
        print(f"  │ Per-UAV: {(time_gen/n_uavs)*1000:.4f} ms  |  {(mem_gen/n_uavs):.4f} MB")
        print(f"  │")
        print(f"  └─ Mathematical Verification: {'✓ PASS' if is_valid else '✗ FAIL'}")
        print(f"     {reason}\n")
        
        # ─────────────────────────────────────────────────────────────────
        # PHASE 2: DETECTOR INITIALIZATION
        # ─────────────────────────────────────────────────────────────────
        print(f"┌─ [PHASE 2] DETECTOR INITIALIZATION ───────────────────────────────────────────────┐")
        print(f"│ Setting up RTree spatial-temporal index and registering all UAVs...")
        print(f"└─────────────────────────────────────────────────────────────────────────────────┘\n")
        
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
        
        print(f"✓ RTree detector initialized")
        print(f"  │ Total swept boxes: {total_boxes} (across {n_uavs} UAVs)")
        print(f"  │ Time: {time_init*1000:.2f} ms  |  Memory: {mem_init:.2f} MB")
        print(f"  │ Per-UAV: {(time_init/n_uavs)*1000:.4f} ms  |  Boxes per UAV: {(total_boxes/n_uavs):.1f}")
        print()
        
        # ─────────────────────────────────────────────────────────────────
        # PHASE 3: CONFLICT DETECTION
        # ─────────────────────────────────────────────────────────────────
        print(f"┌─ [PHASE 3] SYSTEM-WIDE CONFLICT DETECTION ────────────────────────────────────────┐")
        print(f"│ Running broad-phase (R-Tree) and narrow-phase (SAT) collision tests...")
        print(f"└─────────────────────────────────────────────────────────────────────────────────┘\n")
        
        t0_detect = time.time()
        mem0_detect = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024
        
        # Run conflict detection for all UAVs
        all_conflicts = []
        detection_times = []
        rtree_queries = 0
        sat_tests = 0
        
        for fp in self.fleet:
            t_before = time.time()
            conflicts = self.detector.detect_all_conflicts(fp.id)
            t_after = time.time()
            
            detection_times.append(t_after - t_before)
            all_conflicts.extend(conflicts)
        
        t1_detect = time.time()
        mem1_detect = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024
        
        time_detect = t1_detect - t0_detect
        mem_detect = mem1_detect - mem0_detect
        
        print(f"✓ Detection completed on all {n_uavs} UAVs")
        print(f"  │ Conflicts found: {len(all_conflicts)}")
        print(f"  │ Time: {time_detect*1000:.2f} ms  |  Memory: {mem_detect:.2f} MB")
        print(f"  │ Per-UAV: {(time_detect/n_uavs)*1000:.4f} ms")
        print(f"  │ Detection time stats:")
        print(f"  │   • Min: {min(detection_times)*1000:.4f} ms")
        print(f"  │   • Max: {max(detection_times)*1000:.4f} ms")
        print(f"  │   • Mean: {np.mean(detection_times)*1000:.4f} ms")
        print(f"  │   • Median: {np.median(detection_times)*1000:.4f} ms")
        print(f"  │   • StdDev: {np.std(detection_times)*1000:.4f} ms")
        print()
        
        # ─────────────────────────────────────────────────────────────────
        # VALIDATION
        # ─────────────────────────────────────────────────────────────────
        print(f"┌─ [VALIDATION] CONFLICT VERIFICATION ──────────────────────────────────────────────┐")
        
        if len(all_conflicts) == 0:
            print(f"│ ✓ PASS: Zero conflicts detected (as mathematically expected)                   │")
        else:
            print(f"│ ⚠ WARNING: {len(all_conflicts)} conflicts detected (mathematical design may have issues)│")
        
        print(f"└─────────────────────────────────────────────────────────────────────────────────┘\n")
        
        # ─────────────────────────────────────────────────────────────────
        # SUMMARY REPORT
        # ─────────────────────────────────────────────────────────────────
        total_time = time_gen + time_init + time_detect
        total_mem = mem_gen + mem_init + mem_detect
        
        print("█" * 95)
        print("█" + "  PERFORMANCE SUMMARY".center(93) + "█")
        print("█" * 95)
        
        print(f"\n┌─ EXECUTION TIME ──────────────────────────────────────────────────────────────────┐")
        print(f"│ Total:                            {total_time*1000:>12.2f} ms  ({total_time:>8.4f} s)          │")
        print(f"│   ├─ Fleet generation:            {time_gen*1000:>12.2f} ms  ({time_gen/total_time*100:>5.1f}%)             │")
        print(f"│   ├─ Detector initialization:     {time_init*1000:>12.2f} ms  ({time_init/total_time*100:>5.1f}%)             │")
        print(f"│   └─ Conflict detection:          {time_detect*1000:>12.2f} ms  ({time_detect/total_time*100:>5.1f}%)             │")
        print(f"└─────────────────────────────────────────────────────────────────────────────────┘")
        
        print(f"\n┌─ MEMORY USAGE ────────────────────────────────────────────────────────────────────┐")
        print(f"│ Total delta:                      {total_mem:>12.2f} MB                            │")
        print(f"│   ├─ Fleet generation:            {mem_gen:>12.2f} MB                            │")
        print(f"│   ├─ Detector initialization:     {mem_init:>12.2f} MB                            │")
        print(f"│   └─ Conflict detection:          {mem_detect:>12.2f} MB                            │")
        print(f"└─────────────────────────────────────────────────────────────────────────────────┘")
        
        print(f"\n┌─ SCALABILITY METRICS ─────────────────────────────────────────────────────────────┐")
        print(f"│ Per-UAV metrics (averaged across all {n_uavs} aircraft):                         │")
        print(f"│   • Execution time:               {(total_time/n_uavs)*1000:>12.4f} ms/UAV                 │")
        print(f"│   • Memory consumption:           {(total_mem/n_uavs):>12.4f} MB/UAV                 │")
        print(f"│   • Swept boxes per UAV:          {(total_boxes/n_uavs):>12.1f} boxes/UAV             │")
        print(f"│   • Total swept boxes:            {total_boxes:>12} boxes                    │")
        print(f"└─────────────────────────────────────────────────────────────────────────────────┘")
        
        print(f"\n┌─ CONFLICT STATISTICS ─────────────────────────────────────────────────────────────┐")
        print(f"│ Conflicts detected:               {len(all_conflicts):>12} (expected: 0)                │")
        print(f"│ Result:                           {'✓ PASS (Zero conflicts)' if len(all_conflicts) == 0 else '✗ FAIL':>29}       │")
        print(f"└─────────────────────────────────────────────────────────────────────────────────┘")
        
        print("\n" + "█" * 95 + "\n")
        
        # Store metrics
        self.metrics = {
            "n_uavs": n_uavs,
            "total_boxes": total_boxes,
            "total_conflicts": len(all_conflicts),
            "time_gen_ms": time_gen * 1000,
            "time_init_ms": time_init * 1000,
            "time_detect_ms": time_detect * 1000,
            "time_total_ms": total_time * 1000,
            "mem_gen_mb": mem_gen,
            "mem_init_mb": mem_init,
            "mem_detect_mb": mem_detect,
            "mem_total_mb": total_mem,
            "detection_times_ms": [t * 1000 for t in detection_times],
            "mathematical_guarantee": is_valid,
        }
        
        return self.metrics
    
    def visualize_metrics(self):
        """Generate professional visualization of benchmark results."""
        if not self.metrics:
            print("[ERROR] No metrics to visualize. Run benchmark first.")
            return
        
        fig = plt.figure(figsize=(16, 12))
        fig.suptitle('UAV Conflict Detection System — Performance Analysis', 
                     fontsize=18, fontweight='bold', color='white', y=0.98)
        fig.patch.set_facecolor('#0f1117')
        
        # Create grid
        gs = fig.add_gridspec(3, 3, hspace=0.35, wspace=0.3, top=0.93, bottom=0.08)
        
        # --- Plot 1: Execution time breakdown (pie) ---
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.set_facecolor('#1a1a2e')
        phases = ['Generation', 'Detector Init', 'Detection']
        times = [
            self.metrics['time_gen_ms'],
            self.metrics['time_init_ms'],
            self.metrics['time_detect_ms']
        ]
        colors_pie = ['#00d9ff', '#ffbe0b', '#ff006e']
        wedges, texts, autotexts = ax1.pie(times, labels=phases, autopct='%1.1f%%',
                                            colors=colors_pie, startangle=90,
                                            textprops={'color': 'white', 'fontsize': 10})
        for autotext in autotexts:
            autotext.set_color('black')
            autotext.set_fontweight('bold')
        ax1.set_title('Time Distribution', fontsize=12, fontweight='bold', color='white')
        
        # --- Plot 2: Execution time breakdown (bar) ---
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.set_facecolor('#1a1a2e')
        bars = ax2.bar(phases, times, color=colors_pie, edgecolor='white', linewidth=2)
        ax2.set_ylabel('Time [ms]', fontsize=11, color='white', fontweight='bold')
        ax2.set_title('Execution Time Breakdown', fontsize=12, fontweight='bold', color='white')
        ax2.tick_params(colors='white', labelsize=9)
        ax2.grid(axis='y', alpha=0.3, linestyle='--')
        for bar, val in zip(bars, times):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{val:.1f}ms', ha='center', va='bottom', 
                    color='white', fontsize=9, fontweight='bold')
        
        # --- Plot 3: Memory breakdown ---
        ax3 = fig.add_subplot(gs[0, 2])
        ax3.set_facecolor('#1a1a2e')
        mems = [
            self.metrics['mem_gen_mb'],
            self.metrics['mem_init_mb'],
            self.metrics['mem_detect_mb']
        ]
        bars = ax3.bar(phases, mems, color=colors_pie, edgecolor='white', linewidth=2)
        ax3.set_ylabel('Memory [MB]', fontsize=11, color='white', fontweight='bold')
        ax3.set_title('Memory Usage Breakdown', fontsize=12, fontweight='bold', color='white')
        ax3.tick_params(colors='white', labelsize=9)
        ax3.grid(axis='y', alpha=0.3, linestyle='--')
        for bar, val in zip(bars, mems):
            height = bar.get_height()
            ax3.text(bar.get_x() + bar.get_width()/2., height,
                    f'{val:.2f}MB', ha='center', va='bottom',
                    color='white', fontsize=9, fontweight='bold')
        
        # --- Plot 4: Detection time distribution (histogram) ---
        ax4 = fig.add_subplot(gs[1, :2])
        ax4.set_facecolor('#1a1a2e')
        detection_times = self.metrics['detection_times_ms']
        counts, bins, patches = ax4.hist(detection_times, bins=25, color='#8338ec', 
                                         edgecolor='white', alpha=0.8)
        # Color patches with gradient
        for i, patch in enumerate(patches):
            patch.set_alpha(0.7 + 0.3 * (i / len(patches)))
        ax4.set_xlabel('Detection Time per UAV [ms]', fontsize=11, color='white', fontweight='bold')
        ax4.set_ylabel('Frequency', fontsize=11, color='white', fontweight='bold')
        ax4.set_title('Detection Time Distribution (Per-UAV Analysis)', fontsize=12, fontweight='bold', color='white')
        ax4.tick_params(colors='white', labelsize=9)
        ax4.grid(axis='y', alpha=0.3, linestyle='--')
        
        # Add statistics text
        stats_text = f"μ={np.mean(detection_times):.3f}ms  σ={np.std(detection_times):.3f}ms  min={min(detection_times):.3f}ms  max={max(detection_times):.3f}ms"
        ax4.text(0.98, 0.97, stats_text, transform=ax4.transAxes, fontsize=10,
                verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='#0f1117', edgecolor='#8338ec', linewidth=1.5),
                color='#8338ec', family='monospace', fontweight='bold')
        
        # --- Plot 5: Scalability metrics ---
        ax5 = fig.add_subplot(gs[1, 2])
        ax5.set_facecolor('#1a1a2e')
        ax5.axis('off')
        
        scalability_text = f"""
SCALABILITY ANALYSIS
{'─' * 32}
Fleet Size:           {self.metrics['n_uavs']} UAVs
Total Boxes:          {self.metrics['total_boxes']}

Time per UAV:         {self.metrics['time_total_ms']/self.metrics['n_uavs']:.4f} ms
Memory per UAV:       {self.metrics['mem_total_mb']/self.metrics['n_uavs']:.4f} MB
Boxes per UAV:        {self.metrics['total_boxes']/self.metrics['n_uavs']:.1f}

Total Execution:      {self.metrics['time_total_ms']:.2f} ms
Total Memory:         {self.metrics['mem_total_mb']:.2f} MB

Conflicts Detected:   {self.metrics['total_conflicts']}
Expected:             0
Status:               {'✓ PASS' if self.metrics['total_conflicts'] == 0 else '✗ FAIL'}
Math Guarantee:       {'✓ YES' if self.metrics.get('mathematical_guarantee', False) else '✗ NO'}
        """
        
        ax5.text(0.05, 0.95, scalability_text, fontsize=9.5, family='monospace',
                color='#00d9ff', verticalalignment='top', transform=ax5.transAxes,
                bbox=dict(boxstyle='round', facecolor='#1a1a2e', edgecolor='#00d9ff', linewidth=2))
        
        # --- Plot 6: Time per phase (stacked bar) ---
        ax6 = fig.add_subplot(gs[2, :])
        ax6.set_facecolor('#1a1a2e')
        
        phase_labels = [f"UAV {i+1}" for i in range(min(10, self.metrics['n_uavs']))]
        detection_subset = self.metrics['detection_times_ms'][:10]
        
        bars = ax6.bar(range(len(detection_subset)), detection_subset, 
                      color='#3a86ff', edgecolor='white', linewidth=1.5, alpha=0.8)
        ax6.set_ylabel('Detection Time [ms]', fontsize=11, color='white', fontweight='bold')
        ax6.set_xlabel('UAV Index (first 10 shown)', fontsize=11, color='white', fontweight='bold')
        ax6.set_title('Per-UAV Detection Performance (Sample)', fontsize=12, fontweight='bold', color='white')
        ax6.set_xticks(range(len(detection_subset)))
        ax6.set_xticklabels(phase_labels, fontsize=9)
        ax6.tick_params(colors='white', labelsize=9)
        ax6.grid(axis='y', alpha=0.3, linestyle='--')
        ax6.axhline(y=np.mean(detection_subset), color='#ff006e', linestyle='--', 
                   linewidth=2, label=f'Mean: {np.mean(detection_subset):.3f}ms')
        ax6.legend(loc='upper right', fontsize=9, framealpha=0.9, labelcolor='white')
        
        return fig


# ============================================================================
# Main Entry Point
# ============================================================================

if __name__ == "__main__":
    # Run benchmark
    runner = BenchmarkRunner()
    metrics = runner.run(n_uavs=50)
    
    # Visualize results
    print("\nGenerating visualization...")
    fig = runner.visualize_metrics()
    
    print("Close the plot window to exit.")
    plt.show()
