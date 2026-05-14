"""
viz_06_kinematic_bounding.py — Visualizer 6: Kinematic Bounding (Strategy 1)
========================================================================

PURPOSE:
    Demonstrate Strategy 1 (Kinematic Bounding) which resolves conflicts by 
    shifting the timestamp of the waypoint closest to the conflict zone.
    The trajectory is then re-interpolated, accelerating or braking the UAV
    without changing its spatial route.

VISUALIZATION:
    Displays a 2D Time vs Distance plot showing:
      - The conflict temporal window.
      - The original speed profile leading to conflict.
      - The newly adjusted speed profile safely avoiding the VIP.
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from central_manager import CentralManager
from benchmark.flightplan_generator import generate_crossing_pair

def run_visualizer():
    print("\n" + "="*70)
    print(" VISUALIZER 6: Strategy 1 (Kinematic Bounding via Time-Shift) ")
    print("="*70)
    
    # 1. Generate Crossing Pair
    # seed=12 reliably generates a crossing conflict between the two UAVs
    fp1, fp2 = generate_crossing_pair(seed=12) 
    fp1.priority = 2
    fp2.priority = 0

    # Extract original traces
    trace1 = fp1.trace(0.5)
    trace2 = fp2.trace(0.5)
    
    # Distance from start for UAV 2 (Original Plebeian)
    dist_2_orig = np.linalg.norm(trace2[:, 1:4] - trace2[0, 1:4], axis=1)
    time_2_orig = trace2[:, 0]

    # Distance for UAV 1 (VIP) relative to Plebeian start
    dist_1 = np.linalg.norm(trace1[:, 1:4] - trace2[0, 1:4], axis=1)
    time_1 = trace1[:, 0]

    # 2. Resolve via Central Manager
    manager = CentralManager()
    manager.register_uav("UAV_1", fp1)
    manager.register_uav("UAV_2", fp2)
    
    manager._priorities["UAV_1"] = 2
    manager._priorities["UAV_2"] = 0

    # Check for initial conflict
    conflicts = manager._rtree_detector.detect_all_conflicts("UAV_2")
    
    if not conflicts:
        print(" [!] No conflict generated. Try a different seed.")
        return

    c_t = conflicts[0]['time_range']
    print(f" [*] Conflict detected in temporal window: [{c_t[0]:.2f}s, {c_t[1]:.2f}s]")

    # Resolve
    print(" [*] Resolving via Central Manager...")
    result = manager.check_and_resolve("UAV_2")
    
    if result is None or not result.success:
        print(" [!] Resolution failed or no conflict to resolve.")
        return
        
    print(f" [+] Success! Conflict resolved using: {result.strategy_used}")
    
    fp2_safe = manager.get_flight_plan("UAV_2")

    # Extract safe traces
    trace2_safe = fp2_safe.trace(0.5)
    dist_2_safe = np.linalg.norm(trace2_safe[:, 1:4] - trace2_safe[0, 1:4], axis=1)
    time_2_safe = trace2_safe[:, 0]

    # 3. Plotting Setup (Professional Dark Theme)
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(12, 7))
    ax = fig.add_subplot(111)
    
    # Configure aesthetics
    ax.set_facecolor('#1e1e1e')
    fig.patch.set_facecolor('#1e1e1e')
    ax.grid(color='#333333', linestyle='--', linewidth=0.8, alpha=0.7)
    for spine in ax.spines.values():
        spine.set_color('#555555')

    ax.set_title("Strategy 1: Kinematic Bounding (Time-Shift)", 
                 fontsize=16, pad=20, color='white', fontweight='bold')

    # Plot trajectories
    ax.plot(time_2_orig, dist_2_orig, color='#ff5555', linewidth=3, linestyle='--', 
            alpha=0.7, label="Plebeian (Original Collision Path)")
            
    ax.plot(time_2_safe, dist_2_safe, color='#55ff55', linewidth=3.5, 
            label="Plebeian (Safe Time-Shifted Path)")
            
    # Highlight the conflict temporal zone
    ax.axvspan(c_t[0], c_t[1], color='#ffaa00', alpha=0.2, 
               label=f'Conflict Temporal Window ({c_t[0]:.1f}s - {c_t[1]:.1f}s)')
    
    # Mark the start and end of the safe trajectory
    ax.scatter([time_2_safe[0], time_2_safe[-1]], [dist_2_safe[0], dist_2_safe[-1]], 
               color='#55ff55', s=80, zorder=5)

    # Labels and Legend
    ax.set_xlabel("Time (seconds)", fontsize=12, color='#cccccc')
    ax.set_ylabel("Distance Traveled from Origin (meters)", fontsize=12, color='#cccccc')
    ax.tick_params(colors='#aaaaaa', labelsize=10)
    
    legend = ax.legend(loc='upper left', fontsize=11, framealpha=0.9, facecolor='#2d2d2d')
    for text in legend.get_texts():
        text.set_color('white')

    # Add explanatory text box
    info_text = (
        "Kinematic Bounding Mechanism:\n\n"
        "1. Detect temporal collision window.\n"
        "2. Identify the nearest waypoint to the conflict.\n"
        "3. Shift its timestamp by ±5s or ±10s.\n"
        "4. Recompute trajectory quintic polynomials.\n"
        "Result: UAV accelerates or brakes, shifting its\n"
        "arrival time without altering the spatial route."
    )
    props = dict(boxstyle='round,pad=0.8', facecolor='#2d2d2d', alpha=0.9, edgecolor='#555555')
    ax.text(0.95, 0.05, info_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='bottom', horizontalalignment='right', 
            color='#eeeeee', bbox=props, family='monospace')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_visualizer()
