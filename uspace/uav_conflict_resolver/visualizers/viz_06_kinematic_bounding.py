"""
viz_06_kinematic_bounding.py — Visualizer 6: Kinematic Bounding (Strategy 1)
========================================================================

PURPOSE:
    Demonstrate Strategy 1 (ConnectURM2) which resolves conflicts by modifying 
    only the speed (accelerate or brake) along the original straight route 
    and propagating timestamps (Ripple Effect).

VISUALIZATION:
    Displays a 2D Time vs Distance plot showing:
      - The conflict intersection point.
      - The original speed profile leading to conflict.
      - The newly adjusted speed profile avoiding the VIP.
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from central_manager import CentralManager
from visualizers.flightplan_generator import generate_crossing_pair

def run_visualizer():
    print("\n" + "="*60)
    print("VISUALIZER 6: Strategy 1 (ConnectURM2 / Kinematic)")
    print("="*60)
    
    # 1. Generate Crossing Pair
    fp1, fp2 = generate_crossing_pair(seed=12) # Same alt crossing
    fp1.priority = 10
    fp2.priority = 5

    # Original trace distance
    trace1 = fp1.trace(0.5)
    trace2 = fp2.trace(0.5)
    
    # Distance from start for UAV 2 (Original)
    dist_2_orig = np.linalg.norm(trace2[:, 1:4] - trace2[0, 1:4], axis=1)
    time_2_orig = trace2[:, 0]

    # Time vs Distance for UAV 1
    dist_1 = np.linalg.norm(trace1[:, 1:4] - trace2[0, 1:4], axis=1) # relative to uav2 start for common ref
    time_1 = trace1[:, 0]

    # 2. Resolve via Central Manager
    manager = CentralManager()
    manager.register_uav("UAV_1", fp1)
    manager.register_uav("UAV_2", fp2)
    # Set up the priorities
    manager._priorities["UAV_1"] = 10
    manager._priorities["UAV_2"] = 5

    # Access internal detector to display conflict information
    conflicts = manager._rtree_detector.detect_all_conflicts("UAV_2")
    
    if not conflicts:
        print("No conflict generated.")
        return

    # Use public API to resolve
    result = manager.check_and_resolve("UAV_2")
    if result is None or not result.success:
        print("Resolution failed or no conflict to resolve.")
        return
        
    fp2_safe = manager.get_flight_plan("UAV_2")

    # Safe trace distance for UAV 2
    trace2_safe = fp2_safe.trace(0.5)
    dist_2_safe = np.linalg.norm(trace2_safe[:, 1:4] - trace2_safe[0, 1:4], axis=1)
    time_2_safe = trace2_safe[:, 0]

    # 3. Plotting
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111)
    ax.set_title("Strategy 1: Kinematic Bounding via ConnectURM2", fontsize=14, pad=20)

    # Plot continuous trajectory timing
    ax.plot(time_2_orig, dist_2_orig, color='red', linewidth=2, linestyle='--', label="Plebeian (Original Plan)")
    ax.plot(time_2_safe, dist_2_safe, color='green', linewidth=3, label="Plebeian (Safe S1 Plan)")
    
    # Conflict zone highlighter (approximate)
    c_t = conflicts[0]['time_range']
    ax.axvspan(c_t[0], c_t[1], color='orange', alpha=0.3, label='Temporal Conflict Window')
    
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance traveled from start (m)")
    ax.legend()
    ax.grid(True, linestyle=':', alpha=0.7)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_visualizer()
