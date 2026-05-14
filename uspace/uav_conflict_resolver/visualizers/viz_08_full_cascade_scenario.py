"""
viz_08_full_cascade_scenario.py — Visualizer 8: Full Multi-UAV Cascade Matrix
=============================================================================

Demonstrates the CentralManager resolving a chaotic multi-UAV scenario
generated via the realistic 7D flightplan generator.
"""

import sys
import logging
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.waypoint import Waypoint
from core.models.flight_plan import FlightPlan
from central_manager import CentralManager

def generate_straight_fleet(n_uavs: int = 5) -> list[FlightPlan]:
    """
    Generates an array 'SPACE' of 5 straight-line flight plans that intentionally
    collide or cross very close to the center (200, 200, 100) at approximately t=20s.
    The distances and speeds are scaled to respect the 20 m/s physical limits.
    """
    SPACE = []
    
    # (start_pos, mid_pos, end_pos, speed)
    routes = [
        ([0, 200, 100],   [200, 200, 100], [400, 200, 0], 10.0),       # W -> E
        ([200, 0, 100],   [200, 200, 100], [200, 400, 200], 10.0),       # S -> N
        ([0, 0, 100],     [200, 200, 100], [400, 400, 100], 14.14),      # SW -> NE
        ([400, 0, 0],   [200, 200, 100], [0, 400, 200],   14.14),      # SE -> NW
        ([200, 200, 200], [200, 200, 100], [200, 200, 0],   5.0)         # Drop down
    ]
    
    for i in range(min(n_uavs, 5)):
        start_pos, mid_pos, end_pos, speed = routes[i]
        
        # Mid waypoint exactly at t = 20s
        dist_to_mid = np.linalg.norm(np.array(mid_pos) - np.array(start_pos))
        t_start = 20.0 - (dist_to_mid / speed)
        
        dist_from_mid = np.linalg.norm(np.array(end_pos) - np.array(mid_pos))
        t_end = 20.0 + (dist_from_mid / speed)
        
        fp = FlightPlan()
        fp.id = i + 1
        fp.priority = max(2 - i, 0)
        
        fp.set_waypoint(Waypoint("start", round(t_start, 2), start_pos, [0,0,0]))
        fp.set_waypoint(Waypoint("mid", 20.0, mid_pos, [0,0,0]))
        fp.set_waypoint(Waypoint("end", round(t_end, 2), end_pos, [0,0,0]))
        
        fp.max_var_lin_vel = 10.0
        fp.max_var_ang_vel = 2.0
        fp.set_uniform_velocity()
        fp.connect_waypoints()
        
        SPACE.append(fp)
        
    return SPACE

BG = "#0f1117"

def run_visualizer():
    # Configure logging to show only Strategy 1 and Manager logs (silence noisy modules)
    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)-8s [%(name)s] %(message)s'
    )
    
    # Silence noisy modules
    logging.getLogger('matplotlib').setLevel(logging.WARNING)
    logging.getLogger('urllib').setLevel(logging.WARNING)
    logging.getLogger('PIL').setLevel(logging.WARNING)
    
    # Enable DEBUG for our modules
    logging.getLogger('resolution.kinematic.velocity_bounding').setLevel(logging.DEBUG)
    logging.getLogger('central_manager').setLevel(logging.DEBUG)
    
    print("\n" + "=" * 60)
    print("  VISUALIZER 8: Chaotic Multi-UAV Cascade Matrix")
    print("  [Strategy 1 (Kinematic Bounding) DEBUG enabled]")
    print("=" * 60)

    # 1. Generate Fleet
    print("\n[1] Generating SPACE (5 straight-line colliding UAVs)...")
    SPACE = generate_straight_fleet(n_uavs=5)
    
    # Map of original plans
    original_plans = {f"UAV_{i+1}": fp.copy() for i, fp in enumerate(SPACE)}

    # 2. Register in Central Manager
    print("\n[2] Registering in CentralManager and resolving conflicts...")
    manager = CentralManager()
    
    for i, fp in enumerate(SPACE):
        uav_id = f"UAV_{i+1}"
        priority = max(2 - i, 0)
        print(f"  Registering {uav_id} (Priority {priority})...")
        manager.register_uav(uav_id, fp, priority=priority)
        
        orig_fp = original_plans[uav_id]
        
        result = manager.check_and_resolve(uav_id)
        if result is not None:
            if result.success:
                print(f"    -> [SUCCESS] Resolved via {result.strategy_used}")
                print(f"       Detail: {result.message}")
                
                # Check how Kinematic Bounding modified the time
                if result.strategy_used == "S1":
                    new_fp = result.new_fp_pleb
                    for j, (wp_old, wp_new) in enumerate(zip(orig_fp.waypoints, new_fp.waypoints)):
                        diff = wp_new.t - wp_old.t
                        if abs(diff) > 0.01:
                            print(f"       [S1 Action] Shifted WP[{j}] '{wp_old.label}' by {diff:+.1f}s (t={wp_old.t:.1f}s -> {wp_new.t:.1f}s)")
            else:
                print(f"    -> [DEADLOCK] {result.message}")

    # 3. Retrieve resolved plans
    resolved_plans = {uid: manager.get_flight_plan(uid) for uid in original_plans.keys()}

    # 3.5 Verification of secondary conflicts
    print("\n[3] Final System Verification...")
    total_conflicts = 0
    for uid in resolved_plans.keys():
        confs = manager._rtree_detector.detect_all_conflicts(uid)
        total_conflicts += len(confs)
    print(f"  Total secondary/unresolved conflicts across all UAVs: {total_conflicts}")
    if total_conflicts == 0:
        print("  [OK] MATHEMATICAL PROOF: The airspace is 100% collision-free.")
        print("       No secondary conflicts were created by the kinematic shifts.")
    else:
        print("  [X] WARNING: Secondary conflicts detected!")

    # 4. Visualization (Beautiful Before/After like viz_04)
    print("\n[4] Rendering 3D Visualization...")
    fig = plt.figure(figsize=(14, 7), facecolor=BG)
    fig.canvas.manager.set_window_title("UAV Conflict Resolver — Full Cascade Matrix")
    fig.text(0.5, 0.95, "Chaotic Multi-UAV Conflict Resolution Matrix", 
             ha="center", va="center", fontsize=15, fontweight="bold", color="white")

    colors = ["#2ecc71", "#e74c3c", "#3498db", "#f1c40f", "#9b59b6"]

    for col, title, plan_dict in [
        (0, "BEFORE (Original Generated Fleet)", original_plans),
        (1, "AFTER (Resolved via Cascade)", resolved_plans),
    ]:
        ax = fig.add_subplot(1, 2, col + 1, projection="3d")
        ax.set_facecolor(BG)
        ax.set_title(title, color="white", fontsize=11, pad=10)

        for i, (uid, fp) in enumerate(plan_dict.items()):
            if fp is None:
                continue
            
            trace = fp.trace(0.1)
            # Colors for 5 drones
            colors_list = ["#2ecc71", "#e74c3c", "#3498db", "#f1c40f", "#9b59b6"]
            color = colors_list[i % len(colors_list)]
            linestyle = "-" if col == 0 else ("-" if not hasattr(fp, "resolved") else "--")
            
            ax.plot(trace[:, 1], trace[:, 2], trace[:, 3],
                    color=color, linewidth=1.5, linestyle=linestyle, alpha=0.8)

            # Start and End markers (no waypoints to avoid extreme clutter)
            ax.scatter(*trace[0, 1:4], color=color, s=20, marker="o", zorder=5)
            ax.scatter(*trace[-1, 1:4], color=color, s=20, marker="^", zorder=5)

        ax.set_xlabel("X (m)", color="white", labelpad=4)
        ax.set_ylabel("Y (m)", color="white", labelpad=4)
        ax.set_zlabel("Z (m)", color="white", labelpad=4)
        ax.tick_params(colors="white")
        
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor("#2a2d3a")

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    run_visualizer()
