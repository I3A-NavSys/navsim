"""
viz_08_full_cascade_scenario.py — Visualizer 8: Full Multi-UAV Cascade Matrix
=============================================================================

Demonstrates the CentralManager resolving a chaotic multi-UAV scenario
generated via the realistic 7D flightplan generator.
"""

import sys
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
    Generates an array 'SPACE'of 5 straight-line flight plans that intentionally
    collide or cross very close to the center (1000, 1000, 100) at approximately t=20s.
    Each includes a mid waypoint to trigger the C2 continuous motion solver.
    """
    SPACE = []
    
    # (start_pos, mid_pos, end_pos, speed)
    routes = [
        ([0, 1000, 100], [1000, 1000, 100], [2000, 1000, 100], 50.0),       # W -> E
        ([1000, 0, 100], [1000, 1000, 100], [1000, 2000, 100], 50.0),       # S -> N
        ([0, 0, 100], [1000, 1000, 100], [2000, 2000, 100], 70.71),         # SW -> NE
        ([2000, 0, 100], [1000, 1000, 100], [0, 2000, 100], 70.71),         # SE -> NW
        ([1000, 1000, 200], [1000, 1000, 100], [1000, 1000, 0], 5.0)        # Drop down
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
        fp.priority = 10 - i
        
        fp.set_waypoint(Waypoint("start", round(t_start, 2), start_pos, [0,0,0]))
        fp.set_waypoint(Waypoint("mid", 20.0, mid_pos, [0,0,0]))
        fp.set_waypoint(Waypoint("end", round(t_end, 2), end_pos, [0,0,0]))
        
        fp.max_var_lin_vel = 80.0
        fp.max_var_ang_vel = 2.0
        fp.set_uniform_velocity()
        fp.connect_waypoints()
        
        SPACE.append(fp)
        
    return SPACE

BG = "#0f1117"

def run_visualizer():
    print("\n" + "=" * 60)
    print("  VISUALIZER 8: Chaotic Multi-UAV Cascade Matrix")
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
        print(f"  Registering {uav_id} (Priority {10 - i})...")
        manager.register_uav(uav_id, fp, priority=10 - i)
        
        result = manager.check_and_resolve(uav_id)
        if result is not None:
            if result.success:
                print(f"    -> [SUCCESS] Resolved via {result.strategy_used}")
            else:
                print(f"    -> [DEADLOCK] {result.message}")

    # 3. Retrieve resolved plans
    resolved_plans = {uid: manager.get_flight_plan(uid) for uid in original_plans.keys()}

    # 4. Visualization (Beautiful Before/After like viz_04)
    print("\n[3] Rendering 3D Visualization...")
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
            
            trace = fp.trace(0.5)
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
