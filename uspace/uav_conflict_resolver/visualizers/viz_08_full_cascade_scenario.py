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


def _heading_from_velocity(velocity: np.ndarray) -> list[float]:
    velocity_xy = np.array(velocity[:2], dtype=float)
    if np.linalg.norm(velocity_xy) < 1e-9:
        return [0, 0]
    return velocity_xy.tolist()


def _sync_headings_to_velocity(flight_plan: FlightPlan) -> None:
    for waypoint in flight_plan.waypoints:
        waypoint.heading = _heading_from_velocity(waypoint.vel)




def generate_straight_fleet(n_uavs: int = 5) -> list[FlightPlan]:
    """
    Generates an array 'SPACE' of 5 straight-line flight plans that intentionally
    collide or cross very close to the center (200, 200, 100) at approximately t=20s.
    The distances and speeds are scaled to respect the 20 m/s physical limits.
    """
    SPACE = []
    
    # (start_pos, mid_pos, end_pos, speed)
    routes = [
        ([0, 200, 50],   [200, 200, 50], [400, 200, 50], 10.0),       # W -> E
        ([200, 0, 50],   [200, 200, 50], [200, 400, 50], 10.0),       # S -> N
        ([0, 0, 50],     [200, 200, 50], [400, 400, 50], 10.0),       # SW -> NE
        ([400, 0, 50],   [200, 200, 50], [0, 400, 50],   10.0),       # SE -> NW
        ([200, 200, 100], [200, 200, 50], [200, 200, 5],   5.0),        # Drop down
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
        fp.connect_waypoints(strict=True)
        _sync_headings_to_velocity(fp)
        
        SPACE.append(fp)
        
    return SPACE

BG = "#0f1117"
MID_POS = np.array([200.0, 200.0, 100.0])
MID_TIME = 20.0

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

    # 1.5 Create reversed routes: same routes but reversed (start where the other ends)
    print("\n[1.5] Creating 5 reversed routes (same but reversed endpoints)...")
    num_orig = len(SPACE)
    for i in range(num_orig):
        orig_fp = SPACE[i]

        # Prefer labelled 'mid' waypoint when present
        mid_idx = orig_fp.get_index_from_label("mid")
        if mid_idx is None:
            mid_idx = 1 if len(orig_fp.waypoints) >= 3 else max(0, len(orig_fp.waypoints) // 2)

        wp_start = orig_fp.waypoints[0]
        wp_mid = orig_fp.waypoints[mid_idx]
        wp_end = orig_fp.waypoints[-1]

        # Durations in the original direction
        d1 = wp_mid.t - wp_start.t
        d2 = wp_end.t - wp_mid.t

        # Reversed timings so mid stays at the same absolute time (e.g. t=20)
        t_start_rev = round(wp_mid.t - d2, 2)
        t_end_rev = round(wp_mid.t + d1, 2)

        # Detect purely vertical routes (same X,Y but different Z) and skip
        # creating the reversed counterpart to avoid an UAV that goes from
        # down->up in Z (the user requested to remove that drone).
        try:
            start_pos_arr = np.array(wp_start.pos, dtype=float)
            end_pos_arr = np.array(wp_end.pos, dtype=float)
            # If X,Y equal (within tolerance) and Z differs, consider vertical
            if np.allclose(start_pos_arr[:2], end_pos_arr[:2], atol=1e-6) and abs(end_pos_arr[2] - start_pos_arr[2]) > 1e-6:
                print(f"    Skipping reversed vertical route for UAV_{i+1} (vertical-only)")
                continue
        except Exception:
            # If any positional data is unexpected, fall back to creating the route
            pass

        new_fp = FlightPlan()
        new_id = num_orig + i + 1
        new_fp.id = new_id
        new_fp.priority = orig_fp.priority

        def _pos(wp):
            try:
                return wp.pos.tolist()
            except Exception:
                return wp.pos

        new_fp.set_waypoint(Waypoint("start", t_start_rev, _pos(wp_end), [0, 0, 0]))
        new_fp.set_waypoint(Waypoint("mid", wp_mid.t, _pos(wp_mid), [0, 0, 0]))
        new_fp.set_waypoint(Waypoint("end", t_end_rev, _pos(wp_start), [0, 0, 0]))

        new_fp.max_var_lin_vel = orig_fp.max_var_lin_vel
        new_fp.max_var_ang_vel = orig_fp.max_var_ang_vel
        new_fp.set_uniform_velocity()
        new_fp.connect_waypoints()
        _sync_headings_to_velocity(new_fp)

        # Append to SPACE and to the mapping so registration loop can access them
        SPACE.append(new_fp)
        original_plans[f"UAV_{new_id}"] = new_fp.copy()

    

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

    # 2.5 Keep resolving until the system stabilizes or we hit a safety cap.
    # A single successful resolution only guarantees that one local conflict
    # was repaired; new secondary conflicts can still appear elsewhere.
    print("\n[2.5] Global stabilization sweep...")
    max_sweeps = 300
    for sweep_idx in range(max_sweeps):
        conflicts = manager._rtree_detector.detect_all_conflicts_system_wide()
        if not conflicts:
            print(f"  Sweep {sweep_idx + 1}: no conflicts detected.")
            break

        print(f"  Sweep {sweep_idx + 1}: {len(conflicts)} unique conflicts")
        sweep_results = manager.check_and_resolve_all()

        if not sweep_results:
            print(f"  Sweep {sweep_idx + 1}: no successful resolutions in this pass.")
            break

        for uid, sweep_result in sweep_results.items():
            if sweep_result is None:
                continue
            if sweep_result.success:
                print(f"    -> {uid}: {sweep_result.strategy_used} ({sweep_result.message})")
            else:
                print(f"    -> {uid}: DEADLOCK ({sweep_result.message})")

    # Refresh the original snapshot after the stabilization sweep so the
    # "AFTER" view reflects the latest committed plans.
    resolved_plans = {uid: manager.get_flight_plan(uid) for uid in original_plans.keys()}

    # 3.5 Verification of secondary conflicts
    print("\n[3] Final System Verification...")
    total_conflicts = manager._rtree_detector.detect_all_conflicts_system_wide()
    print(f"  Total unique secondary/unresolved conflicts across all UAVs: {len(total_conflicts)}")
    if len(total_conflicts) == 0:
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

    colors = [
        "#2ecc71", "#e74c3c", "#3498db", "#f1c40f", "#9b59b6",
        "#1abc9c", "#e67e22", "#34495e", "#7f8c8d", "#16a085"
    ]

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
            # Colors list for drones (supports up to 10)
            colors_list = [
                "#2ecc71", "#e74c3c", "#3498db", "#f1c40f", "#9b59b6",
                "#1abc9c", "#e67e22", "#34495e", "#7f8c8d", "#16a085"
            ]
            color = colors_list[i % len(colors_list)]
            linestyle = "-" if col == 0 else ("-" if not hasattr(fp, "resolved") else "--")

            # Apply tiny vertical offset per UAV index to avoid exact overlap hiding traces
            z_offset = i * 0.1
            z_plot = trace[:, 3] + z_offset
            ax.plot(trace[:, 1], trace[:, 2], z_plot,
                    color=color, linewidth=1.5, linestyle=linestyle, alpha=0.9)

            # Start and End markers (no waypoints to avoid extreme clutter)
            ax.scatter(trace[0, 1], trace[0, 2], trace[0, 3] + z_offset, color=color, s=20, marker="o", zorder=5)
            ax.scatter(trace[-1, 1], trace[-1, 2], trace[-1, 3] + z_offset, color=color, s=20, marker="^", zorder=5)

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