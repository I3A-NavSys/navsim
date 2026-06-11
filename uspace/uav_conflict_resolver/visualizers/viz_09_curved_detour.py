import sys
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from core.config import ANCHOR_DELTA, RIGID_SHIFT_MTV_SCALE, EPSILON_ABSOLUTE
from detection.rtree_detector import RTreeDetector
from resolution.geometry.sat_mtv import generate_mtv_candidates
from resolution.geometry.path_geometry import build_rigid_shift_detour

BG = "#0f1117"

def generate_obb_faces(corners):
    """Generate the 6 faces of an OBB from its 8 corners for 3D plotting."""
    corners = np.array(corners)
    faces = [
        [corners[0], corners[1], corners[3], corners[2]], # Bottom
        [corners[4], corners[5], corners[7], corners[6]], # Top
        [corners[0], corners[1], corners[5], corners[4]], # Front
        [corners[2], corners[3], corners[7], corners[6]], # Back
        [corners[0], corners[2], corners[6], corners[4]], # Left
        [corners[1], corners[3], corners[7], corners[5]], # Right
    ]
    return faces

def run_visualizer():
    print("\n" + "=" * 60)
    print("  VISUALIZER 9: Strategy 2 on TWO CROSSING CURVED paths")
    print("=" * 60)

    # ── 1. Create TWO Flight Plans: CURVED (plebeian) + CURVED (VIP) ──
    print("Generating CURVED flight plan #1 (plebeian)...")
    fp_curved = FlightPlan()
    fp_curved.id = 1
    fp_curved.priority = 0  # Plebeian
    fp_curved.radius = 5.0  # Larger collision radius
    fp_curved.max_var_lin_vel = 15.0
    fp_curved.max_var_ang_vel = 1.0

    # Trajectory: Curves through (150, 100). We extend it at both ends (X=-100 and Y=350)
    # to provide ample straight flight before and after the conflict for clean anchor/return.
    wp0 = Waypoint("START_1", 0.0, [-100.0, 100.0, 100.0], [10.0, 0.0, 0])
    wp1 = Waypoint("CRUISE_1", 10.0, [0.0, 100.0, 100.0], [10.0, 0.0, 0])
    wp2 = Waypoint("TURN_1", 25.0, [150.0, 100.0, 100.0], [0.0, 10.0, 0])
    wp3 = Waypoint("CRUISE_2", 40.0, [150.0, 250.0, 100.0], [0.0, 10.0, 0])
    wp4 = Waypoint("END_1", 50.0, [150.0, 350.0, 100.0], [0.0, 10.0, 0])
    
    fp_curved.set_waypoint(wp0)
    fp_curved.set_waypoint(wp1)
    fp_curved.set_waypoint(wp2)
    fp_curved.set_waypoint(wp3)
    fp_curved.set_waypoint(wp4)
    fp_curved.connect_waypoints()

    # Create a SECOND curved trajectory that crosses the first
    print("Generating CURVED flight plan #2 (VIP)...")
    fp_vip = FlightPlan()
    fp_vip.id = 2
    fp_vip.priority = 10  # VIP
    fp_vip.radius = 5.0  # Larger collision radius
    fp_vip.max_var_lin_vel = 15.0
    fp_vip.max_var_ang_vel = 1.0

    # Trajectory: curves from West to South, passing through same center point
    # Offset adjusted (+10s) so they still perfectly cross at (150, 100) around t=25-30
    vip0 = Waypoint("START_2", 15.0, [300.0, 100.0, 100.0], [-10.0, 0.0, 0])
    vip1 = Waypoint("TURN_2", 30.0, [150.0, 100.0, 100.0], [0.0, -10.0, 0])
    vip2 = Waypoint("END_2", 45.0, [150.0, -50.0, 100.0], [0.0, -10.0, 0])
    
    fp_vip.set_waypoint(vip0)
    fp_vip.set_waypoint(vip1)
    fp_vip.set_waypoint(vip2)
    fp_vip.connect_waypoints()

    # ── 2. Register both in RTreeDetector and detect conflicts ─────────
    print("Registering in RTreeDetector with balanced swept volume proportions...")
    detector = RTreeDetector()
    # Use interval=1.0s to match viz_01 proportions
    # distance = 12 m/s × 1s = 12m, half_extents = [6, 5, 5] ≈ cubic
    detector.register_uav(fp_curved.id, fp_curved, interval=0.5)
    detector.register_uav(fp_vip.id, fp_vip, interval=0.5)
    
    print("Detecting conflicts...")
    conflicts = detector.detect_all_conflicts(fp_curved.id)
    if not conflicts:
        print("[ERROR] No conflicts detected between curved and straight paths.")
        return
    
    conflict = conflicts[0]
    t_start = fp_curved.init_time()
    t_conflict = conflict["time_range"][0]
    t_anchor = max(t_start + ANCHOR_DELTA, t_conflict - ANCHOR_DELTA)
    
    print(f"✓ Conflict detected at t={t_conflict:.2f}s")
    print(f"✓ t_anchor placed at t={t_anchor:.2f}s")

    # ── 3. Get conflict OBBs to define the aggregated conflict window ────
    t_min, t_max = conflict["time_range"]
    boxes = detector.uavs[fp_curved.id]["boxes"]
    overlapping_indices = [i for i, b in enumerate(boxes) if b.t_range[1] > t_min and b.t_range[0] < t_max]
    start_idx = min(overlapping_indices)
    end_idx = min(max(overlapping_indices) + 2, len(boxes) - 1)
    conflict_obbs = [boxes[i] for i in range(start_idx, end_idx + 1)]

    # Compute the true temporal midpoint (apex) of the aggregated conflict window
    t_aggregated_start = conflict_obbs[0].t_range[0]
    t_aggregated_end = conflict_obbs[-1].t_range[1]
    t_apex = (t_aggregated_start + t_aggregated_end) / 2.0

    # ── 4. Calculate MTV automatically via SAT + perpendicularity filter ──
    print("\nCalculating MTV with automatic SAT evaluated at conflict apex...")
    vel_at_apex = fp_curved.status_at_time(t_apex).vel
    print(f"  Plebeian velocity at conflict apex (t={t_apex:.2f}s): {vel_at_apex}")
    print(f"  Magnitude: {np.linalg.norm(vel_at_apex):.2f} m/s")
    
    # We pass t_ref=t_apex so generate_mtv_candidates dynamically executes the SAT 
    # on the specific OBB pair active exactly at the center of the conflict.
    sat_res = generate_mtv_candidates(conflict, detector, flight_plan_pleb=fp_curved, t_ref=t_apex)
    if not sat_res.horizontal_mtvs:
        print("[ERROR] No valid perpendicular MTV found.")
        return
    
    mtv_raw = sat_res.horizontal_mtvs[0]
    mtv = mtv_raw * RIGID_SHIFT_MTV_SCALE
    print(f"  Raw MTV (generated from apex OBB): {mtv_raw}")
    print(f"  Scaled MTV ({RIGID_SHIFT_MTV_SCALE}×): {mtv}")

    # ── 5. Build the Rigid Shift Detour ────────────────────────────────
    print("\nCalculating Rigid Shift Detour on curved path...")
    fp_new = build_rigid_shift_detour(
        fp=fp_curved,
        t_detour_starthor=t_anchor,
        mtv=mtv,
        conflict_obbs=conflict_obbs
    )

    if fp_new is None:
        print("[ERROR] Could not generate the detour.")
        return

    # Print resulting waypoints
    print("\nGenerated Waypoints:")
    for wp in fp_new.waypoints:
        if wp.label in ["anc", "det", "ret"]:
            print(f"  -> [{wp.label:4}] t={wp.t:6.2f}s | Pos: ({wp.pos[0]:6.1f}, {wp.pos[1]:6.1f}, {wp.pos[2]:5.1f})")
        else:
            print(f"     [{wp.label:4}] t={wp.t:6.2f}s | Pos: ({wp.pos[0]:6.1f}, {wp.pos[1]:6.1f}, {wp.pos[2]:5.1f})")

    # ── 6. Generate OBBs for the NEW flight plan ───────────────────────────
    print("\nGenerating OBBs for modified flight plan...")
    # Ensure the new flight plan inherits all kinematic properties from original
    fp_new.radius = fp_curved.radius
    fp_new.max_var_lin_vel = fp_curved.max_var_lin_vel
    fp_new.max_var_ang_vel = fp_curved.max_var_ang_vel
    # Use interval=1.0 like original to get well-proportioned cubic boxes (distance/2 ≈ radius)
    obbs_new = fp_new.generate_swept_boxes_obb(interval=1.0)
    print(f"Generated {len(obbs_new)} OBBs for the detour")

    # ── 7. Traces for Plotting ─────────────────────────────────────────────
    trace_curved = fp_curved.trace(0.1)
    trace_new = fp_new.trace(0.1)
    trace_vip = fp_vip.trace(0.1)

    # ── 8. Plot the Scene (BEFORE/AFTER Comparison) ──────────────────────────
    fig = plt.figure(figsize=(16, 7), facecolor=BG)
    fig.canvas.manager.set_window_title("Curved Conflict Resolution - Before & After")
    fig.text(0.5, 0.97, "Strategy 2: TWO CROSSING CURVED Paths — Conflict Resolution", 
             ha="center", va="center", fontsize=14, fontweight="bold", color="white")

    # Shared axis configuration function
    def configure_ax(ax):
        max_range = np.array([450 - (-150), 450 - (-150), 120 - 80]).max() / 2.0
        mid_x = (450 + (-150)) / 2.0
        mid_y = (450 + (-150)) / 2.0
        mid_z = (120 + 80) / 2.0
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        ax.set_xlabel("X (m)", color="white", fontsize=10, fontweight="bold")
        ax.set_ylabel("Y (m)", color="white", fontsize=10, fontweight="bold")
        ax.set_zlabel("Z (m)", color="white", fontsize=10, fontweight="bold")
        ax.tick_params(colors="white")
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor("#2a2d3a")
        ax.view_init(elev=25, azim=45)

    # ========== SUBPLOT 1: BEFORE (Original Conflict) ==========
    ax_before = fig.add_subplot(1, 2, 1, projection="3d")
    ax_before.set_facecolor(BG)
    ax_before.set_title("❌ BEFORE: Original Conflicting Routes", color="#e74c3c", fontsize=12, fontweight="bold", pad=10)

    # Plot original trajectories
    ax_before.plot(trace_curved[:, 1], trace_curved[:, 2], trace_curved[:, 3],
            color="#7f8c8d", linewidth=2.5, label="Plebeian Route")
    ax_before.plot(trace_vip[:, 1], trace_vip[:, 2], trace_vip[:, 3],
            color="#e74c3c", linewidth=2.5, label="VIP Route (Obstacle)")

    # Draw ONLY the conflicting OBBs - HIGHLY VISIBLE IN RED
    print("\nDrawing BEFORE subplot (conflict highlighted)...")
    for i, box in enumerate(conflict_obbs):
        corners = box.get_corners()
        faces = generate_obb_faces(corners)
        poly = Poly3DCollection(faces, facecolors="#c0392b", linewidths=2.5, 
                               edgecolors="#e74c3c", alpha=0.40)
        ax_before.add_collection3d(poly)
    
    # Conflict zone box from VIP
    vip_conflict_box = detector.uavs[fp_vip.id]["boxes"][conflict["box_b_idx"]]
    corners_vip = vip_conflict_box.get_corners()
    faces_vip = generate_obb_faces(corners_vip)
    poly_vip = Poly3DCollection(faces_vip, facecolors="#c0392b", linewidths=2.5, 
                               edgecolors="#e74c3c", alpha=0.40)
    ax_before.add_collection3d(poly_vip)
    
    # Add text annotation for conflict zone
    conflict_center = (conflict_obbs[0].center + vip_conflict_box.center) / 2
    ax_before.text(conflict_center[0], conflict_center[1], conflict_center[2]+15, 
                   "⚠️ CONFLICT ZONE", color="#e74c3c", fontsize=10, fontweight="bold",
                   ha="center", bbox=dict(boxstyle="round,pad=0.3", facecolor=BG, edgecolor="#e74c3c", alpha=0.8))

    configure_ax(ax_before)
    ax_before.legend(facecolor=BG, edgecolor="#2a2d3a", labelcolor="white", loc="upper left", fontsize=9)

    # ========== SUBPLOT 2: AFTER (Resolved) ==========
    ax_after = fig.add_subplot(1, 2, 2, projection="3d")
    ax_after.set_facecolor(BG)
    ax_after.set_title("✓ AFTER: Resolved via Rigid Shift Detour", color="#2ecc71", fontsize=12, fontweight="bold", pad=10)

    # Plot resolved trajectories
    ax_after.plot(trace_curved[:, 1], trace_curved[:, 2], trace_curved[:, 3],
            color="#7f8c8d", linewidth=1.5, linestyle="--", alpha=0.5, label="Original (avoided)")
    ax_after.plot(trace_new[:, 1], trace_new[:, 2], trace_new[:, 3],
            color="#2ecc71", linewidth=3.0, label="Detour Solution")
    ax_after.plot(trace_vip[:, 1], trace_vip[:, 2], trace_vip[:, 3],
            color="#e74c3c", linewidth=2.5, label="VIP Route (safe)")

    # Draw NEW Plebeian swept OBBs (prominent green)
    print("Drawing AFTER subplot (resolution)...")
    print(f"  NEW Modified Plebeian swept OBBs: {len(obbs_new)}")
    for i, box in enumerate(obbs_new):
        corners = box.get_corners()
        faces = generate_obb_faces(corners)
        poly = Poly3DCollection(faces, facecolors="#27ae60", linewidths=1.0, 
                               edgecolors="#2ecc71", alpha=0.25)
        ax_after.add_collection3d(poly)
    
    # Draw VIP OBBs (transparent, no conflict)
    all_boxes_vip = detector.uavs[fp_vip.id]["boxes"]
    for i, box in enumerate(all_boxes_vip):
        corners = box.get_corners()
        faces = generate_obb_faces(corners)
        poly = Poly3DCollection(faces, facecolors="#d35400", linewidths=0.5, 
                               edgecolors="#e67e22", alpha=0.08)
        ax_after.add_collection3d(poly)

    # Mark the detour waypoints
    for wp in fp_new.waypoints:
        if wp.label == "anc":
            ax_after.scatter(*wp.pos, color="#e67e22", s=80, marker="s", zorder=5, edgecolors="white", linewidths=1.5)
        elif wp.label == "det":
            ax_after.scatter(*wp.pos, color="#3498db", s=80, marker="^", zorder=5, edgecolors="white", linewidths=1.5)
        elif wp.label == "ret":
            ax_after.scatter(*wp.pos, color="#1abc9c", s=80, marker="D", zorder=5, edgecolors="white", linewidths=1.5)

    configure_ax(ax_after)
    ax_after.legend(facecolor=BG, edgecolor="#2a2d3a", labelcolor="white", loc="upper left", fontsize=9)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    run_visualizer()
