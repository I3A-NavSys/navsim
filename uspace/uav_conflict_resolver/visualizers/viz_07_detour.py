"""
viz_07_rigid_shift_detour.py — Visualizer: Strategy 1 (Rigid Shift/Triangle) Internals
========================================================================

PURPOSE:
    This visualizer isolates the `build_rigid_shift_detour` function but uses 
    the system's real RTreeDetector and SAT collision detection to 
    automatically calculate the true `t_anchor` and `MTV`.

    It displays the new 3-point topology (anc -> det -> ret) with the 
    kinematically smoothed apex, using a realistic physical scale.
"""

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
from benchmark.flightplan_generator import generate_crossing_pair
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
    print("  VISUALIZER 7: Rigid Shift (Triangle) Geometry (Automatic)")
    print("=" * 60)

    # ── 1. Create a dynamic crossing pair ──────────────────────────────
    print("Generating a realistic crossing pair...")
    fp_vip, fp_orig = generate_crossing_pair(seed=42)
    # Let's make fp_vip the VIP and fp_orig the plebeian
    fp_vip.priority = 10
    fp_orig.priority = 1
    
    # ── 2. Register in R-Tree and Detect Conflict ──────────────────────
    print("Registering plans in the RTreeDetector...")
    detector = RTreeDetector()
    detector.register_uav(fp_vip.id, fp_vip)
    detector.register_uav(fp_orig.id, fp_orig)
    
    print("Detecting conflicts...")
    conflicts = detector.detect_all_conflicts(fp_orig.id)
    if not conflicts:
        print("[ERROR] No conflicts detected. Cannot calculate automatically.")
        return
    
    conflict = conflicts[0]
    t_start = fp_orig.init_time()
    t_conflict = conflict["time_range"][0]
    
    # Calculate t_anchor automatically using the system's logic
    t_anchor = max(t_start + ANCHOR_DELTA, t_conflict - ANCHOR_DELTA)
    print(f"Conflict found at t={t_conflict:.2f}s. Placing t_anchor at t={t_anchor:.2f}s.")

    # ── 3. Calculate MTV automatically via generate_mtv_candidates ────
    print("\n" + "─" * 60)
    print("CALCULATING MTV")
    print("─" * 60)
    
    # Get instantaneous velocity at conflict time
    vel_at_conflict = fp_orig.status_at_time(t_conflict).vel
    print(f"Plebeian velocity at t_conflict={t_conflict:.2f}s: {vel_at_conflict}")
    print(f"  Velocity magnitude: {np.linalg.norm(vel_at_conflict):.2f} m/s")
    
    sat_res = generate_mtv_candidates(conflict, detector, flight_plan_pleb=fp_orig)
    if not sat_res.horizontal_mtvs:
        print("[ERROR] No horizontal MTV found after filtering.")
        return
    
    print(f"MTVs found: {len(sat_res.horizontal_mtvs)} horizontal, {len(sat_res.vertical_mtvs)} vertical")
    print(f"First (best) horizontal MTV: {sat_res.horizontal_mtvs[0]}")
        
    # FIX: Scale the MTV by a physically realistic amount (e.g., 2.0)
    # Using 15.0 caused kinematic failure (overshoot/loops) because it forced 
    # the drone to move too far laterally in too little time.
    mtv = sat_res.horizontal_mtvs[0] * RIGID_SHIFT_MTV_SCALE
    print(f"After kinematic scaling ({RIGID_SHIFT_MTV_SCALE}×): {mtv}\n")

    # ── 3.5 VISUALIZE MTV CALCULATION (3D) ─────────────────────────────
    print("\nVisualizing MTV calculation in 3D...")
    
    # Retrieve colliding OBBs in the conflict time range (same as in final viz)
    t_min, t_max = conflict["time_range"]
    boxes = detector.uavs[fp_orig.id]["boxes"]
    overlapping_indices = [i for i, b in enumerate(boxes) if b.t_range[1] > t_min and b.t_range[0] < t_max]
    start_idx = min(overlapping_indices)
    end_idx = min(max(overlapping_indices) + 2, len(boxes) - 1)
    conflict_obbs = [boxes[i] for i in range(start_idx, end_idx + 1)]
    
    # Get the main conflicting OBB and VIP OBB
    box_orig = detector.uavs[fp_orig.id]["boxes"][conflict["box_a_idx"]]
    box_vip = detector.uavs[fp_vip.id]["boxes"][conflict["box_b_idx"]]
    
    # Create MTV visualization figure
    fig_mtv = plt.figure(figsize=(16, 12), facecolor=BG)
    
    # ── LEFT: 3D Scene with all conflicting OBBs ──
    ax_mtv_3d = fig_mtv.add_subplot(121, projection="3d")
    ax_mtv_3d.set_facecolor(BG)
    ax_mtv_3d.set_title("SAT MTV Calculation: Conflict Geometry", color="white", fontsize=14, fontweight="bold", pad=15)
    
    # Draw ALL conflicting plebeian OBBs (lighter blue)
    for i, box in enumerate(conflict_obbs):
        faces = generate_obb_faces(box.get_corners())
        alpha = 0.08 if i != conflict["box_a_idx"] else 0.25  # Highlight the main conflict box
        edge_color = '#1abc9c' if i != conflict["box_a_idx"] else '#3498db'
        face_color = '#0e6251' if i != conflict["box_a_idx"] else '#2980b9'
        poly = Poly3DCollection(faces, alpha=alpha, linewidths=1.5, edgecolors=edge_color, facecolors=face_color)
        ax_mtv_3d.add_collection3d(poly)
    
    # Draw VIP OBB (Red) - semi-transparent
    corners_vip = np.array(box_vip.get_corners())
    faces_vip = generate_obb_faces(corners_vip)
    poly_vip = Poly3DCollection(faces_vip, alpha=0.25, linewidths=2, edgecolors='#e74c3c', facecolors='#c0392b')
    ax_mtv_3d.add_collection3d(poly_vip)
    
    # Get original and VIP centers
    center_orig = box_orig.center
    center_vip = box_vip.center
    
    # Plot centers
    ax_mtv_3d.scatter(*center_orig, color="#3498db", s=250, marker="o", zorder=5, edgecolors="white", linewidths=2)
    ax_mtv_3d.text(center_orig[0]-20, center_orig[1], center_orig[2]-10, "Plebeian\nCenter", 
                   color="#3498db", fontsize=10, fontweight="bold", ha="right")
    
    ax_mtv_3d.scatter(*center_vip, color="#e74c3c", s=250, marker="o", zorder=5, edgecolors="white", linewidths=2)
    ax_mtv_3d.text(center_vip[0]+20, center_vip[1], center_vip[2]-10, "VIP\nCenter", 
                   color="#e74c3c", fontsize=10, fontweight="bold", ha="left")
    
    # MTV vectors
    mtv_raw = sat_res.horizontal_mtvs[0]
    
    # Draw MTV as arrow from plebeian center
    # Raw MTV (orange, thin)
    scale_factor = 8
    
    # Draw velocity vector (reference for perpendicularity check)
    vel_at_conflict_arr = np.array(vel_at_conflict)
    if np.linalg.norm(vel_at_conflict_arr) > EPSILON_ABSOLUTE:
        ax_mtv_3d.quiver(center_orig[0], center_orig[1], center_orig[2],
                         vel_at_conflict_arr[0], vel_at_conflict_arr[1], vel_at_conflict_arr[2],
                         color="#9b59b6", arrow_length_ratio=0.15, linewidth=2, alpha=0.7, label="Instantaneous Velocity")
    
    end_point_raw = center_orig + mtv_raw * scale_factor
    ax_mtv_3d.quiver(center_orig[0], center_orig[1], center_orig[2],
                     mtv_raw[0]*scale_factor, mtv_raw[1]*scale_factor, mtv_raw[2]*scale_factor,
                     color="#f39c12", arrow_length_ratio=0.15, linewidth=2.5, alpha=0.8)
    
    # Scaled MTV (cyan, thick)
    end_point_scaled = center_orig + mtv * scale_factor
    ax_mtv_3d.quiver(center_orig[0], center_orig[1], center_orig[2],
                     mtv[0]*scale_factor, mtv[1]*scale_factor, mtv[2]*scale_factor,
                     color="#1abc9c", arrow_length_ratio=0.15, linewidth=3.5, alpha=0.95, linestyle='dashed')
    
    # Show the shifted plebeian center
    shifted_orig = center_orig + mtv
    ax_mtv_3d.scatter(*shifted_orig, color="#1abc9c", s=200, marker="^", zorder=6, edgecolors="white", linewidths=2)
    ax_mtv_3d.text(shifted_orig[0], shifted_orig[1]+20, shifted_orig[2], "After MTV\nShift", 
                   color="#1abc9c", fontsize=10, fontweight="bold", ha="center")
    
    # Connection line
    ax_mtv_3d.plot([center_orig[0], shifted_orig[0]], [center_orig[1], shifted_orig[1]], [center_orig[2], shifted_orig[2]],
                   color="#1abc9c", linewidth=2, linestyle=":", alpha=0.6)
    
    ax_mtv_3d.set_xlabel("X (m)", color="white", fontsize=11, fontweight="bold")
    ax_mtv_3d.set_ylabel("Y (m)", color="white", fontsize=11, fontweight="bold")
    ax_mtv_3d.set_zlabel("Z (m)", color="white", fontsize=11, fontweight="bold")
    ax_mtv_3d.tick_params(colors="white", labelsize=9)
    for pane in [ax_mtv_3d.xaxis.pane, ax_mtv_3d.yaxis.pane, ax_mtv_3d.zaxis.pane]:
        pane.fill = False
        pane.set_edgecolor("#2a2d3a")
    
    # Zoom into conflict zone
    center = box_orig.center
    zoom = 120
    ax_mtv_3d.set_xlim(center[0] - zoom, center[0] + zoom)
    ax_mtv_3d.set_ylim(center[1] - zoom, center[1] + zoom)
    ax_mtv_3d.set_zlim(center[2] - zoom, center[2] + zoom)
    
    ax_mtv_3d.view_init(elev=30, azim=45)
    
    # Add legend for 3D view
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='#3498db', linewidth=3, label='Plebeian OBBs (sequence)'),
        Line2D([0], [0], color='#e74c3c', linewidth=3, label='VIP OBB'),
        Line2D([0], [0], color='#9b59b6', linewidth=3, label='Instantaneous Velocity'),
        Line2D([0], [0], color='#f39c12', linewidth=3, label=f'Raw MTV (mag={np.linalg.norm(mtv_raw):.2f}m)'),
        Line2D([0], [0], color='#1abc9c', linewidth=3, linestyle='--', label=f'Scaled MTV (mag={np.linalg.norm(mtv):.2f}m)'),
    ]
    ax_mtv_3d.legend(handles=legend_elements, loc='upper left', facecolor="#1a1d27", edgecolor="gray", labelcolor="white", fontsize=10)
    
    # ── RIGHT: 2D Close-up and MTV Explanation ──
    ax_mtv_2d = fig_mtv.add_subplot(122)
    ax_mtv_2d.set_facecolor(BG)
    ax_mtv_2d.axis('off')
    
    # Title for explanation
    ax_mtv_2d.text(0.5, 0.95, "How is MTV Calculated?", transform=ax_mtv_2d.transAxes,
                   color="white", fontsize=14, fontweight="bold", ha="center",
                   bbox=dict(boxstyle="round", facecolor="#1a1d27", edgecolor="#1abc9c", linewidth=2, alpha=0.9))
    
    # MTV explanation
    mtv_raw_norm = np.linalg.norm(mtv_raw)
    mtv_scaled_norm = np.linalg.norm(mtv)
    vel_norm = np.linalg.norm(vel_at_conflict)
    
    # Calculate alignment between MTV and velocity
    if vel_norm > EPSILON_ABSOLUTE:
        vel_dir = vel_at_conflict_arr / vel_norm
        mtv_dir = mtv_raw / mtv_raw_norm
        alignment = abs(np.dot(mtv_dir, vel_dir))
    else:
        alignment = 0
    
    explanation = f"""
SEPARATING AXIS THEOREM (SAT)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1️⃣  TEST ALL AXES
   • Check all face normals from both OBBs
   • Project both boxes onto each axis
   
2️⃣  FIND OVERLAPS
   • If projections overlap on all axes → COLLISION
   • Calculate overlap distance on each axis

3️⃣  IDENTIFY MTV (Minimum Translation Vector)
   • MTV = axis with SMALLEST overlap distance
   • Direction: perpendicular to collision surface
   • Magnitude: minimum distance to separate boxes

4️⃣  FILTER BY PERPENDICULARITY
   • Discard MTVs that are aligned with current velocity
   • Keep only MTVs perpendicular to flight direction
   • Prevents "acceleration" solutions; demands true evasion
   
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

RESULTS FOR THIS CONFLICT:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Instantaneous Velocity (t={t_conflict:.2f}s):
  Vector: [{vel_at_conflict_arr[0]:7.3f}, {vel_at_conflict_arr[1]:7.3f}, {vel_at_conflict_arr[2]:7.3f}]
  Magnitude: {vel_norm:.3f} m/s

Raw MTV (from SAT):
  Vector: [{mtv_raw[0]:7.3f}, {mtv_raw[1]:7.3f}, {mtv_raw[2]:7.3f}]
  Magnitude: {mtv_raw_norm:.3f} m
  Alignment with velocity: {alignment:.3f} (0=perpendicular, 1=parallel)

Kinematic Scaling Factor: {RIGID_SHIFT_MTV_SCALE}×

Scaled MTV (physically feasible):
  Vector: [{mtv[0]:7.3f}, {mtv[1]:7.3f}, {mtv[2]:7.3f}]
  Magnitude: {mtv_scaled_norm:.3f} m

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

INTERPRETATION:
  The drone is shifted {mtv_scaled_norm:.1f}m in the direction
  ({mtv[0]/mtv_scaled_norm:.2f}, {mtv[1]/mtv_scaled_norm:.2f}, {mtv[2]/mtv_scaled_norm:.2f})
  to safely avoid the VIP aircraft.
"""
    
    ax_mtv_2d.text(0.05, 0.85, explanation, transform=ax_mtv_2d.transAxes,
                   color="#ecf0f1", fontsize=10, family="monospace",
                   verticalalignment='top',
                   bbox=dict(boxstyle="round", facecolor="#16213e", edgecolor="#f39c12", linewidth=1.5, alpha=0.95))
    
    plt.tight_layout()
    plt.show()
    
    print(f"\n{'='*60}")
    print(f"MTV CALCULATION SUMMARY")
    print(f"{'='*60}")
    print(f"Raw MTV magnitude:       {mtv_raw_norm:.3f}m")
    print(f"Scaled MTV magnitude:    {mtv_scaled_norm:.3f}m")
    print(f"Scale factor applied:    {RIGID_SHIFT_MTV_SCALE}×")
    print(f"Direction (normalized):  [{mtv[0]/mtv_scaled_norm:.4f}, {mtv[1]/mtv_scaled_norm:.4f}, {mtv[2]/mtv_scaled_norm:.4f}]")
    print(f"{'='*60}\n")



    # ── 4. Build the Rigid Shift Detour ────────────────────────────────
    print("\nCalculating the Rigid Shift (Triangle) Detour with automatically-derived MTV...")
    fp_new = build_rigid_shift_detour(
        fp=fp_orig,
        t_anchor=t_anchor,
        mtv=mtv,
        conflict_obbs=conflict_obbs
    )

    if fp_new is None:
        print("[ERROR] Could not generate the detour. (Kinematics failed)")
        return

    # Print the resulting waypoints
    print("\nGenerated Waypoints:")
    for wp in fp_new.waypoints:
        if wp.label in ["anc", "det", "ret"]:
            print(f"  -> [{wp.label:4}] t={wp.t:6.2f}s | Pos: ({wp.pos[0]:5.1f}, {wp.pos[1]:5.1f}, {wp.pos[2]:4.1f})")
        else:
            print(f"     [{wp.label:4}] t={wp.t:6.2f}s | Pos: ({wp.pos[0]:5.1f}, {wp.pos[1]:5.1f}, {wp.pos[2]:4.1f})")

    # ── 5. Traces for Plotting ─────────────────────────────────────────
    trace_orig = fp_orig.trace(0.2)
    trace_new  = fp_new.trace(0.2)
    trace_vip  = fp_vip.trace(0.2)

    # Get first conflict OBB for reference
    first_conflict_obb = conflict_obbs[0]

    # ── 6. Plot the Scene ──────────────────────────────────────────────
    fig = plt.figure(figsize=(10, 8), facecolor=BG)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(BG)
    ax.set_title("Rigid Shift Detour (3-Point Topology)", color="white", fontsize=14, pad=15)

    # 1. Original route (Gray dashed)
    ax.plot(trace_orig[:, 1], trace_orig[:, 2], trace_orig[:, 3],
            color="#7f8c8d", linewidth=2.0, linestyle="--", label="Original Route (Plebeian)")
    
    # 2. Original conflict center (Red X)
    ax.scatter(*first_conflict_obb.center, color="#e74c3c", s=100, marker="X", label="Conflict Center")

    # 3. Evasive route (Green)
    ax.plot(trace_new[:, 1], trace_new[:, 2], trace_new[:, 3],
            color="#2ecc71", linewidth=3.5, label="New Trajectory (Evasion)")

    # 4. VIP Obstacle Route (Red solid)
    ax.plot(trace_vip[:, 1], trace_vip[:, 2], trace_vip[:, 3],
            color="#e74c3c", linewidth=2.5, linestyle="-", label="VIP Route (Obstacle)")

    # 5. Plot the colliding OBBs
    # Plebeian conflicting OBB sequence (Blue)
    for i, box in enumerate(conflict_obbs):
        faces = generate_obb_faces(box.get_corners())
        poly = Poly3DCollection(faces, alpha=0.15, linewidths=1.2, edgecolors='#3498db', facecolors='#2980b9')
        ax.add_collection3d(poly)

    # VIP conflicting OBB (Red)
    box_vip = detector.uavs[fp_vip.id]["boxes"][conflict["box_b_idx"]]
    faces_vip = generate_obb_faces(box_vip.get_corners())
    poly_vip = Poly3DCollection(faces_vip, alpha=0.1, linewidths=1.2, edgecolors='#e74c3c', facecolors='#c0392b')
    ax.add_collection3d(poly_vip)
    
    # 6. Mark the 3 magic waypoints in 3D space
    for wp in fp_new.waypoints:
        if wp.label == "anc":
            ax.scatter(*wp.pos, color="#e67e22", s=70, marker="s", zorder=5)
            ax.text(wp.pos[0], wp.pos[1]+5, wp.pos[2], "anc", color="#e67e22", fontsize=10, fontweight="bold")
        elif wp.label == "det":
            ax.scatter(*wp.pos, color="#3498db", s=70, marker="^", zorder=5)
            ax.text(wp.pos[0]-15, wp.pos[1]+5, wp.pos[2], "det", color="#3498db", fontsize=10, fontweight="bold")
        elif wp.label == "ret":
            ax.scatter(*wp.pos, color="#1abc9c", s=70, marker="D", zorder=5)
            ax.text(wp.pos[0], wp.pos[1]-15, wp.pos[2], "ret", color="#1abc9c", fontsize=10, fontweight="bold")

    ax.set_xlabel("X (m)", color="white")
    ax.set_ylabel("Y (m)", color="white")
    ax.set_zlabel("Z (m)", color="white")
    ax.tick_params(colors="white")
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = False
        pane.set_edgecolor("#2a2d3a")
    ax.legend(facecolor="#1a1d27", edgecolor="gray", labelcolor="white")
    
    # Zoom exactly onto the conflict zone
    center = first_conflict_obb.center
    ax.set_xlim(center[0] - 150, center[0] + 150)
    ax.set_ylim(center[1] - 150, center[1] + 150)
    ax.set_zlim(center[2] - 150, center[2] + 150)

    ax.view_init(elev=35, azim=45)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_visualizer()