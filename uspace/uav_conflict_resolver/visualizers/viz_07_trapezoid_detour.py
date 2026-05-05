"""
viz_07_trapezoid_detour.py — Visualizer: Strategy 2 (Trapezoid) Internals
========================================================================

PURPOSE:
    This visualizer isolates the `build_trapezoid_detour` function but uses 
    the system's real RTreeDetector and SAT collision detection to 
    automatically calculate the true `t_anchor` and `MTV`.

    By applying the new waypoint-skipping fix, waypoints within the conflict 
    time window are completely ignored, preventing available time budget 
    compression.
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
from core.config import ANCHOR_DELTA
from visualizers.flightplan_generator import generate_crossing_pair
from detection.rtree_detector import RTreeDetector
from resolution.geometry.sat_mtv import generate_mtv_candidates
from resolution.geometry.path_geometry import build_trapezoid_detour

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
    print("  VISUALIZER 7: Strategy 2 (Trapezoid) Geometry (Automatic)")
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
    sat_res = generate_mtv_candidates(conflict, detector)
    if not sat_res.horizontal_mtvs:
        print("[ERROR] No horizontal MTV found.")
        return
        
    # Pick the first horizontal MTV and scale it up for visualization visibility
    mtv = sat_res.horizontal_mtvs[0] * 15.0
    print(f"Using automatically computed and scaled horizontal MTV: {mtv}")

    # Retrieve colliding OBBs in the conflict time range
    t_min, t_max = conflict["time_range"]
    conflict_obbs = []
    boxes = detector.uavs[fp_orig.id]["boxes"]
    for box in boxes:
        if box.t_range[1] > t_min and box.t_range[0] < t_max:
            conflict_obbs.append(box)

    # Add 2 extra consecutive boxes to ensure the safe region completely covers the collision zone
    overlapping_indices = [i for i, b in enumerate(boxes) if b.t_range[1] > t_min and b.t_range[0] < t_max]
    start_idx = min(overlapping_indices)
    end_idx = min(max(overlapping_indices) + 2, len(boxes) - 1)
    
    conflict_obbs = [boxes[i] for i in range(start_idx, end_idx + 1)]

    # ── 4. Build the Trapezoid Detour ──────────────────────────────────
    print("\nCalculating the heuristic Trapezoid with automatically-derived MTV...")
    fp_new = build_trapezoid_detour(
        fp=fp_orig,
        t_anchor=t_anchor,
        mtv=mtv,
        conflict_obbs=conflict_obbs
    )

    if fp_new is None:
        print("[ERROR] Could not generate the trapezoid.")
        return

    # Print the resulting waypoints
    print("\nGenerated Waypoints:")
    for wp in fp_new.waypoints:
        if wp.label in ["anc", "det1", "det2", "ret"]:
            print(f"  -> [{wp.label:4}] t={wp.t:6.2f}s | Pos: ({wp.pos[0]:5.1f}, {wp.pos[1]:5.1f}, {wp.pos[2]:4.1f})")
        else:
            print(f"     [{wp.label:4}] t={wp.t:6.2f}s | Pos: ({wp.pos[0]:5.1f}, {wp.pos[1]:5.1f}, {wp.pos[2]:4.1f})")

    # ── 5. Traces for Plotting ─────────────────────────────────────────
    trace_orig = fp_orig.trace(0.2)
    trace_new  = fp_new.trace(0.2)
    trace_vip  = fp_vip.trace(0.2)

    # Safe Center of the first conflicting box
    first_conflict_obb = conflict_obbs[0]
    safe_center = first_conflict_obb.center + mtv

    # ── 6. Plot the Scene ──────────────────────────────────────────────
    fig = plt.figure(figsize=(10, 8), facecolor=BG)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(BG)
    ax.set_title("Strategy 2 (Trapezoid) Detour with Dynamic Waypoint Filtering", color="white", fontsize=14, pad=15)

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

    # 5. Safe Center
    ax.scatter(*safe_center, color="#f39c12", s=80, marker="o", label="Safe Center (Original + MTV)")

    # 6. Plot the colliding OBBs
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
    
    # 7. Mark the 4 magic waypoints in 3D space
    for wp in fp_new.waypoints:
        if wp.label == "anc":
            ax.scatter(*wp.pos, color="#e67e22", s=70, marker="s", zorder=5)
            ax.text(wp.pos[0], wp.pos[1]+5, wp.pos[2], "anc", color="#e67e22", fontsize=10, fontweight="bold")
        elif wp.label == "det1":
            ax.scatter(*wp.pos, color="#3498db", s=70, marker="^", zorder=5)
            ax.text(wp.pos[0]-15, wp.pos[1]+5, wp.pos[2], "det1", color="#3498db", fontsize=10, fontweight="bold")
        elif wp.label == "det2":
            ax.scatter(*wp.pos, color="#9b59b6", s=70, marker="^", zorder=5)
            ax.text(wp.pos[0]+5, wp.pos[1]-15, wp.pos[2], "det2", color="#9b59b6", fontsize=10, fontweight="bold")
        elif wp.label == "ret":
            ax.scatter(*wp.pos, color="#1abc9c", s=70, marker="D", zorder=5)
            ax.text(wp.pos[0], wp.pos[1]-15, wp.pos[2], "ret", color="#1abc9c", fontsize=10, fontweight="bold")

    # Draw the "flat-top" line
    try:
        det1_wp = next(w for w in fp_new.waypoints if w.label == "det1")
        det2_wp = next(w for w in fp_new.waypoints if w.label == "det2")
        ax.plot([det1_wp.pos[0], det2_wp.pos[0]], 
                [det1_wp.pos[1], det2_wp.pos[1]], 
                [det1_wp.pos[2], det2_wp.pos[2]],
                color="white", linewidth=1.0, linestyle=":", label="Ideal Flat-top (det1 -> det2)")
    except StopIteration:
        pass

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
