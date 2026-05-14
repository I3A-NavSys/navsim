"""
viz_10_long_curved_conflict.py
==============================

SCENARIO: Long conflict on a CURVED path.

Two UAVs fly almost the same L-curve (W→E then N), one starting 3 s later.
Because they follow the same path, their swept OBBs overlap for the ENTIRE
curve duration (~30+ seconds, many OBB pairs).

This stresses Strategy 2 (Rigid Shift / Triangle) because:
  - t_ret - t_anc is very large
  - The apex falls on the turning portion of the curve
  - Many consecutive OBB pairs overlap
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
from detection.rtree_detector import RTreeDetector
from resolution.geometry.sat_mtv import generate_mtv_candidates
from resolution.geometry.path_geometry import build_rigid_shift_detour

BG = "#0f1117"


def generate_obb_faces(corners):
    corners = np.array(corners)
    return [
        [corners[0], corners[1], corners[3], corners[2]],
        [corners[4], corners[5], corners[7], corners[6]],
        [corners[0], corners[1], corners[5], corners[4]],
        [corners[2], corners[3], corners[7], corners[6]],
        [corners[0], corners[2], corners[6], corners[4]],
        [corners[1], corners[3], corners[7], corners[5]],
    ]


def make_l_curve(uid, priority, t_offset=0.0):
    """L-curve: East 150 m then North 150 m, v=10 m/s. Optional time offset."""
    fp = FlightPlan()
    fp.id = uid
    fp.priority = priority
    fp.radius = 5.0
    fp.max_var_lin_vel = 15.0
    fp.max_var_ang_vel = 1.0
    fp.set_waypoint(Waypoint("S", t_offset + 0.0,  [  0.0,   0.0, 100.0], [10.0,  0.0, 0.0]))
    fp.set_waypoint(Waypoint("T", t_offset + 15.0, [150.0,   0.0, 100.0], [ 0.0, 10.0, 0.0]))
    fp.set_waypoint(Waypoint("E", t_offset + 30.0, [150.0, 150.0, 100.0], [ 0.0, 10.0, 0.0]))
    fp.connect_waypoints()
    return fp


def run_visualizer():
    print("\n" + "=" * 65)
    print("  VISUALIZER 10: Strategy 2 — LONG CONFLICT on CURVED PATH")
    print("=" * 65)

    # UAV 1 (plebeian): starts at t=0
    # UAV 2 (VIP):      same curve, starts at t=1  → overlap for ~29 s
    fp_pleb = make_l_curve(uid=1, priority=0,  t_offset=0.0)
    fp_vip  = make_l_curve(uid=2, priority=10, t_offset=1.0)
    print("FP plebeian: t=0..30 s,  FP VIP: t=1..31 s — conflict spans full curve")

    # ── Detect ──────────────────────────────────────────────────────────
    detector = RTreeDetector()
    detector.register_uav(1, fp_pleb, interval=0.5)
    detector.register_uav(2, fp_vip,  interval=0.5)

    conflicts = detector.detect_all_conflicts(1)
    if not conflicts:
        print("[ERROR] No conflicts detected.")
        return

    t_all_min = min(c["time_range"][0] for c in conflicts)
    t_all_max = max(c["time_range"][1] for c in conflicts)
    print(f"[OK] {len(conflicts)} conflict pairs | window: {t_all_min:.1f}s -> {t_all_max:.1f}s "
          f"(dt={t_all_max - t_all_min:.1f}s)")

    # Use first conflict for MTV + anchor
    conflict = conflicts[0]
    t_conflict = conflict["time_range"][0]
    t_anchor   = max(fp_pleb.init_time() + ANCHOR_DELTA, t_conflict - ANCHOR_DELTA)

    # ── MTV ─────────────────────────────────────────────────────────────
    # Calculate the exact apex of the aggregated curved conflict
    t_apex = (t_all_min + t_all_max) / 2.0
    print(f"[OK] Conflict apex located at t={t_apex:.2f}s")
    
    # Pass t_ref=t_apex so SAT operates on the precise OBB pair active at the apex
    sat_res = generate_mtv_candidates(conflict, detector, flight_plan_pleb=fp_pleb, t_ref=t_apex)
    if not sat_res.horizontal_mtvs:
        print("[ERROR] No MTV found.")
        return
    mtv = sat_res.horizontal_mtvs[0]  # Use MTV as-is; no extra RIGID_SHIFT_MTV_SCALE
    print(f"[OK] MTV: {np.round(mtv, 2)}")

    # ── All conflicting OBBs ─────────────────────────────────────────────
    boxes = detector.uavs[1]["boxes"]
    ov_idx = [i for i, b in enumerate(boxes)
               if b.t_range[1] > t_all_min and b.t_range[0] < t_all_max]
    conflict_obbs = [boxes[i] for i in range(min(ov_idx), min(max(ov_idx) + 2, len(boxes) - 1) + 1)]
    print(f"[OK] Conflict OBBs: {len(conflict_obbs)}")

    # ── Detour Iteration Logic (mirroring cascade solver) ───────────────────
    print("\nBuilding Rigid Shift Detour (Iterating scales)...")
    best_fp = None
    scales_to_try = [1.0, 1.25, 1.5, 2.0]
    
    for scale in scales_to_try:
        scaled_mtv = mtv * scale
        print(f"-> Testing scale {scale}x | Scaled MTV: {np.round(scaled_mtv, 2)}")
        
        # Build detour candidate
        fp_cand = build_rigid_shift_detour(fp_pleb, t_anchor, scaled_mtv, conflict_obbs)
        if fp_cand is None:
            print(f"   [FAIL] Kinematically infeasible at scale {scale}x")
            continue
            
        # Keep latest successfully built flight plan as the fallback plot target
        best_fp = fp_cand
        
        # Validate with shadow R-Tree detector
        detector_post = RTreeDetector()
        detector_post.register_uav(1, fp_cand, interval=0.5)
        detector_post.register_uav(2, fp_vip,  interval=0.5)
        
        remaining = detector_post.detect_all_conflicts(1)
        if not remaining:
            print(f"   [SUCCESS] 0 conflicts remaining at scale {scale}x!")
            break
        else:
            print(f"   [WARN] {len(remaining)} conflict(s) remain at scale {scale}x")

    if best_fp is None:
        print("[ERROR] Strategy 2 could not generate ANY kinematically valid detour.")
        return
        
    # Assign the best found candidate (or last attempted valid one) to fp_new for plotting
    fp_new = best_fp

    print("\nDetour waypoints:")
    for wp in fp_new.waypoints:
        m = "->" if wp.label in ("anc", "det", "ret") else "  "
        print(f"  {m} [{wp.label:4}] t={wp.t:6.2f}s  pos=({wp.pos[0]:.1f}, {wp.pos[1]:.1f}, {wp.pos[2]:.1f})")

    fp_new.radius = fp_pleb.radius
    fp_new.max_var_lin_vel = fp_pleb.max_var_lin_vel
    fp_new.max_var_ang_vel = fp_pleb.max_var_ang_vel
    obbs_new = fp_new.generate_swept_boxes_obb(interval=0.5)

    # RTree Final Summary for console output
    print("\n--- RTree Post-Resolution Verification (Final Plotting State) ---")
    detector_post = RTreeDetector()
    detector_post.register_uav(1, fp_new, interval=0.5)
    detector_post.register_uav(2, fp_vip, interval=0.5)
    final_remaining = detector_post.detect_all_conflicts(1)
    if final_remaining:
        print(f"[WARN] {len(final_remaining)} conflict(s) STILL REMAIN in the plotted detour!")
    else:
        print("[RESOLVED] Plotted detour is 100% SAFE.")
    print("------------------------------------------------------------------")

    # ── Traces ──────────────────────────────────────────────────────────
    tr_pleb = fp_pleb.trace(0.1)
    tr_new  = fp_new.trace(0.1)
    tr_vip  = fp_vip.trace(0.1)

    # ── Plot ────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 8), facecolor=BG)
    fig.canvas.manager.set_window_title("VIZ 10 — Long Curved Conflict — Strategy 2")
    fig.text(0.5, 0.97,
             "Strategy 2 (Rigid Shift / Triangle) — LONG CONFLICT on CURVED PATH",
             ha="center", fontsize=14, fontweight="bold", color="white")

    def configure_ax(ax):
        ax.set_xlim(-20, 250); ax.set_ylim(-50, 200); ax.set_zlim(80, 180)
        ax.set_xlabel("X (m)", color="white", fontsize=10, fontweight="bold")
        ax.set_ylabel("Y (m)", color="white", fontsize=10, fontweight="bold")
        ax.set_zlabel("Z (m)", color="white", fontsize=10, fontweight="bold")
        ax.tick_params(colors="white")
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False; pane.set_edgecolor("#2a2d3a")
        ax.view_init(elev=28, azim=40)

    vip_boxes = detector.uavs[2]["boxes"]

    # ── BEFORE ──────────────────────────────────────────────────────────
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax1.set_facecolor(BG)
    ax1.set_title("❌ BEFORE: Long Curved Conflict", color="#e74c3c",
                  fontsize=12, fontweight="bold", pad=10)
    ax1.plot(tr_pleb[:, 1], tr_pleb[:, 2], tr_pleb[:, 3],
             color="#7f8c8d", linewidth=2.5, label="Plebeian Route")
    ax1.plot(tr_vip[:, 1], tr_vip[:, 2], tr_vip[:, 3],
             color="#e74c3c", linewidth=2.5, label="VIP Route (Obstacle)")

    for box in conflict_obbs:
        poly = Poly3DCollection(generate_obb_faces(box.get_corners()),
                                facecolors="#c0392b", linewidths=1.5,
                                edgecolors="#e74c3c", alpha=0.40)
        ax1.add_collection3d(poly)

    for box in vip_boxes:
        poly = Poly3DCollection(generate_obb_faces(box.get_corners()),
                                facecolors="#922b21", linewidths=0.8,
                                edgecolors="#e74c3c", alpha=0.15)
        ax1.add_collection3d(poly)

    mid = conflict_obbs[len(conflict_obbs) // 2].center
    ax1.text(mid[0], mid[1], mid[2] + 25,
             f"⚠️ CONFLICT\n{len(conflict_obbs)} OBBs\nΔt={t_all_max-t_all_min:.0f}s",
             color="#e74c3c", fontsize=9, fontweight="bold", ha="center",
             bbox=dict(boxstyle="round,pad=0.3", facecolor=BG, edgecolor="#e74c3c", alpha=0.85))
    configure_ax(ax1)
    ax1.legend(facecolor=BG, edgecolor="#2a2d3a", labelcolor="white", loc="upper left", fontsize=9)

    # ── AFTER ───────────────────────────────────────────────────────────
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    ax2.set_facecolor(BG)
    ax2.set_title("✅ AFTER: Resolved via Rigid Shift (Triangle)", color="#2ecc71",
                  fontsize=12, fontweight="bold", pad=10)
    ax2.plot(tr_pleb[:, 1], tr_pleb[:, 2], tr_pleb[:, 3],
             color="#7f8c8d", linewidth=1.5, linestyle="--", alpha=0.4, label="Original (avoided)")
    ax2.plot(tr_new[:, 1], tr_new[:, 2], tr_new[:, 3],
             color="#2ecc71", linewidth=3.0, label="Detour Solution")
    ax2.plot(tr_vip[:, 1], tr_vip[:, 2], tr_vip[:, 3],
             color="#e74c3c", linewidth=2.5, label="VIP Route (safe)")

    for box in obbs_new:
        poly = Poly3DCollection(generate_obb_faces(box.get_corners()),
                                facecolors="#27ae60", linewidths=0.8,
                                edgecolors="#2ecc71", alpha=0.20)
        ax2.add_collection3d(poly)
    for box in vip_boxes:
        poly = Poly3DCollection(generate_obb_faces(box.get_corners()),
                                facecolors="#d35400", linewidths=0.4,
                                edgecolors="#e67e22", alpha=0.06)
        ax2.add_collection3d(poly)

    for wp in fp_new.waypoints:
        if wp.label == "anc":
            ax2.scatter(*wp.pos, color="#e67e22", s=100, marker="s", zorder=5,
                        edgecolors="white", linewidths=1.5, label="Anchor (anc)")
        elif wp.label == "det":
            ax2.scatter(*wp.pos, color="#3498db", s=140, marker="^", zorder=5,
                        edgecolors="white", linewidths=1.5, label="Apex (det)")
        elif wp.label == "ret":
            ax2.scatter(*wp.pos, color="#1abc9c", s=100, marker="D", zorder=5,
                        edgecolors="white", linewidths=1.5, label="Return (ret)")

    configure_ax(ax2)
    ax2.legend(facecolor=BG, edgecolor="#2a2d3a", labelcolor="white", loc="upper left", fontsize=9)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


if __name__ == "__main__":
    run_visualizer()
