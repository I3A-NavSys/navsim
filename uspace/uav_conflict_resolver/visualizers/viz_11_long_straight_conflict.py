"""
viz_11_long_straight_conflict.py
=================================

SCENARIO: Long conflict on a STRAIGHT path with MULTIPLE OBB pairs.

Two UAVs fly almost the same straight East route, one starting 3 s later.
Because they follow the same path, their swept OBBs overlap for the ENTIRE
straight segment (~57 seconds, many OBB pairs).

This stresses Strategy 2 (Rigid Shift / Triangle) because:
  - t_ret - t_anc is very large (most of the flight)
  - The apex is at the midpoint of a long straight segment
  - Many consecutive OBB pairs overlap → the MTV must clear a wide corridor
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

BG = "white"
FG = "#111111"


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


def make_straight(uid, priority, t_offset=0.0):
    """Straight East route: 480 m at v=8 m/s (60 s). Optional time offset."""
    fp = FlightPlan()
    fp.id = uid
    fp.priority = priority
    fp.radius = 5.0
    fp.max_var_lin_vel = 15.0
    fp.max_var_ang_vel = 1.0
    fp.set_waypoint(Waypoint("S", t_offset + 0.0,  [  0.0, 0.0, 100.0], [8.0, 0.0, 0.0]))
    fp.set_waypoint(Waypoint("E", t_offset + 60.0, [480.0, 0.0, 100.0], [8.0, 0.0, 0.0]))
    fp.connect_waypoints()
    return fp


def run_visualizer():
    print("\n" + "=" * 65)
    print("  VISUALIZER 11: Strategy 2 — LONG CONFLICT on STRAIGHT PATH")
    print("  (Multiple overlapping OBB pairs)")
    print("=" * 65)

    # ── Trajectories: Extending the plebeian path at both ends ──────────────
    # UAV 1 (plebeian): Travels from X=-100 to X=580.
    fp_pleb = FlightPlan()
    fp_pleb.id = 1
    fp_pleb.priority = 0
    fp_pleb.radius = 5.0
    fp_pleb.max_var_lin_vel = 15.0
    fp_pleb.max_var_ang_vel = 1.0
    fp_pleb.set_waypoint(Waypoint("S1", 0.0,  [-100.0, 0.0, 100.0], [8.0, 0.0, 0.0]))
    fp_pleb.set_waypoint(Waypoint("E1", 85.0, [ 580.0, 0.0, 100.0], [8.0, 0.0, 0.0]))
    fp_pleb.connect_waypoints()

    # UAV 2 (VIP): Realigned so it only occupies the central part (X=0 to X=480).
    # Plebeian crosses X=0 at t=12.5s. VIP joins the corridor at t=13.5s to trigger conflict.
    fp_vip = FlightPlan()
    fp_vip.id = 2
    fp_vip.priority = 10
    fp_vip.radius = 5.0
    fp_vip.max_var_lin_vel = 15.0
    fp_vip.max_var_ang_vel = 1.0
    fp_vip.set_waypoint(Waypoint("S2", 13.5, [  0.0, 0.0, 100.0], [8.0, 0.0, 0.0]))
    fp_vip.set_waypoint(Waypoint("E2", 73.5, [480.0, 0.0, 100.0], [8.0, 0.0, 0.0]))
    fp_vip.connect_waypoints()

    print("FP plebeian: t=0..85 s (Extended X=[-100, 580])")
    print("FP VIP:      t=13.5..73.5 s (Occupies corridor X=[0, 480])")
    print("-> Creates conflict-free buffer zones at plebeian's start and end!")

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

    conflict = conflicts[0]
    t_conflict = conflict["time_range"][0]
    t_anchor   = max(fp_pleb.init_time() + ANCHOR_DELTA, t_conflict - ANCHOR_DELTA)

    # ── MTV ─────────────────────────────────────────────────────────────
    # Calculate the exact apex of the aggregated corridor conflict
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
    print(f"[OK] Conflict OBBs (plebeian): {len(conflict_obbs)}")

    vip_boxes = detector.uavs[2]["boxes"]
    vip_ov    = [i for i, b in enumerate(vip_boxes)
                 if b.t_range[1] > t_all_min and b.t_range[0] < t_all_max]
    print(f"[OK] Conflict OBBs (VIP):     {len(vip_ov)}")

    # ── Detour ──────────────────────────────────────────────────────────
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
        
        # Validate with live R-Tree detector
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
        print(f"  {m} [{wp.label:5}] t={wp.t:6.2f}s  pos=({wp.pos[0]:.1f}, {wp.pos[1]:.1f}, {wp.pos[2]:.1f})")

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
    fig = plt.figure(figsize=(20, 8), facecolor=BG)
    fig.canvas.manager.set_window_title("VIZ 11 — Long Straight Conflict — Strategy 2")
    fig.text(0.5, 0.97,
             "Strategy 2 (Rigid Shift / Triangle) — LONG STRAIGHT CONFLICT (Multiple OBB Pairs)",
             ha="center", fontsize=13, fontweight="bold", color=FG)

    def configure_ax(ax):
        ax.set_xlim(-130, 610)
        ax.set_ylim(-200, 200)
        ax.set_zlim(80, 200)
        ax.set_xlabel("X (m)", color=FG, fontsize=10, fontweight="bold")
        ax.set_ylabel("Y (m)", color=FG, fontsize=10, fontweight="bold")
        ax.set_zlabel("Z (m)", color=FG, fontsize=10, fontweight="bold")
        ax.tick_params(colors=FG)
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False; pane.set_edgecolor("#2a2d3a")
        ax.view_init(elev=25, azim=20)

    # ── BEFORE ──────────────────────────────────────────────────────────
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax1.set_facecolor(BG)
    ax1.set_title("❌ BEFORE: Long Straight Corridor Conflict", color="#e74c3c",
                  fontsize=12, fontweight="bold", pad=10)
    ax1.plot(tr_pleb[:, 1], tr_pleb[:, 2], tr_pleb[:, 3],
             color="#7f8c8d", linewidth=2.5, label="Plebeian Route")
    ax1.plot(tr_vip[:, 1], tr_vip[:, 2], tr_vip[:, 3],
             color="#e74c3c", linewidth=2.5, label="VIP Route (same corridor)")

    for box in conflict_obbs:
        poly = Poly3DCollection(generate_obb_faces(box.get_corners()),
                                facecolors="#c0392b", linewidths=1.0,
                                edgecolors="#e74c3c", alpha=0.35)
        ax1.add_collection3d(poly)
    for i in vip_ov:
        poly = Poly3DCollection(generate_obb_faces(vip_boxes[i].get_corners()),
                                facecolors="#922b21", linewidths=0.8,
                                edgecolors="#e74c3c", alpha=0.18)
        ax1.add_collection3d(poly)

    mid = conflict_obbs[len(conflict_obbs) // 2].center
    ax1.text(mid[0], mid[1], mid[2] + 40,
             f"⚠️ CONFLICT\n{len(conflict_obbs)} pleb OBBs\n{len(vip_ov)} VIP OBBs\nΔt={t_all_max-t_all_min:.0f}s",
             color="#e74c3c", fontsize=9, fontweight="bold", ha="center",
             bbox=dict(boxstyle="round,pad=0.3", facecolor=BG, edgecolor="#e74c3c", alpha=0.85))
    configure_ax(ax1)
    ax1.legend(facecolor=BG, edgecolor="#cfcfcf", labelcolor=FG, loc="upper left", fontsize=9)

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
                                facecolors="#27ae60", linewidths=0.6,
                                edgecolors="#2ecc71", alpha=0.18)
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
    ax2.legend(facecolor=BG, edgecolor="#cfcfcf", labelcolor=FG, loc="upper left", fontsize=9)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


if __name__ == "__main__":
    run_visualizer()
