#!/usr/bin/env python3
"""
viz14_horizontal_shift.py
==========================

SCENARIO: Perpendicular crossing resolved with Strategy 2 (Rigid Shift).

Two UAVs cross at right angles at the same altitude:
  - Plebeian route: West -> East
  - VIP route: South -> North

The lower-priority UAV is repaired with the S2 detour builder. The plot shows
the conflict OBBs, the SAT MTV used for S2, and the anc / det / ret points.
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
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


def make_straight_route(uid, priority, axis, t_offset=0.0, speed=10.0, length=400.0, altitude=100.0):
    fp = FlightPlan()
    fp.id = uid
    fp.priority = priority
    fp.radius = 5.0
    fp.max_var_lin_vel = 15.0
    fp.max_var_ang_vel = 1.0

    if axis == "x":
        start_pos = [0.0, 200.0, altitude]
        end_pos = [length, 200.0, altitude]
        start_vel = [speed, 0.0, 0.0]
        end_vel = [speed, 0.0, 0.0]
    elif axis == "y":
        start_pos = [200.0, 0.0, altitude]
        end_pos = [200.0, length, altitude]
        start_vel = [0.0, speed, 0.0]
        end_vel = [0.0, speed, 0.0]
    else:
        raise ValueError(f"Unsupported axis: {axis}")

    duration = length / speed
    fp.set_waypoint(Waypoint("S", t_offset + 0.0, start_pos, start_vel))
    fp.set_waypoint(Waypoint("E", t_offset + duration, end_pos, end_vel))
    fp.connect_waypoints()
    return fp


def _display_label(label: str) -> str:
    return {
        "detour_start": "anc",
        "det": "det",
        "detour_end": "ret",
    }.get(label, label)


def _configure_ax(ax):
    ax.set_xlim(-40, 440)
    ax.set_ylim(-40, 440)
    ax.set_zlim(80, 140)
    ax.set_xlabel("X (m)", color=FG, fontsize=10, fontweight="bold")
    ax.set_ylabel("Y (m)", color=FG, fontsize=10, fontweight="bold")
    ax.set_zlabel("Z (m)", color=FG, fontsize=10, fontweight="bold")
    ax.tick_params(colors=FG)
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = False
        pane.set_edgecolor("#2a2d3a")
    ax.view_init(elev=26, azim=35)


def _select_apex_conflict(conflicts):
    if len(conflicts) == 1:
        return conflicts[0]
    return min(conflicts, key=lambda c: abs(((c["time_range"][0] + c["time_range"][1]) / 2.0) - 20.0))


def run_visualizer():
    print("\n" + "=" * 65)
    print("  VISUALIZER 14: Strategy 2 — PERPENDICULAR CROSSING")
    print("  (anc / det / ret shown with the conflicting OBBs)")
    print("=" * 65)

    fp_pleb = make_straight_route(uid=1, priority=0, axis="x", t_offset=0.0)
    fp_vip = make_straight_route(uid=2, priority=10, axis="y", t_offset=0.0)

    print("FP plebeian: W -> E | t=0..40 s")
    print("FP VIP:      S -> N | t=0..40 s")
    print("-> Perpendicular crossing at the center, both at z=100 m")

    detector = RTreeDetector()
    detector.register_uav(1, fp_pleb, interval=0.5)
    detector.register_uav(2, fp_vip, interval=0.5)

    conflicts = detector.detect_all_conflicts(1)
    if not conflicts:
        print("[ERROR] No conflicts detected.")
        return

    t_all_min = min(c["time_range"][0] for c in conflicts)
    t_all_max = max(c["time_range"][1] for c in conflicts)
    t_apex = (t_all_min + t_all_max) / 2.0
    print(f"[OK] {len(conflicts)} conflict pairs | window: {t_all_min:.1f}s -> {t_all_max:.1f}s")
    print(f"[OK] Apex time for S2: t={t_apex:.2f}s")

    conflict = _select_apex_conflict(conflicts)
    t_conflict = conflict["time_range"][0]
    t_anchor = max(fp_pleb.init_time() + ANCHOR_DELTA, t_conflict - ANCHOR_DELTA)

    sat_res = generate_mtv_candidates(conflict, detector, flight_plan_pleb=fp_pleb, t_ref=t_apex)
    if not sat_res.horizontal_mtvs:
        print("[ERROR] No horizontal MTV found for Strategy 2.")
        return

    mtv = sat_res.horizontal_mtvs[0]
    print(f"[OK] MTV used by S2: {np.round(mtv, 2)}")

    pleb_boxes = detector.uavs[1]["boxes"]
    vip_boxes = detector.uavs[2]["boxes"]

    pleb_idx = [i for i, box in enumerate(pleb_boxes) if box.t_range[1] > t_all_min and box.t_range[0] < t_all_max]
    vip_idx = [i for i, box in enumerate(vip_boxes) if box.t_range[1] > t_all_min and box.t_range[0] < t_all_max]

    conflict_obbs_pleb = [pleb_boxes[i] for i in range(min(pleb_idx), min(max(pleb_idx) + 2, len(pleb_boxes) - 1) + 1)]
    conflict_obbs_vip = [vip_boxes[i] for i in range(min(vip_idx), min(max(vip_idx) + 2, len(vip_boxes) - 1) + 1)]
    print(f"[OK] Conflict OBBs (pleb): {len(conflict_obbs_pleb)}")
    print(f"[OK] Conflict OBBs (vip):  {len(conflict_obbs_vip)}")

    print("\nBuilding S2 detour (iterating MTV scales)...")
    best_fp = None
    best_scale = None
    scales_to_try = [1.0, 1.25, 1.5, 2.0]

    for scale in scales_to_try:
        scaled_mtv = mtv * scale
        print(f"-> Testing scale {scale}x | MTV: {np.round(scaled_mtv, 2)}")

        fp_cand = build_rigid_shift_detour(fp_pleb, t_anchor, scaled_mtv, conflict_obbs_pleb)
        if fp_cand is None:
            print(f"   [FAIL] Kinematically infeasible at scale {scale}x")
            continue

        best_fp = fp_cand
        best_scale = scale

        detector_post = RTreeDetector()
        detector_post.register_uav(1, fp_cand, interval=0.5)
        detector_post.register_uav(2, fp_vip, interval=0.5)
        remaining = detector_post.detect_all_conflicts(1)
        if not remaining:
            print(f"   [SUCCESS] 0 conflicts remaining at scale {scale}x")
            break
        print(f"   [WARN] {len(remaining)} conflict(s) remain at scale {scale}x")

    if best_fp is None:
        print("[ERROR] S2 could not generate a valid detour.")
        return

    fp_new = best_fp
    print(f"\n[OK] Selected S2 scale: {best_scale}x")
    print("Detour waypoints:")
    for wp in fp_new.waypoints:
        display = _display_label(wp.label)
        arrow = "->" if display in ("anc", "det", "ret") else "  "
        print(
            f"  {arrow} [{display:3}] t={wp.t:6.2f}s  pos=({wp.pos[0]:.1f}, {wp.pos[1]:.1f}, {wp.pos[2]:.1f})"
        )

    fp_new.radius = fp_pleb.radius
    fp_new.max_var_lin_vel = fp_pleb.max_var_lin_vel
    fp_new.max_var_ang_vel = fp_pleb.max_var_ang_vel

    detector_post = RTreeDetector()
    detector_post.register_uav(1, fp_new, interval=0.5)
    detector_post.register_uav(2, fp_vip, interval=0.5)
    final_remaining = detector_post.detect_all_conflicts(1)
    print("\n--- RTree Post-Resolution Verification ---")
    if final_remaining:
        print(f"[WARN] {len(final_remaining)} conflict(s) STILL REMAIN in the plotted detour!")
    else:
        print("[RESOLVED] Plotted detour is conflict-free.")
    print("------------------------------------------------")

    tr_pleb = fp_pleb.trace(0.1)
    tr_new = fp_new.trace(0.1)
    tr_vip = fp_vip.trace(0.1)

    fig = plt.figure(figsize=(18, 8), facecolor=BG)
    fig.canvas.manager.set_window_title("VIZ 14 — Perpendicular Crossing — Strategy 2")
    fig.text(
        0.5,
        0.97,
        "Strategy 2 (Rigid Shift / Triangle) — PERPENDICULAR CROSSING",
        ha="center",
        fontsize=14,
        fontweight="bold",
        color=FG,
    )

    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax1.set_facecolor(BG)
    ax1.set_title("❌ BEFORE: Perpendicular Conflict", color="#e74c3c", fontsize=12, fontweight="bold", pad=10)
    ax1.plot(tr_pleb[:, 1], tr_pleb[:, 2], tr_pleb[:, 3], color="#7f8c8d", linewidth=2.5, label="Plebeian Route")
    ax1.plot(tr_vip[:, 1], tr_vip[:, 2], tr_vip[:, 3], color="#e74c3c", linewidth=2.5, label="VIP Route")

    mid = np.array([200.0, 200.0, 100.0])
    ax1.text(
        mid[0],
        mid[1],
        mid[2] + 14,
        f"⚠️ CONFLICT\n{len(conflict_obbs_pleb)} pleb OBBs\n{len(conflict_obbs_vip)} vip OBBs\nΔt={t_all_max - t_all_min:.1f}s",
        color="#e74c3c",
        fontsize=9,
        fontweight="bold",
        ha="center",
    )
    _configure_ax(ax1)
    ax1.legend(facecolor=BG, edgecolor="#cfcfcf", labelcolor=FG, loc="upper left", fontsize=9)

    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    ax2.set_facecolor(BG)
    ax2.set_title("✅ AFTER: Resolved via S2", color="#2ecc71", fontsize=12, fontweight="bold", pad=10)
    ax2.plot(tr_pleb[:, 1], tr_pleb[:, 2], tr_pleb[:, 3], color="#7f8c8d", linewidth=1.5, linestyle="--", alpha=0.35, label="Original (avoided)")
    ax2.plot(tr_new[:, 1], tr_new[:, 2], tr_new[:, 3], color="#2ecc71", linewidth=3.0, label="S2 Detour")
    ax2.plot(tr_vip[:, 1], tr_vip[:, 2], tr_vip[:, 3], color="#e74c3c", linewidth=2.5, label="VIP Route")

    seen_labels = set()
    for wp in fp_new.waypoints:
        display = _display_label(wp.label)
        label = None if display in seen_labels else {
            "anc": "Anchor (anc)",
            "det": "Apex (det)",
            "ret": "Return (ret)",
        }.get(display, display)
        seen_labels.add(display)

        if display == "anc":
            ax2.scatter(*wp.pos, color="#e67e22", s=100, marker="s", zorder=5, edgecolors="white", linewidths=1.5, label=label)
        elif display == "det":
            ax2.scatter(*wp.pos, color="#3498db", s=140, marker="^", zorder=5, edgecolors="white", linewidths=1.5, label=label)
        elif display == "ret":
            ax2.scatter(*wp.pos, color="#1abc9c", s=100, marker="D", zorder=5, edgecolors="white", linewidths=1.5, label=label)

    _configure_ax(ax2)
    ax2.legend(facecolor=BG, edgecolor="#cfcfcf", labelcolor=FG, loc="upper left", fontsize=9)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


if __name__ == "__main__":
    run_visualizer()
