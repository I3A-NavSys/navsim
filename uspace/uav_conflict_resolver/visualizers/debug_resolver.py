"""
debug_resolver.py — Verbose step-by-step debug of the ConflictResolver cascade
================================================================================
Run from the uav_conflict_resolver/ directory:
    python -m visualizers.debug_resolver
or:
    python visualizers/debug_resolver.py
"""

import sys
from pathlib import Path
import numpy as np
import traceback

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

# ─── Monkey-patch key functions with verbose wrappers ────────────────────────

from core.config import (
    ANCHOR_DELTA, OBB_INTERVAL, HOVER_MAX_TIMEOUT, HOVER_TIME_STEP,
    WAYPOINT_TIME_EPSILON, FORWARD_PROGRESS_MARGIN, RECTILINEAR_THRESHOLD,
)

SEP  = "─" * 70
SEP2 = "═" * 70


def section(title: str):
    print(f"\n{SEP2}")
    print(f"  {title}")
    print(SEP2)


def subsection(title: str):
    print(f"\n{SEP}")
    print(f"  {title}")
    print(SEP)


# ─── Imports ─────────────────────────────────────────────────────────────────

from central_manager import CentralManager
from visualizers.flightplan_generator import generate_crossing_pair
from detection.rtree_detector import RTreeDetector
from resolution.conflict_resolver import ConflictResolver, ResolveResult
from resolution.geometry.sat_mtv import generate_mtv_candidates
from resolution.geometry.path_geometry import (
    build_trapezoid_detour, validate_curve_kinematics,
)
from resolution.kinematic.velocity_bounding import (
    run_strategy1, _find_segment_index, _is_segment_rectilinear,
    _decide_maneuver_direction, _compute_required_delta_v, _build_shadow_rtree,
    _validate_against_shadow, connect_urm2,
)
from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint


# =============================================================================
# Verbose helpers
# =============================================================================

def _show_flight_plan(label: str, fp: FlightPlan):
    print(f"\n  [{label}]  id={fp.id}  priority={fp.priority}  "
          f"t=[{fp.init_time():.3f}, {fp.finish_time():.3f}]s  "
          f"#wps={len(fp.waypoints)}")
    for i, wp in enumerate(fp.waypoints):
        spd = float(np.linalg.norm(wp.vel))
        print(f"    WP[{i}] '{wp.label}'  t={wp.t:.3f}s  "
              f"pos=({wp.pos[0]:.1f},{wp.pos[1]:.1f},{wp.pos[2]:.1f})  "
              f"|v|={spd:.2f} m/s")


def _show_conflict(conflict: dict):
    print(f"  uav_a={conflict.get('uav_a')}  uav_b={conflict.get('uav_b')}")
    print(f"  time_range={conflict.get('time_range')}")
    mtv = conflict.get('mtv')
    if mtv is not None:
        print(f"  mtv={np.array(mtv).round(4)}")
    print(f"  box_a_idx={conflict.get('box_a_idx')}  box_b_idx={conflict.get('box_b_idx')}")


# =============================================================================
# Main debug routine
# =============================================================================

def run_debug():
    section("STEP 0 — Generate crossing pair (seed=42)")
    fp1, fp2 = generate_crossing_pair(seed=42)
    fp1.priority = 10   # VIP
    fp2.priority = 5    # Plebeian
    _show_flight_plan("FP1 (VIP)", fp1)
    _show_flight_plan("FP2 (PLEB)", fp2)

    # ─── Setup manager ───────────────────────────────────────────────────────
    section("STEP 1 — Register UAVs in CentralManager")
    manager = CentralManager()
    manager.register_uav("UAV_1", fp1, priority=10)
    manager.register_uav("UAV_2", fp2, priority=5)

    n_boxes_vip  = len(manager._rtree_detector.uavs.get("UAV_1", {}).get("boxes", []))
    n_boxes_pleb = len(manager._rtree_detector.uavs.get("UAV_2", {}).get("boxes", []))
    print(f"  UAV_1 OBB boxes: {n_boxes_vip}")
    print(f"  UAV_2 OBB boxes: {n_boxes_pleb}")

    # ─── Detect conflicts ────────────────────────────────────────────────────
    section("STEP 2 — detect_all_conflicts for UAV_2")
    conflicts = manager._rtree_detector.detect_all_conflicts("UAV_2")
    print(f"  Conflicts found: {len(conflicts)}")
    for i, c in enumerate(conflicts):
        print(f"\n  Conflict #{i}:")
        _show_conflict(c)

    if not conflicts:
        print("  ✘  No conflicts detected — cannot test resolver.")
        return

    conflicts.sort(key=lambda c: c["time_range"][0])
    conflict = conflicts[0]
    print(f"\n  → Using conflict #0 (earliest t={conflict['time_range'][0]:.3f}s)")

    # ─── Priority & anchor ───────────────────────────────────────────────────
    section("STEP 3 — Determine VIP/Pleb and t_anchor")
    vip_id, pleb_id = "UAV_1", "UAV_2"
    fp_vip  = manager._flight_plans[vip_id]
    fp_pleb = manager._flight_plans[pleb_id]

    t_start    = fp_pleb.init_time()
    t_conflict = conflict["time_range"][0]
    t_anchor   = max(t_conflict, t_start) + ANCHOR_DELTA

    print(f"  t_start    = {t_start:.3f}s")
    print(f"  t_conflict = {t_conflict:.3f}s")
    print(f"  ANCHOR_DELTA = {ANCHOR_DELTA}s")
    print(f"  t_anchor   = {t_anchor:.3f}s")
    print(f"  fp_pleb.finish_time() = {fp_pleb.finish_time():.3f}s")

    if t_anchor >= fp_pleb.finish_time():
        print("  ✘  GUARD FAILED: t_anchor >= fp_pleb.finish_time() → immediate DEADLOCK")
        print(f"     Δ = {t_anchor - fp_pleb.finish_time():.3f}s beyond flight end")
        return
    else:
        print(f"  ✔  t_anchor within flight window "
              f"(margin = {fp_pleb.finish_time() - t_anchor:.3f}s)")

    # ─── Hold VIP ────────────────────────────────────────────────────────────
    section("STEP 4 — Hold VIP (Semaphore)")
    resolver = ConflictResolver(manager._rtree_detector)
    try:
        fp_vip_held = resolver._hold_vip(fp_vip, t_anchor)
        _show_flight_plan("VIP (HELD)", fp_vip_held)
    except Exception as e:
        print(f"  ✘  _hold_vip CRASHED: {e}")
        traceback.print_exc()
        return

    # ─── Strategy 1 ──────────────────────────────────────────────────────────
    section("STEP 5 — Strategy 1: Kinematic Bounding (ConnectURM2)")

    # Sub-checks before calling run_strategy1
    seg_idx = _find_segment_index(fp_pleb, t_anchor)
    print(f"  segment_idx at t_anchor: {seg_idx}")

    if seg_idx is None:
        print("  ✘  t_anchor is outside fp_pleb bounds → S1 immediately returns None")
    else:
        wp_i    = fp_pleb.waypoints[seg_idx]
        wp_next = fp_pleb.waypoints[seg_idx + 1]
        is_rect = _is_segment_rectilinear(wp_i, wp_next)
        print(f"  Segment [{seg_idx}] '{wp_i.label}' → [{seg_idx+1}] '{wp_next.label}'")
        print(f"    acel={np.array(wp_i.acel).round(6)}")
        print(f"    jerk={np.array(wp_i.jerk).round(6)}")
        print(f"    snap={np.array(wp_i.snap).round(6)}")
        print(f"    crakle={np.array(wp_i.crakle).round(6)}")
        print(f"    RECTILINEAR_THRESHOLD = {RECTILINEAR_THRESHOLD}")
        print(f"  is_segment_rectilinear: {is_rect}")
        if not is_rect:
            print("  ✘  Segment is curved → S1 guard clause fires → returns None immediately")
        else:
            maneuver = _decide_maneuver_direction(fp_pleb, fp_vip_held, t_anchor)
            print(f"  Maneuver direction: {maneuver}")
            delta_v = _compute_required_delta_v(fp_pleb, fp_vip_held, t_anchor, maneuver)
            print(f"  Required Δv: {delta_v}")
            v_current = float(np.linalg.norm(fp_pleb.status_at_time(t_anchor).vel))
            max_dv    = fp_pleb.max_var_lin_vel
            print(f"  v_current = {v_current:.3f} m/s   max_var_lin_vel = {max_dv:.3f} m/s")
            if delta_v is not None and delta_v > max_dv:
                print(f"  ✘  Required Δv={delta_v:.3f} > max_dv={max_dv:.3f} → Golden Rule inversion")

    # Now actually run S1
    try:
        result_s1 = run_strategy1(
            fp_pleb=fp_pleb,
            fp_vip=fp_vip_held,
            pleb_id=pleb_id,
            t_anchor=t_anchor,
            manager=manager._rtree_detector,
            t_conflict=t_conflict,
        )
    except Exception as e:
        print(f"  ✘  run_strategy1 CRASHED: {e}")
        traceback.print_exc()
        result_s1 = None

    if result_s1 is not None:
        print("  ✔  Strategy 1 SUCCEEDED!")
        _show_flight_plan("S1 result", result_s1)
        return
    else:
        print("  ✘  Strategy 1 FAILED → cascade continues")

    # ─── Build shadow R-Tree ─────────────────────────────────────────────────
    section("STEP 6 — Build shadow R-Tree (excluding plebeian)")
    try:
        shadow = resolver._build_shadow_rtree(pleb_id)
        n_shadow = len(shadow.uavs)
        print(f"  Shadow R-Tree UAVs: {list(shadow.uavs.keys())}  (total={n_shadow})")
        for uid, data in shadow.uavs.items():
            print(f"    {uid}: {len(data['boxes'])} boxes")
    except Exception as e:
        print(f"  ✘  _build_shadow_rtree CRASHED: {e}")
        traceback.print_exc()
        return

    # ─── SAT MTV candidates ──────────────────────────────────────────────────
    section("STEP 7 — SAT MTV Generation")
    try:
        sat_result = generate_mtv_candidates(
            conflict=conflict,
            manager=manager._rtree_detector,
        )
        print(f"  SAT success: {sat_result.success}")
        print(f"  Horizontal MTVs ({len(sat_result.horizontal_mtvs)}):")
        for i, m in enumerate(sat_result.horizontal_mtvs):
            print(f"    [{i}] {np.array(m).round(4)}  |mag|={np.linalg.norm(m):.4f}m")
        print(f"  Vertical MTVs ({len(sat_result.vertical_mtvs)}):")
        for i, m in enumerate(sat_result.vertical_mtvs):
            print(f"    [{i}] {np.array(m).round(4)}  |mag|={np.linalg.norm(m):.4f}m")

        if not sat_result.success:
            print("  ✘  SAT failed — no MTVs generated")
            sat_result.horizontal_mtvs = []
            sat_result.vertical_mtvs   = []
    except Exception as e:
        print(f"  ✘  generate_mtv_candidates CRASHED: {e}")
        traceback.print_exc()
        sat_result = type('SR', (), {'success': False, 'horizontal_mtvs': [], 'vertical_mtvs': []})()

    # ─── Strategy 2 ──────────────────────────────────────────────────────────
    section("STEP 8 — Strategy 2: Horizontal Path Stretch")

    # Retrieve conflict OBB
    pleb_id_key = conflict.get("uav_a", pleb_id)
    box_a_idx   = conflict.get("box_a_idx", None)
    conflict_obb = None
    if box_a_idx is not None:
        try:
            conflict_obb = manager._rtree_detector.uavs[pleb_id_key]["boxes"][box_a_idx]
            print(f"  conflict_obb retrieved (box_a_idx={box_a_idx})")
            print(f"    center={np.array(conflict_obb.center).round(2)}")
            print(f"    half_extents={np.array(conflict_obb.half_extents).round(2)}")
        except (KeyError, IndexError) as e:
            print(f"  ✘  Could not retrieve conflict_obb: {e}")
    else:
        print("  conflict_obb: None (box_a_idx not in conflict dict)")

    for i, mtv in enumerate(sat_result.horizontal_mtvs):
        print(f"\n  ── H-MTV [{i}]: {np.array(mtv).round(4)}")
        try:
            candidate = build_trapezoid_detour(
                fp=fp_pleb,
                t_anchor=t_anchor,
                mtv=mtv,
                conflict_obb=conflict_obb,
            )
        except Exception as e:
            print(f"    ✘  build_trapezoid_detour CRASHED: {e}")
            traceback.print_exc()
            continue

        if candidate is None:
            print("    ✘  build_trapezoid_detour → None (geometry degenerate or Bézier failed)")
            continue

        print(f"    ✔  Candidate built: {len(candidate.waypoints)} waypoints")
        _show_flight_plan(f"S2 candidate [{i}]", candidate)

        # Kinematics check
        kin_ok = resolver._validate_detour_kinematics(candidate, t_anchor, fp_pleb)
        print(f"    Kinematics check: {'✔ OK' if kin_ok else '✘ FAILED'}")
        if not kin_ok:
            continue

        # Shadow validation
        shadow.register_uav(pleb_id, candidate, interval=OBB_INTERVAL)
        new_conflicts = shadow.detect_all_conflicts(pleb_id)
        print(f"    Shadow conflicts after candidate: {len(new_conflicts)}")
        for ci, cc in enumerate(new_conflicts):
            print(f"      Conflict #{ci}: t={cc['time_range'][0]:.3f}s "
                  f"(solving t={t_conflict:.3f}s, margin={FORWARD_PROGRESS_MARGIN}s)")
            is_fwd = cc['time_range'][0] >= t_conflict + FORWARD_PROGRESS_MARGIN
            print(f"        forward_progress: {is_fwd}")

        valid = resolver._validate_shadow(candidate, pleb_id, shadow, t_conflict)
        print(f"    _validate_shadow: {'✔ ACCEPTED' if valid else '✘ REJECTED'}")
        if valid:
            print(f"\n  ✔  Strategy 2 SUCCEEDED via H-MTV [{i}]!")
            return

    if not sat_result.horizontal_mtvs:
        print("  (no horizontal MTVs to try)")
    print("  ✘  Strategy 2 FAILED → cascade continues")

    # ─── Fallback 1 ──────────────────────────────────────────────────────────
    section("STEP 9 — Fallback 1: Vertical MTVs")
    for i, mtv in enumerate(sat_result.vertical_mtvs):
        print(f"\n  ── V-MTV [{i}]: {np.array(mtv).round(4)}")
        try:
            candidate = build_trapezoid_detour(
                fp=fp_pleb,
                t_anchor=t_anchor,
                mtv=mtv,
                conflict_obb=conflict_obb,
            )
        except Exception as e:
            print(f"    ✘  build_trapezoid_detour CRASHED: {e}")
            traceback.print_exc()
            continue

        if candidate is None:
            print("    ✘  build_trapezoid_detour → None")
            continue

        print(f"    ✔  Candidate built: {len(candidate.waypoints)} waypoints")
        shadow.register_uav(pleb_id, candidate, interval=OBB_INTERVAL)
        new_conflicts = shadow.detect_all_conflicts(pleb_id)
        print(f"    Shadow conflicts after candidate: {len(new_conflicts)}")

        valid = resolver._validate_shadow(candidate, pleb_id, shadow, t_conflict)
        print(f"    _validate_shadow: {'✔ ACCEPTED' if valid else '✘ REJECTED'}")
        if valid:
            print(f"\n  ✔  Fallback 1 SUCCEEDED via V-MTV [{i}]!")
            return

    if not sat_result.vertical_mtvs:
        print("  (no vertical MTVs to try)")
    print("  ✘  Fallback 1 FAILED → cascade continues")

    # ─── Fallback 2 ──────────────────────────────────────────────────────────
    section("STEP 10 — Fallback 2: Time-Shift (Hovering)")
    max_iters     = int(HOVER_MAX_TIMEOUT / HOVER_TIME_STEP)
    position_at_anchor = fp_pleb.status_at_time(t_anchor).pos.copy()
    print(f"  hover position: {position_at_anchor.round(2)}")
    print(f"  max_iters = {max_iters}  (HOVER_MAX_TIMEOUT={HOVER_MAX_TIMEOUT}s / HOVER_TIME_STEP={HOVER_TIME_STEP}s)")

    accepted_fb2 = False
    for iteration in range(1, min(max_iters + 1, 6)):   # Show first 5 iterations only
        hover_duration = iteration * HOVER_TIME_STEP
        t_resume       = t_anchor + hover_duration

        candidate = fp_pleb.copy()
        hover_start = Waypoint(label="HOV_S", t=t_anchor,
                               pos=position_at_anchor, vel=[0., 0., 0.])
        hover_end   = Waypoint(label="HOV_E", t=round(t_resume, 3),
                               pos=position_at_anchor, vel=[0., 0., 0.])
        candidate.set_waypoint(hover_start)
        candidate.set_waypoint(hover_end)
        candidate.postpone_from(t_anchor + WAYPOINT_TIME_EPSILON, hover_duration)
        candidate.connect_waypoints()

        shadow.register_uav(pleb_id, candidate, interval=OBB_INTERVAL)
        new_conflicts = shadow.detect_all_conflicts(pleb_id)
        valid = resolver._validate_shadow(candidate, pleb_id, shadow, t_anchor)

        print(f"  Iter {iteration:3d}  hover={hover_duration:.1f}s  "
              f"t_resume={t_resume:.3f}s  "
              f"shadow_conflicts={len(new_conflicts)}  "
              f"valid={valid}")
        if valid:
            print(f"  ✔  Fallback 2 SUCCEEDED at iteration {iteration}!")
            accepted_fb2 = True
            break

    if not accepted_fb2:
        print(f"\n  (continuing hover search — showing first 5 iters only)")
        # Run the real FB2 silently
        fb2_result, iters = resolver._fallback2_timeshift(
            fp_pleb=fp_pleb,
            fp_vip=fp_vip_held,
            pleb_id=pleb_id,
            t_anchor=t_anchor,
            shadow=shadow,
        )
        if fb2_result is not None:
            print(f"  ✔  Fallback 2 SUCCEEDED at iteration {iters}")
        else:
            print(f"  ✘  Fallback 2 FAILED after {iters} iterations → DEADLOCK")

    section("DEBUG COMPLETE")
    print("  If DEADLOCK was reached, look at the steps above for the root cause.")


if __name__ == "__main__":
    run_debug()
