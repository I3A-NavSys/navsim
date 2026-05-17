"""
benchmark_resolution_cases.py - Three conflict-resolution cases
===============================================================

PURPOSE:
    This script creates one deterministic crossing pair of flight plans and
    shows three different outcomes built from the same input plans:

    Case 1: The original pair is registered and the conflict is reported.
    Case 2: The CentralManager resolves the conflict automatically.
    Case 3: The same conflict is resolved explicitly through the MTV branch.

WHY THIS FILE EXISTS:
    The existing demo script is useful, but the third case is fragile because it
    depends on a monkey patch. This version keeps the demonstration focused and
    makes the MTV branch explicit so the result is easier to reproduce.

USAGE:
    Run from the project root:

        python benchmark/benchmark_resolution_cases.py

OUTPUT:
    Each case prints the relevant flight plans and their waypoints so you can
    compare the unresolved conflict, the manager solution, and the MTV-based
    detour side by side.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional, Tuple

# ---------------------------------------------------------------------------
# Make the project root importable when this file is executed directly.
# ---------------------------------------------------------------------------
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from benchmark.flightplan_generator import generate_crossing_pair
from central_manager import CentralManager
from core.config import ANCHOR_DELTA, WAYPOINT_TIME_EPSILON
from core.models.flight_plan import FlightPlan
from resolution.geometry.path_geometry import build_rigid_shift_detour
from resolution.geometry.sat_mtv import generate_mtv_candidates


SEED = 42
VIP_ID = "VIP_UAV"
PLEB_ID = "PLEB_UAV"


def print_fp_summary(fp: FlightPlan, title: str) -> None:
    """Print a compact waypoint summary for one flight plan."""
    print(f"  [{title}] Waypoints ({len(fp.waypoints)}):")
    for wp in fp.waypoints:
        pos = f"({wp.pos[0]:.1f}, {wp.pos[1]:.1f}, {wp.pos[2]:.1f})"
        print(f"    - {wp.label:8s} | t={wp.t:6.2f}s | pos={pos}")


def build_demo_manager() -> Tuple[CentralManager, FlightPlan, FlightPlan]:
    """Create the same deterministic crossing pair used in every case."""
    fp_vip, fp_pleb = generate_crossing_pair(seed=SEED)

    manager = CentralManager()
    manager.register_uav(VIP_ID, fp_vip, priority=2)
    manager.register_uav(PLEB_ID, fp_pleb, priority=0)
    return manager, fp_vip, fp_pleb


def get_primary_conflict(manager: CentralManager) -> dict:
    """Return the earliest conflict for the demo plebeian UAV."""
    conflicts = manager._rtree_detector.detect_all_conflicts(PLEB_ID)
    if not conflicts:
        raise RuntimeError("No conflict detected for the demo pair.")

    conflicts.sort(key=lambda item: item["time_range"][0])
    return conflicts[0]


def split_vip_and_pleb(manager: CentralManager, conflict: dict) -> Tuple[str, str]:
    """Apply the same priority and tie-break rules used by CentralManager."""
    uav_a_id = conflict["uav_a"]
    uav_b_id = conflict["uav_b"]

    priority_a = manager._priorities.get(uav_a_id, 0)
    priority_b = manager._priorities.get(uav_b_id, 0)

    if priority_a >= priority_b:
        if priority_a == priority_b and uav_a_id > uav_b_id:
            return uav_b_id, uav_a_id
        return uav_a_id, uav_b_id

    return uav_b_id, uav_a_id


def compute_t_anchor(fp_pleb: FlightPlan, t_conflict: float) -> float:
    """Mirror the resolver's anchor-time logic for the explicit MTV demo."""
    t_start = fp_pleb.init_time()
    t_anchor = max(t_start + ANCHOR_DELTA, t_conflict - ANCHOR_DELTA)

    if t_anchor >= t_conflict:
        t_anchor = t_conflict - WAYPOINT_TIME_EPSILON

    if t_anchor < t_start:
        raise RuntimeError("Could not place the anchor before the conflict.")

    if t_anchor >= fp_pleb.finish_time():
        raise RuntimeError("Anchor time lies beyond the plebeian flight end.")

    return t_anchor


def run_case_1() -> None:
    print("====================================================================")
    print("CASE 1: ORIGINAL CROSSING PAIR")
    print("====================================================================")
    print("Description: The two UAVs fly through the same area at the same time.")

    manager, fp_vip, fp_pleb = build_demo_manager()
    conflict = get_primary_conflict(manager)

    print(f"\n=> Detected {1 if conflict else 0} conflict(s).")
    print(f"   Earliest conflict window: {conflict['time_range']}")

    print("\nFlight plans (original):")
    print_fp_summary(fp_vip, f"{VIP_ID} (original)")
    print_fp_summary(fp_pleb, f"{PLEB_ID} (original)")
    print()


def run_case_2() -> None:
    print("====================================================================")
    print("CASE 2: CENTRAL MANAGER RESOLUTION")
    print("====================================================================")
    print("Description: The manager resolves the same conflict automatically.")
    print("The system will usually try Strategy 1 first (kinematic bounding).")

    manager, _, _ = build_demo_manager()
    result = manager.check_and_resolve(PLEB_ID)

    if result is None:
        print("\n=> No conflict was detected.")
        print()
        return

    if result.success:
        print(f"\n=> Resolution success: {result.strategy_used}")
        print(f"   Message: {result.message}")
        print("\nFlight plans (resolved by manager):")
        print_fp_summary(result.new_fp_vip, f"{VIP_ID} (held then released)")
        print_fp_summary(result.new_fp_pleb, f"{PLEB_ID} (manager solution)")
    else:
        print("\n=> Resolution failed.")
        print(f"   Message: {result.message}")

    print()


def run_case_3() -> None:
    print("====================================================================")
    print("CASE 3: EXPLICIT MTV-BASED RESOLUTION")
    print("====================================================================")
    print("Description: The same conflict is solved by extracting an MTV and")
    print("passing it directly to the rigid-shift detour builder.")

    manager, fp_vip, fp_pleb = build_demo_manager()
    conflict = get_primary_conflict(manager)

    t_conflict = conflict["time_range"][0]
    t_anchor = compute_t_anchor(fp_pleb, t_conflict)

    sat_result = generate_mtv_candidates(
        conflict=conflict,
        manager=manager._rtree_detector,
        flight_plan_pleb=fp_pleb,
        t_ref=(conflict["time_range"][0] + conflict["time_range"][1]) / 2.0,
    )

    if not sat_result.success or (not sat_result.horizontal_mtvs and not sat_result.vertical_mtvs):
        print("\n=> MTV extraction failed.")
        print()
        return

    mtv = sat_result.horizontal_mtvs[0] if sat_result.horizontal_mtvs else sat_result.vertical_mtvs[0]

    pleb_id = split_vip_and_pleb(manager, conflict)[1]
    t_min, t_max = conflict["time_range"]
    conflict_obbs = [
        box
        for box in manager._rtree_detector.uavs[pleb_id]["boxes"]
        if box.t_range[1] > t_min and box.t_range[0] < t_max
    ]

    print(f"\n=> MTV extracted: {mtv}")
    print(f"   t_anchor:      {t_anchor:.2f}s")

    candidate = build_rigid_shift_detour(
        fp=fp_pleb,
        t_anchor=t_anchor,
        mtv=mtv,
        conflict_obbs=conflict_obbs,
    )

    if candidate is None:
        print("\n=> MTV-based detour could not be built.")
        print()
        return

    print("\n=> MTV detour generated successfully.")
    print("\nFlight plans (MTV solution):")
    print_fp_summary(fp_vip, f"{VIP_ID} (reference route)")
    print_fp_summary(candidate, f"{PLEB_ID} (MTV detour)")
    print()


def main() -> None:
    print("Starting UAV conflict-resolution benchmark...\n")
    run_case_1()
    run_case_2()
    run_case_3()
    print("Benchmark completed.")


if __name__ == "__main__":
    main()