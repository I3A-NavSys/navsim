"""
benchmark_resolutions.py — 3-Case Resolution Benchmark
======================================================

PURPOSE:
    This script demonstrates the behavior of the conflict resolution system
    by creating three distinct scenarios based on the exact same initial flight
    plans. It explicitly shows:
    
    - Case 1: The original crossing flight plans causing a conflict.
    - Case 2: The default resolution provided by the CentralManager 
              (typically Strategy 1 - Kinematic Bounding / Velocity Adjustments).
    - Case 3: Forced spatial resolution using the Minimum Translation Vectors 
              (MTVs) strategy (Strategy 2 - Rigid Shift / Path Stretch).

USAGE:
    Run this file directly:
    $ python benchmark/benchmark_resolutions.py
"""

import sys
from pathlib import Path
import unittest.mock

# Ensure the parent directory is in sys.path so we can import 'core' and others
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.flight_plan import FlightPlan
from central_manager import CentralManager
from benchmark.flightplan_generator import generate_crossing_pair

# Common seed to ensure we get the exact same conflict in all 3 cases
SEED = 42

def print_fp_summary(fp: FlightPlan, name: str):
    """Helper to print a quick summary of a FlightPlan's waypoints."""
    print(f"  [{name}] Waypoints ({len(fp.waypoints)}):")
    for wp in fp.waypoints:
        # wp.pos is a list or array, wp.vel as well
        pos_str = f"({wp.pos[0]:.1f}, {wp.pos[1]:.1f}, {wp.pos[2]:.1f})"
        print(f"    - {wp.label:8s} | t={wp.t:6.2f}s | pos={pos_str}")

def run_case_1():
    print("====================================================================")
    print(" CASE 1: UNRESOLVED CONFLICT")
    print("====================================================================")
    print("Description: Two UAVs are on a collision course.")
    
    # 1. Generate the initial crossing pair
    fp_vip, fp_pleb = generate_crossing_pair(seed=SEED)
    
    # 2. Register them in the CentralManager
    cm = CentralManager()
    cm.register_uav("VIP_UAV", fp_vip, priority=2)
    cm.register_uav("PLEB_UAV", fp_pleb, priority=0)
    
    # 3. Detect conflicts without resolving
    conflicts = cm._rtree_detector.detect_all_conflicts("PLEB_UAV")
    
    if conflicts:
        print(f"\n=> DETECTED {len(conflicts)} CONFLICT(S).")
        print(f"   Earliest conflict time range: {conflicts[0]['time_range']}")
    else:
        print("\n=> NO CONFLICT DETECTED.")
        
    print("\nFlight Plans (Original):")
    print_fp_summary(fp_vip, "VIP_UAV (Original)")
    print_fp_summary(fp_pleb, "PLEB_UAV (Original)")
    print("\n")


def run_case_2():
    print("====================================================================")
    print(" CASE 2: CENTRAL MANAGER RESOLUTION (KINEMATIC BOUNDING)")
    print("====================================================================")
    print("Description: The CentralManager resolves the conflict automatically.")
    print("By default, Strategy 1 (Kinematic Bounding) will try to solve it by")
    print("adjusting the velocity (accelerating/decelerating) without leaving")
    print("the planned path.")
    
    fp_vip, fp_pleb = generate_crossing_pair(seed=SEED)
    
    cm = CentralManager()
    cm.register_uav("VIP_UAV", fp_vip, priority=2)
    cm.register_uav("PLEB_UAV", fp_pleb, priority=0)
    
    # Check and resolve
    result = cm.check_and_resolve("PLEB_UAV")
    
    if result and result.success:
        print(f"\n=> RESOLUTION SUCCESS: {result.strategy_used}")
        print(f"   Message: {result.message}")
        
        print("\nFlight Plans (Resolved):")
        print_fp_summary(result.new_fp_vip, "VIP_UAV (Held & Released)")
        print_fp_summary(result.new_fp_pleb, "PLEB_UAV (Kinematic Adjusted)")
    else:
        print("\n=> RESOLUTION FAILED OR NO CONFLICT.")
    print("\n")


def run_case_3():
    print("====================================================================")
    print(" CASE 3: FORCED MTV STRATEGY (SPATIAL DETOUR)")
    print("====================================================================")
    print("Description: We resolve the exact same conflict, but this time we")
    print("simulate that Strategy 1 (Kinematic Bounding) failed. This forces")
    print("the system to use the MTVs to create a geometric detour (Strategy 2).")
    
    fp_vip, fp_pleb = generate_crossing_pair(seed=SEED)
    
    cm = CentralManager()
    cm.register_uav("VIP_UAV", fp_vip, priority=2)
    cm.register_uav("PLEB_UAV", fp_pleb, priority=0)
    
    # We patch 'run_strategy1' to always return None. This makes the ConflictResolver
    # believe that velocity adjustments are impossible, pushing it to Strategy 2.
    with unittest.mock.patch('resolution.conflict_resolver.run_strategy1', return_value=None):
        result = cm.check_and_resolve("PLEB_UAV")
        
    if result and result.success:
        print(f"\n=> RESOLUTION SUCCESS: {result.strategy_used}")
        print(f"   Message: {result.message}")
        
        print("\nFlight Plans (Resolved with Detour):")
        print_fp_summary(result.new_fp_vip, "VIP_UAV (Held & Released)")
        print_fp_summary(result.new_fp_pleb, "PLEB_UAV (Detour via MTV)")
    else:
        print("\n=> RESOLUTION FAILED OR NO CONFLICT.")
    print("\n")


if __name__ == "__main__":
    print("Starting UAV Conflict Resolution Benchmark...\n")
    run_case_1()
    run_case_2()
    run_case_3()
    print("Benchmark completed.")
