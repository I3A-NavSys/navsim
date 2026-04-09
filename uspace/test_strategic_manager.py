"""
Test Suite for StrategicManager with 4D R-Tree Collision Detection

This test module validates the StrategicManager class against the 15 standard
test scenarios. The StrategicManager uses a 4D R-Tree for efficient broad-phase
collision detection combined with SAT (Separating Axis Theorem) for precise
narrow-phase collision tests.

Scenarios tested:
1. Direct collision - Same trajectory, same time
2. Head-on collision - UAVs flying toward each other
3. Crossing paths - Perpendicular trajectories
4. Near miss - Close but no collision
5. Different altitudes - Vertical separation
6. Parallel paths - Safe horizontal separation
7. Same path, different times - Temporal separation
8. Complex maneuver - Multiple waypoints intersection
9. Overtaking - Same direction with lateral separation
10. Spiral maneuver - Convergent helical trajectories
11. Takeoff conflict - Uncoordinated takeoff
12. Landing conflict - Uncoordinated landing
13. Near miss at different times - Temporal separation at critical point
14. Large safety radius - Collision with large safety zones
15. Sharp turn conflict - Crossing at turn point
"""

import sys
from pathlib import Path
import time

# Add the parent directory to the path for imports
sys.path.insert(0, str(Path(__file__).parent))

from flight_plan import FlightPlan
from flight_plan.manager import StrategicManager


# =============================================================================
# SCENARIO 1: DIRECT COLLISION
# =============================================================================
def test_scenario_1_direct_collision():
    """
    Two UAVs flying on the SAME straight line and time.
    EXPECTED RESULT: CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 1: DIRECT COLLISION")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 0, 0], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 0], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[0, 0, 0], vel=[10, 0, 0])
    fp2.set_waypoint(time=10, pos=[100, 0, 0], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: Two UAVs on the same trajectory")
    print(f"Expected: CONFLICT ✓")
    print(f"Result: {'CONFLICT DETECTED ✓' if conflicts else 'NO CONFLICT ✗'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    if conflicts:
        print(f"Conflicts found: {len(conflicts)}")
        for c in conflicts[:3]:  # Show first 3 conflicts
            print(f"  - UAVs {c['uav_a']} ↔ {c['uav_b']}: boxes {c['box_a_idx']} ↔ {c['box_b_idx']}, time: {c['time_range']}")
    
    return len(conflicts) > 0, detect_time


# =============================================================================
# SCENARIO 2: HEAD-ON COLLISION
# =============================================================================
def test_scenario_2_head_on_collision():
    """
    Two UAVs flying toward each other on the same line.
    EXPECTED RESULT: CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 2: HEAD-ON COLLISION")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[100, 2, 10], vel=[-10, 0, 0])
    fp2.set_waypoint(time=10, pos=[0, 2, 10], vel=[-10, 0, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: UAVs flying toward each other")
    print(f"Expected: CONFLICT ✓")
    print(f"Result: {'CONFLICT DETECTED ✓' if conflicts else 'NO CONFLICT ✗'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    if conflicts:
        print(f"Conflicts found: {len(conflicts)}")
    
    return len(conflicts) > 0, detect_time


# =============================================================================
# SCENARIO 3: CROSSING PATHS
# =============================================================================
def test_scenario_3_crossing_paths():
    """
    Two UAVs crossing at a point (perpendicular).
    EXPECTED RESULT: CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 3: CROSSING PATHS")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 50, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 50, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[50, 0, 10], vel=[0, 10, 0])
    fp2.set_waypoint(time=10, pos=[50, 100, 10], vel=[0, 10, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: UAVs with perpendicular paths crossing at (50, 50, 10)")
    print(f"Expected: CONFLICT ✓")
    print(f"Result: {'CONFLICT DETECTED ✓' if conflicts else 'NO CONFLICT ✗'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    if conflicts:
        print(f"Conflicts found: {len(conflicts)}")
    
    return len(conflicts) > 0, detect_time


# =============================================================================
# SCENARIO 4: NEAR MISS
# =============================================================================
def test_scenario_4_near_miss():
    """
    Two UAVs passing very close but managing to avoid each other.
    EXPECTED RESULT: NO CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 4: NEAR MISS (SAFE)")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 1.5
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 1.5
    fp2.set_waypoint(time=0, pos=[50, -5, 10], vel=[0, 10, 0])
    fp2.set_waypoint(time=12, pos=[50, 115, 10], vel=[0, 10, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: UAVs pass 5m apart")
    print(f"Expected: NO CONFLICT ✓")
    print(f"Result: {'NO CONFLICT DETECTED ✓' if not conflicts else f'CONFLICT DETECTED ✗ ({len(conflicts)} conflicts)'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    
    return len(conflicts) == 0, detect_time


# =============================================================================
# SCENARIO 5: DIFFERENT ALTITUDES
# =============================================================================
def test_scenario_5_different_altitudes():
    """
    Two UAVs on the same horizontal trajectory but at different altitudes.
    EXPECTED RESULT: NO CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 5: DIFFERENT ALTITUDES (SAFE)")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[0, 0, 20], vel=[10, 0, 0])
    fp2.set_waypoint(time=10, pos=[100, 0, 20], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: UAV1 at 10m altitude, UAV2 at 20m altitude")
    print(f"Expected: NO CONFLICT ✓")
    print(f"Result: {'NO CONFLICT DETECTED ✓' if not conflicts else f'CONFLICT DETECTED ✗ ({len(conflicts)} conflicts)'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    
    return len(conflicts) == 0, detect_time


# =============================================================================
# SCENARIO 6: PARALLEL PATHS (SAFE)
# =============================================================================
def test_scenario_6_parallel_paths():
    """
    Two UAVs flying in parallel, well separated.
    EXPECTED RESULT: NO CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 6: PARALLEL PATHS (SAFE)")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[0, 20, 10], vel=[10, 0, 0])
    fp2.set_waypoint(time=10, pos=[100, 20, 10], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: Parallel paths 20m apart")
    print(f"Expected: NO CONFLICT ✓")
    print(f"Result: {'NO CONFLICT DETECTED ✓' if not conflicts else f'CONFLICT DETECTED ✗ ({len(conflicts)} conflicts)'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    
    return len(conflicts) == 0, detect_time


# =============================================================================
# SCENARIO 7: SAME PATH, DIFFERENT TIMES
# =============================================================================
def test_scenario_7_same_path_different_times():
    """
    Two UAVs on the same trajectory but at different times.
    EXPECTED RESULT: NO CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 7: SAME PATH, DIFFERENT TIMES (SAFE)")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=20, pos=[0, 0, 10], vel=[10, 0, 0])
    fp2.set_waypoint(time=30, pos=[100, 0, 10], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: UAV1 t=0-10s, UAV2 t=20-30s on same path")
    print(f"Expected: NO CONFLICT ✓")
    print(f"Result: {'NO CONFLICT DETECTED ✓' if not conflicts else f'CONFLICT DETECTED ✗ ({len(conflicts)} conflicts)'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    
    return len(conflicts) == 0, detect_time


# =============================================================================
# SCENARIO 8: COMPLEX MANEUVER WITH 3 WAYPOINTS
# =============================================================================
def test_scenario_8_complex_maneuver():
    """
    UAVs with more complex trajectories with multiple waypoints.
    EXPECTED RESULT: CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 8: COMPLEX MANEUVER (3 waypoints)")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[5, 5, 0])
    fp1.set_waypoint(time=10, pos=[50, 50, 10], vel=[5, -5, 0])
    fp1.set_waypoint(time=20, pos=[100, 0, 10], vel=[5, -5, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[100, 50, 10], vel=[-5, 0, 0])
    fp2.set_waypoint(time=10, pos=[50, 50, 10], vel=[-5, 0, 0])
    fp2.set_waypoint(time=20, pos=[0, 50, 10], vel=[-5, 0, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: Complex paths crossing at (50,50,10) at t=10s")
    print(f"Expected: CONFLICT ✓")
    print(f"Result: {'CONFLICT DETECTED ✓' if conflicts else 'NO CONFLICT ✗'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    if conflicts:
        print(f"Conflicts found: {len(conflicts)}")
    
    return len(conflicts) > 0, detect_time


# =============================================================================
# SCENARIO 9: OVERTAKING (SAFE)
# =============================================================================
def test_scenario_9_overtaking():
    """
    One UAV attempts to overtake another in the same direction by changing lanes.
    EXPECTED RESULT: NO CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 9: OVERTAKING (SAFE)")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[5, 0, 0])
    fp1.set_waypoint(time=20, pos=[100, 0, 10], vel=[5, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[0, 5, 10], vel=[10, 0, 0])
    fp2.set_waypoint(time=10, pos=[100, 5, 10], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: Overtaking with 5m lateral separation")
    print(f"Expected: NO CONFLICT ✓")
    print(f"Result: {'NO CONFLICT DETECTED ✓' if not conflicts else f'CONFLICT DETECTED ✗ ({len(conflicts)} conflicts)'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    
    return len(conflicts) == 0, detect_time


# =============================================================================
# SCENARIO 10: SPIRAL MANEUVER
# =============================================================================
def test_scenario_10_spiral_maneuver():
    """
    UAVs in convergent helical trajectories.
    EXPECTED RESULT: CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 10: SPIRAL MANEUVER")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[80, 50, 10], vel=[-5, 5, 0])
    fp1.set_waypoint(time=5, pos=[50, 80, 10], vel=[-5, -5, 0])
    fp1.set_waypoint(time=10, pos=[20, 50, 10], vel=[5, -5, 0])
    fp1.set_waypoint(time=15, pos=[50, 20, 10], vel=[5, 5, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[20, 50, 10], vel=[5, 5, 0])
    fp2.set_waypoint(time=5, pos=[50, 20, 10], vel=[5, -5, 0])
    fp2.set_waypoint(time=10, pos=[80, 50, 10], vel=[-5, -5, 0])
    fp2.set_waypoint(time=15, pos=[50, 80, 10], vel=[-5, 5, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: Convergent spiral trajectories")
    print(f"Expected: CONFLICT ✓")
    print(f"Result: {'CONFLICT DETECTED ✓' if conflicts else 'NO CONFLICT ✗'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    if conflicts:
        print(f"Conflicts found: {len(conflicts)}")
    
    return len(conflicts) > 0, detect_time


# =============================================================================
# SCENARIO 11: TAKEOFF CONFLICT
# =============================================================================
def test_scenario_11_takeoff_conflict():
    """
    Two UAVs taking off from the same point without coordination.
    EXPECTED RESULT: CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 11: TAKEOFF CONFLICT")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 0, 0], vel=[0, 0, 0])
    fp1.set_waypoint(time=1, pos=[0, 0, 1], vel=[0, 0, 1])
    fp1.set_waypoint(time=10, pos=[0, 0, 50], vel=[0, 0, 5])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[0, 0, 0], vel=[0, 0, 0])
    fp2.set_waypoint(time=1, pos=[1, 1, 1], vel=[1, 1, 1])
    fp2.set_waypoint(time=10, pos=[10, 10, 50], vel=[1, 1, 5])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: Uncoordinated takeoff - both starting from ground")
    print(f"Expected: CONFLICT ✓")
    print(f"Result: {'CONFLICT DETECTED ✓' if conflicts else 'NO CONFLICT ✗'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    if conflicts:
        print(f"Conflicts found: {len(conflicts)}")
    
    return len(conflicts) > 0, detect_time


# =============================================================================
# SCENARIO 12: LANDING CONFLICT
# =============================================================================
def test_scenario_12_landing_conflict():
    """
    Two UAVs landing at the same point without coordination.
    EXPECTED RESULT: CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 12: LANDING CONFLICT")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 0, 50], vel=[0, 0, -5])
    fp1.set_waypoint(time=10, pos=[0, 0, 0], vel=[0, 0, -5])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[10, 10, 50], vel=[-1, -1, -5])
    fp2.set_waypoint(time=10, pos=[0, 0, 0], vel=[-1, -1, -5])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: Uncoordinated landing - both converging to ground")
    print(f"Expected: CONFLICT ✓")
    print(f"Result: {'CONFLICT DETECTED ✓' if conflicts else 'NO CONFLICT ✗'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    if conflicts:
        print(f"Conflicts found: {len(conflicts)}")
    
    return len(conflicts) > 0, detect_time


# =============================================================================
# SCENARIO 13: NEAR MISS AT DIFFERENT TIMES
# =============================================================================
def test_scenario_13_near_miss_different_times():
    """
    Two UAVs on similar trajectories but reaching the critical point at different times.
    EXPECTED RESULT: NO CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 13: NEAR MISS AT DIFFERENT TIMES (SAFE)")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 1.5
    fp1.set_waypoint(time=0, pos=[0, 50, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 50, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 1.5
    fp2.set_waypoint(time=10, pos=[0, 50, 10], vel=[10, 0, 0])
    fp2.set_waypoint(time=20, pos=[100, 50, 10], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: UAV1 t=0-10s, UAV2 t=10-20s on same path")
    print(f"Expected: NO CONFLICT ✓")
    print(f"Result: {'NO CONFLICT DETECTED ✓' if not conflicts else f'CONFLICT DETECTED ✗ ({len(conflicts)} conflicts)'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    
    return len(conflicts) == 0, detect_time


# =============================================================================
# SCENARIO 14: LARGE SAFETY RADIUS COLLISION
# =============================================================================
def test_scenario_14_large_safety_radius():
    """
    Two UAVs with large safety radii (4m).
    EXPECTED RESULT: CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 14: LARGE SAFETY RADIUS COLLISION")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 4.0
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 4.0
    fp2.set_waypoint(time=0, pos=[50, -6, 10], vel=[0, 10, 0])
    fp2.set_waypoint(time=12, pos=[50, 114, 10], vel=[0, 10, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: Large safety radii (4m each) at 6m distance")
    print(f"Expected: CONFLICT ✓")
    print(f"Result: {'CONFLICT DETECTED ✓' if conflicts else 'NO CONFLICT ✗'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    if conflicts:
        print(f"Conflicts found: {len(conflicts)}")
    
    return len(conflicts) > 0, detect_time


# =============================================================================
# SCENARIO 15: SHARP TURN CONFLICT
# =============================================================================
def test_scenario_15_sharp_turn_conflict():
    """
    Two UAVs, one turns sharply crossing the other.
    EXPECTED RESULT: CONFLICT
    """
    print("\n" + "="*70)
    print("SCENARIO 15: SHARP TURN CONFLICT")
    print("="*70)
    
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[0, 10, 0])
    fp1.set_waypoint(time=10, pos=[0, 100, 10], vel=[0, 10, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    fp2.set_waypoint(time=0, pos=[-50, 50, 10], vel=[10, 0, 0])
    fp2.set_waypoint(time=5, pos=[50, 50, 10], vel=[0, 10, 0])
    fp2.set_waypoint(time=15, pos=[50, 150, 10], vel=[0, 10, 0])
    fp2.connect_waypoints()
    
    manager = StrategicManager()
    manager.register_uav("uav1", fp1, interval=0.5)
    manager.register_uav("uav2", fp2, interval=0.5)
    
    start_time = time.perf_counter()
    conflicts = manager.detect_all_conflicts("uav1")
    detect_time = time.perf_counter() - start_time
    
    print(f"Description: UAV2 makes sharp turn crossing UAV1 at (50,50,10)")
    print(f"Expected: CONFLICT ✓")
    print(f"Result: {'CONFLICT DETECTED ✓' if conflicts else 'NO CONFLICT ✗'}")
    print(f"Detection time: {detect_time*1000:.3f} ms")
    if conflicts:
        print(f"Conflicts found: {len(conflicts)}")
    
    return len(conflicts) > 0, detect_time


# =============================================================================
# TEST RUNNER
# =============================================================================
def run_all_tests():
    """Run all 15 test scenarios and report results."""
    
    print("\n")
    print("#" * 70)
    print("# STRATEGIC MANAGER 4D R-TREE COLLISION DETECTION TEST SUITE")
    print("#" * 70)
    print("# Testing StrategicManager with 15 standard scenarios")
    print("#" * 70)
    
    test_functions = [
        ("Direct Collision", test_scenario_1_direct_collision, True),
        ("Head-on Collision", test_scenario_2_head_on_collision, True),
        ("Crossing Paths", test_scenario_3_crossing_paths, True),
        ("Near Miss", test_scenario_4_near_miss, False),
        ("Different Altitudes", test_scenario_5_different_altitudes, False),
        ("Parallel Paths", test_scenario_6_parallel_paths, False),
        ("Same Path Different Times", test_scenario_7_same_path_different_times, False),
        ("Complex Maneuver", test_scenario_8_complex_maneuver, True),
        ("Overtaking", test_scenario_9_overtaking, False),
        ("Spiral Maneuver", test_scenario_10_spiral_maneuver, True),
        ("Takeoff Conflict", test_scenario_11_takeoff_conflict, True),
        ("Landing Conflict", test_scenario_12_landing_conflict, True),
        ("Near Miss Different Times", test_scenario_13_near_miss_different_times, False),
        ("Large Safety Radius", test_scenario_14_large_safety_radius, True),
        ("Sharp Turn Conflict", test_scenario_15_sharp_turn_conflict, True),
    ]
    
    results = []
    start_time = time.time()
    
    for i, (name, test_func, expected_conflict) in enumerate(test_functions, 1):
        try:
            result_data = test_func()
            result, detect_time = result_data
            passed = result == expected_conflict
            results.append((name, passed, expected_conflict, result, detect_time))
        except Exception as e:
            print(f"\n❌ ERROR in scenario {i}: {str(e)}")
            results.append((name, False, expected_conflict, None, 0))
    
    elapsed_time = time.time() - start_time
    
    # Print summary
    print("\n\n")
    print("#" * 70)
    print("# TEST SUMMARY")
    print("#" * 70)
    
    passed_count = sum(1 for _, passed, _, _, _ in results if passed)
    total_count = len(results)
    
    for i, (name, passed, expected, result, detect_time) in enumerate(results, 1):
        status = "✓ PASS" if passed else "✗ FAIL"
        expected_str = "CONFLICT" if expected else "NO CONFLICT"
        result_str = "CONFLICT" if result else "NO CONFLICT"
        time_str = f"{detect_time*1000:.3f} ms"
        print(f"{i:2d}. {status} - {name:35s} | Expected: {expected_str:11s} | Got: {result_str:11s} | Time: {time_str:>10s}")
    
    print("#" * 70)
    print(f"# Results: {passed_count}/{total_count} tests passed")
    print(f"# Execution time: {elapsed_time:.2f} seconds")
    print("#" * 70)
    
    return passed_count == total_count


if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)
