"""
Test Flight Plans for Conflict Detection

This module contains a battery of flight plans to test different scenarios
for conflict detection. Each function generates a pair of flight plans representing
different types of conflicts or safe situations.

Scenarios:
- Direct collision: two UAVs on the same trajectory
- Head-on collision: two UAVs flying toward each other
- Crossing paths: two UAVs crossing each other
- Near miss: two UAVs passing very close but without colliding
- Different altitudes: two UAVs at different altitudes
- Parallel paths: two UAVs flying in parallel
- Same path different times: two UAVs on the same trajectory but at different times
- Complex maneuver: UAVs with direction changes
- Spiral patterns: UAVs on spiral routes
- Take-off and landing: takeoff and landing scenarios
"""


from flight_plan import FlightPlan

import numpy as np
import time


# =============================================================================
# SCENARIO 1: DIRECT COLLISION
# =============================================================================
def test_direct_collision():
    """
    Two UAVs flying on the SAME straight line and time.
    EXPECTED RESULT: CONFLICT
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0  # safety radius 2m
    
    # UAV 1: (0,0,0) -> (100,0,0) in 10 seconds
    fp1.set_waypoint(time=0, pos=[0, 0, 0], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 0], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: same path, same time
    fp2.set_waypoint(time=0, pos=[0, 0, 0], vel=[10, 0, 0])
    fp2.set_waypoint(time=10, pos=[100, 0, 0], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "EXPECTED CONFLICT: Direct collision"


# =============================================================================
# SCENARIO 2: HEAD-ON COLLISION
# =============================================================================
def test_head_on_collision():
    """
    Two UAVs flying toward each other on the same line.
    EXPECTED RESULT: CONFLICT
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: (0,0,10) -> (100,0,10)
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: (100,0,10) -> (0,0,10) opposite movement
    fp2.set_waypoint(time=0, pos=[100, 0, 10], vel=[-10, 0, 0])
    fp2.set_waypoint(time=10, pos=[0, 0, 10], vel=[-10, 0, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "EXPECTED CONFLICT: Head-on collision at t=5s"


# =============================================================================
# SCENARIO 3: CROSSING PATHS
# =============================================================================
def test_crossing_paths():
    """
    Two UAVs crossing at a point (perpendicular).
    EXPECTED RESULT: CONFLICT (if they reach the intersection point at similar times)
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: (0,50,10) -> (100,50,10) movement in X
    fp1.set_waypoint(time=0, pos=[0, 50, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 50, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: (50,0,10) -> (50,100,10) movement in Y, they cross at (50,50,10)
    fp2.set_waypoint(time=0, pos=[50, 0, 10], vel=[0, 10, 0])
    fp2.set_waypoint(time=10, pos=[50, 100, 10], vel=[0, 10, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "EXPECTED CONFLICT: Crossing at (50,50,10) at t=5s"


# =============================================================================
# SCENARIO 4: NEAR MISS
# =============================================================================
def test_near_miss():
    """
    Two UAVs passing very close but managing to avoid each other.
    EXPECTED RESULT: NO CONFLICT (if distance is greater than sum of radii)
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 1.5
    
    # UAV 1: (0,0,10) -> (100,0,10)
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 1.5
    
    # UAV 2: (50,-5,10) -> (50,100,10) passes at 5m distance
    fp2.set_waypoint(time=0, pos=[50, -5, 10], vel=[0, 10, 0])
    fp2.set_waypoint(time=12, pos=[50, 115, 10], vel=[0, 10, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "NO CONFLICT expected: minimum distance > sum of radii"


# =============================================================================
# SCENARIO 5: DIFFERENT ALTITUDES
# =============================================================================
def test_different_altitudes():
    """
    Two UAVs on the same horizontal trajectory but at different altitudes.
    EXPECTED RESULT: NO CONFLICT
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: (0,0,10) -> (100,0,10) at 10m altitude
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: same path but at 20m altitude
    fp2.set_waypoint(time=0, pos=[0, 0, 20], vel=[10, 0, 0])
    fp2.set_waypoint(time=10, pos=[100, 0, 20], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "NO CONFLICT expected: sufficient vertical separation"


# =============================================================================
# SCENARIO 6: PARALLEL PATHS (SAFE)
# =============================================================================
def test_parallel_paths_safe():
    """
    Two UAVs flying in parallel, well separated.
    EXPECTED RESULT: NO CONFLICT
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: (0,0,10) -> (100,0,10)
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: (0,20,10) -> (100,20,10) parallel but 20m away
    fp2.set_waypoint(time=0, pos=[0, 20, 10], vel=[10, 0, 0])
    fp2.set_waypoint(time=10, pos=[100, 20, 10], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "NO CONFLICT expected: parallel paths well separated"


# =============================================================================
# SCENARIO 7: SAME PATH, DIFFERENT TIMES (SAFE)
# =============================================================================
def test_same_path_different_times():
    """
    Two UAVs on the same trajectory but at different times.
    EXPECTED RESULT: NO CONFLICT (if there is sufficient temporal separation)
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: (0,0,10) -> (100,0,10) from t=0 to t=10
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: same path but starts 20 seconds later
    fp2.set_waypoint(time=20, pos=[0, 0, 10], vel=[10, 0, 0])
    fp2.set_waypoint(time=30, pos=[100, 0, 10], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "NO CONFLICT expected: sufficient temporal separation"


# =============================================================================
# SCENARIO 8: COMPLEX MANEUVER WITH 3 WAYPOINTS
# =============================================================================
def test_complex_maneuver():
    """
    UAVs with more complex trajectories with multiple waypoints.
    EXPECTED RESULT: CONFLICT at the intersection point
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: (0,0,10) -> (50,50,10) -> (100,0,10)
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[5, 5, 0])
    fp1.set_waypoint(time=10, pos=[50, 50, 10], vel=[5, -5, 0])
    fp1.set_waypoint(time=20, pos=[100, 0, 10], vel=[5, -5, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: (100,50,10) -> (50,50,10) -> (0,50,10) 
    # They cross at (50,50,10)
    fp2.set_waypoint(time=0, pos=[100, 50, 10], vel=[-5, 0, 0])
    fp2.set_waypoint(time=10, pos=[50, 50, 10], vel=[-5, 0, 0])
    fp2.set_waypoint(time=20, pos=[0, 50, 10], vel=[-5, 0, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "EXPECTED CONFLICT: crossing at common waypoint (50,50,10) at t=10s"


# =============================================================================
# SCENARIO 9: OVERTAKING (SAFE)
# =============================================================================
def test_overtaking():
    """
    One UAV attempts to overtake another in the same direction by changing lanes.
    EXPECTED RESULT: NO CONFLICT (if it changes altitude or lateral distance)
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: flying slowly at 5 m/s
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[5, 0, 0])
    fp1.set_waypoint(time=20, pos=[100, 0, 10], vel=[5, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: overtaking on the right at 10 m/s, 5m to the right
    fp2.set_waypoint(time=0, pos=[0, 5, 10], vel=[10, 0, 0])
    fp2.set_waypoint(time=10, pos=[100, 5, 10], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "NO CONFLICT expected: overtaking with lateral separation"


# =============================================================================
# SCENARIO 10: SPIRAL MANEUVER
# =============================================================================
def test_spiral_maneuver():
    """
    UAVs in convergent helical trajectories.
    EXPECTED RESULT: CONFLICT at the center of the spirals
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: spiral approaching the center (50,50,10)
    fp1.set_waypoint(time=0, pos=[80, 50, 10], vel=[-5, 5, 0])
    fp1.set_waypoint(time=5, pos=[50, 80, 10], vel=[-5, -5, 0])
    fp1.set_waypoint(time=10, pos=[20, 50, 10], vel=[5, -5, 0])
    fp1.set_waypoint(time=15, pos=[50, 20, 10], vel=[5, 5, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: similar spiral but from opposite angle
    fp2.set_waypoint(time=0, pos=[20, 50, 10], vel=[5, 5, 0])
    fp2.set_waypoint(time=5, pos=[50, 20, 10], vel=[5, -5, 0])
    fp2.set_waypoint(time=10, pos=[80, 50, 10], vel=[-5, -5, 0])
    fp2.set_waypoint(time=15, pos=[50, 80, 10], vel=[-5, 5, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "EXPECTED CONFLICT: spiral convergence"


# =============================================================================
# SCENARIO 11: TAKEOFF CONFLICT
# =============================================================================
def test_takeoff_conflict():
    """
    Two UAVs taking off from the same point without coordination.
    EXPECTED RESULT: CONFLICT during takeoff
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: takes off vertically
    fp1.set_waypoint(time=0, pos=[0, 0, 0], vel=[0, 0, 0])
    fp1.set_waypoint(time=1, pos=[0, 0, 1], vel=[0, 0, 1])
    fp1.set_waypoint(time=10, pos=[0, 0, 50], vel=[0, 0, 5])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: takes off from the same point at a different angle
    fp2.set_waypoint(time=0, pos=[0, 0, 0], vel=[0, 0, 0])
    fp2.set_waypoint(time=1, pos=[1, 1, 1], vel=[1, 1, 1])
    fp2.set_waypoint(time=10, pos=[10, 10, 50], vel=[1, 1, 5])
    fp2.connect_waypoints()
    
    return fp1, fp2, "EXPECTED CONFLICT: uncoordinated takeoff"


# =============================================================================
# SCENARIO 12: LANDING CONFLICT
# =============================================================================
def test_landing_conflict():
    """
    Two UAVs landing at the same point without coordination.
    EXPECTED RESULT: CONFLICT during landing
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: descends vertically
    fp1.set_waypoint(time=0, pos=[0, 0, 50], vel=[0, 0, -5])
    fp1.set_waypoint(time=10, pos=[0, 0, 0], vel=[0, 0, -5])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: descends to the same point from a different angle
    fp2.set_waypoint(time=0, pos=[10, 10, 50], vel=[-1, -1, -5])
    fp2.set_waypoint(time=10, pos=[0, 0, 0], vel=[-1, -1, -5])
    fp2.connect_waypoints()
    
    return fp1, fp2, "EXPECTED CONFLICT: uncoordinated landing"


# =============================================================================
# SCENARIO 13: NEAR MISS AT DIFFERENT TIMES
# =============================================================================
def test_near_miss_different_times():
    """
    Two UAVs on similar trajectories but reaching the critical point at different times.
    EXPECTED RESULT: NO CONFLICT (if temporal separation is sufficient)
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 1.5
    
    # UAV 1: passes through (50,50,10) at t=5
    fp1.set_waypoint(time=0, pos=[0, 50, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 50, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 1.5
    
    # UAV 2: passes through the same point but much later (t=15)
    fp2.set_waypoint(time=10, pos=[0, 50, 10], vel=[10, 0, 0])
    fp2.set_waypoint(time=20, pos=[100, 50, 10], vel=[10, 0, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "NO CONFLICT expected: temporal separation at critical point"


# =============================================================================
# SCENARIO 14: LARGE SAFETY RADIUS COLLISION
# =============================================================================
def test_large_safety_radius():
    """
    Two UAVs with large safety radii (4m).
    EXPECTED RESULT: CONFLICT (even though they wouldn't touch with small radii)
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 4.0  # Radio grande
    
    # UAV 1: (0,0,10) -> (100,0,10)
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[10, 0, 0])
    fp1.set_waypoint(time=10, pos=[100, 0, 10], vel=[10, 0, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 4.0  # Large radius
    
    # UAV 2: (50,-6,10) -> (50,100,10) 6m away
    fp2.set_waypoint(time=0, pos=[50, -6, 10], vel=[0, 10, 0])
    fp2.set_waypoint(time=12, pos=[50, 114, 10], vel=[0, 10, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "EXPECTED CONFLICT: large safety radii overlap"


# =============================================================================
# SCENARIO 15: SHARP TURN CONFLICT
# =============================================================================
def test_sharp_turn_conflict():
    """
    Two UAVs, one turns sharply crossing the other.
    EXPECTED RESULT: CONFLICT at the turn point
    """
    fp1 = FlightPlan()
    fp1.id = 1
    fp1.priority = 1
    fp1.radius = 2.0
    
    # UAV 1: flying north
    fp1.set_waypoint(time=0, pos=[0, 0, 10], vel=[0, 10, 0])
    fp1.set_waypoint(time=10, pos=[0, 100, 10], vel=[0, 10, 0])
    fp1.connect_waypoints()
    
    fp2 = FlightPlan()
    fp2.id = 2
    fp2.priority = 2
    fp2.radius = 2.0
    
    # UAV 2: flying east, sharp turn at (50,50)
    fp2.set_waypoint(time=0, pos=[-50, 50, 10], vel=[10, 0, 0])
    fp2.set_waypoint(time=5, pos=[50, 50, 10], vel=[0, 10, 0])  # Sharp turn
    fp2.set_waypoint(time=15, pos=[50, 150, 10], vel=[0, 10, 0])
    fp2.connect_waypoints()
    
    return fp1, fp2, "EXPECTED CONFLICT: crossing at turn point (50,50,10)"


# =============================================================================
# Test runner
# =============================================================================

def run_all_tests(conflict_detector):
    """
    Runs all flight plan tests and reports results.
    
    Parameters:
    -----------
    conflict_detector : function
        Function that detects conflicts between two flight plans.
        Must return True if there is a conflict, False otherwise.
    
    Example:
    --------
    >>> from conflictDetection import detect_conflict
    >>> run_all_tests(detect_conflict)
    """
    
    tests = [
        ("1. Direct collision", test_direct_collision),
        ("2. Head-on collision", test_head_on_collision),
        ("3. Crossing paths", test_crossing_paths),
        ("4. Near Miss", test_near_miss),
        ("5. Different altitudes", test_different_altitudes),
        ("6. Safe parallel paths", test_parallel_paths_safe),
        ("7. Same path, different times", test_same_path_different_times),
        ("8. Complex maneuver", test_complex_maneuver),
        ("9. Safe overtaking", test_overtaking),
        ("10. Convergent spirals", test_spiral_maneuver),
        ("11. Takeoff conflict", test_takeoff_conflict),
        ("12. Landing conflict", test_landing_conflict),
        ("13. Near Miss at different times", test_near_miss_different_times),
        ("14. Large safety radius", test_large_safety_radius),
        ("15. Sharp turn with crossing", test_sharp_turn_conflict),
    ]
    
    print("\n" + "="*80)
    print("TEST SUITE: Conflict Detection")
    print("="*80 + "\n")
    
    results = []
    total_time = 0.0
    total_detection_time = 0.0
    
    for i, (name, test_func) in enumerate(tests, 1):
        # Time the flight plan creation
        start_creation = time.time()
        fp1, fp2, description = test_func()
        creation_time = time.time() - start_creation
        
        # Time the conflict detection
        start_detection = time.time()
        conflict_detected = conflict_detector(fp1, fp2)
        detection_time = time.time() - start_detection
        
        total_time += creation_time + detection_time
        total_detection_time += detection_time
        
        results.append({
            'name': name,
            'description': description,
            'conflict': conflict_detected,
            'fp1': fp1,
            'fp2': fp2,
            'creation_time': creation_time,
            'detection_time': detection_time
        })
        
        status = "✓ CONFLICT" if conflict_detected else "✗ NO CONFLICT"
        print(f"Test {i:2d}: {name}")
        print(f"         {description}")
        print(f"         Result: {status}")
        print(f"         Time: Creation {creation_time*1000:.2f}ms | Detection {detection_time*1000:.2f}ms\n")
    
    # Print summary
    print("="*80)
    print("SUMMARY")
    print("="*80)
    print(f"Total tests: {len(results)}")
    print(f"Total execution time: {total_time*1000:.2f}ms")
    print(f"Average detection time: {(total_detection_time/len(results))*1000:.2f}ms")
    print(f"Total detection time: {total_detection_time*1000:.2f}ms")
    print("="*80 + "\n")
    
    return results


if __name__ == "__main__":
    # Example usage without detection function
    print("\n" + "="*80)
    print("Available Test Flight Plans")
    print("="*80 + "\n")
    
    test_functions = [
        ("test_direct_collision", "Direct collision"),
        ("test_head_on_collision", "Head-on collision"),
        ("test_crossing_paths", "Crossing paths"),
        ("test_near_miss", "Near Miss"),
        ("test_different_altitudes", "Different altitudes"),
        ("test_parallel_paths_safe", "Parallel paths"),
        ("test_same_path_different_times", "Same path, different times"),
        ("test_complex_maneuver", "Complex maneuver"),
        ("test_overtaking", "Overtaking"),
        ("test_spiral_maneuver", "Convergent spirals"),
        ("test_takeoff_conflict", "Takeoff conflict"),
        ("test_landing_conflict", "Landing conflict"),
        ("test_near_miss_different_times", "Near Miss at different times"),
        ("test_large_safety_radius", "Large safety radius"),
        ("test_sharp_turn_conflict", "Sharp turn with crossing"),
    ]
    
    for func_name, description in test_functions:
        print(f"• {func_name}()")
        print(f"  → {description}\n")



