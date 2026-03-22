"""
Interactive visualization of conflict detection scenarios.

This script allows you to select and visualize different UAV collision scenarios
with swept boxes (Continuous Collision Detection).
"""

import matplotlib.pyplot as plt
from test_flight_plans import (
    test_direct_collision,
    test_head_on_collision,
    test_crossing_paths,
    test_near_miss,
    test_different_altitudes,
    test_parallel_paths_safe,
    test_same_path_different_times,
    test_complex_maneuver,
    test_overtaking,
    test_spiral_maneuver,
    test_takeoff_conflict,
    test_landing_conflict,
    test_near_miss_different_times,
    test_large_safety_radius,
    test_sharp_turn_conflict,
)
from flight_plan import FlightPlan


# Dictionary mapping scenario names to test functions
SCENARIOS = {
    "1": {
        "name": "Direct collision",
        "func": test_direct_collision,
        "description": "Two UAVs flying on SAME trajectory and time - SHOULD COLLIDE"
    },
    "2": {
        "name": "Head-on collision",
        "func": test_head_on_collision,
        "description": "Two UAVs flying toward each other - SHOULD COLLIDE"
    },
    "3": {
        "name": "Crossing paths",
        "func": test_crossing_paths,
        "description": "Two UAVs crossing perpendicular paths - SHOULD COLLIDE"
    },
    "4": {
        "name": "Near miss",
        "func": test_near_miss,
        "description": "Two UAVs passing close but avoiding collision - NO CONFLICT"
    },
    "5": {
        "name": "Different altitudes",
        "func": test_different_altitudes,
        "description": "Same horizontal path but different altitudes - NO CONFLICT"
    },
    "6": {
        "name": "Parallel paths (safe)",
        "func": test_parallel_paths_safe,
        "description": "Two UAVs flying in parallel, well separated - NO CONFLICT"
    },
    "7": {
        "name": "Same path, different times",
        "func": test_same_path_different_times,
        "description": "Same trajectory but UAVs arrive at different times - NO CONFLICT"
    },
    "8": {
        "name": "Complex maneuver",
        "func": test_complex_maneuver,
        "description": "Multiple waypoints with intersection - SHOULD COLLIDE"
    },
    "9": {
        "name": "Overtaking (safe)",
        "func": test_overtaking,
        "description": "One UAV overtakes another with lateral separation - NO CONFLICT"
    },
    "10": {
        "name": "Spiral maneuver",
        "func": test_spiral_maneuver,
        "description": "Convergent helical trajectories - SHOULD COLLIDE"
    },
    "11": {
        "name": "Takeoff conflict",
        "func": test_takeoff_conflict,
        "description": "Two UAVs taking off without coordination - SHOULD COLLIDE"
    },
    "12": {
        "name": "Landing conflict",
        "func": test_landing_conflict,
        "description": "Two UAVs landing without coordination - SHOULD COLLIDE"
    },
    "13": {
        "name": "Near miss (different times)",
        "func": test_near_miss_different_times,
        "description": "Critical point reached at different times - NO CONFLICT"
    },
    "14": {
        "name": "Large safety radius",
        "func": test_large_safety_radius,
        "description": "Large safety margins cause collision detection - SHOULD COLLIDE"
    },
    "15": {
        "name": "Sharp turn",
        "func": test_sharp_turn_conflict,
        "description": "UAV makes sharp turn crossing another - SHOULD COLLIDE"
    },
}


def print_menu():
    """Print available scenarios menu."""
    print("\n" + "="*80)
    print("CONFLICT DETECTION SCENARIOS - VISUALIZATION")
    print("="*80 + "\n")
    
    print("Available scenarios:\n")
    for key, scenario in SCENARIOS.items():
        print(f"{key:>2}. {scenario['name']:<30} | {scenario['description']}")
    
    print("\n" + "-"*80)
    print("Choose a scenario (1-15), or:")
    print("  'all'  - Visualize all scenarios sequentially")
    print("  'q'    - Quit")
    print("-"*80 + "\n")


def visualize_scenario(choice):
    """Visualize a specific scenario."""
    if choice not in SCENARIOS:
        print(f"Invalid choice: {choice}")
        return False
    
    scenario = SCENARIOS[choice]
    print(f"\nLoading scenario {choice}: {scenario['name']}")
    print(f"Description: {scenario['description']}\n")
    
    # Generate flight plans
    fp1, fp2, expected_result = scenario["func"]()
    
    # Print info
    print(f"UAV 1: id={fp1.id}, radius={fp1.radius}m, waypoints={len(fp1.waypoints)}")
    print(f"UAV 2: id={fp2.id}, radius={fp2.radius}m, waypoints={len(fp2.waypoints)}")
    print(f"Expected: {expected_result}\n")
    
    # Visualize trajectories
    print("Generating visualization...")
    print("(Close the plot window to continue)\n")
    
    try:
        FlightPlan.compare_flight_plans(
            [fp1, fp2],
            f"Scenario {choice}: {scenario['name']}"
        )
        plt.show()
        return True
    except Exception as e:
        print(f"Error during visualization: {e}")
        import traceback
        traceback.print_exc()
        return False


def visualize_all():
    """Visualize all scenarios sequentially."""
    print("\nVisualizing all scenarios...")
    print("Close each plot window to proceed to the next scenario.\n")
    
    for choice in sorted(SCENARIOS.keys(), key=lambda x: int(x)):
        visualize_scenario(choice)
        response = input("Press Enter to continue to next scenario, or 'q' to quit: ").strip().lower()
        if response == 'q':
            print("Stopped.")
            break


def main():
    """Main interactive menu loop."""
    print("\n" + "="*80)
    print("CONFLICT DETECTION SCENARIOS VISUALIZER")
    print("="*80)
    print("\nThis tool lets you visualize different UAV collision scenarios")
    print("with Continuous Collision Detection (CCD) using Oriented Bounding Boxes (OBB).")
    
    while True:
        print_menu()
        choice = input("Enter your choice: ").strip().lower()
        
        if choice == 'q':
            print("Goodbye!")
            break
        elif choice == 'all':
            visualize_all()
        elif choice in SCENARIOS:
            visualize_scenario(choice)
        else:
            print(f"Invalid choice: {choice}. Please try again.")


if __name__ == "__main__":
    main()
