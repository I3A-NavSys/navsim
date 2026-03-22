import matplotlib.pyplot as plt
from flight_plan import FlightPlan
import time  # For performance benchmarking
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

# Simulation of multiple UAVs with complex trajectories for conflict detection using swept volumes (CCD).

#
# : Pensar como tratar la informacion del conflicto de cara a la resolucion. Por ejemplo, si detectamos un conflicto entre UAV A y UAV B en el intervalo t=2-3s,

# ========== INTERACTIVE SCENARIO SELECTION ==========

# List of all available test scenarios
TEST_SCENARIOS = [
    ("Direct collision", test_direct_collision),
    ("Head-on collision", test_head_on_collision),
    ("Crossing paths", test_crossing_paths),
    ("Near Miss", test_near_miss),
    ("Different altitudes", test_different_altitudes),
    ("Safe parallel paths", test_parallel_paths_safe),
    ("Same path, different times", test_same_path_different_times),
    ("Complex maneuver", test_complex_maneuver),
    ("Safe overtaking", test_overtaking),
    ("Convergent spirals", test_spiral_maneuver),
    ("Takeoff conflict", test_takeoff_conflict),
    ("Landing conflict", test_landing_conflict),
    ("Near Miss at different times", test_near_miss_different_times),
    ("Large safety radius", test_large_safety_radius),
    ("Sharp turn with crossing", test_sharp_turn_conflict),
]

def display_menu():
    """Display the scenario selection menu"""
    print("\n" + "="*80)
    print("AVAILABLE TEST SCENARIOS")
    print("="*80 + "\n")
    
    for i, (name, _) in enumerate(TEST_SCENARIOS, 1):
        print(f"{i:2d}. {name}")
    
    print(f"\n{len(TEST_SCENARIOS) + 1}. Custom collision scenario (original)")
    print(f"{len(TEST_SCENARIOS) + 2}. Exit")
    print("\n" + "="*80 + "\n")

def select_scenario():
    """Interactive scenario selection"""
    while True:
        display_menu()
        try:
            choice = input(f"Select a scenario (1-{len(TEST_SCENARIOS) + 2}): ").strip()
            choice_num = int(choice)
            
            if choice_num == len(TEST_SCENARIOS) + 1:
                return "custom", None  # Original collision scenario
            elif choice_num == len(TEST_SCENARIOS) + 2:
                print("Exiting...")
                return "exit", None
            elif 1 <= choice_num <= len(TEST_SCENARIOS):
                scenario_name, scenario_func = TEST_SCENARIOS[choice_num - 1]
                return "test", scenario_func
            else:
                print(f"Invalid choice. Please enter a number between 1 and {len(TEST_SCENARIOS) + 2}.")
        except ValueError:
            print("Invalid input. Please enter a number.")

# Create two flight plans that collide in time and space
uav_collision1 = FlightPlan()
uav_collision1.radius = 1.0
uav_collision1.set_waypoint(label="C1_1", time=0, pos=[0, 0, 0], vel=[10, 5, 0])
uav_collision1.set_waypoint(label="C1_2", time=3, pos=[30, 15, 0], vel=[10, 5, 0])
uav_collision1.connect_waypoints()

uav_collision2 = FlightPlan()
uav_collision2.radius = 1.0
uav_collision2.set_waypoint(label="C2_1", time=0, pos=[30, 0, 0], vel=[-10, 5, 0])
uav_collision2.set_waypoint(label="C2_2", time=3, pos=[0, 15, 0], vel=[-10, 5, 0])
uav_collision2.connect_waypoints()

# 1. UAV A - Long curved path (spiral-like)
uav_a = FlightPlan()
uav_a.radius = 1.0
uav_a.set_waypoint(label="A1", time=0, pos=[0, 0, 0], vel=[15, 0, 0])
uav_a.set_waypoint(label="A2", time=3, pos=[30, 10, 5], vel=[12, 8, 2])
uav_a.set_waypoint(label="A3", time=6, pos=[50, 20, 10], vel=[10, 10, 3])
uav_a.set_waypoint(label="A4", time=9, pos=[60, 30, 15], vel=[8, 12, 3])
uav_a.set_waypoint(label="A5", time=12, pos=[65, 45, 18], vel=[5, 15, 2])
uav_a.connect_waypoints()

# 2. UAV B - Crossing path with curves
uav_b = FlightPlan()
uav_b.radius = 1.0
uav_b.set_waypoint(label="B1", time=0, pos=[25, -30, 2], vel=[0, 12, 0.5])
uav_b.set_waypoint(label="B2", time=2, pos=[28, -15, 5], vel=[2, 12, 1])
uav_b.set_waypoint(label="B3", time=5, pos=[32, 0, 8], vel=[3, 12, 1.5])
uav_b.set_waypoint(label="B4", time=8, pos=[35, 15, 10], vel=[3, 12, 1])
uav_b.set_waypoint(label="B5", time=11, pos=[38, 30, 12], vel=[2, 12, 0.5])
uav_b.connect_waypoints()

# 3. UAV C - Diagonal ascending path
uav_c = FlightPlan()
uav_c.radius = 1.0
uav_c.set_waypoint(label="C1", time=1, pos=[-20, 10, 0], vel=[10, 5, 3])
uav_c.set_waypoint(label="C2", time=4, pos=[5, 20, 8], vel=[12, 5, 4])
uav_c.set_waypoint(label="C3", time=7, pos=[25, 25, 14], vel=[12, 3, 4])
uav_c.set_waypoint(label="C4", time=10, pos=[45, 28, 18], vel=[12, 2, 3])
uav_c.set_waypoint(label="C5", time=13, pos=[65, 30, 20], vel=[12, 1, 1])
uav_c.connect_waypoints()

# 4. UAV D - Complex sinusoidal path
uav_d = FlightPlan()
uav_d.radius = 1.0
uav_d.set_waypoint(label="D1", time=0.5, pos=[10, 40, 8], vel=[10, -10, 0])
uav_d.set_waypoint(label="D2", time=3.5, pos=[28, 25, 6], vel=[12, -8, -1])
uav_d.set_waypoint(label="D3", time=6.5, pos=[45, 15, 5], vel=[12, -5, -0.5])
uav_d.set_waypoint(label="D4", time=9.5, pos=[60, 5, 4], vel=[12, -3, -0.2])
uav_d.set_waypoint(label="D5", time=12.5, pos=[75, 0, 3], vel=[12, 0, -0.1])
uav_d.connect_waypoints()

# 5. Generate the swept boxes for all UAVs
# COMPULSORY: We need to use the same interval for all UAVs to ensure that
# we are comparing boxes that correspond to the same time intervals.

# Choose box type: "aabb" or "obb"
BOX_TYPE = "obb"  # Change to "aabb" to use traditional axis-aligned boxes

# ========== OPTIMIZED CONFLICT DETECTION ==========
import numpy as np
from bisect import bisect_left, bisect_right

def detect_conflicts_optimized(all_boxes):
    """
    Ultra-Optimized Continuous Collision Detection (CCD) with Binary Search:
    
    KEY STRATEGIES:
    1. TEMPORAL SORTING: Boxes are sorted by start time - enables binary search
    2. BINARY SEARCH (bisect): Jump directly to the time-relevant box range using O(log M) search
    3. SPATIAL FILTERING: Only run expensive SAT checks on temporally-overlapping boxes
    
    OPTIMIZATION COMPARISON:
    - Naive approach: O(N² × M²) with all comparisons
    - Previous version: O(N² × M²) with linear continue/break filtering
    - THIS VERSION: O(N² × M × log M) - binary search reduces inner loop significantly
    
    Result: ~5-10x faster for large box counts (100+ boxes per UAV)
    """
    conflict_count = 0
    conflicts_list = []
    
    # STEP 1: Pre-extract time information into tuples (t_start, t_end, box_index, box_object)
    # PURPOSE: Avoid repeated attribute access in tight loops, saving function call overhead
    time_ranges = []
    for boxes_list, _ in all_boxes:
        time_ranges.append([(box.t_range[0], box.t_range[1], idx, box) 
                           for idx, box in enumerate(boxes_list)])
    
    # STEP 2: Compare each pair of UAVs (i.e., their box lists)
    # PURPOSE: Generate all unique pairs (A-B, A-C, B-C, etc.) without redundant comparisons
    for i in range(len(all_boxes)):
        for j in range(i+1, len(all_boxes)): # i+1 to avoid redundant comparisons and self-comparison
            boxes1, name1 = all_boxes[i]
            boxes2, name2 = all_boxes[j]
            ranges1 = time_ranges[i]
            ranges2 = time_ranges[j]
            
            # STEP 3: Iterate through boxes of UAV 1
            # All boxes are pre-sorted by start time (generated sequentially by FlightPlan)
            for t1_start, t1_end, idx1, box1 in ranges1:
                
                # ============ BINARY SEARCH OPTIMIZATION ============
                # Instead of iterating through ALL boxes in ranges2 with continue/break,
                # use bisect to jump directly to the relevant time range
                # bisect_left/bisect_right use binary search on SORTED data to find insertion point
                # This runs in O(log n) time instead of O(n) linear scan
                
                # STEP 4A: Binary search finds the FIRST box in ranges2 that COULD overlap with box1
                # We need the leftmost index where a box's t_start >= box1.t_start
                # 
                # KEY INSIGHT: ranges2 contains tuples like (t_start, t_end, idx, box)
                # bisect_left() compares tuples element-by-element:
                #   - First compares t_start values
                #   - If t_start values are equal, compares second element (t_end)
                #
                # WHY -float('inf')?
                # - We create (t1_start, -float('inf')) as search key
                # - If multiple boxes have the SAME t_start, we want the FIRST one
                # - Using -float('inf') as second element ensures we land at the leftmost position
                #   (since -infinity is less than any t_end value)
                #
                start_idx = bisect_left(ranges2, (t1_start, -float('inf')))
                
                # STEP 4B: Binary search finds the LAST box in ranges2 that COULD overlap with box1
                # We need the rightmost index where a box's t_end <= box1.t_end
                # More precisely: first index where t_start > box1.t_end
                #
                # WHY float('inf')?
                # - We create (t1_end, float('inf')) as search key
                # - bisect_right() returns the insertion point AFTER all equal elements
                # - Using +infinity as second element ensures we skip past all boxes with t_start == t1_end
                #   (since infinity is greater than any t_end value)
                # - This gives us the index RIGHT AFTER the last relevant box
                #
                end_idx = bisect_right(ranges2, (t1_end, float('inf')))
                
                # STEP 5: Iterate ONLY through the temporally-relevant boxes
                # This is a SMALL slice compared to the full ranges2 list
                # NOTE: Since all boxes are uniformly generated with the same interval,
                # bisect guarantees we only get boxes that temporally overlap with box1
                for box_tuple in ranges2[start_idx:end_idx]:
                    t2_start, t2_end, idx2, box2 = box_tuple
                    
                    # ============ SPATIAL COLLISION CHECK ============
                    # At this point, we've confirmed TEMPORAL overlap
                    # Now perform the expensive SAT (Separating Axis Theorem) check
                    # SAT tests 15 axes for OBB collision or checks AABB bounds
                    if box1.collides_with(box2):
                        # COLLISION FOUND: Store all relevant information
                        conflicts_list.append((idx1, box1.t_range, idx2, box2.t_range, name1, name2))
                        conflict_count += 1
    
    return conflict_count, conflicts_list

# Import necessary modules for visualization
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def visualize_with_colored_conflicts(flight_plans, all_boxes, conflicts):
    """
    Visualize flight plans with OBBs colored by collision status.
    - RED boxes: contain collisions
    - GREEN/BLUE boxes: no collisions
    """
    fig = plt.figure("UAV Trajectories with Conflict Detection")
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("OBB Visualization - Red boxes = Collisions, Green/Blue = Safe")
    
    # Color palette for different UAVs (non-collision)
    base_colors = [
        [0, 0.7, 1],      # Blue
        [0, 1, 0],         # Green
    ]
    
    # Create a set of collision box indices for quick lookup
    # Format: (uav_index, box_index)
    collision_boxes = set()
    for idx1, t_range1, idx2, t_range2, name1, name2 in conflicts:
        # Extract UAV indices from names (e.g., "UAV A" -> 0, "UAV B" -> 1)
        uav1_idx = 0 if "A" in name1 else (1 if "B" in name1 else 2)
        uav2_idx = 0 if "A" in name2 else (1 if "B" in name2 else 2)
        collision_boxes.add((uav1_idx, idx1))
        collision_boxes.add((uav2_idx, idx2))
    
    # Plot each UAV's trajectory and boxes
    for uav_idx, (fp, (boxes_list, name)) in enumerate(zip(flight_plans, all_boxes)):
        color = base_colors[uav_idx % len(base_colors)]
        
        # Get trace
        tr = fp.trace(0.1)
        tr_x = tr[:, 1]
        tr_y = tr[:, 2]
        tr_z = tr[:, 3]
        
        # Plot trajectory
        ax.plot(tr_x, tr_y, tr_z, linewidth=2, color=color, label=name, zorder=10)
        
        # Plot waypoints
        xPos = [wp.pos[0] for wp in fp.waypoints]
        yPos = [wp.pos[1] for wp in fp.waypoints]
        zPos = [wp.pos[2] for wp in fp.waypoints]
        ax.scatter(xPos, yPos, zPos, marker="o", color=color, s=50, zorder=15)
        
        # Plot boxes with collision status coloring
        for box_idx, box in enumerate(boxes_list):
            # Determine color based on collision status
            is_colliding = (uav_idx, box_idx) in collision_boxes
            box_color = [1, 0, 0] if is_colliding else color  # RED if colliding, base color otherwise
            
            # Get corners and draw box
            corners = box.get_corners()
            corners = np.array(corners)
            
            edges = [
                [0, 1], [1, 3], [3, 2], [2, 0],  # Bottom face
                [4, 5], [5, 7], [7, 6], [6, 4],  # Top face
                [0, 4], [1, 5], [2, 6], [3, 7],  # Vertical edges
            ]
            
            # Draw edges
            for edge in edges:
                points = corners[edge]
                linewidth = 2.0 if is_colliding else 0.8
                alpha_val = 1.0 if is_colliding else 0.6
                ax.plot3D(*points.T, color=box_color, linewidth=linewidth, alpha=alpha_val)
            
            # Draw faces
            alpha = 0.15 if is_colliding else 0.05
            faces = [
                [corners[0], corners[1], corners[5], corners[4]],
                [corners[2], corners[3], corners[7], corners[6]],
                [corners[0], corners[2], corners[6], corners[4]],
                [corners[1], corners[3], corners[7], corners[5]],
                [corners[0], corners[1], corners[3], corners[2]],
                [corners[4], corners[5], corners[7], corners[6]],
            ]
            
            poly = Poly3DCollection(faces, alpha=alpha, facecolor=box_color, 
                                   edgecolor=box_color, linewidth=0.8)
            ax.add_collection3d(poly)
    
    # Add legend
    ax.legend(loc='upper right', fontsize=10)
    
    # Set equal aspect ratio
    xLim = ax.get_xlim()
    yLim = ax.get_ylim()
    zLim = ax.get_zlim()
    maxLim = max(max(np.abs(xLim)), max(np.abs(yLim)), max(np.abs(zLim)))
    ax.set_xlim(-maxLim, maxLim)
    ax.set_ylim(-maxLim, maxLim)
    ax.set_zlim(-maxLim, maxLim)
    
    plt.show()

# ========== MAIN EXECUTION ==========
def main():
    """Main interactive scenario selector and visualizer"""
    while True:
        scenario_type, scenario_func = select_scenario()
        
        if scenario_type == "exit":
            break
        
        print("\n" + "="*80)
        
        if scenario_type == "custom":
            # Use the original collision scenario
            print("Using original collision scenario")
            fp1 = uav_collision1
            fp2 = uav_collision2
            flight_plans = [fp1, fp2]
            scenario_name = "Original Collision"
        else:
            # Get the test scenario
            fp1, fp2, description = scenario_func()
            flight_plans = [fp1, fp2]
            scenario_name = description
            print(f"Test: {scenario_name}")
        
        print("="*80 + "\n")
        
        # Generate swept boxes
        if BOX_TYPE == "obb":
            boxes1 = fp1.generate_swept_boxes_obb(interval=0.5)
            boxes2 = fp2.generate_swept_boxes_obb(interval=0.5)
            print("Using Oriented Bounding Boxes (OBB) for collision detection\n")
        else:
            boxes1 = fp1.generate_swept_boxes(interval=0.5)
            boxes2 = fp2.generate_swept_boxes(interval=0.5)
            print("Using Axis-Aligned Bounding Boxes (AABB) for collision detection\n")
        
        all_boxes = [
            (boxes1, "UAV A"),
            (boxes2, "UAV B"),
        ]
        
        # Detect conflicts
        print("Starting conflict detection...")
        start_time = time.time()
        conflict_count, conflicts = detect_conflicts_optimized(all_boxes)
        elapsed_time = time.time() - start_time
        
        print(f"Conflict detection completed in {elapsed_time:.4f} seconds ({elapsed_time*1000:.2f} ms)\n")
        
        if conflict_count > 0:
            print(f"CONFLICTS DETECTED: {conflict_count}\n")
            for idx1, t_range1, idx2, t_range2, name1, name2 in conflicts:
                print(f"  {name1}[{idx1}] {t_range1} collides with {name2}[{idx2}] {t_range2}")
        else:
            print("NO CONFLICTS DETECTED\n")
        
        # Visualize
        print("\nGenerating visualization...\n")
        visualize_with_colored_conflicts(flight_plans, all_boxes, conflicts)

if __name__ == "__main__":
    main()
