"""
Quick benchmark script to test performance improvement
"""
import time
from flight_plan import FlightPlan

# Create two flight plans that collide in time and space (original scenario)
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

# Generate OBB boxes
print("Generating OBB boxes with interval=0.5...")
boxes1 = uav_collision1.generate_swept_boxes_obb(interval=0.5)
boxes2 = uav_collision2.generate_swept_boxes_obb(interval=0.5)

print(f"UAV 1: {len(boxes1)} boxes")
print(f"UAV 2: {len(boxes2)} boxes")

# Conflict detection function (optimized version from playground.py)
from bisect import bisect_left, bisect_right

def detect_conflicts_optimized(all_boxes):
    """Optimized conflict detection with binary search"""
    conflict_count = 0
    conflicts_list = []
    
    time_ranges = []
    for boxes_list, _ in all_boxes:
        time_ranges.append([(box.t_range[0], box.t_range[1], idx, box) 
                           for idx, box in enumerate(boxes_list)])
    
    for i in range(len(all_boxes)):
        for j in range(i+1, len(all_boxes)):
            boxes1, name1 = all_boxes[i]
            boxes2, name2 = all_boxes[j]
            ranges1 = time_ranges[i]
            ranges2 = time_ranges[j]
            
            for t1_start, t1_end, idx1, box1 in ranges1:
                start_idx = bisect_left(ranges2, (t1_start, -float('inf')))
                end_idx = bisect_right(ranges2, (t1_end, float('inf')))
                
                for box_tuple in ranges2[start_idx:end_idx]:
                    t2_start, t2_end, idx2, box2 = box_tuple
                    
                    if box1.collides_with(box2):
                        conflicts_list.append((idx1, box1.t_range, idx2, box2.t_range, name1, name2))
                        conflict_count += 1
    
    return conflict_count, conflicts_list

all_boxes = [
    (boxes1, "UAV A"),
    (boxes2, "UAV B"),
]

# Benchmark: Run multiple times to get stable timing
print("\nRunning 5 iterations to benchmark...")
times = []

for iteration in range(5):
    start_time = time.time()
    conflict_count, conflicts = detect_conflicts_optimized(all_boxes)
    elapsed_time = time.time() - start_time
    times.append(elapsed_time)
    print(f"  Iteration {iteration+1}: {elapsed_time:.4f}s ({elapsed_time*1000:.2f}ms) - Conflicts: {conflict_count}")

avg_time = sum(times) / len(times)
print(f"\nAverage time: {avg_time:.4f}s ({avg_time*1000:.2f}ms)")
print(f"Min time: {min(times):.4f}s")
print(f"Max time: {max(times):.4f}s")

if conflict_count > 0:
    print(f"\nConflicts detected: {conflict_count}")
    for idx1, t_range1, idx2, t_range2, name1, name2 in conflicts:
        print(f"  {name1}[{idx1}] {t_range1} collides with {name2}[{idx2}] {t_range2}")
else:
    print("\nNo conflicts detected")
