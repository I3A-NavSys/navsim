import matplotlib.pyplot as plt
from flight_plan import FlightPlan

# Simulation of multiple UAVs with complex trajectories for conflict detection using swept volumes (CCD).

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
boxes_a = uav_a.generate_swept_boxes(interval=0.5)
boxes_b = uav_b.generate_swept_boxes(interval=0.5)
boxes_c = uav_c.generate_swept_boxes(interval=0.5)
boxes_d = uav_d.generate_swept_boxes(interval=0.5)

# 6. CCD: Compare boxes and detect conflicts
print(f"\n=== CONFLICT DETECTION ===")
all_boxes = [
    (boxes_a, "UAV A"),
    (boxes_b, "UAV B"),
    (boxes_c, "UAV C"),
    (boxes_d, "UAV D"),
]

conflict_count = 0 # Just for counting conflicts without printing every single one
for i in range(len(all_boxes)):
    for j in range(i+1, len(all_boxes)):
        boxes1, name1 = all_boxes[i]
        boxes2, name2 = all_boxes[j]
        for box1 in boxes1:
            for box2 in boxes2:
                if box1.t_range == box2.t_range:
                    if box1.collides_with(box2):
                        print(f"CONFLICT: {name1} <-> {name2} at t={box1.t_range[0]}-{box1.t_range[1]}s")
                        conflict_count += 1

print(f"Total conflicts detected: {conflict_count}\n")

# 7. Visualize all trajectories in the same figure (with swept boxes)
FlightPlan.compare_flight_plans([uav_a, uav_b, uav_c, uav_d ], 
                                 "Complex UAV Trajectories with Swept Boxes", 
                                 timeStep=0.1, 
                                 show_swept_boxes=True, 
                                 box_interval=0.5)
plt.show()