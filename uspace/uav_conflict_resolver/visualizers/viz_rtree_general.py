import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rtree import index
import random
import numpy as np

def draw_cube(ax, bbox, color, alpha, linewidth, label=None):
    """Draws an AABB (box) in 3D space."""
    x1, y1, z1, x2, y2, z2 = bbox
    edges = [
        ((x1,y1,z1), (x2,y1,z1)), ((x2,y1,z1), (x2,y2,z1)), ((x2,y2,z1), (x1,y2,z1)), ((x1,y2,z1), (x1,y1,z1)),
        ((x1,y1,z2), (x2,y1,z2)), ((x2,y1,z2), (x2,y2,z2)), ((x2,y2,z2), (x1,y2,z2)), ((x1,y2,z2), (x1,y1,z2)),
        ((x1,y1,z1), (x1,y1,z2)), ((x2,y1,z1), (x2,y1,z2)), ((x2,y2,z1), (x2,y2,z2)), ((x1,y2,z1), (x1,y2,z2))
    ]
    for i, (start, end) in enumerate(edges):
        ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], 
                 color=color, alpha=alpha, linewidth=linewidth, 
                 label=label if i == 0 and label else "")

def visualize_realistic_fleet():
    # 1. R-Tree Configuration (Legal minimum = 4)
    p = index.Property()
    p.dimension = 3
    p.leaf_capacity = 4
    p.index_capacity = 4
    p.near_minimum_overlap_factor = 2
    idx = index.Index(properties=p)

    # 2. Generate Realistic Drone Routes (With Noise and Randomness)
    # random.seed(42) # Uncomment this if you want it to be the same in every execution
    route_colors = ['#ff0055', '#00ffcc', '#ffcc00', '#9900ff']
    route_history = [] 

    # Central reference point for the mission
    mission_center = np.array([50, 50, 50])

    for r in range(4): 
        route_points = []
        
        # --- BASE RANDOMNESS PER DRONE ---
        # Random start around the mission center
        start_pos = mission_center + np.random.uniform(-15, 15, 3)
        # Random base direction (where it mainly flies)
        # We force an advance in X but with variations in Y and Z
        base_dir = np.array([random.uniform(10, 20), random.uniform(-8, 8), random.uniform(-4, 4)])
        # Random curvature (frequency and amplitude)
        curve_freq = random.uniform(0.3, 0.7)
        curve_amp_y = random.uniform(3, 8)
        curve_amp_z = random.uniform(2, 5)

        num_steps = random.randint(40, 50) # Much more steps for continuous coverage

        current_pos = start_pos
        
        for step in range(num_steps):
            # --- NOISE AT EACH STEP (Wind, GPS error) ---
            noise = np.random.uniform(-1.5, 1.5, 3)
            
            # Base movement + Curvature + Noise
            x = current_pos[0] + base_dir[0] * 0.5  # Reduced step size for denser coverage
            y = current_pos[1] + base_dir[1] * 0.5 + np.sin(step * curve_freq) * curve_amp_y * 0.5
            z = current_pos[2] + base_dir[2] * 0.5 + np.cos(step * curve_freq) * curve_amp_z * 0.5
            
            # Update current position (with accumulated noise)
            current_pos = np.array([x, y, z]) + noise
            
            # Smaller, more consistent box size for continuous coverage
            size = 2.0  # Fixed size for uniform continuous line
            half_size = size / 2
            coords = (current_pos[0]-half_size, current_pos[1]-half_size, current_pos[2]-half_size, 
                      current_pos[0]+half_size, current_pos[1]+half_size, current_pos[2]+half_size)
            idx.insert(r * 100 + step, coords)
            route_points.append(coords)
        
        route_history.append((route_points, route_colors[r]))

    # 3. 3D Plot
    fig = plt.figure(figsize=(14, 11))
    ax = fig.add_subplot(111, projection='3d')

    # --- DRAW TRAJECTORIES AND LEAVES ---
    for i, (points, col) in enumerate(route_history):
        centers = [((p[0]+p[3])/2, (p[1]+p[4])/2, (p[2]+p[5])/2) for p in points]
        cx, cy, cz = zip(*centers)
        # Continuous trajectory (Dashed line)
        ax.plot(cx, cy, cz, color=col, linewidth=1.5, linestyle='--', alpha=0.7)
        
        # Individual boxes (Leaves)
        for p in points:
            draw_cube(ax, p, color=col, alpha=0.45, linewidth=1)
        
        if i == 0: ax.plot([], [], color='gray', label='Realistic Trajectory (Leaves)')

    # --- DRAW INTERMEDIATE NODES (Level 1 - Approximation) ---
    # Due to randomness, the MBRs will no longer be perfect.
    # We simulate the R-Tree grouping by calculating the MBRs of route sections.
    level2_mbrs = []  # Store Level 2 MBRs for visualization
    for r in range(4):
        points, col = route_history[r]
        # We divide each route into sections of 4 points (legal capacity)
        num_sections = int(np.ceil(len(points) / 4))
        
        # Group Level 1 into Level 2 (pairs within same drone)
        for t in range(0, num_sections, 2):
            section_pair = points[t*4 : (t+2)*4]  # Take 2 sections at a time (8 points)
            if not section_pair: continue
            
            # Draw Level 1 MBRs for the pair
            for section_idx in range(min(2, num_sections - t)):
                section = points[(t+section_idx)*4 : (t+section_idx+1)*4]
                if not section: continue
                
                min_p = [min(b[i] for b in section) for i in range(3)]
                max_p = [max(b[i+3] for b in section) for i in range(3)]
                
                intermediate_mbr = (min_p[0]-0.5, min_p[1]-0.5, min_p[2]-0.5, 
                                  max_p[0]+0.5, max_p[1]+0.5, max_p[2]+0.5)
                
                draw_cube(ax, intermediate_mbr, color='green', alpha=0.6, linewidth=2, 
                          label="Intermediate MBR (Level 1)" if (r==0 and t==0 and section_idx==0) else None)
            
            # Draw Level 2 MBR (grouping the pair)
            min_coords = [min(b[i] for b in section_pair) for i in range(3)]
            max_coords = [max(b[i+3] for b in section_pair) for i in range(3)]
            
            level2_mbr = (min_coords[0]-0.3, min_coords[1]-0.3, min_coords[2]-0.3,
                         max_coords[0]+0.3, max_coords[1]+0.3, max_coords[2]+0.3)
            
            level2_mbrs.append(level2_mbr)
            draw_cube(ax, level2_mbr, color='blue', alpha=0.4, linewidth=2.5,
                     label="Intermediate MBR (Level 2)" if (r==0 and t==0) else None)

    # --- DRAW INTERMEDIATE NODES (Level 3 - Group of Level 2) ---
    # Group Level 2 MBRs to create Level 3
    for i in range(0, len(level2_mbrs), 3):
        mbrs_group = level2_mbrs[i : i+3]  # Take 3 Level 2s at a time
        if not mbrs_group: continue
        
        min_coords = [min(b[j] for b in mbrs_group) for j in range(3)]
        max_coords = [max(b[j+3] for b in mbrs_group) for j in range(3)]
        
        level3_mbr = (min_coords[0]-0.5, min_coords[1]-0.5, min_coords[2]-0.5,
                     max_coords[0]+0.5, max_coords[1]+0.5, max_coords[2]+0.5)
        
        draw_cube(ax, level3_mbr, color='orange', alpha=0.3, linewidth=3,
                 label="Intermediate MBR (Level 3)" if i == 0 else None)

    # --- DRAW ROOT (Global) ---
    root_mbr = idx.bounds
    # Small margin so it doesn't touch the outer boxes
    root_mbr_vis = (root_mbr[0]-1, root_mbr[1]-1, root_mbr[2]-1, 
                    root_mbr[3]+1, root_mbr[4]+1, root_mbr[5]+1)
    draw_cube(ax, root_mbr_vis, color='red', alpha=0.85, linewidth=4, label="Root (Root MBR)")

    # Aesthetics and limits
    ax.set_xlabel('X Axis (m)'); ax.set_ylabel('Y Axis (m)'); ax.set_zlabel('Z Axis (m)')
    
    
    # Adjust view
    ax.view_init(elev=20, azim=-120)
    
    # Set limits based on the Root (with margin)
    ax.set_xlim(root_mbr[0]-10, root_mbr[3]+10)
    ax.set_ylim(root_mbr[1]-10, root_mbr[4]+10)
    ax.set_zlim(root_mbr[2]-10, root_mbr[5]+10)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    visualize_realistic_fleet()