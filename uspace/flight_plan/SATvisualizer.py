import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def draw_obb_projection():
    """Visualize SAT projection in 2D"""
    # 1. OBB Configuration
    center = np.array([0, 0])
    half_lengths = np.array([5, 2]) # e1, e2 (Length and Height)
    angle = np.radians(30)          # 30 degree rotation
    
    # Local box axes (u1, u2)
    u1 = np.array([np.cos(angle), np.sin(angle)])
    u2 = np.array([-np.sin(angle), np.cos(angle)])
    
    # 2. Define the Projection Axis (L) - For example, X axis
    L = np.array([1, 0]) 
    
    # 3. CALCULATE THE PROJECTION
    # Projection = e1 * |u1 · L| + e2 * |u2 · L|
    proj_r = half_lengths[0] * abs(np.dot(u1, L)) + \
             half_lengths[1] * abs(np.dot(u2, L))
    
    # --- DRAWING ---
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Draw the OBB (4 corners)
    corners = np.array([
        center + u1*half_lengths[0] + u2*half_lengths[1],
        center - u1*half_lengths[0] + u2*half_lengths[1],
        center - u1*half_lengths[0] - u2*half_lengths[1],
        center + u1*half_lengths[0] - u2*half_lengths[1],
        center + u1*half_lengths[0] + u2*half_lengths[1]
    ])
    ax.plot(corners[:, 0], corners[:, 1], 'b', label='OBB (Box)')
    
    # Draw the "Shadow" on the X axis
    ax.plot([-proj_r, proj_r], [-3, -3], 'r', lw=4, label='Projection (Shadow)')
    ax.axvline(x=-proj_r, color='r', linestyle='--', alpha=0.3)
    ax.axvline(x=proj_r, color='r', linestyle='--', alpha=0.3)
    
    # Styling
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.axhline(0, color='black', lw=1)
    ax.axvline(0, color='black', lw=1)
    ax.set_title(f"Projection Radius: {proj_r:.2f}")
    ax.legend()
    ax.set_aspect('equal')
    plt.grid(True)
    plt.show()


def get_obb_corners_3d(center, axes, half_extents):
    """Generate 8 corners of an OBB in 3D"""
    corners = []
    for i in [-1, 1]:
        for j in [-1, 1]:
            for k in [-1, 1]:
                corner = (center + 
                         i * half_extents[0] * axes[0] +
                         j * half_extents[1] * axes[1] +
                         k * half_extents[2] * axes[2])
                corners.append(corner)
    return np.array(corners)


def project_obb_on_axis_3d(center, axes, half_extents, axis):
    """Project OBB onto an axis. Returns (min_proj, max_proj)"""
    axis = np.array(axis)
    axis_norm = axis / (np.linalg.norm(axis) + 1e-10)
    
    # Project center
    proj_center = np.dot(center, axis_norm)
    
    # Project half-extents
    proj_half = sum(abs(np.dot(axes[i], axis_norm)) * half_extents[i] 
                   for i in range(3))
    
    return proj_center - proj_half, proj_center + proj_half


def draw_obb_projection_3d():
    """Visualize SAT collision detection in 3D with two OBBs"""
    
    # OBB 1 Configuration
    center1 = np.array([0, 0, 0])
    angle1 = np.radians(20)
    axes1 = np.array([
        [np.cos(angle1), np.sin(angle1), 0],           # forward
        [-np.sin(angle1), np.cos(angle1), 0],          # right
        [0, 0, 1]                                       # up
    ])
    half_extents1 = np.array([4, 2, 1.5])
    
    # OBB 2 Configuration (rotated differently)
    center2 = np.array([6, 3, 2])
    angle2_x = np.radians(30)
    angle2_z = np.radians(45)
    # Rotation around X
    rot_x = np.array([
        [1, 0, 0],
        [0, np.cos(angle2_x), -np.sin(angle2_x)],
        [0, np.sin(angle2_x), np.cos(angle2_x)]
    ])
    # Rotation around Z
    rot_z = np.array([
        [np.cos(angle2_z), -np.sin(angle2_z), 0],
        [np.sin(angle2_z), np.cos(angle2_z), 0],
        [0, 0, 1]
    ])
    axes2 = rot_z @ rot_x
    half_extents2 = np.array([3, 2.5, 1.2])
    
    # Get corners
    corners1 = get_obb_corners_3d(center1, axes1, half_extents1)
    corners2 = get_obb_corners_3d(center2, axes2, half_extents2)
    
    # ===== FIGURE 1: 3D View =====
    fig_3d = plt.figure(figsize=(12, 10))
    fig_3d.suptitle('3D OBBs Configuration - SAT Visualization', fontsize=16, fontweight='bold')
    
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    
    # Draw OBB 1
    edges1 = [
        [0, 1], [1, 3], [3, 2], [2, 0],  # Bottom face
        [4, 5], [5, 7], [7, 6], [6, 4],  # Top face
        [0, 4], [1, 5], [2, 6], [3, 7]   # Vertical edges
    ]
    for edge in edges1:
        points = corners1[edge]
        ax_3d.plot3D(*points.T, 'b-', linewidth=2.5, label='OBB1' if edge == edges1[0] else '')
    
    # Draw OBB 2
    for edge in edges1:
        points = corners2[edge]
        ax_3d.plot3D(*points.T, 'r-', linewidth=2.5, label='OBB2' if edge == edges1[0] else '')
    
    # Draw centers
    ax_3d.scatter(*center1, color='blue', s=150, label='OBB1 Center', marker='o', edgecolors='black', linewidth=1.5)
    ax_3d.scatter(*center2, color='red', s=150, label='OBB2 Center', marker='s', edgecolors='black', linewidth=1.5)
    
    # Draw axes for OBB1
    axis_length = 3
    colors = ['red', 'green', 'blue']
    labels_axes = ['Forward', 'Right', 'Up']
    for i, (color, label) in enumerate(zip(colors, labels_axes)):
        ax_3d.quiver(center1[0], center1[1], center1[2], 
                    axes1[i, 0]*axis_length, axes1[i, 1]*axis_length, axes1[i, 2]*axis_length,
                    color=color, arrow_length_ratio=0.15, linewidth=2, alpha=0.8)
    
    ax_3d.set_xlabel('X [m]', fontsize=11, fontweight='bold')
    ax_3d.set_ylabel('Y [m]', fontsize=11, fontweight='bold')
    ax_3d.set_zlabel('Z [m]', fontsize=11, fontweight='bold')
    ax_3d.set_title('OBB1 (Blue) vs OBB2 (Red)', fontsize=12, fontweight='bold', pad=20)
    ax_3d.legend(loc='upper left', fontsize=10)
    ax_3d.set_xlim(-5, 10)
    ax_3d.set_ylim(-5, 10)
    ax_3d.set_zlim(-3, 5)
    ax_3d.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # ===== FIGURE 2: All 15 Projection Axes =====
    fig_proj = plt.figure(figsize=(22, 14))
    fig_proj.suptitle('SAT: 15 Projection Axes Test\n"If ANY axis shows SEPARATION → NO COLLISION"', 
                      fontsize=18, fontweight='bold', color='#2c3e50')
    
    # Generate all 15 axes
    test_axes = []
    axis_labels = []
    
    # OBB1 axes (3)
    for i, label in enumerate(['Forward', 'Right', 'Up']):
        test_axes.append(axes1[i])
        axis_labels.append(f'OBB1 {label}')
    
    # OBB2 axes (3)
    for i, label in enumerate(['Forward', 'Right', 'Up']):
        test_axes.append(axes2[i])
        axis_labels.append(f'OBB2 {label}')
    
    # Cross products (9)
    for i in range(3):
        for j in range(3):
            cross_axis = np.cross(axes1[i], axes2[j])
            if np.linalg.norm(cross_axis) > 1e-10:
                test_axes.append(cross_axis)
                axis_labels.append(f'OBB1[{i}] × OBB2[{j}]')
    
    # Plot all 15 projections
    separated_count = 0
    for idx, (axis, label) in enumerate(zip(test_axes, axis_labels)):
        ax = fig_proj.add_subplot(3, 5, idx + 1)
        
        min1, max1 = project_obb_on_axis_3d(center1, axes1, half_extents1, axis)
        min2, max2 = project_obb_on_axis_3d(center2, axes2, half_extents2, axis)
        
        # Check overlap
        overlap = not (max1 < min2 or max2 < min1)
        if not overlap:
            separated_count += 1
        
        # Background color based on result
        bg_color = '#ffe6e6' if overlap else '#e6ffe6'
        ax.set_facecolor(bg_color)
        
        # Draw projections as rectangles
        size1 = max1 - min1
        size2 = max2 - min2
        
        ax.barh(1.3, size1, left=min1, height=0.4, color='#3498db', alpha=0.85, 
                edgecolor='#2c3e50', linewidth=2.5, label='OBB1')
        ax.barh(0.7, size2, left=min2, height=0.4, color='#e74c3c', alpha=0.85, 
                edgecolor='#2c3e50', linewidth=2.5, label='OBB2')
        
        # Title based on result
        title_color = '#c0392b' if overlap else '#27ae60'
        title_bg = '#ffcccc' if overlap else '#ccffcc'
        status_text = '❌ OVERLAP\n(Could collide)' if overlap else '✓ SEPARATED\n(No collision)'
        
        ax.set_ylim(0.3, 2.5)
        ax.set_xlabel('Position on Axis', fontsize=8, fontweight='bold')
        
        # Create title with background box
        title = ax.set_title(f'{label}\n{status_text}', 
                    color=title_color, fontweight='bold', fontsize=9, pad=12)
        title.set_bbox(dict(boxstyle='round,pad=0.5', facecolor=title_bg, alpha=0.8, edgecolor=title_color, linewidth=1.5))
        
        # Grid
        ax.grid(True, alpha=0.2, linestyle='--', axis='x')
        ax.set_yticks([])
        
        # Legend only on first subplot
        if idx == 0:
            ax.legend(loc='upper left', fontsize=7, framealpha=0.95, title='Shadows')
    
    # Add detailed explanation
    explanation = (
        'WHAT ARE THESE GRAPHS?\n\n'
        '• Each shows ONE test axis from SAT\n'
        '• Blue bar = OBB1 "shadow" on that axis\n'
        '• Red bar = OBB2 "shadow" on that axis\n\n'
        '• NO OVERLAP → Axis separates boxes → ✓ No collision\n'
        '• OVERLAP → Keep testing other axes\n\n'
        '• THEOREM: If ANY axis separates them,\n'
        '   the objects 100% DO NOT collide\n'
        '• If ALL axes overlap, they collide'
    )
    
    fig_proj.text(0.02, 0.97, explanation, transform=fig_proj.transFigure,
                  fontsize=9.5, verticalalignment='top', fontfamily='monospace',
                  bbox=dict(boxstyle='round', facecolor='#f0f0f0', alpha=0.95, edgecolor='#333', linewidth=2, pad=12))
    
    # Add summary result
    result_text = f'✅ RESULT: Found {separated_count} separating axes → Safely NO Collision' if separated_count > 0 else f'❌ RESULT: No separating axis → Collision Likely'
    result_color = 'darkgreen' if separated_count > 0 else 'darkred'
    fig_proj.text(0.5, 0.01, result_text, 
                  ha='center', fontsize=12, fontweight='bold', color=result_color, 
                  bbox=dict(boxstyle='round', facecolor='#ccffcc' if separated_count > 0 else '#ffcccc', 
                           alpha=0.95, edgecolor=result_color, linewidth=2.5))
    
    plt.tight_layout(rect=[0.16, 0.04, 1, 0.96])
    plt.show()


def draw_obb_projection_3d_separated():
    """
    Visualize SAT collision detection with two SEPARATED OBBs.
    
    This demonstrates how the Separating Axis Theorem correctly identifies
    non-colliding boxes.
    """
    
    # OBB 1 Configuration (on the left)
    center1 = np.array([-6, 0, 0])
    angle1 = np.radians(15)
    axes1 = np.array([
        [np.cos(angle1), np.sin(angle1), 0],
        [-np.sin(angle1), np.cos(angle1), 0],
        [0, 0, 1]
    ])
    half_extents1 = np.array([3, 1.5, 1.2])
    
    # OBB 2 Configuration (far away on the right)
    center2 = np.array([10, 5, 3])
    angle2_x = np.radians(25)
    angle2_z = np.radians(35)
    rot_x = np.array([
        [1, 0, 0],
        [0, np.cos(angle2_x), -np.sin(angle2_x)],
        [0, np.sin(angle2_x), np.cos(angle2_x)]
    ])
    rot_z = np.array([
        [np.cos(angle2_z), -np.sin(angle2_z), 0],
        [np.sin(angle2_z), np.cos(angle2_z), 0],
        [0, 0, 1]
    ])
    axes2 = rot_z @ rot_x
    half_extents2 = np.array([2.5, 2, 1])
    
    # Get corners
    corners1 = get_obb_corners_3d(center1, axes1, half_extents1)
    corners2 = get_obb_corners_3d(center2, axes2, half_extents2)
    
    # ===== FIGURE 1: 3D View of Separated OBBs =====
    fig_3d = plt.figure(figsize=(12, 10))
    fig_3d.suptitle('Separated OBBs - SAT Visualization (NO COLLISION)', 
                    fontsize=16, fontweight='bold', color='green')
    
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    
    # Draw OBB 1
    edges1 = [
        [0, 1], [1, 3], [3, 2], [2, 0],
        [4, 5], [5, 7], [7, 6], [6, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]
    for edge in edges1:
        points = corners1[edge]
        ax_3d.plot3D(*points.T, 'b-', linewidth=2.5, label='OBB1' if edge == edges1[0] else '')
    
    # Draw OBB 2
    for edge in edges1:
        points = corners2[edge]
        ax_3d.plot3D(*points.T, 'r-', linewidth=2.5, label='OBB2' if edge == edges1[0] else '')
    
    # Draw centers
    ax_3d.scatter(*center1, color='blue', s=150, label='OBB1 Center', marker='o', edgecolors='black', linewidth=1.5)
    ax_3d.scatter(*center2, color='red', s=150, label='OBB2 Center', marker='s', edgecolors='black', linewidth=1.5)
    
    # Draw a line between centers to show distance
    ax_3d.plot([center1[0], center2[0]], [center1[1], center2[1]], [center1[2], center2[2]], 
               'k--', linewidth=1.5, alpha=0.5, label='Distance')
    
    # Draw axes
    axis_length = 2.5
    colors = ['red', 'green', 'blue']
    for i, color in enumerate(zip(colors)):
        ax_3d.quiver(center1[0], center1[1], center1[2], 
                    axes1[i, 0]*axis_length, axes1[i, 1]*axis_length, axes1[i, 2]*axis_length,
                    color=color, arrow_length_ratio=0.15, linewidth=1.5, alpha=0.6)
    
    ax_3d.set_xlabel('X [m]', fontsize=11, fontweight='bold')
    ax_3d.set_ylabel('Y [m]', fontsize=11, fontweight='bold')
    ax_3d.set_zlabel('Z [m]', fontsize=11, fontweight='bold')
    ax_3d.set_title('OBB1 (Blue) vs OBB2 (Red) - SEPARATED', fontsize=12, fontweight='bold', pad=20)
    ax_3d.legend(loc='upper left', fontsize=10)
    ax_3d.set_xlim(-12, 15)
    ax_3d.set_ylim(-5, 10)
    ax_3d.set_zlim(-2, 6)
    ax_3d.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # ===== FIGURE 2: All 15 Projection Axes for Separated OBBs =====
    fig_proj = plt.figure(figsize=(22, 14))
    fig_proj.suptitle('SAT: 15 Axes for Separated OBBs\n"All axes should show SEPARATION"', 
                      fontsize=18, fontweight='bold', color='green')
    
    # Generate all 15 axes
    test_axes = []
    axis_labels = []
    
    # OBB1 axes (3)
    for i, label in enumerate(['Forward', 'Right', 'Up']):
        test_axes.append(axes1[i])
        axis_labels.append(f'OBB1 {label}')
    
    # OBB2 axes (3)
    for i, label in enumerate(['Forward', 'Right', 'Up']):
        test_axes.append(axes2[i])
        axis_labels.append(f'OBB2 {label}')
    
    # Cross products (9)
    for i in range(3):
        for j in range(3):
            cross_axis = np.cross(axes1[i], axes2[j])
            if np.linalg.norm(cross_axis) > 1e-10:
                test_axes.append(cross_axis)
                axis_labels.append(f'OBB1[{i}] × OBB2[{j}]')
    
    # Plot all 15 projections
    separated_count = 0
    for idx, (axis, label) in enumerate(zip(test_axes, axis_labels)):
        ax = fig_proj.add_subplot(3, 5, idx + 1)
        
        min1, max1 = project_obb_on_axis_3d(center1, axes1, half_extents1, axis)
        min2, max2 = project_obb_on_axis_3d(center2, axes2, half_extents2, axis)
        
        # Check overlap
        overlap = not (max1 < min2 or max2 < min1)
        if not overlap:
            separated_count += 1
        
        # Background color based on result
        bg_color = '#ffe6e6' if overlap else '#e6ffe6'
        ax.set_facecolor(bg_color)
        
        # Draw projections
        ax.barh(1.3, max1 - min1, left=min1, height=0.4, color='#3498db', alpha=0.85, 
                edgecolor='#2c3e50', linewidth=2.5, label='OBB1')
        ax.barh(0.7, max2 - min2, left=min2, height=0.4, color='#e74c3c', alpha=0.85, 
                edgecolor='#2c3e50', linewidth=2.5, label='OBB2')
        
        # Title color based on overlap
        title_color = '#c0392b' if overlap else '#27ae60'
        title_bg = '#ffcccc' if overlap else '#ccffcc'
        status_text = '❌ OVERLAP\n(Could collide)' if overlap else '✓ SEPARATED\n(No collision)'
        
        ax.set_ylim(0.3, 2.5)
        ax.set_xlabel('Position on Axis', fontsize=8, fontweight='bold')
        
        title = ax.set_title(f'{label}\n{status_text}', 
                    color=title_color, fontweight='bold', fontsize=9, pad=12)
        title.set_bbox(dict(boxstyle='round,pad=0.5', facecolor=title_bg, alpha=0.8, edgecolor=title_color, linewidth=1.5))
        
        ax.grid(True, alpha=0.2, linestyle='--', axis='x')
        ax.set_yticks([])
        
        if idx == 0:
            ax.legend(loc='upper left', fontsize=7, framealpha=0.95, title='Shadows')
    
    # Add explanation
    explanation = (
        'WHAT ARE THESE GRAPHS?\n\n'
        '• Each shows ONE test axis from SAT\n'
        '• Blue bar = OBB1 "shadow" on that axis\n'
        '• Red bar = OBB2 "shadow" on that axis\n\n'
        '• NO OVERLAP → Axis separates → ✓ No collision\n'
        '• OVERLAP → Keep testing\n\n'
        '• THEOREM: If ANY axis shows NO overlap,\n'
        '   objects 100% DO NOT collide'
    )
    
    fig_proj.text(0.02, 0.97, explanation, transform=fig_proj.transFigure,
                  fontsize=9.5, verticalalignment='top', fontfamily='monospace',
                  bbox=dict(boxstyle='round', facecolor='#f0f0f0', alpha=0.95, edgecolor='#333', linewidth=2, pad=12))
    
    # Add summary
    result_text = f'✅ SUCCESS: {separated_count}/15 axes confirm NO collision'
    fig_proj.text(0.5, 0.01, result_text, 
                  ha='center', fontsize=12, fontweight='bold', color='darkgreen', 
                  bbox=dict(boxstyle='round', facecolor='#ccffcc', alpha=0.95, edgecolor='green', linewidth=2.5))
    
    plt.tight_layout(rect=[0.16, 0.04, 1, 0.96])
    plt.show()


def draw_projection_explanation():
    """
    Educational visualization that explains what projection means in SAT
    """
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('Understanding SAT: What is a Projection?', fontsize=18, fontweight='bold')
    
    # ===== LEFT: 3D View =====
    ax_3d = fig.add_subplot(1, 2, 1, projection='3d')
    
    # Create a simple rotated box
    center = np.array([2, 2, 2])
    angle = np.radians(35)
    axes_box = np.array([
        [np.cos(angle), np.sin(angle), 0],
        [-np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    half_extents_box = np.array([2, 1, 1])
    
    corners = get_obb_corners_3d(center, axes_box, half_extents_box)
    
    # Draw the box
    edges = [
        [0, 1], [1, 3], [3, 2], [2, 0],
        [4, 5], [5, 7], [7, 6], [6, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]
    for edge in edges:
        points = corners[edge]
        ax_3d.plot3D(*points.T, 'b-', linewidth=2.5)
    
    # Draw the test axis (red arrow)
    test_axis = np.array([1, 0.5, 0.3])
    test_axis = test_axis / np.linalg.norm(test_axis)
    ax_3d.quiver(0, 0, 0, test_axis[0]*5, test_axis[1]*5, test_axis[2]*5, 
                color='red', arrow_length_ratio=0.15, linewidth=3, label='Test Axis')
    
    # Project corners onto the axis
    projections = np.array([np.dot(c, test_axis) for c in corners])
    min_proj, max_proj = projections.min(), projections.max()
    
    # Draw projection points
    for i, corner in enumerate(corners):
        proj_point = test_axis * np.dot(corner, test_axis)
        ax_3d.scatter(*proj_point, color='red', s=50, alpha=0.5)
        ax_3d.plot([corner[0], proj_point[0]], [corner[1], proj_point[1]], [corner[2], proj_point[2]], 
                  'r--', alpha=0.3, linewidth=1)
    
    # Highlight min and max projections
    min_corner = test_axis * min_proj
    max_corner = test_axis * max_proj
    ax_3d.scatter(*min_corner, color='green', s=200, marker='o', edgecolors='black', linewidth=2, label='Min Projection')
    ax_3d.scatter(*max_corner, color='orange', s=200, marker='o', edgecolors='black', linewidth=2, label='Max Projection')
    
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    ax_3d.set_title('3D: Box with Test Axis', fontweight='bold')
    ax_3d.legend()
    ax_3d.set_xlim(-2, 6)
    ax_3d.set_ylim(-2, 6)
    ax_3d.set_zlim(-2, 4)
    
    # ===== RIGHT: 1D Projection =====
    ax_proj = fig.add_subplot(1, 2, 2)
    
    # Draw the projection axis (1D)
    ax_proj.arrow(-0.5, 0, 6, 0, head_width=0.15, head_length=0.2, fc='red', ec='red', linewidth=2)
    ax_proj.text(5.8, -0.4, 'Projection Axis', fontsize=12, fontweight='bold', color='red')
    
    # Draw the projection range as a bar
    ax_proj.barh(0, max_proj - min_proj, left=min_proj, height=0.3, 
                color='#3498db', alpha=0.7, edgecolor='black', linewidth=2.5, label='Box Shadow')
    
    # Mark min and max
    ax_proj.scatter([min_proj], [0], color='green', s=200, marker='o', 
                   edgecolors='black', linewidth=2, zorder=5, label='Min')
    ax_proj.scatter([max_proj], [0], color='orange', s=200, marker='o', 
                   edgecolors='black', linewidth=2, zorder=5, label='Max')
    
    # Add measurements
    ax_proj.annotate('', xy=(max_proj, 0.5), xytext=(min_proj, 0.5),
                    arrowprops=dict(arrowstyle='<->', color='black', lw=2))
    ax_proj.text((min_proj + max_proj) / 2, 0.7, f'Size: {max_proj - min_proj:.2f}', 
                ha='center', fontsize=11, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    ax_proj.set_xlim(-1, 6)
    ax_proj.set_ylim(-1, 1.5)
    ax_proj.set_xlabel('Position on Axis', fontsize=11, fontweight='bold')
    ax_proj.set_title('1D Projection: "Shadow" on the Axis', fontweight='bold')
    ax_proj.legend(loc='upper left', fontsize=10)
    ax_proj.set_yticks([])
    ax_proj.grid(True, alpha=0.3, axis='x')
    
    # Add explanation box
    explanation = (
        'HOW SAT WORKS:\n\n'
        '1. Pick a test axis (red line)\n'
        '2. Project all corners onto that axis\n'
        '3. Create a 1D "shadow" (blue bar)\n'
        '4. Compare shadows of two boxes\n'
        '5. If shadows DON\'T overlap →\n'
        '   Objects CANNOT collide!\n\n'
        'We test 15 axes to be sure.'
    )
    
    fig.text(0.5, 0.02, explanation, ha='center', fontsize=11, fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='#ffffcc', alpha=0.9, edgecolor='black', linewidth=2, pad=10))
    
    plt.tight_layout(rect=[0, 0.15, 1, 0.96])
    plt.show()


if __name__ == "__main__":
    print("=" * 60)
    print("SAT (Separating Axis Theorem) Visualizer")
    print("=" * 60)
    
    print("\n[0] Educational: What is a Projection?")
    try:
        draw_projection_explanation()
    except Exception as e:
        print(f"Error: {e}")
    
    print("\n[1] 2D SAT Visualization:")
    try:
        draw_obb_projection()
    except Exception as e:
        print(f"Error in 2D visualization: {e}")
    
    print("\n[2] 3D SAT Visualization with two OBBs (CLOSE/OVERLAPPING):")
    try:
        draw_obb_projection_3d()
    except Exception as e:
        print(f"Error in 3D visualization: {e}")
    
    print("\n[3] 3D SAT Visualization with two SEPARATED OBBs:")
    try:
        draw_obb_projection_3d_separated()
    except Exception as e:
        print(f"Error in 3D separated visualization: {e}")
    
    print("\n" + "=" * 60)
    print("Done!")
    print("=" * 60)