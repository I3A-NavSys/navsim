# -*- coding: utf-8 -*-
"""
SATvisualizer.py — Visualizer: Separating Axis Theorem (SAT)
==============================================================================

PURPOSE:
    Comprehensive demonstration of the Separating Axis Theorem (SAT) for 3D 
    Oriented Bounding Box (OBB) collision detection. Includes:
      - 2D projection visualization (pedagogical intro)
      - 3D OBB configurations with axis visualizations
      - All 15 test axes (3 per box face + 9 cross-product axes)
      - Separated and overlapping scenarios

VISUALIZATION:
    - Interactive 3D plots showing OBB configurations and their relative 
      positioning
    - 15-panel projection tests showing shadow overlaps on each test axis
    - Color-coded feedback: green = separation confirmed, red = overlap
    - Educational diagrams explaining the projection concept
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Path setup targeting the project root
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

# -- Color Palette ---------------------------------------------------------------
C_BOX_A         = "#3498db"   # Blue
C_BOX_B         = "#e74c3c"   # Red
C_SEPARATED     = "#27ae60"   # Green (no collision)
C_OVERLAPPING   = "#c0392b"   # Dark red (collision likely)
C_AXIS_A        = "#f39c12"   # Orange
C_CROSS_AXIS    = "#9b59b6"   # Purple
C_SEPARATOR     = "#2ecc71"   # Bright green
C_BG_DARK       = "#ecf0f1"   # Light gray background
C_BG_LIGHT      = "#ffffff"   # White


# ==============================================================================
# Geometry Helpers
# ==============================================================================

def get_obb_corners_3d(center, axes, half_extents):
    """Generate the 8 corners of an OBB in 3D space.
    
    Args:
        center: 3D position of box center
        axes: 3x3 matrix of normalized local axes (rows are forward, right, up)
        half_extents: 3D array of half-widths along each axis
    
    Returns:
        8x3 array of corner positions
    """


def get_obb_corners_3d(center, axes, half_extents):
    """Generate the 8 corners of an OBB in 3D space.
    
    Args:
        center: 3D position of box center
        axes: 3x3 matrix of normalized local axes (rows are forward, right, up)
        half_extents: 3D array of half-widths along each axis
    
    Returns:
        8x3 array of corner positions
    """
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
    """Project an OBB onto a test axis and return the interval.
    
    Args:
        center: 3D position of box center
        axes: 3x3 matrix of box local axes
        half_extents: 3D array of half-widths
        axis: Test axis (3D vector)
    
    Returns:
        Tuple (min_projection, max_projection)
    """
    axis = np.array(axis)
    axis_norm = axis / (np.linalg.norm(axis) + 1e-10)
    
    proj_center = np.dot(center, axis_norm)
    proj_half = sum(abs(np.dot(axes[i], axis_norm)) * half_extents[i] 
                   for i in range(3))
    
    return proj_center - proj_half, proj_center + proj_half


# ==============================================================================
# Visualization Functions
# ==============================================================================

def visualize_projection_concept():
    """Educational visualization explaining the SAT projection concept."""
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('Understanding SAT: What is a Projection?', 
                 fontsize=18, fontweight='bold')
    
    # 3D View
    ax_3d = fig.add_subplot(1, 2, 1, projection='3d')
    
    center = np.array([2, 2, 2])
    angle = np.radians(35)
    axes_box = np.array([
        [np.cos(angle), np.sin(angle), 0],
        [-np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    half_extents_box = np.array([2, 1, 1])
    
    corners = get_obb_corners_3d(center, axes_box, half_extents_box)
    
    # Draw box edges
    edges = [
        [0, 1], [1, 3], [3, 2], [2, 0],
        [4, 5], [5, 7], [7, 6], [6, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]
    for edge in edges:
        points = corners[edge]
        ax_3d.plot3D(*points.T, color=C_BOX_A, linewidth=2.5)
    
    # Test axis
    test_axis = np.array([1, 0.5, 0.3])
    test_axis = test_axis / np.linalg.norm(test_axis)
    ax_3d.quiver(0, 0, 0, test_axis[0]*5, test_axis[1]*5, test_axis[2]*5, 
                color='red', arrow_length_ratio=0.15, linewidth=3, label='Test Axis')
    
    # Project corners and visualize
    projections = np.array([np.dot(c, test_axis) for c in corners])
    min_proj, max_proj = projections.min(), projections.max()
    
    for corner in corners:
        proj_point = test_axis * np.dot(corner, test_axis)
        ax_3d.scatter(*proj_point, color='red', s=50, alpha=0.5)
        ax_3d.plot([corner[0], proj_point[0]], [corner[1], proj_point[1]], 
                  [corner[2], proj_point[2]], 'r--', alpha=0.3, linewidth=1)
    
    min_corner = test_axis * min_proj
    max_corner = test_axis * max_proj
    ax_3d.scatter(*min_corner, color=C_SEPARATOR, s=200, marker='o', 
                 edgecolors='black', linewidth=2, label='Min Projection')
    ax_3d.scatter(*max_corner, color='orange', s=200, marker='o', 
                 edgecolors='black', linewidth=2, label='Max Projection')
    
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    ax_3d.set_title('3D: Box with Test Axis', fontweight='bold')
    ax_3d.legend()
    ax_3d.set_xlim(-2, 6)
    ax_3d.set_ylim(-2, 6)
    ax_3d.set_zlim(-2, 4)
    
    # 1D Projection View
    ax_proj = fig.add_subplot(1, 2, 2)
    
    ax_proj.arrow(-0.5, 0, 6, 0, head_width=0.15, head_length=0.2, 
                 fc='red', ec='red', linewidth=2)
    ax_proj.text(5.8, -0.4, 'Projection Axis', fontsize=12, fontweight='bold', color='red')
    
    ax_proj.barh(0, max_proj - min_proj, left=min_proj, height=0.3, 
                color=C_BOX_A, alpha=0.7, edgecolor='black', linewidth=2.5, label='Box Shadow')
    
    ax_proj.scatter([min_proj], [0], color=C_SEPARATOR, s=200, marker='o', 
                   edgecolors='black', linewidth=2, zorder=5, label='Min')
    ax_proj.scatter([max_proj], [0], color='orange', s=200, marker='o', 
                   edgecolors='black', linewidth=2, zorder=5, label='Max')
    
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
    
    explanation = (
        'HOW SAT WORKS:\n\n'
        '1. Pick a test axis\n'
        '2. Project all corners onto that axis\n'
        '3. Create a 1D "shadow" (blue bar)\n'
        '4. Compare shadows of two boxes\n'
        '5. If shadows DON\'T overlap →\n'
        '   Objects CANNOT collide!\n\n'
        'We test 15 axes total.'
    )
    
    fig.text(0.5, 0.02, explanation, ha='center', fontsize=11, fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='#ffffcc', alpha=0.9, 
                     edgecolor='black', linewidth=2, pad=10))
    
    plt.tight_layout(rect=[0, 0.15, 1, 0.96])
    plt.show()


def visualize_overlapping_obbs():
    """Visualize SAT collision detection with two overlapping OBBs."""
    
    # OBB 1
    center1 = np.array([0, 0, 0])
    angle1 = np.radians(20)
    axes1 = np.array([
        [np.cos(angle1), np.sin(angle1), 0],
        [-np.sin(angle1), np.cos(angle1), 0],
        [0, 0, 1]
    ])
    half_extents1 = np.array([4, 2, 1.5])
    
    # OBB 2
    center2 = np.array([6, 3, 2])
    angle2_x = np.radians(30)
    angle2_z = np.radians(45)
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
    half_extents2 = np.array([3, 2.5, 1.2])
    
    corners1 = get_obb_corners_3d(center1, axes1, half_extents1)
    corners2 = get_obb_corners_3d(center2, axes2, half_extents2)
    
    # 3D Plot
    fig_3d = plt.figure(figsize=(14, 10))
    fig_3d.suptitle('3D OBBs: SAT Collision Detection (Overlapping Case)', 
                    fontsize=16, fontweight='bold')
    
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    
    edges1 = [
        [0, 1], [1, 3], [3, 2], [2, 0],
        [4, 5], [5, 7], [7, 6], [6, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]
    
    for edge in edges1:
        points = corners1[edge]
        ax_3d.plot3D(*points.T, color=C_BOX_A, linewidth=2.5, label='OBB1' if edge == edges1[0] else '')
    
    for edge in edges1:
        points = corners2[edge]
        ax_3d.plot3D(*points.T, color=C_BOX_B, linewidth=2.5, label='OBB2' if edge == edges1[0] else '')
    
    ax_3d.scatter(*center1, color=C_BOX_A, s=150, label='OBB1 Center', 
                 marker='o', edgecolors='black', linewidth=1.5)
    ax_3d.scatter(*center2, color=C_BOX_B, s=150, label='OBB2 Center', 
                 marker='s', edgecolors='black', linewidth=1.5)
    
    axis_length = 3
    colors_ax = ['red', 'green', 'blue']
    for i, color in enumerate(colors_ax):
        ax_3d.quiver(center1[0], center1[1], center1[2], 
                    axes1[i, 0]*axis_length, axes1[i, 1]*axis_length, axes1[i, 2]*axis_length,
                    color=color, arrow_length_ratio=0.15, linewidth=2, alpha=0.8)
    
    ax_3d.set_xlabel('X [m]', fontsize=11, fontweight='bold')
    ax_3d.set_ylabel('Y [m]', fontsize=11, fontweight='bold')
    ax_3d.set_zlabel('Z [m]', fontsize=11, fontweight='bold')
    ax_3d.legend(loc='upper left', fontsize=10)
    ax_3d.set_xlim(-5, 10)
    ax_3d.set_ylim(-5, 10)
    ax_3d.set_zlim(-3, 5)
    ax_3d.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # 15-Axis Projection Analysis
    fig_proj = plt.figure(figsize=(22, 14))
    fig_proj.suptitle('SAT Analysis: 15 Projection Axes (Overlapping Case)\n'
                      '"If ANY axis shows SEPARATION → NO COLLISION"', 
                      fontsize=18, fontweight='bold')
    
    test_axes = []
    axis_labels = []
    
    # Face normals of OBB1
    for i, label in enumerate(['Forward', 'Right', 'Up']):
        test_axes.append(axes1[i])
        axis_labels.append(f'OBB1 {label}')
    
    # Face normals of OBB2
    for i, label in enumerate(['Forward', 'Right', 'Up']):
        test_axes.append(axes2[i])
        axis_labels.append(f'OBB2 {label}')
    
    # Edge cross-products
    for i in range(3):
        for j in range(3):
            cross_axis = np.cross(axes1[i], axes2[j])
            if np.linalg.norm(cross_axis) > 1e-10:
                test_axes.append(cross_axis)
                axis_labels.append(f'OBB1[{i}] × OBB2[{j}]')
    
    separated_count = 0
    for idx, (axis, label) in enumerate(zip(test_axes, axis_labels)):
        ax = fig_proj.add_subplot(3, 5, idx + 1)
        
        min1, max1 = project_obb_on_axis_3d(center1, axes1, half_extents1, axis)
        min2, max2 = project_obb_on_axis_3d(center2, axes2, half_extents2, axis)
        
        overlap = not (max1 < min2 or max2 < min1)
        if not overlap:
            separated_count += 1
        
        bg_color = '#ffe6e6' if overlap else '#e6ffe6'
        ax.set_facecolor(bg_color)
        
        ax.barh(1.3, max1 - min1, left=min1, height=0.4, color=C_BOX_A, 
               alpha=0.85, edgecolor='#2c3e50', linewidth=2.5, label='OBB1')
        ax.barh(0.7, max2 - min2, left=min2, height=0.4, color=C_BOX_B, 
               alpha=0.85, edgecolor='#2c3e50', linewidth=2.5, label='OBB2')
        
        title_color = C_OVERLAPPING if overlap else C_SEPARATED
        title_bg = '#ffcccc' if overlap else '#ccffcc'
        status_text = '❌ OVERLAP\n(Could collide)' if overlap else '✓ SEPARATED\n(No collision)'
        
        ax.set_ylim(0.3, 2.5)
        ax.set_xlabel('Position on Axis', fontsize=8, fontweight='bold')
        
        title = ax.set_title(f'{label}\n{status_text}', 
                    color=title_color, fontweight='bold', fontsize=9, pad=12)
        title.set_bbox(dict(boxstyle='round,pad=0.5', facecolor=title_bg, 
                           alpha=0.8, edgecolor=title_color, linewidth=1.5))
        
        ax.grid(True, alpha=0.2, linestyle='--', axis='x')
        ax.set_yticks([])
        
        if idx == 0:
            ax.legend(loc='upper left', fontsize=7, framealpha=0.95, title='Shadows')
    
    explanation = (
        'UNDERSTANDING THE CHARTS:\n\n'
        '• Each panel = one SAT test axis\n'
        '• Blue bar = OBB1 projection\n'
        '• Red bar = OBB2 projection\n\n'
        '• NO OVERLAP → Separating axis found\n'
        '  → Boxes definitely don\'t collide\n'
        '• OVERLAP → Axes still compatible\n\n'
        '• THEOREM: If ANY axis separates\n'
        '  the boxes, collision is impossible\n'
        '• If ALL axes overlap, collision\n'
        '  is likely'
    )
    
    fig_proj.text(0.02, 0.97, explanation, transform=fig_proj.transFigure,
                  fontsize=9.5, verticalalignment='top', fontfamily='monospace',
                  bbox=dict(boxstyle='round', facecolor='#f0f0f0', alpha=0.95, 
                           edgecolor='#333', linewidth=2, pad=12))
    
    result_text = (f'✅ RESULT: Found {separated_count} separating axes → Safely NO Collision' 
                   if separated_count > 0 
                   else f'❌ RESULT: No separating axis → Collision Likely')
    result_color = 'darkgreen' if separated_count > 0 else 'darkred'
    fig_proj.text(0.5, 0.01, result_text, 
                  ha='center', fontsize=12, fontweight='bold', color=result_color, 
                  bbox=dict(boxstyle='round', facecolor='#ccffcc' if separated_count > 0 else '#ffcccc', 
                           alpha=0.95, edgecolor=result_color, linewidth=2.5))
    
    plt.tight_layout(rect=[0.16, 0.04, 1, 0.96])
    plt.show()


def visualize_separated_obbs():
    """Visualize SAT with two separated (non-colliding) OBBs."""
    
    # OBB 1 (left side)
    center1 = np.array([-6, 0, 0])
    angle1 = np.radians(15)
    axes1 = np.array([
        [np.cos(angle1), np.sin(angle1), 0],
        [-np.sin(angle1), np.cos(angle1), 0],
        [0, 0, 1]
    ])
    half_extents1 = np.array([3, 1.5, 1.2])
    
    # OBB 2 (right side, far away)
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
    
    corners1 = get_obb_corners_3d(center1, axes1, half_extents1)
    corners2 = get_obb_corners_3d(center2, axes2, half_extents2)
    
    # 3D Plot
    fig_3d = plt.figure(figsize=(14, 10))
    fig_3d.suptitle('3D OBBs: SAT Collision Detection (Separated Case)', 
                    fontsize=16, fontweight='bold', color=C_SEPARATED)
    
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    
    edges1 = [
        [0, 1], [1, 3], [3, 2], [2, 0],
        [4, 5], [5, 7], [7, 6], [6, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]
    
    for edge in edges1:
        points = corners1[edge]
        ax_3d.plot3D(*points.T, color=C_BOX_A, linewidth=2.5, label='OBB1' if edge == edges1[0] else '')
    
    for edge in edges1:
        points = corners2[edge]
        ax_3d.plot3D(*points.T, color=C_BOX_B, linewidth=2.5, label='OBB2' if edge == edges1[0] else '')
    
    ax_3d.scatter(*center1, color=C_BOX_A, s=150, label='OBB1 Center', 
                 marker='o', edgecolors='black', linewidth=1.5)
    ax_3d.scatter(*center2, color=C_BOX_B, s=150, label='OBB2 Center', 
                 marker='s', edgecolors='black', linewidth=1.5)
    
    ax_3d.plot([center1[0], center2[0]], [center1[1], center2[1]], [center1[2], center2[2]], 
               'k--', linewidth=1.5, alpha=0.5, label='Distance')
    
    axis_length = 2.5
    colors_ax = ['red', 'green', 'blue']
    for i, color in enumerate(colors_ax):
        ax_3d.quiver(center1[0], center1[1], center1[2], 
                    axes1[i, 0]*axis_length, axes1[i, 1]*axis_length, axes1[i, 2]*axis_length,
                    color=color, arrow_length_ratio=0.15, linewidth=1.5, alpha=0.6)
    
    ax_3d.set_xlabel('X [m]', fontsize=11, fontweight='bold')
    ax_3d.set_ylabel('Y [m]', fontsize=11, fontweight='bold')
    ax_3d.set_zlabel('Z [m]', fontsize=11, fontweight='bold')
    ax_3d.legend(loc='upper left', fontsize=10)
    ax_3d.set_xlim(-12, 15)
    ax_3d.set_ylim(-5, 10)
    ax_3d.set_zlim(-2, 6)
    ax_3d.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # 15-Axis Projection Analysis
    fig_proj = plt.figure(figsize=(22, 14))
    fig_proj.suptitle('SAT Analysis: 15 Projection Axes (Separated Case)\n'
                      '"All axes should show SEPARATION"', 
                      fontsize=18, fontweight='bold', color=C_SEPARATED)
    
    test_axes = []
    axis_labels = []
    
    for i, label in enumerate(['Forward', 'Right', 'Up']):
        test_axes.append(axes1[i])
        axis_labels.append(f'OBB1 {label}')
    
    for i, label in enumerate(['Forward', 'Right', 'Up']):
        test_axes.append(axes2[i])
        axis_labels.append(f'OBB2 {label}')
    
    for i in range(3):
        for j in range(3):
            cross_axis = np.cross(axes1[i], axes2[j])
            if np.linalg.norm(cross_axis) > 1e-10:
                test_axes.append(cross_axis)
                axis_labels.append(f'OBB1[{i}] × OBB2[{j}]')
    
    separated_count = 0
    for idx, (axis, label) in enumerate(zip(test_axes, axis_labels)):
        ax = fig_proj.add_subplot(3, 5, idx + 1)
        
        min1, max1 = project_obb_on_axis_3d(center1, axes1, half_extents1, axis)
        min2, max2 = project_obb_on_axis_3d(center2, axes2, half_extents2, axis)
        
        overlap = not (max1 < min2 or max2 < min1)
        if not overlap:
            separated_count += 1
        
        bg_color = '#ffe6e6' if overlap else '#e6ffe6'
        ax.set_facecolor(bg_color)
        
        ax.barh(1.3, max1 - min1, left=min1, height=0.4, color=C_BOX_A, 
               alpha=0.85, edgecolor='#2c3e50', linewidth=2.5, label='OBB1')
        ax.barh(0.7, max2 - min2, left=min2, height=0.4, color=C_BOX_B, 
               alpha=0.85, edgecolor='#2c3e50', linewidth=2.5, label='OBB2')
        
        title_color = C_OVERLAPPING if overlap else C_SEPARATED
        title_bg = '#ffcccc' if overlap else '#ccffcc'
        status_text = '❌ OVERLAP\n(Could collide)' if overlap else '✓ SEPARATED\n(No collision)'
        
        ax.set_ylim(0.3, 2.5)
        ax.set_xlabel('Position on Axis', fontsize=8, fontweight='bold')
        
        title = ax.set_title(f'{label}\n{status_text}', 
                    color=title_color, fontweight='bold', fontsize=9, pad=12)
        title.set_bbox(dict(boxstyle='round,pad=0.5', facecolor=title_bg, 
                           alpha=0.8, edgecolor=title_color, linewidth=1.5))
        
        ax.grid(True, alpha=0.2, linestyle='--', axis='x')
        ax.set_yticks([])
        
        if idx == 0:
            ax.legend(loc='upper left', fontsize=7, framealpha=0.95, title='Shadows')
    
    explanation = (
        'UNDERSTANDING THE CHARTS:\n\n'
        '• Each panel = one SAT test axis\n'
        '• Blue bar = OBB1 projection\n'
        '• Red bar = OBB2 projection\n\n'
        '• NO OVERLAP → Separating axis\n'
        '• OVERLAP → Boxes still aligned\n\n'
        '• In separated case: Most/all axes\n'
        '  show clear separation'
    )
    
    fig_proj.text(0.02, 0.97, explanation, transform=fig_proj.transFigure,
                  fontsize=9.5, verticalalignment='top', fontfamily='monospace',
                  bbox=dict(boxstyle='round', facecolor='#f0f0f0', alpha=0.95, 
                           edgecolor='#333', linewidth=2, pad=12))
    
    result_text = f'✅ SUCCESS: {separated_count}/15 axes confirm NO collision'
    fig_proj.text(0.5, 0.01, result_text, 
                  ha='center', fontsize=12, fontweight='bold', color='darkgreen', 
                  bbox=dict(boxstyle='round', facecolor='#ccffcc', alpha=0.95, 
                           edgecolor='green', linewidth=2.5))
    
    plt.tight_layout(rect=[0.16, 0.04, 1, 0.96])
    plt.show()


# ==============================================================================
# Main Visualization Runner
# ==============================================================================

def run_visualizer():
    """Main entry point for SAT visualizer demonstrations."""
    print("\n" + "="*70)
    print("SAT (Separating Axis Theorem) Visualizer")
    print("="*70)
    
    demos = [
        ("Educational: What is a Projection?", visualize_projection_concept),
        ("3D OBBs: Overlapping Case", visualize_overlapping_obbs),
        ("3D OBBs: Separated Case", visualize_separated_obbs),
    ]
    
    for i, (name, func) in enumerate(demos, 1):
        print(f"\n[{i}] {name}...")
        try:
            func()
        except Exception as e:
            print(f"  ⚠ Error: {e}")


# ==============================================================================
# Script Entry Point
# ==============================================================================

if __name__ == "__main__":
    run_visualizer()
    print("\n" + "="*70)
    print("Visualization complete!")
    print("="*70)