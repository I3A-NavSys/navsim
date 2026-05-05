"""
viz_09_obb_within_mbr.py — Visualizer: OBB Enclosed by MBR
========================================================================

PURPOSE:
    Demonstrate how an Oriented Bounding Box (OBB) lies strictly inside its 
    Minimum Bounding Rectangle (MBR) / Axis-Aligned Bounding Box (AABB) 
    for broad-phase and narrow-phase collision detection filtering.

VISUALIZATION:
    Displays a 3D plot showing:
      - The OBB in limegreen (rotated according to the trajectory direction).
      - The MBR in semi-transparent red (aligned with the world axes).
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

# Path setup targeting the project root
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from detection.conflictDetection import SweptBox_OBB


def generate_obb_faces(corners):
    """Generate the 6 faces of an OBB from its 8 corners for 3D plotting."""
    corners = np.array(corners)
    # Mapping of corner indices to faces
    faces = [
        [corners[0], corners[1], corners[3], corners[2]], # Bottom
        [corners[4], corners[5], corners[7], corners[6]], # Top
        [corners[0], corners[1], corners[5], corners[4]], # Front
        [corners[2], corners[3], corners[7], corners[6]], # Back
        [corners[0], corners[2], corners[6], corners[4]], # Left
        [corners[1], corners[3], corners[7], corners[5]], # Right
    ]
    return faces


def run_visualizer():
    print("\n" + "="*60)
    print("VISUALIZER: OBB Enclosed by MBR")
    print("="*60)

    # 1. Create a perfectly proportional OBB
    # This prevents the box from being too long and thin (which looks like a 2D line)
    print("Creating a sample OBB with proportional 3D dimensions...")
    center = np.array([50.0, 50.0, 50.0])
    
    # 30-degree rotation around the Z axis to orient the OBB
    theta = np.radians(30)
    forward = np.array([np.cos(theta), np.sin(theta), 0.0])
    right   = np.array([-np.sin(theta), np.cos(theta), 0.0])
    up      = np.array([0.0, 0.0, 1.0])
    axes = np.array([forward, right, up])
    
    # Well-proportioned dimensions: 40m (Length) x 20m (Width) x 24m (Height)
    half_extents = np.array([20.0, 10.0, 12.0])
    box = SweptBox_OBB(center, axes, half_extents, t_start=0.0, t_end=1.0)

    # 2. Calculate the MBR corners using get_4d_bounds()
    print("Calculating the MBR (AABB) envelope in 4D...")
    x_min, y_min, z_min, t_min, x_max, y_max, z_max, t_max = box.get_4d_bounds()
    
    # Generate the 8 corners of the axis-aligned MBR following the exact same iteration pattern
    mbr_corners = []
    for i in [x_min, x_max]:
        for j in [y_min, y_max]:
            for k in [z_min, z_max]:
                mbr_corners.append([i, j, k])
    mbr_corners = np.array(mbr_corners)

    # 3. Plotting
    print("Rendering 3D visualization...")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("OBB Enclosed by its Axis-Aligned MBR (3D View)", fontsize=14, pad=20)

    # Use explicit box aspect to prevent axis distortion
    ax.set_box_aspect([1, 1, 1])

    # Plot the OBB
    obb_corners = box.get_corners()
    obb_faces = generate_obb_faces(obb_corners)
    poly_obb = Poly3DCollection(obb_faces, alpha=0.45, linewidths=2.0, edgecolors='forestgreen', facecolors='lime')
    ax.add_collection3d(poly_obb)

    # Plot the MBR
    mbr_faces = generate_obb_faces(mbr_corners)
    poly_mbr = Poly3DCollection(mbr_faces, alpha=0.15, linewidths=1.5, edgecolors='crimson', facecolors='salmon')
    ax.add_collection3d(poly_mbr)

    # Plot Center
    ax.scatter([box.center[0]], [box.center[1]], [box.center[2]], color='black', s=80, marker='o', label="Box Center")

    # Configure axes
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # Set consistent aspect ratio
    center = box.center
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min) * 0.6
    ax.set_xlim(center[0] - max_range, center[0] + max_range)
    ax.set_ylim(center[1] - max_range, center[1] + max_range)
    ax.set_zlim(center[2] - max_range, center[2] + max_range)

    # Set a custom view angle so the 3D volume is perfectly visible
    ax.view_init(elev=35, azim=55)

    # Legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='lime', edgecolor='forestgreen', alpha=0.45, label='Oriented Bounding Box (OBB)'),
        Patch(facecolor='salmon', edgecolor='crimson', alpha=0.15, label='Minimum Bounding Rectangle (MBR/AABB)')
    ]
    ax.legend(handles=legend_elements, loc='upper left')

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_visualizer()
