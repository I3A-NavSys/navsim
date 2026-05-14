"""
viz_01_obb_swept_volumes.py — Visualizer 1: Trajectory Discretization
========================================================================

PURPOSE:
    Demonstrate how a continuous flight plan is discretized into a sequence
    of Oriented Bounding Boxes (OBBs) representing swept volumes over time.
    This is the foundation of the continuous collision detection (CCD) system.

VISUALIZATION:
    Displays a 3D plot showing:
      - The continuous Bézier trajectory (curved path).
      - Waypoints marked as points.
      - A sequence of OBBs overlaid along the path.
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

from benchmark.flightplan_generator import generate_flight_plan


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
    print("VISUALIZER 1: OBB Swept Volumes & Trajectory")
    print("="*60)

    # 1. Generate a sample route
    print("Generating a kinematically-valid flight plan...")
    fp = generate_flight_plan(
        num_waypoints=5,
        speed_range=(10, 15),
        radius=5.0,
        seed=101
    )

    # 2. Get the trajectory trace
    time_step = 0.5
    trace = fp.trace(time_step)
    trace_x = trace[:, 1]
    trace_y = trace[:, 2]
    trace_z = trace[:, 3]

    # 3. Generate the Swept Boxes (CCD)
    print("Discretizing trajectory into Oriented Bounding Boxes (OBBs)...")
    interval = 1 
    obbs = fp.generate_swept_boxes_obb(interval=interval)

    # 4. Plotting
    print("Rendering 3D visualization...")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Trajectory Discretization via OBB Swept Volumes", fontsize=14, pad=20)

    # Plot continuous trajectory
    ax.plot(trace_x, trace_y, trace_z, color='dodgerblue', linewidth=3, label="Continuous Trajectory")

    # Plot specific waypoints
    wp_x = [wp.pos[0] for wp in fp.waypoints]
    wp_y = [wp.pos[1] for wp in fp.waypoints]
    wp_z = [wp.pos[2] for wp in fp.waypoints]
    ax.scatter(wp_x, wp_y, wp_z, color='red', s=50, marker='o', label="Waypoints")

    # Plot OBBs
    for i, box in enumerate(obbs):
        corners = box.get_corners()
        faces = generate_obb_faces(corners)

        color = 'orange' if i % 2 == 0 else 'limegreen'

        # Create a 3D polygon collection for the OBB
        poly3d = Poly3DCollection(faces, alpha=0.15, linewidths=1.0, edgecolors=color, facecolors=color)
        ax.add_collection3d(poly3d)

    # Configure axes
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # Set consistent aspect ratio
    max_range = np.array([trace_x.max()-trace_x.min(), trace_y.max()-trace_y.min(), trace_z.max()-trace_z.min()]).max() / 2.0
    mid_x = (trace_x.max() + trace_x.min()) * 0.5
    mid_y = (trace_y.max() + trace_y.min()) * 0.5
    mid_z = (trace_z.max() + trace_z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Add a custom legend entry for the OBBs
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    legend_elements = [
        Line2D([0], [0], color='dodgerblue', lw=3, label='Continuous Trajectory'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=8, label='Waypoints'),
        Patch(facecolor='orange', edgecolor='orange', alpha=0.3, label='Swept Volume OBBs')
    ]
    ax.legend(handles=legend_elements, loc='upper left')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_visualizer()
