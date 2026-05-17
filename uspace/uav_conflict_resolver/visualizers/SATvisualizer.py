# -*- coding: utf-8 -*-
"""
SATvisualizer.py - Visualizer: SAT Collision Detection
=======================================================

PURPOSE:
    Demonstrate the Separating Axis Theorem (SAT) for 3D Oriented Bounding
    Box (OBB) collision detection using the 15 candidate axes.

VISUALIZATION:
    - 3D OBB configuration for overlapping and separated cases
    - 15-axis projection panels with overlap/separation status
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Path setup targeting the project root
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from detection.conflictDetection import SweptBox_OBB

# Palette
C_BOX_A = "#2ecc71"
C_BOX_B = "#e74c3c"
C_AXIS_A = "#3498db"
C_AXIS_B = "#f39c12"
C_AXIS_CROSS = "#9b59b6"
C_SEPARATED = "#27ae60"
C_OVERLAP = "#c0392b"


def build_obb_faces(obb: SweptBox_OBB):
    """Generate 6 quad faces of an OBB from corner ordering in get_corners()."""
    c = obb.get_corners()
    return [
        [c[0], c[1], c[3], c[2]],
        [c[4], c[5], c[7], c[6]],
        [c[0], c[1], c[5], c[4]],
        [c[2], c[3], c[7], c[6]],
        [c[0], c[2], c[6], c[4]],
        [c[1], c[3], c[7], c[5]],
    ]


def build_sat_axes(box_a: SweptBox_OBB, box_b: SweptBox_OBB):
    """Build SAT axis metadata: 3 from A, 3 from B, and up to 9 cross-axes."""
    axes_list = []

    for i, ax in enumerate(box_a.axes):
        axes_list.append({"axis": ax / np.linalg.norm(ax), "label": f"A{i}", "color": C_AXIS_A})

    for i, ax in enumerate(box_b.axes):
        axes_list.append({"axis": ax / np.linalg.norm(ax), "label": f"B{i}", "color": C_AXIS_B})

    for i in range(3):
        for j in range(3):
            cross = np.cross(box_a.axes[i], box_b.axes[j])
            n = np.linalg.norm(cross)
            if n > 1e-10:
                axes_list.append({"axis": cross / n, "label": f"A{i}xB{j}", "color": C_AXIS_CROSS})

    return axes_list


def project_on_axis(center, axes, half_extents, test_axis):
    """Project an OBB onto a normalized axis and return [min, max]."""
    axis = np.asarray(test_axis, dtype=float)
    axis = axis / (np.linalg.norm(axis) + 1e-10)

    proj_center = float(np.dot(center, axis))
    proj_radius = sum(abs(np.dot(axes[i], axis)) * half_extents[i] for i in range(3))
    return proj_center - proj_radius, proj_center + proj_radius


def is_overlap(min_a, max_a, min_b, max_b):
    """Interval overlap on 1D line."""
    return not (max_a < min_b or max_b < min_a)


def visualize_3d_obbs(box_a: SweptBox_OBB, box_b: SweptBox_OBB, title: str):
    """Render two OBBs in 3D."""
    fig = plt.figure(figsize=(11, 8.5))
    ax = fig.add_subplot(111, projection="3d")

    poly_a = Poly3DCollection(
        build_obb_faces(box_a), alpha=0.28, linewidths=1.8, edgecolors=C_BOX_A, facecolors=C_BOX_A
    )
    poly_b = Poly3DCollection(
        build_obb_faces(box_b), alpha=0.28, linewidths=1.8, edgecolors=C_BOX_B, facecolors=C_BOX_B
    )
    ax.add_collection3d(poly_a)
    ax.add_collection3d(poly_b)

    ax.scatter(*box_a.center, color=C_BOX_A, s=90, marker="o", label="Box A")
    ax.scatter(*box_b.center, color=C_BOX_B, s=90, marker="s", label="Box B")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(title, fontsize=13, fontweight="bold", pad=16)
    ax.legend(loc="upper left", fontsize=10)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def visualize_sat_axes(box_a: SweptBox_OBB, box_b: SweptBox_OBB, title: str):
    """Render 15 SAT axis projections as 1D overlap tests."""
    axes_list = build_sat_axes(box_a, box_b)

    fig = plt.figure(figsize=(18, 10.5))
    fig.suptitle(f"{title}\nSAT 15-Axis Projection Analysis", fontsize=15, fontweight="bold")

    separated_count = 0

    for idx, axis_data in enumerate(axes_list[:15], 1):
        ax = fig.add_subplot(3, 5, idx)
        axis = axis_data["axis"]

        min_a, max_a = project_on_axis(box_a.center, box_a.axes, box_a.half_extents, axis)
        min_b, max_b = project_on_axis(box_b.center, box_b.axes, box_b.half_extents, axis)

        overlap = is_overlap(min_a, max_a, min_b, max_b)
        if not overlap:
            separated_count += 1

        ax.set_facecolor("#ffe6e6" if overlap else "#e6ffe6")
        ax.barh(1.2, max_a - min_a, left=min_a, height=0.25, color=C_BOX_A, alpha=0.85, edgecolor="black")
        ax.barh(0.8, max_b - min_b, left=min_b, height=0.25, color=C_BOX_B, alpha=0.85, edgecolor="black")

        status = "OVERLAP" if overlap else "SEPARATED"
        status_color = C_OVERLAP if overlap else C_SEPARATED
        ax.set_title(f"{axis_data['label']}\n{status}", color=status_color, fontsize=9, fontweight="bold")

        ax.set_ylim(0.4, 1.6)
        ax.set_xlabel("Projection", fontsize=8)
        ax.set_yticks([])
        ax.grid(True, axis="x", alpha=0.2)

        if idx == 1:
            ax.legend(["Box A", "Box B"], loc="upper right", fontsize=8)

    result_color = "darkgreen" if separated_count > 0 else "darkred"
    fig.text(
        0.5,
        0.01,
        f"Separated: {separated_count}/15 axes",
        ha="center",
        fontsize=11,
        fontweight="bold",
        color=result_color,
    )

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    return fig


def run_visualizer():
    """Main entry point."""
    print("\n" + "=" * 70)
    print("SAT (Separating Axis Theorem) - 3D OBB Collision Detection")
    print("=" * 70)

    # Case 1: overlapping
    print("\n[1/2] Overlapping scenario...")
    center_a = np.array([0.0, 0.0, 0.0])
    angle_a = np.radians(20.0)
    axes_a = np.array(
        [
            [np.cos(angle_a), np.sin(angle_a), 0.0],
            [-np.sin(angle_a), np.cos(angle_a), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    box_a = SweptBox_OBB(center_a, axes_a, np.array([4.0, 2.0, 1.5]), t_start=0.0, t_end=1.0)

    angle_bx = np.radians(30.0)
    angle_bz = np.radians(45.0)
    rot_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(angle_bx), -np.sin(angle_bx)],
            [0.0, np.sin(angle_bx), np.cos(angle_bx)],
        ]
    )
    rot_z = np.array(
        [
            [np.cos(angle_bz), -np.sin(angle_bz), 0.0],
            [np.sin(angle_bz), np.cos(angle_bz), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    axes_b = rot_z @ rot_x
    box_b = SweptBox_OBB(np.array([5.0, 2.0, 1.5]), axes_b, np.array([3.0, 2.5, 1.2]), t_start=0.0, t_end=1.0)

    visualize_3d_obbs(box_a, box_b, "Overlapping OBBs")
    visualize_sat_axes(box_a, box_b, "Case 1: Overlapping Boxes")

    # Case 2: separated but close (small positive gap)
    print("[2/2] Separated scenario (close gap)...")
    angle_a2 = np.radians(15.0)
    axes_a2 = np.array(
        [
            [np.cos(angle_a2), np.sin(angle_a2), 0.0],
            [-np.sin(angle_a2), np.cos(angle_a2), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    box_a2 = SweptBox_OBB(np.array([-3.3, 0.0, 0.0]), axes_a2, np.array([3.0, 1.5, 1.2]), t_start=0.0, t_end=1.0)

    angle_b2 = np.radians(-20.0)
    axes_b2 = np.array(
        [
            [np.cos(angle_b2), np.sin(angle_b2), 0.0],
            [-np.sin(angle_b2), np.cos(angle_b2), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    # Center distance in X is tuned to keep a tiny gap without contact.
    box_b2 = SweptBox_OBB(np.array([3.3, 0.15, 0.0]), axes_b2, np.array([2.5, 2.0, 1.0]), t_start=0.0, t_end=1.0)

    visualize_3d_obbs(box_a2, box_b2, "Separated OBBs (Close, No Contact)")
    visualize_sat_axes(box_a2, box_b2, "Case 2: Separated Boxes (Close Gap)")

    print("\n" + "=" * 70)
    print("Visualization complete!")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    run_visualizer()
    plt.show()
