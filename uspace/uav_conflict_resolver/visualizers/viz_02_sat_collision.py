# -*- coding: utf-8 -*-
"""
viz_02_sat_collision.py - Visualizer 2: SAT 15-Axis Collision Detection
========================================================================

PURPOSE:
    Demonstrate the Narrow-Phase collision detection using the Separating
    Axis Theorem (SAT) for 3D Oriented Bounding Boxes (OBBs).
    Specifically shows the extraction of the Minimum Translation Vector (MTV).

VISUALIZATION:
    Displays an interactive 3D plot with:
      - Two overlapping OBBs.
      - The 15 projection axes tested by SAT.
      - The exact MTV (vector pushing the boxes apart).
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# -- Path setup ----------------------------------------------------------------
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from detection.conflictDetection import SweptBox_OBB

# -- Palette -------------------------------------------------------------------
C_BOX_A      = "#2ecc71"   # green
C_BOX_A_EDGE = "#1a8a4a"
C_BOX_B      = "#e74c3c"   # red
C_BOX_B_EDGE = "#9b1c1c"
C_MTV        = "#d4ac0d"   # dark gold
C_AX_A       = "#3498db"   # blue   -- 3 face-normal axes of A
C_AX_B       = "#e67e22"   # orange -- 3 face-normal axes of B
C_AX_CROSS   = "#9b59b6"   # purple -- 9 cross-product edge axes
C_BG         = "#ffffff"   # white background


# ------------------------------------------------------------------------------
# Geometry helpers
# ------------------------------------------------------------------------------

def build_obb_faces(obb: SweptBox_OBB):
    """Return the 6 quad-faces of an OBB as lists of 4 corners each."""
    c = obb.get_corners()
    # Corner ordering from get_corners() iterates i in {-1,1}, j in {-1,1}, k in {-1,1}
    # Indices:  0=(-,-,-) 1=(-,-,+) 2=(-,+,-) 3=(-,+,+)
    #           4=(+,-,-) 5=(+,-,+) 6=(+,+,-) 7=(+,+,+)
    return [
        [c[0], c[1], c[3], c[2]],  # face -X
        [c[4], c[5], c[7], c[6]],  # face +X
        [c[0], c[1], c[5], c[4]],  # face -Y
        [c[2], c[3], c[7], c[6]],  # face +Y
        [c[0], c[2], c[6], c[4]],  # face -Z
        [c[1], c[3], c[7], c[5]],  # face +Z
    ]


def build_15_axes(box_a: SweptBox_OBB, box_b: SweptBox_OBB):
    """
    Build the full list of 15 SAT candidate axes with labels and colours.

    Returns
    -------
    list of dicts: {axis, label, color, origin, group}
        axis   - normalised direction
        origin - 3-D point to draw the arrow from
        label  - short human-readable name
        color  - matplotlib colour string
        group  - group description string
    """
    entries = []

    # -- Group 1: 3 face-normal axes of Box A ----------------------------------
    for i, ax in enumerate(box_a.axes):
        entries.append({
            "axis":   ax / np.linalg.norm(ax),
            "origin": box_a.center,
            "label":  f"A{i+1}",
            "color":  C_AX_A,
            "group":  "Face-normals of A",
        })

    # -- Group 2: 3 face-normal axes of Box B ----------------------------------
    for i, ax in enumerate(box_b.axes):
        entries.append({
            "axis":   ax / np.linalg.norm(ax),
            "origin": box_b.center,
            "label":  f"B{i+1}",
            "color":  C_AX_B,
            "group":  "Face-normals of B",
        })

    # -- Group 3: 9 cross-product edge axes ------------------------------------
    # Origin = midpoint between the two box centres (visually centred)
    mid = (box_a.center + box_b.center) / 2.0
    idx = 1
    for i in range(3):
        for j in range(3):
            raw = np.cross(box_a.axes[i], box_b.axes[j])
            n   = np.linalg.norm(raw)
            if n < 1e-9:
                # Parallel edges -> degenerate, use fallback direction
                entries.append({
                    "axis":   box_a.axes[i],
                    "origin": mid,
                    "label":  f"E{idx}x",
                    "color":  C_AX_CROSS,
                    "group":  "Edge cross-products (degenerate)",
                })
            else:
                entries.append({
                    "axis":   raw / n,
                    "origin": mid,
                    "label":  f"E{idx}",
                    "color":  C_AX_CROSS,
                    "group":  "Edge cross-products",
                })
            idx += 1

    return entries  # len == 15


# ------------------------------------------------------------------------------
# Text table printed to console
# ------------------------------------------------------------------------------

def print_sat_table(axes_info, box_a, box_b, mtv):
    header = (f"{'Axis':>5}  {'Group':<28}  "
              f"{'Direction (normalised)':>34}  {'Overlap':>10}")
    print()
    print("=" * len(header))
    print("  SAT -- 15 Tested Axes")
    print("=" * len(header))
    print(header)
    print("-" * len(header))
    for entry in axes_info:
        ax = entry["axis"]
        min1, max1 = box_a.project_on_axis(ax)
        min2, max2 = box_b.project_on_axis(ax)
        overlap = min(max1 - min2, max2 - min1)
        sep = "SEPARATED" if overlap < 0 else f"{overlap:+.3f}"
        print(f"  {entry['label']:>4}  {entry['group']:<28}  "
              f"({ax[0]:+.4f}, {ax[1]:+.4f}, {ax[2]:+.4f})  {sep:>10}")
    print("-" * len(header))
    print(f"\n  MTV = ({mtv[0]:+.4f}, {mtv[1]:+.4f}, {mtv[2]:+.4f})  "
          f"  |MTV| = {np.linalg.norm(mtv):.4f} m")
    print()


# ------------------------------------------------------------------------------
# Main visualiser
# ------------------------------------------------------------------------------

def run_visualizer():
    print("\n" + "=" * 60)
    print("  VISUALIZER 2: SAT Narrow-Phase Collision & MTV")
    print("=" * 60)

    # -- 1. Define two overlapping OBBs ----------------------------------------
    # Box A: axis-aligned, centred at origin
    center_a  = np.array([0., 0., 0.])
    axes_a    = np.eye(3)                   # identity -- world-aligned
    extents_a = np.array([5., 4., 3.])      # half-extents (m)

    # Box B: rotated 45 deg around Z and 30 deg around Y, partially penetrating A
    angle_z = np.radians(45)
    angle_y = np.radians(30)
    Rz = np.array([
        [ np.cos(angle_z), -np.sin(angle_z), 0],
        [ np.sin(angle_z),  np.cos(angle_z), 0],
        [ 0,                0,               1],
    ])
    Ry = np.array([
        [ np.cos(angle_y), 0, np.sin(angle_y)],
        [ 0,               1, 0              ],
        [-np.sin(angle_y), 0, np.cos(angle_y)],
    ])
    axes_b    = (Ry @ Rz)                   # each ROW is a local axis
    center_b  = np.array([7., 5., 4.])      # penetrates A's corner
    extents_b = np.array([4., 3., 3.])

    box_a = SweptBox_OBB(center_a, axes_a, extents_a, 0, 1)
    box_b = SweptBox_OBB(center_b, axes_b, extents_b, 0, 1)

    # -- 2. Run SAT ------------------------------------------------------------
    print("Executing 15-axis SAT check ...")
    collision, mtv = box_a.collides_with(box_b)

    if collision:
        print(f"  [COLLISION] Detected!")
        print(f"  MTV = {mtv}  |MTV| = {np.linalg.norm(mtv):.4f} m")
    else:
        print("  [NO COLLISION] Boxes are separated -- adjust geometry.")
        mtv = np.zeros(3)

    # -- 3. Build 15 axes info -------------------------------------------------
    axes_info = build_15_axes(box_a, box_b)
    assert len(axes_info) == 15, f"Expected 15 axes, got {len(axes_info)}"
    print_sat_table(axes_info, box_a, box_b, mtv)

    # -- 4. Figure layout: main 3D + inset overlap bar chart -------------------
    fig = plt.figure(figsize=(16, 9), facecolor=C_BG)
    fig.suptitle(
        "Narrow-Phase SAT  |  15 Axes Tested  |  MTV Extraction",
        color="black", fontsize=16, fontweight="bold", y=0.97
    )

    # Left: 3D scene
    ax3d = fig.add_axes([0.00, 0.00, 0.65, 1.00], projection="3d")
    ax3d.set_facecolor(C_BG)
    ax3d.patch.set_alpha(0)
    for pane in (ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor("black")

    # Right: overlap chart
    ax_bar = fig.add_axes([0.67, 0.10, 0.30, 0.80])
    ax_bar.set_facecolor("white")

    # -- 4a. Draw OBBs ---------------------------------------------------------
    poly_a = Poly3DCollection(
        build_obb_faces(box_a), alpha=0.25,
        facecolors=C_BOX_A, edgecolors=C_BOX_A_EDGE, linewidths=1.2
    )
    poly_b = Poly3DCollection(
        build_obb_faces(box_b), alpha=0.25,
        facecolors=C_BOX_B, edgecolors=C_BOX_B_EDGE, linewidths=1.2
    )
    ax3d.add_collection3d(poly_a)
    ax3d.add_collection3d(poly_b)

    # -- 4b. Draw the 15 SAT axes ----------------------------------------------
    ARROW_LEN       = 5.5   # visual length for face-normal axes (m)
    ARROW_LEN_CROSS = 4.5   # visual length for cross-product axes (m)

    for entry in axes_info:
        origin    = entry["origin"]
        is_cross  = "cross" in entry["group"].lower()
        direction = entry["axis"] * (ARROW_LEN_CROSS if is_cross else ARROW_LEN)
        color     = entry["color"]
        label     = entry["label"]

        ax3d.quiver(
            origin[0], origin[1], origin[2],
            direction[0], direction[1], direction[2],
            color=color, linewidth=1.4, arrow_length_ratio=0.18,
            alpha=0.85
        )
        # Label at arrow tip
        tip = origin + direction * 1.15
        ax3d.text(tip[0], tip[1], tip[2], label,
                  color=color, fontsize=7, ha="center", va="center",
                  fontweight="bold")

    # -- 4c. Draw MTV arrow ----------------------------------------------------
    if collision:
        ax3d.quiver(
            center_b[0], center_b[1], center_b[2],
            mtv[0], mtv[1], mtv[2],
            color=C_MTV, linewidth=3.5, arrow_length_ratio=0.12,
            label=f"MTV  |{np.linalg.norm(mtv):.2f} m|"
        )
        # Mark the resolved position of B
        resolved_center = center_b + mtv
        ax3d.scatter(*resolved_center, s=80, c=C_MTV, marker="*", zorder=5,
                     label="B resolved centre")
        ax3d.text(resolved_center[0] + 0.3, resolved_center[1] + 0.3,
                  resolved_center[2] + 0.3,
                  "B'", color=C_MTV, fontsize=9, fontweight="bold")

    # -- 4d. Box centre labels -------------------------------------------------
    ax3d.text(*center_a, "A", color=C_BOX_A, fontsize=12, fontweight="bold",
              ha="center", va="center")
    ax3d.text(*center_b, "B", color=C_BOX_B, fontsize=12, fontweight="bold",
              ha="center", va="center")

    # -- 4e. Limits & cosmetics ------------------------------------------------
    ax3d.set_xlim(-12, 15)
    ax3d.set_ylim(-12, 14)
    ax3d.set_zlim(-10, 12)
    ax3d.set_xlabel("X", color="black", labelpad=6)
    ax3d.set_ylabel("Y", color="black", labelpad=6)
    ax3d.set_zlabel("Z", color="black", labelpad=6)
    ax3d.tick_params(colors="black", labelsize=7)
    ax3d.xaxis.label.set_color("black")
    ax3d.yaxis.label.set_color("black")
    ax3d.zaxis.label.set_color("black")
    ax3d.set_title("3D Scene -- OBBs & all 15 SAT axes",
                   color="black", fontsize=11, pad=10)

    # Legend patches
    legend_patches = [
        mpatches.Patch(color=C_BOX_A,    label="Box A (stationary)"),
        mpatches.Patch(color=C_BOX_B,    label="Box B (intruder)"),
        mpatches.Patch(color=C_AX_A,     label="A1-A3  face normals of A"),
        mpatches.Patch(color=C_AX_B,     label="B1-B3  face normals of B"),
        mpatches.Patch(color=C_AX_CROSS, label="E1-E9  edge cross-products"),
        mpatches.Patch(color=C_MTV,      label=f"MTV  |{np.linalg.norm(mtv):.2f} m|"),
    ]
    ax3d.legend(handles=legend_patches, loc="upper left",
                facecolor="white", edgecolor="black",
                labelcolor="black", fontsize=8, framealpha=0.85)

    # -- 5. Overlap bar chart --------------------------------------------------
    labels   = [e["label"] for e in axes_info]
    colors   = [e["color"] for e in axes_info]
    overlaps = []
    for entry in axes_info:
        ax = entry["axis"]
        min1, max1 = box_a.project_on_axis(ax)
        min2, max2 = box_b.project_on_axis(ax)
        overlaps.append(min(max1 - min2, max2 - min1))

    overlaps_arr = np.array(overlaps)
    bar_colors = [
        "#cccccc" if o < 0 else c          # grey if separated, group colour if overlapping
        for o, c in zip(overlaps_arr, colors)
    ]

    bars = ax_bar.barh(labels, overlaps_arr, color=bar_colors,
                       edgecolor="black", height=0.65, zorder=3)

    # Highlight MTV axis (minimum positive overlap)
    if collision:
        positive = [o if o >= 0 else float("inf") for o in overlaps_arr]
        min_idx  = int(np.argmin(positive))
        bars[min_idx].set_edgecolor(C_MTV)
        bars[min_idx].set_linewidth(2.5)
        ax_bar.annotate(
            "<- MTV axis",
            xy=(overlaps_arr[min_idx], min_idx),
            xytext=(overlaps_arr[min_idx] + 0.5, min_idx),
            color=C_MTV, fontsize=8, fontweight="bold",
            arrowprops=dict(arrowstyle="->", color=C_MTV, lw=1.5),
            va="center"
        )

    ax_bar.axvline(0, color="black", linewidth=0.8, linestyle="--")
    ax_bar.set_xlabel(
        "Overlap depth (m)\n(negative = separation = no collision)",
        color="black", fontsize=8
    )
    ax_bar.set_title("Overlap on Each of the 15 SAT Axes",
                     color="black", fontsize=10, pad=8)
    ax_bar.tick_params(colors="black", labelsize=8)
    ax_bar.set_facecolor("white")
    ax_bar.spines[:].set_color("black")
    ax_bar.grid(axis="x", color="#dddddd", linewidth=0.6, zorder=0)
    ax_bar.invert_yaxis()

    # Colour the y-tick labels to match axis groups
    for tick_label, col in zip(ax_bar.get_yticklabels(), colors):
        tick_label.set_color(col)

    # SAT result annotation box
    mtv_norm = np.linalg.norm(mtv)
    if collision:
        result_str  = "SAT result: COLLISION"
        mtv_str     = (f"MTV = ({mtv[0]:+.2f}, {mtv[1]:+.2f}, {mtv[2]:+.2f})\n"
                       f"|MTV| = {mtv_norm:.4f} m")
        box_edge    = C_MTV
        box_face    = "white"
    else:
        result_str  = "SAT result: SEPARATED"
        mtv_str     = "No MTV (boxes do not overlap)"
        box_edge    = "black"
        box_face    = "white"

    ax_bar.text(
        0.98, 0.02,
        f"{result_str}\n{mtv_str}",
        transform=ax_bar.transAxes,
        ha="right", va="bottom", fontsize=8,
        color="black",
        bbox=dict(boxstyle="round,pad=0.5",
                  facecolor=box_face,
                  edgecolor=box_edge,
                  alpha=0.9)
    )

    plt.show()


if __name__ == "__main__":
    run_visualizer()
