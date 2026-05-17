"""
viz_12_rtree_full_hierarchy.py — Visualizer 12: R-Tree Full Hierarchy
========================================================================

PURPOSE:
    Demonstrate how the 4D R-Tree efficiently filters out spatial-temporal
    non-conflicts by visualizing the full hierarchy. 
    Combines exact data from libspatialindex (Level 0 and Level 1) 
    with heuristically grouped intermediate nodes (Level 2 and Level 3) 
    to provide a clean, professional, and academic visualization of the 
    underlying spatial data structure.

    - Level 0 (Leaf entries) : individual OBB MBRs (Exact)
    - Level 1 (Leaf nodes)   : groupings from tree_index.leaves() (Exact)
    - Level 2 & 3 (Intermediate) : Heuristic grouping of lower levels
    - Root                   : tree_index.bounds (Exact)
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import random
import matplotlib.colors as mcolors

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from detection.rtree_detector import RTreeDetector
from benchmark.flightplan_generator import generate_random_fleet
from rtree import index

# Force small leaf capacity to create a deep tree structure
old_init = RTreeDetector.__init__
def new_init(self):
    p = index.Property()
    p.dimension = 4
    p.leaf_capacity = 4
    p.index_capacity = 4
    p.near_minimum_overlap_factor = 2
    self.tree_index = index.Index(properties=p)
    self.uavs = {}
RTreeDetector.__init__ = new_init


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _faces(x1, y1, z1, x2, y2, z2):
    """Return the 6 quad faces of an AABB for Poly3DCollection."""
    c = np.array([
        [x1, y1, z1], [x2, y1, z1], [x1, y2, z1], [x2, y2, z1],
        [x1, y1, z2], [x2, y1, z2], [x1, y2, z2], [x2, y2, z2],
    ])
    return [
        [c[0], c[1], c[3], c[2]],  # Bottom
        [c[4], c[5], c[7], c[6]],  # Top
        [c[0], c[1], c[5], c[4]],  # Front
        [c[2], c[3], c[7], c[6]],  # Back
        [c[0], c[2], c[6], c[4]],  # Left
        [c[1], c[3], c[7], c[5]],  # Right
    ]


def add_box(ax, x1, y1, z1, x2, y2, z2, color, alpha, lw, ls='-', label=""):
    faces = _faces(x1, y1, z1, x2, y2, z2)
    
    # We use wireframes (transparent faces) and solid colored edges to match the aesthetic
    edge_rgba = list(mcolors.to_rgba(color))
    edge_rgba[3] = alpha 
    
    # Use (0,0,0,0) instead of 'none' to avoid Matplotlib 3D Poly3DCollection bug
    poly = Poly3DCollection(
        faces,
        facecolors=(0, 0, 0, 0), edgecolors=tuple(edge_rgba),
        linewidths=lw, linestyles=ls
    )
    ax.add_collection3d(poly)
    
    if label:
        # Matplotlib 3D has a bug generating legends for transparent Poly3DCollections
        # We add a dummy 2D line to act as a proxy artist for the legend
        ax.plot([], [], color=edge_rgba, linewidth=lw, linestyle=ls, label=label)


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def run_visualizer():
    print("\n" + "="*70)
    print("VISUALIZER 12: 4D R-Tree Full Hierarchy (Real + Intermediate)")
    print("="*70)

    print("Generating fleet of 4 UAVs to reduce visual clutter...")
    fleet = generate_random_fleet(n_uavs=4, radius=5.0, seed=42)

    print("Registering swept volumes into 4D R-Tree...")
    detector = RTreeDetector()
    for fp in fleet:
        detector.register_uav(f"UAV_{fp.id}", fp, interval=8.0) # Larger interval = fewer boxes

    # ── Reverse map: entry_id  →  SweptBox_OBB ───────────────────────────────
    entry_map = {}
    for uav_id, data in detector.uavs.items():
        for i, box in enumerate(data["boxes"]):
            eid = hash(f"{uav_id}_{i}")
            entry_map[eid] = box

    # ── Level 1: Real leaf-node groupings from libspatialindex ───────────────
    raw_leaves = list(detector.tree_index.leaves())
    print(f"  Leaf nodes in the R-Tree : {len(raw_leaves)}")

    leaf_node_mbrs = []
    for node_id, item_ids, _ in raw_leaves:
        node_boxes = [entry_map[eid] for eid in item_ids if eid in entry_map]
        if not node_boxes:
            continue
        all_b = [b.get_4d_bounds() for b in node_boxes]
        x1 = min(b[0] for b in all_b);  y1 = min(b[1] for b in all_b); z1 = min(b[2] for b in all_b)
        x2 = max(b[4] for b in all_b);  y2 = max(b[5] for b in all_b); z2 = max(b[6] for b in all_b)
        leaf_node_mbrs.append((x1, y1, z1, x2, y2, z2))

    # ── Level 2: Intermediate groupings (Heuristic based on Level 1) ─────────
    level2_mbrs = []
    chunk_size_L2 = 4
    for i in range(0, len(leaf_node_mbrs), chunk_size_L2):
        group = leaf_node_mbrs[i:i+chunk_size_L2]
        x1 = min(b[0] for b in group); y1 = min(b[1] for b in group); z1 = min(b[2] for b in group)
        x2 = max(b[3] for b in group); y2 = max(b[4] for b in group); z2 = max(b[5] for b in group)
        level2_mbrs.append((x1, y1, z1, x2, y2, z2))
    print(f"  Level 2 intermediate nodes : {len(level2_mbrs)}")

    # ── Level 3: Intermediate groupings (Heuristic based on Level 2) ─────────
    level3_mbrs = []
    chunk_size_L3 = 4
    for i in range(0, len(level2_mbrs), chunk_size_L3):
        group = level2_mbrs[i:i+chunk_size_L3]
        x1 = min(b[0] for b in group); y1 = min(b[1] for b in group); z1 = min(b[2] for b in group)
        x2 = max(b[3] for b in group); y2 = max(b[4] for b in group); z2 = max(b[5] for b in group)
        level3_mbrs.append((x1, y1, z1, x2, y2, z2))
    print(f"  Level 3 intermediate nodes : {len(level3_mbrs)}")

    # ── Root: Global MBR from the R-Tree ─────────────────────────────────────
    root_b = detector.tree_index.bounds
    rx1, ry1, rz1 = root_b[0], root_b[1], root_b[2]
    rx2, ry2, rz2 = root_b[4], root_b[5], root_b[6]
    print(f"  Root MBR (3D) : X[{rx1:.0f},{rx2:.0f}] Y[{ry1:.0f},{ry2:.0f}] Z[{rz1:.0f},{rz2:.0f}]")

    # ── Rendering ─────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(
        "Architecture of the 4D R-Tree Hierarchy\n"
        "Levels: Leaf Entries (Real) → Leaf Nodes (Real) → Intermediate Nodes (Simulated) → Root (Real)",
        fontsize=14, pad=20, fontweight='bold'
    )

    uav_colors = ['#ff0055', '#00ffcc', '#ffcc00', '#9900ff'] # Bright route colors

    # Level 0 — individual leaf entries (Exact)
    for i, fp in enumerate(fleet):
        uav_id = f"UAV_{fp.id}"
        boxes  = detector.uavs[uav_id]["boxes"]
        color  = uav_colors[i % len(uav_colors)]

        trace = fp.trace(1.0)
        ax.plot(trace[:, 1], trace[:, 2], trace[:, 3],
                color=color, linewidth=1.0, linestyle='--', alpha=0.6, label=uav_id)

        first = True
        for box in boxes:
            b = box.get_4d_bounds()
            add_box(ax, b[0], b[1], b[2], b[4], b[5], b[6],
                    color=color, alpha=0.5, lw=0.8,
                    label="L0: Leaf Entry MBRs (Exact)" if (i == 0 and first) else "")
            first = False

    # Level 1 — leaf nodes (Exact groupings from rtree.leaves())
    for k, (x1, y1, z1, x2, y2, z2) in enumerate(leaf_node_mbrs):
        add_box(ax, x1, y1, z1, x2, y2, z2,
                color='green', alpha=0.6, lw=1.2,
                label="L1: Leaf Nodes (Exact)" if k == 0 else "")

    # Level 2 — Intermediate nodes (Simulated)
    for k, (x1, y1, z1, x2, y2, z2) in enumerate(level2_mbrs):
        add_box(ax, x1, y1, z1, x2, y2, z2,
                color='blue', alpha=0.7, lw=1.8, ls='-',
                label="L2: Intermediate Nodes (Simulated)" if k == 0 else "")

    # Level 3 — Intermediate nodes (Simulated)
    for k, (x1, y1, z1, x2, y2, z2) in enumerate(level3_mbrs):
        add_box(ax, x1, y1, z1, x2, y2, z2,
                color='orange', alpha=0.8, lw=2.5, ls='-',
                label="L3: Intermediate Nodes (Simulated)" if k == 0 else "")

    # Root — single global MBR (Exact)
    add_box(ax, rx1, ry1, rz1, rx2, ry2, rz2,
            color='red', alpha=1.0, lw=3.5,
            label="Root Node MBR (Exact)")

    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 1000)
    ax.set_zlim(0, 200)
    ax.set_xlabel("X (m)", fontweight='bold')
    ax.set_ylabel("Y (m)", fontweight='bold')
    ax.set_zlabel("Z (m)", fontweight='bold')
    
    # Customize the legend to make it look professional
    leg = ax.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), fontsize=10, 
                    title="R-Tree Elements", title_fontsize='12', frameon=True, shadow=True)
    leg.get_frame().set_edgecolor('black')

    # Better viewing angle
    ax.view_init(elev=25, azim=-45)

    plt.tight_layout()
    plt.subplots_adjust(right=0.75) # make room for the legend
    plt.show()


if __name__ == "__main__":
    run_visualizer()
