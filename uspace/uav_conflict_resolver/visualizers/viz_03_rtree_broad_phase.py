"""
viz_03_rtree_broad_phase.py — Visualizer 3: R-Tree Broad-Phase
========================================================================

PURPOSE:
    Demonstrate how the 4D R-Tree efficiently filters out spatial-temporal
    non-conflicts. Visualizes the REAL 3-level hierarchy:

      Level 0 (Leaf entries) : individual OBB MBRs  → colored by UAV
      Level 1 (Leaf nodes)   : groupings from tree_index.leaves()  → gold
      Root                   : tree_index.bounds                   → red

    No hierarchy is simulated — all data comes directly from libspatialindex
    via the rtree Python library public API.
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from detection.rtree_detector import RTreeDetector
from visualizers.flightplan_generator import generate_random_fleet


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


def faces_from_8d(bounds):
    """Faces from an 8-tuple (xmin,ymin,zmin,tmin, xmax,ymax,zmax,tmax)."""
    x1, y1, z1, _, x2, y2, z2, _ = bounds
    return _faces(x1, y1, z1, x2, y2, z2)


def add_box(ax, x1, y1, z1, x2, y2, z2, color, alpha, lw, label=""):
    faces = _faces(x1, y1, z1, x2, y2, z2)
    poly = Poly3DCollection(
        faces, alpha=alpha,
        facecolors=color, edgecolors=color,
        linewidths=lw,
    )
    if label:
        poly.set_label(label)
    ax.add_collection3d(poly)


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def run_visualizer():
    print("\n" + "="*60)
    print("VISUALIZER 3: 4D R-Tree Hierarchy (real structure)")
    print("="*60)

    print("Generating fleet of 6 UAVs...")
    fleet = generate_random_fleet(n_uavs=6, radius=5.0, seed=42)

    print("Registering swept volumes into 4D R-Tree...")
    detector = RTreeDetector()
    for fp in fleet:
        detector.register_uav(f"UAV_{fp.id}", fp, interval=5.0)

    # ── Reverse map: entry_id  →  SweptBox_OBB ───────────────────────────────
    # Uses the same hash formula as rtree_detector.register_uav()
    entry_map = {}
    for uav_id, data in detector.uavs.items():
        for i, box in enumerate(data["boxes"]):
            eid = hash(f"{uav_id}_{i}")
            entry_map[eid] = box

    # ── Level 1: Real leaf-node groupings from libspatialindex ───────────────
    # tree_index.leaves() → [(node_id, [entry_ids], [interleaved_bounds]), …]
    # entry_ids are exactly the IDs passed to tree_index.insert()
    raw_leaves = list(detector.tree_index.leaves())
    print(f"  Leaf nodes in the R-Tree : {len(raw_leaves)}")

    leaf_node_mbrs = []
    for node_id, item_ids, _ in raw_leaves:
        node_boxes = [entry_map[eid] for eid in item_ids if eid in entry_map]
        if not node_boxes:
            continue
        all_b = [b.get_4d_bounds() for b in node_boxes]
        # Union of all 4D bounds (project to 3D by ignoring T)
        x1 = min(b[0] for b in all_b);  y1 = min(b[1] for b in all_b)
        z1 = min(b[2] for b in all_b)
        x2 = max(b[4] for b in all_b);  y2 = max(b[5] for b in all_b)
        z2 = max(b[6] for b in all_b)
        leaf_node_mbrs.append((x1, y1, z1, x2, y2, z2))
        print(f"    Node {node_id:6d}: {len(item_ids):3d} entries  "
              f"X[{x1:.0f},{x2:.0f}] Y[{y1:.0f},{y2:.0f}] Z[{z1:.0f},{z2:.0f}]")

    # ── Root: Global MBR from the R-Tree ─────────────────────────────────────
    # For a 4D index, bounds = (xmin,ymin,zmin,tmin, xmax,ymax,zmax,tmax)
    root_b = detector.tree_index.bounds
    rx1, ry1, rz1 = root_b[0], root_b[1], root_b[2]
    rx2, ry2, rz2 = root_b[4], root_b[5], root_b[6]
    print(f"  Root MBR (3D) : X[{rx1:.0f},{rx2:.0f}] Y[{ry1:.0f},{ry2:.0f}] Z[{rz1:.0f},{rz2:.0f}]")

    # ── Rendering ─────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 11))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(
        "4D R-Tree Hierarchy — Level 0 (entries) · Level 1 (leaf nodes) · Root\n"
        "Leaf groupings from tree_index.leaves()  |  Root from tree_index.bounds",
        fontsize=12, pad=20,
    )

    uav_colors = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd', '#ff7f0e', '#17becf']

    # Level 0 — individual leaf entries (one MBR per swept OBB)
    for i, fp in enumerate(fleet):
        uav_id = f"UAV_{fp.id}"
        boxes  = detector.uavs[uav_id]["boxes"]
        color  = uav_colors[i % len(uav_colors)]

        trace = fp.trace(1.0)
        ax.plot(trace[:, 1], trace[:, 2], trace[:, 3],
                color=color, linewidth=2, label=uav_id)

        first = True
        for box in boxes:
            b = box.get_4d_bounds()
            add_box(ax, b[0], b[1], b[2], b[4], b[5], b[6],
                    color=color, alpha=0.06, lw=0.5,
                    label="Leaf entry MBR" if (i == 0 and first) else "")
            first = False

    # Level 1 — leaf nodes (real groupings from rtree.leaves())
    for k, (x1, y1, z1, x2, y2, z2) in enumerate(leaf_node_mbrs):
        add_box(ax, x1, y1, z1, x2, y2, z2,
                color='goldenrod', alpha=0.10, lw=2.0,
                label="Leaf node MBR (Level 1)" if k == 0 else "")

    # Root — single global MBR
    add_box(ax, rx1, ry1, rz1, rx2, ry2, rz2,
            color='red', alpha=0.04, lw=2.5,
            label="Root MBR")

    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 1000)
    ax.set_zlim(0, 200)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend(loc='upper right', fontsize=9)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_visualizer()
