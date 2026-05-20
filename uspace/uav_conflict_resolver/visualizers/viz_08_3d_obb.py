#!/usr/bin/env python3
"""Visualizer: 3D comparison with Oriented Bounding Boxes (OBBs)
Mimics the style of viz_08 but draws OBB swept volumes along UAV traces.
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.waypoint import Waypoint
from core.models.flight_plan import FlightPlan

BG = "#0f1117"

def generate_straight_fleet(n_uavs: int = 5) -> list[FlightPlan]:
    import numpy as _np
    SPACE = []
    routes = [
        ([0, 200, 100],   [200, 200, 100], [400, 200, 0], 10.0),
        ([200, 0, 100],   [200, 200, 100], [200, 400, 200], 10.0),
        ([0, 0, 100],     [200, 200, 100], [400, 400, 100], 14.14),
        ([400, 0, 0],     [200, 200, 100], [0, 400, 200],   14.14),
        ([200, 200, 200], [200, 200, 100], [200, 200, 0],   5.0)
    ]

    for i in range(min(n_uavs, len(routes))):
        start_pos, mid_pos, end_pos, speed = routes[i]
        dist_to_mid = np.linalg.norm(np.array(mid_pos) - np.array(start_pos))
        t_start = 20.0 - (dist_to_mid / speed)
        dist_from_mid = np.linalg.norm(np.array(end_pos) - np.array(mid_pos))
        t_end = 20.0 + (dist_from_mid / speed)

        fp = FlightPlan()
        fp.id = i + 1
        fp.priority = max(2 - i, 0)
        fp.set_waypoint(Waypoint("start", round(t_start, 2), start_pos, [0,0,0]))
        fp.set_waypoint(Waypoint("mid", 20.0, mid_pos, [0,0,0]))
        fp.set_waypoint(Waypoint("end", round(t_end, 2), end_pos, [0,0,0]))
        fp.max_var_lin_vel = 10.0
        fp.max_var_ang_vel = 2.0
        fp.set_uniform_velocity()
        fp.connect_waypoints()
        SPACE.append(fp)
    return SPACE

def draw_obb(ax, center, dims, yaw, color='#888', alpha=0.6, linewidth=1):
    # dims = (lx, ly, lz)
    lx, ly, lz = dims
    hx, hy, hz = lx/2.0, ly/2.0, lz/2.0
    # Local corners
    corners = np.array([
        [+hx, +hy, +hz], [+hx, -hy, +hz], [-hx, -hy, +hz], [-hx, +hy, +hz],
        [+hx, +hy, -hz], [+hx, -hy, -hz], [-hx, -hy, -hz], [-hx, +hy, -hz]
    ])
    # Rotation about Z (yaw)
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    world = (R @ corners.T).T + np.array(center)

    # Edges between corner indices
    edges = [
        (0,1),(1,2),(2,3),(3,0),
        (4,5),(5,6),(6,7),(7,4),
        (0,4),(1,5),(2,6),(3,7)
    ]
    for i,j in edges:
        xs = [world[i,0], world[j,0]]
        ys = [world[i,1], world[j,1]]
        zs = [world[i,2], world[j,2]]
        ax.plot(xs, ys, zs, color=color, alpha=alpha, linewidth=linewidth)

def main():
    print('Generating fleet and rendering 3D OBB comparison...')
    SPACE = generate_straight_fleet(n_uavs=5)
    original_plans = {f'UAV_{i+1}': fp.copy() for i, fp in enumerate(SPACE)}

    # Simulate a simple 'resolved' scenario by shifting some plans laterally
    resolved_plans = {}
    for i, (uid, fp) in enumerate(original_plans.items()):
        new_fp = fp.copy()
        offset = ((i % 2) * 1 - 0.5) * 30.0  # +-15m lateral offset
        for wp in new_fp.waypoints:
            wp.pos[1] += offset
        resolved_plans[uid] = new_fp

    colors = ["#2ecc71", "#e74c3c", "#3498db", "#f1c40f", "#9b59b6"]

    fig = plt.figure(figsize=(14,7), facecolor=BG)
    fig.suptitle('3D OBB Visualization — Before (left) vs After (right)', color='white', fontsize=16)

    for col, title, plan_dict in [(0, 'BEFORE (Original)', original_plans), (1, 'AFTER (Resolved)', resolved_plans)]:
        ax = fig.add_subplot(1,2,col+1, projection='3d')
        ax.set_facecolor(BG)
        ax.set_title(title, color='white', fontsize=12)

        for i, (uid, fp) in enumerate(plan_dict.items()):
            trace = fp.trace(0.2)
            xs, ys, zs = trace[:,1], trace[:,2], trace[:,3]
            color = colors[i % len(colors)]
            ax.plot(xs, ys, zs, color=color, linewidth=1.5, alpha=0.9)

            # sample OBBs along the trace to show swept volume
            n_samples = max(4, len(xs)//50)
            idxs = np.linspace(0, len(xs)-1, n_samples, dtype=int)
            for idx in idxs:
                pos = np.array([xs[idx], ys[idx], zs[idx]])
                # Estimate yaw from velocity vector
                if idx < len(xs)-1:
                    v = np.array([xs[idx+1]-xs[idx], ys[idx+1]-ys[idx], zs[idx+1]-zs[idx]])
                else:
                    v = np.array([xs[idx]-xs[idx-1], ys[idx]-ys[idx-1], zs[idx]-zs[idx-1]])
                yaw = np.arctan2(v[1], v[0]) if np.linalg.norm(v[:2])>1e-6 else 0.0
                # dimensions: forward length, lateral width, height
                dims = (18.0, 6.0, 2.0)
                draw_obb(ax, pos, dims, yaw, color=color, alpha=0.25, linewidth=0.8)

            # markers
            ax.scatter(xs[0], ys[0], zs[0], color=color, s=30, marker='o')
            ax.scatter(xs[-1], ys[-1], zs[-1], color=color, s=30, marker='^')

        ax.set_xlabel('X (m)', color='white')
        ax.set_ylabel('Y (m)', color='white')
        ax.set_zlabel('Z (m)', color='white')
        ax.tick_params(colors='white')
        # stylize panes
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.set_edgecolor('#2a2d3a')
            pane.fill = False

    plt.tight_layout(rect=[0,0,1,0.95])
    out = 'visualizers/obb_3d_comparison.png'
    fig.savefig(out, dpi=300)
    print(f'Saved {out}')

if __name__ == '__main__':
    main()
