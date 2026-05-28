"""
viz_multi_conflicts.py — Visual explanations for conflict handling
===============================================================

Interactive 2D visualizer that demonstrates how the manager treats:
 - conflicts between UAVs with the same priority (tie-breaking), and
 - multi-UAV simultaneous conflicts (not just pairwise resolution).

Usage:
    python -m visualizers.viz_multi_conflicts

The script creates simple straight-line flight plans, registers them in
`CentralManager`, and performs priority-ordered sweeps while plotting the
state before/after each sweep. Conflicts detected by the 4D detector are
marked and the UAVs whose plans were changed are highlighted.
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# Project path setup (same pattern as other visualizers)
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from core.models.flight_plan import FlightPlan
from core.models.waypoint import Waypoint
from central_manager import CentralManager


def make_straight_fp(uav_id, start_pos, end_pos, t_start, t_end, priority=1):
    fp = FlightPlan()
    fp.id = uav_id
    fp.priority = priority
    wp0 = Waypoint(label="S", t=t_start, pos=np.array(start_pos, dtype=float), vel=(np.array(end_pos)-np.array(start_pos))/(t_end-t_start))
    wp1 = Waypoint(label="E", t=t_end, pos=np.array(end_pos, dtype=float), vel=np.array([0.0, 0.0, 0.0]))
    fp.set_waypoint(wp=wp0)
    fp.set_waypoint(wp=wp1)
    fp.connect_waypoints(v_max=10, strict=False)
    return fp


def plot_state(ax, manager: CentralManager, title="State", modified_ids=None):
    ax.clear()
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

    uav_ids = sorted(manager._flight_plans.keys())
    for i, uid in enumerate(uav_ids):
        fp = manager.get_flight_plan(uid)
        trace = fp.trace(0.5)
        if trace.size == 0:
            continue
        pos = trace[:, 1:3]
        color = colors[i % len(colors)]
        lw = 3 if modified_ids and uid in modified_ids else 1.5
        alpha = 1.0 if modified_ids and uid in modified_ids else 0.8
        ax.plot(pos[:, 0], pos[:, 1], color=color, linewidth=lw, alpha=alpha, label=f"{uid} (p={fp.priority})")
        # start/end markers
        ax.scatter(pos[0, 0], pos[0, 1], marker='s', color=color, s=80)
        ax.scatter(pos[-1, 0], pos[-1, 1], marker='^', color=color, s=80)

    # Plot detected conflicts
    conflicts = manager._rtree_detector.detect_all_conflicts_system_wide()
    for c in conflicts:
        t_mid = 0.5 * (c['time_range'][0] + c['time_range'][1])
        # compute approximate collision point as average of positions at t_mid
        a_fp = manager.get_flight_plan(c['uav_a'])
        b_fp = manager.get_flight_plan(c['uav_b'])
        pa = a_fp.status_at_time(t_mid).pos[:2]
        pb = b_fp.status_at_time(t_mid).pos[:2]
        pm = 0.5 * (pa + pb)
        ax.scatter(pm[0], pm[1], marker='x', color='red', s=100)
        ax.text(pm[0], pm[1], f"{c['uav_a']}<> {c['uav_b']}\n{t_mid:.1f}s", color='red')

    ax.set_aspect('equal', 'box')
    ax.set_title(title)
    ax.legend(loc='upper left')
    ax.grid(True)


def scenario_same_priority(manager: CentralManager):
    # Three UAVs crossing at similar times with same priority -> tie-break by ID
    f1 = make_straight_fp('A', [-200, 0, 0], [200, 0, 0], 0.0, 40.0, priority=2)
    f2 = make_straight_fp('B', [0, -200, 0], [0, 200, 0], 5.0, 45.0, priority=2)
    f3 = make_straight_fp('C', [200, -50, 0], [-200, 50, 0], 2.0, 42.0, priority=2)
    manager.register_uav('A', f1, priority=2)
    manager.register_uav('B', f2, priority=2)
    manager.register_uav('C', f3, priority=2)


def scenario_multi_uav_simultaneous(manager: CentralManager):
    # Three UAVs converge to the same point at the same time
    t0 = 30.0
    p_target = [0.0, 0.0, 0.0]
    f1 = make_straight_fp('U1', [-300, 0, 0], p_target, 0.0, t0, priority=1)
    f2 = make_straight_fp('U2', [0, -300, 0], p_target, 0.0, t0, priority=1)
    f3 = make_straight_fp('U3', [300, 0, 0], p_target, 0.0, t0, priority=1)
    manager.register_uav('U1', f1, priority=1)
    manager.register_uav('U2', f2, priority=1)
    manager.register_uav('U3', f3, priority=1)


def interactive_demo(scenario='same'):
    mgr = CentralManager()

    if scenario == 'same':
        scenario_same_priority(mgr)
    else:
        scenario_multi_uav_simultaneous(mgr)

    fig, ax = plt.subplots(figsize=(10, 8))

    sweep = 0
    modified = set()
    while True:
        title = f"Sweep {sweep} — conflicts: {len(mgr._rtree_detector.detect_all_conflicts_system_wide())}"
        plot_state(ax, mgr, title=title, modified_ids=modified)
        plt.pause(0.8)

        # Run one priority-ordered sweep
        results = mgr.single_solve_sweep()
        if not results:
            print("No conflicts in this sweep. Finished.")
            break

        modified = set()
        for uid, res in results.items():
            print(f"Sweep {sweep}: {uid} -> {res.strategy_used} success={res.success}")
            if res.success:
                # manager.register_uav already called; mark plebeian(s) changed
                # We conservatively mark the target uav as modified
                modified.add(uid)

        sweep += 1
        if sweep > 10:
            print("Reached max sweeps, stopping demo.")
            break

    # Final plot
    plot_state(ax, mgr, title=f"Final state after {sweep} sweeps", modified_ids=modified)
    plt.show()


if __name__ == '__main__':
    print("Visualizador: muestra cómo se resuelven conflictos entre UAVs (misma prioridad y multi-UAV).")
    print("Opciones: 'same' (tie-break) o 'multi' (convergencia simultánea)")
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--scenario', choices=['same', 'multi'], default='same')
    args = p.parse_args()
    interactive_demo(scenario=args.scenario)
