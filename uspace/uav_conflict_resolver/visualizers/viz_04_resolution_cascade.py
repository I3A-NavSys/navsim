"""
viz_04_resolution_cascade.py — Visualizer 4: Full Resolution Cascade
========================================================================

PURPOSE:
    Demonstrates the main orchestration capability: CentralManager resolving
    conflicts dynamically via the S1 -> S2 -> FB1 -> FB2 cascade.

API NOTE:
    CentralManager exposes:
        cm.register_uav(uav_id, fp, priority)
        cm.check_and_resolve(uav_id)  → ResolveResult | None
        cm.get_flight_plan(uav_id)    → FlightPlan | None
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from central_manager import CentralManager
from visualizers.flightplan_generator import generate_crossing_pair

BG = "#0f1117"


def run_visualizer():
    print("\n" + "=" * 60)
    print("  VISUALIZER 4: CentralManager Resolution Cascade")
    print("=" * 60)

    # ── 1. Generate a guaranteed crossing conflict ─────────────────────
    fp1, fp2 = generate_crossing_pair(seed=42)
    fp1.priority = 10   # VIP
    fp2.priority = 5    # Plebeian

    # ── 2. Capture BEFORE traces ───────────────────────────────────────
    trace1_before = fp1.trace(1.0)
    trace2_before = fp2.trace(1.0)

    # ── 3. Register UAVs with priorities ──────────────────────────────
    manager = CentralManager()
    manager.register_uav("UAV_1", fp1, priority=10)
    manager.register_uav("UAV_2", fp2, priority=5)

    # ── 4. Detect & resolve via the correct API ────────────────────────
    print("\nRunning check_and_resolve for plebeian UAV_2 …")
    result = manager.check_and_resolve("UAV_2")

    if result is None:
        print("  [FAILED]  No conflict detected — check generate_crossing_pair geometry.")
        resolved = False
    elif result.success:
        print(f"  [SUCCESS] Resolved via strategy: {result.strategy_used}")
        print(f"            Iterations: {result.iterations}")
        print(f"            Message: {result.message}")
        resolved = True
        
        # Imprimir detalles de la resolución
        print("\n  " + "-" * 40)
        print("  DETALLES DE LA RESOLUCIÓN (UAV_2 Plebeyo)")
        print("  " + "-" * 40)
        
        fp2_after = result.new_fp_pleb
        dur_orig = fp2.finish_time() - fp2.init_time()
        dur_new = fp2_after.finish_time() - fp2_after.init_time()
        delay = fp2_after.finish_time() - fp2.finish_time()
        
        print(f"  Duración original: {dur_orig:.2f}s")
        print(f"  Nueva duración:    {dur_new:.2f}s")
        print(f"  Retraso añadido:   {delay:.2f}s")
        
        print("\n  Waypoints Originales:")
        for wp in fp2.waypoints:
            speed = np.linalg.norm(wp.vel)
            print(f"    [{wp.label:5}] t={wp.t:6.2f}s | Pos: ({wp.pos[0]:6.1f}, {wp.pos[1]:6.1f}, {wp.pos[2]:5.1f}) | Vel: {speed:4.1f} m/s")
            
        print("\n  Waypoints Modificados (Ruta Evasiva):")
        for wp in fp2_after.waypoints:
            speed = np.linalg.norm(wp.vel)
            print(f"    [{wp.label:5}] t={wp.t:6.2f}s | Pos: ({wp.pos[0]:6.1f}, {wp.pos[1]:6.1f}, {wp.pos[2]:5.1f}) | Vel: {speed:4.1f} m/s")
        print("  " + "-" * 40 + "\n")
        
    else:
        print(f"  [DEADLOCK] after {result.iterations} iteration(s): {result.message}")
        resolved = False

    # ── 5. Capture AFTER trace (may be the same if no conflict) ───────
    fp2_after  = manager.get_flight_plan("UAV_2")
    trace2_after = fp2_after.trace(1.0) if fp2_after is not None else trace2_before

    # ── 6. Plot ────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 7), facecolor=BG)
    fig.suptitle(
        "CentralManager Resolution Cascade  ·  S1 → S2 → FB1 → FB2",
        fontsize=15, fontweight="bold", color="white", y=0.98
    )

    strategy_label = result.strategy_used if (result and resolved) else "—"

    for col, title, t2 in [
        (0, "BEFORE  (conflict present)",         trace2_before),
        (1, f"AFTER   (strategy: {strategy_label})", trace2_after),
    ]:
        ax = fig.add_subplot(1, 2, col + 1, projection="3d")
        ax.set_facecolor(BG)
        ax.set_title(title, color="white", fontsize=11, pad=6)

        ax.plot(trace1_before[:, 1], trace1_before[:, 2], trace1_before[:, 3],
                color="#2ecc71", linewidth=2.5, label="UAV 1 — VIP")
        ax.plot(t2[:, 1], t2[:, 2], t2[:, 3],
                color="#e74c3c" if col == 0 else "#3498db",
                linewidth=2.5,
                linestyle="-" if col == 0 else "--",
                label="UAV 2 — Plebeian" + (" (detoured)" if col == 1 and resolved else ""))

        # Start / end markers
        for trace, col_m in [(trace1_before, "#2ecc71"), (t2, "#e74c3c" if col == 0 else "#3498db")]:
            ax.scatter(*trace[0, 1:4],  color=col_m, s=50, marker="o", zorder=5)
            ax.scatter(*trace[-1, 1:4], color=col_m, s=70, marker="^", zorder=5)

        ax.set_xlabel("X (m)", color="white", labelpad=4)
        ax.set_ylabel("Y (m)", color="white", labelpad=4)
        ax.set_zlabel("Z (m)", color="white", labelpad=4)
        ax.tick_params(colors="white")
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor("#2a2d3a")
        ax.legend(fontsize=8, facecolor="#1a1d27",
                  edgecolor="gray", labelcolor="white")

    # Status box
    if result and resolved:
        msg = (f"Strategy : {result.strategy_used}\n"
               f"Iterations: {result.iterations}\n"
               f"{result.message}")
        color = "#2ecc71"
    elif result:
        msg = f"DEADLOCK — {result.message}"
        color = "#e74c3c"
    else:
        msg = "No conflict detected."
        color = "#f1c40f"

    fig.text(0.5, 0.01, msg, ha="center", va="bottom", fontsize=9,
             color=color,
             bbox=dict(boxstyle="round,pad=0.4", facecolor="#1a1d27",
                       edgecolor=color, linewidth=1.2))

    plt.tight_layout(rect=[0, 0.06, 1, 0.96])
    plt.show()
    print("\nDone.")


if __name__ == "__main__":
    run_visualizer()
