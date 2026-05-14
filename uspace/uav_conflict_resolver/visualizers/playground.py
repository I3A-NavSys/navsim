"""
playground.py — Flight Plan Generation and Visualization Sandbox
==================================================================

Quick-start script to generate a fleet of UAVs and visualize their
trajectories in 3D space.
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from benchmark.flightplan_generator import (
    generate_flight_plan,
    generate_crossing_pair,
    generate_random_fleet,
    DEFAULT_PRISM,
)


# ============================================================================
# Main Playground
# ============================================================================

def main():
    """Generate and visualize a fleet of flight plans."""
    
    print("\n" + "=" * 80)
    print("  FLIGHT PLAN PLAYGROUND — Professional Fleet Visualization")
    print("=" * 80)
    
    # ───────────────────────────────────────────────────────────────────
    # SCENARIO: Random Fleet
    # ───────────────────────────────────────────────────────────────────
    print("\n[1] SCENARIO: Random Fleet Generation")
    print("─" * 80)
    print("Generating 6 UAVs with realistic flight plans...\n")
    
    fleet = generate_random_fleet(
        n_uavs=6,
        prism=DEFAULT_PRISM,
        t_start=0.0,
        t_end=150.0,
        seed=42
    )
    
    print(f"✓ Generated {len(fleet)} UAVs\n")
    print(f"{'ID':<4} {'Priority':<8} {'Waypoints':<10} {'Duration [s]':<15} {'Time Window':<25}")
    print("─" * 80)
    for fp in fleet:
        duration = fp.finish_time() - fp.init_time()
        time_window = f"[{fp.init_time():.1f}, {fp.finish_time():.1f}]"
        print(f"{fp.id:<4} {fp.priority:<8} {len(fp.waypoints):<10} {duration:<15.2f} {time_window:<25}")
    
    print("\n" + "─" * 80)
    print("Rendering 3D visualization with smooth polynomial trajectories...")
    print("─" * 80)
    visualize_fleet(fleet, title="Fleet of 6 UAVs | Smooth 7D Polynomial Trajectories")
    
    print("\n" + "=" * 80)
    print("  Visualization Complete!")
    print("=" * 80)
    print("\nClose plots to exit.")
    
    plt.show()


def visualize_fleet(fleet, title="Flight Plan Fleet", sample_rate=0.1):
    """
    Visualize a fleet of flight plans in 3D with smooth polynomial trajectories.
    
    Args:
        fleet:       List of FlightPlan objects.
        title:       Title for the plot.
        sample_rate: Time interval for trace sampling (seconds). Lower = smoother.
    """
    fig = plt.figure(figsize=(16, 12))
    fig.patch.set_facecolor('#0f1117')
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('#0f1117')
    
    # Professional color palette
    colors = ['#00d9ff', '#ff006e', '#ffbe0b', '#8338ec', '#3a86ff', '#fb5607',
              '#06ffa5', '#ff006e', '#c1121f', '#023e8a']
    
    for fp_idx, fp in enumerate(fleet):
        color = colors[fp_idx % len(colors)]
        
        # Get smooth trajectory via .trace() which uses polynomial interpolation
        t_start = fp.init_time()
        t_end = fp.finish_time()
        trace = fp.trace(sample_rate)  # Returns [t, x, y, z, ...] rows
        
        if len(trace) > 0:
            # Extract positions from trace
            positions = trace[:, 1:4]  # columns 1, 2, 3 are x, y, z
            
            # Plot smooth trajectory
            ax.plot(
                positions[:, 0], positions[:, 1], positions[:, 2],
                color=color, linewidth=2.5, alpha=0.9, 
                label=f"UAV {fp.id}", zorder=100
            )
        
        # Plot waypoints as discrete markers
        for wp_idx, wp in enumerate(fp.waypoints):
            if wp_idx == 0:
                # Start point: green square
                ax.scatter(
                    wp.pos[0], wp.pos[1], wp.pos[2],
                    color='#06ffa5', s=200, marker='s', 
                    edgecolor='white', linewidth=1.5, zorder=105,
                    label=f"Start (UAV {fp.id})" if fp_idx == 0 else ""
                )
            elif wp_idx == len(fp.waypoints) - 1:
                # End point: red triangle
                ax.scatter(
                    wp.pos[0], wp.pos[1], wp.pos[2],
                    color='#ff006e', s=200, marker='^', 
                    edgecolor='white', linewidth=1.5, zorder=105,
                    label=f"End (UAV {fp.id})" if fp_idx == 0 else ""
                )
            else:
                # Intermediate waypoints: small circles
                ax.scatter(
                    wp.pos[0], wp.pos[1], wp.pos[2],
                    color=color, s=80, marker='o', 
                    edgecolor='white', linewidth=0.5, alpha=0.7, zorder=104
                )
            
            # Add waypoint labels
            if wp_idx == 0 or wp_idx == len(fp.waypoints) - 1:
                ax.text(
                    wp.pos[0], wp.pos[1], wp.pos[2] + 15,
                    f"{wp.label}\nt={wp.t:.1f}s", fontsize=8,
                    color='white', ha='center', zorder=110
                )
    
    # Plot prism boundaries with enhanced styling
    prism = DEFAULT_PRISM
    _plot_prism_wireframe_enhanced(ax, prism, alpha=0.25)
    
    # Professional labels and formatting
    ax.set_xlabel('X [m]', fontsize=12, color='white', fontweight='bold')
    ax.set_ylabel('Y [m]', fontsize=12, color='white', fontweight='bold')
    ax.set_zlabel('Z [m]', fontsize=12, color='white', fontweight='bold')
    ax.set_title(title, fontsize=16, fontweight='bold', color='white', pad=20)
    
    # Customize tick colors
    ax.tick_params(colors='white', labelsize=10)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('white')
    ax.yaxis.pane.set_edgecolor('white')
    ax.zaxis.pane.set_edgecolor('white')
    ax.xaxis.pane.set_alpha(0.1)
    ax.yaxis.pane.set_alpha(0.1)
    ax.zaxis.pane.set_alpha(0.1)
    
    # Legend with dark background
    legend = ax.legend(
        loc='upper left', fontsize=9, ncol=2,
        framealpha=0.9, fancybox=True, shadow=True,
        labelcolor='white'
    )
    legend.get_frame().set_facecolor('#1a1a2e')
    legend.get_frame().set_edgecolor('white')
    
    ax.grid(True, alpha=0.2, color='white')
    
    # Set aspect ratio and limits
    all_positions = []
    for fp in fleet:
        for wp in fp.waypoints:
            all_positions.append(wp.pos)
    all_positions = np.array(all_positions)
    
    x_min, x_max = all_positions[:, 0].min(), all_positions[:, 0].max()
    y_min, y_max = all_positions[:, 1].min(), all_positions[:, 1].max()
    z_min, z_max = all_positions[:, 2].min(), all_positions[:, 2].max()
    
    x_pad = (x_max - x_min) * 0.1 or 50
    y_pad = (y_max - y_min) * 0.1 or 50
    z_pad = (z_max - z_min) * 0.1 or 20
    
    ax.set_xlim([x_min - x_pad, x_max + x_pad])
    ax.set_ylim([y_min - y_pad, y_max + y_pad])
    ax.set_zlim([z_min - z_pad, z_max + z_pad])
    
    # Adjust viewing angle for better perspective
    ax.view_init(elev=20, azim=45)


def _plot_prism_wireframe_enhanced(ax, prism, alpha=0.2, color='cyan'):
    """Plot an enhanced wireframe box representing the prism bounds."""
    (x_min, x_max), (y_min, y_max), (z_min, z_max) = prism
    
    # Define the 8 corners of the prism
    corners = [
        [x_min, y_min, z_min], [x_max, y_min, z_min],
        [x_max, y_max, z_min], [x_min, y_max, z_min],
        [x_min, y_min, z_max], [x_max, y_min, z_max],
        [x_max, y_max, z_max], [x_min, y_max, z_max],
    ]
    corners = np.array(corners)
    
    # Define the 12 edges
    edges = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom
        [4, 5], [5, 6], [6, 7], [7, 4],  # Top
        [0, 4], [1, 5], [2, 6], [3, 7],  # Vertical
    ]
    
    for edge in edges:
        points = corners[edge]
        ax.plot3D(
            *points.T, color=color, alpha=alpha, linewidth=1.5,
            linestyle='--', zorder=50
        )


# ============================================================================
# Entry Point
# ============================================================================

if __name__ == "__main__":
    main()
