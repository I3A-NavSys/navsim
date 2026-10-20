import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

import matplotlib.pyplot as plt
from matplotlib.text import Annotation
import numpy as np
import mplcursors
from flight_plan.flight_plan import FlightPlan

time_step = 0.01

def build_plot():
    # Build the plot
    pos_fig = plt.figure("CURVE_STRAIGHT")
    xyz_pos_plot = pos_fig.add_subplot(2, 2, (1, 3), projection="3d")
    dist_sep_plot = pos_fig.add_subplot(2, 2, (2, 4))

    # Indicate axes' name
    xyz_pos_plot.set_xlabel("x [m]")
    xyz_pos_plot.set_ylabel("y [m]")
    xyz_pos_plot.set_zlabel("z [m]")
    dist_sep_plot.set_xlabel("Time [cs]")
    dist_sep_plot.set_ylabel("Distance [m]")

    # Set title
    xyz_pos_plot.set_title("Position 3D")
    dist_sep_plot.set_title("Distance Separation")

    # Set grid to True
    xyz_pos_plot.grid(True)
    dist_sep_plot.grid(True)

def add_trace(fp):
    # Figure settings
    color = [0, 0.7, 1]

    # Get the trace
    tr = fp.trace(time_step)
    tr_x = tr[:, 1]
    tr_y = tr[:, 2]
    tr_z = tr[:, 3]

    # POSITION 3D
    # Get plot
    pos_fig = plt.figure("CURVE_STRAIGHT")
    subplots = pos_fig.get_axes()
    xyz_pos_plot = subplots[0]

    # Set plot info
    xyz_pos_plot.plot(tr_x, tr_y, tr_z, linewidth=2)

    # Get waypoints positions to highlight
    x_pos = []
    y_pos = []
    z_pos = []

    for wp in fp.waypoints:
        x_pos.append(wp.pos[0])
        y_pos.append(wp.pos[1])
        z_pos.append(wp.pos[2])

        if wp.pos[2] == 60:
            dy = 0

            if (wp.pos[1] // 100) % 2 == 0: dx = 100
            else:                           dx = -100

        else:
            dx = 0

            if (wp.pos[0] // 100) % 2 == 0: dy = 100
            else:                           dy = -100

        # Highlight waypoints positions
        xyz_pos_plot.scatter(wp.pos[0], wp.pos[1], wp.pos[2], marker="o", color="blue", s=25)
        # xyz_pos_plot.arrow(wp.pos[0], wp.pos[1], dx, dy, head_width=10, head_length=15, linewidth=0.5, linestyle=(0, (5, 10)),
        #                         length_includes_head=True, zorder=2)

    # Update limits to maintain scale in all axes
    x_lim = max(np.abs(xyz_pos_plot.get_xlim3d()))
    y_lim = max(np.abs(xyz_pos_plot.get_ylim3d()))
    z_lim = max(np.abs(xyz_pos_plot.get_zlim3d()))
    max_lim = max(x_lim, y_lim, z_lim)

    xyz_pos_plot.set_xlim3d(-max_lim, max_lim)
    xyz_pos_plot.set_ylim3d(-max_lim, max_lim)
    xyz_pos_plot.set_zlim3d(-max_lim, max_lim)

def add_distance_separation(dist_sep, fp1, fp2, times):
    pos_fig = plt.figure("CURVE_STRAIGHT")
    subplots = pos_fig.get_axes()
    xyz_pos_plot = subplots[0]
    dist_sep_plot = subplots[1]

    dist_sep_plot.plot(times, dist_sep, linewidth=2)

    # Highlight minimum distance
    min_dist = np.argmin(dist_sep)
    min_dist_x = times[min_dist]
    min_dist_y = dist_sep[min_dist]

    dist_sep_plot.scatter(min_dist_x, min_dist_y, marker="o", color="blue", s=20, zorder=3)

    dist_sep_plot.annotate(round(min_dist_y, 2), (min_dist_x, min_dist_y), textcoords="offset points", xytext=(0,10), ha='center')

    tr1 = fp1.trace(time_step)
    tr2 = fp2.trace(time_step)

    decimals = len(str(time_step).split(".")[1])

    tr1_times = np.round(tr1[:, 0], decimals)
    tr2_times = np.round(tr2[:, 0], decimals)
    tr1_min_dist_i = np.where(tr1_times == min_dist_x)
    tr2_min_dist_i = np.where(tr2_times == min_dist_x)

    tr1_x = tr1[tr1_min_dist_i, 1][0][0]
    tr1_y = tr1[tr1_min_dist_i, 2][0][0]
    tr1_z = tr1[tr1_min_dist_i, 3][0][0]
    tr2_x = tr2[tr2_min_dist_i, 1][0][0]
    tr2_y = tr2[tr2_min_dist_i, 2][0][0]
    tr2_z = tr2[tr2_min_dist_i, 3][0][0]

    xyz_pos_plot.scatter(tr1_x, tr1_y, tr1_z, marker="o", color="black", s=20, zorder=3)
    xyz_pos_plot.scatter(tr2_x, tr2_y, tr2_z, marker="o", color="black", s=20, zorder=3)
    xyz_pos_plot.plot([tr1_x, tr2_x], [tr1_y, tr2_y], [tr1_z, tr2_z], color="black", linestyle="--")

if __name__ == "__main__":
    # Flightplan 1
    fp1 = FlightPlan()
    fp1.set_waypoint(label="wp0", time=0, pos=[0, 0, 60], vel=[10, 0, 0])
    fp1.set_waypoint(label="wp1", time=10, pos=[100, 0, 60], vel=[10, 0, 0])
    # fp1.set_waypoint(label="wp2", time=20, pos=[200, 0, 60], vel=[10, 0, 0])
    # fp1.set_waypoint(label="wp3", time=30, pos=[250, 50, 100], vel=[0, 10, 0])
    fp1.set_waypoint(label="wp4", time=40, pos=[250, 150, 100], vel=[0, 10, 0])
    fp1.set_waypoint(label="wp5", time=50, pos=[250, 250, 100], vel=[0, 10, 0])

    # Flightplan 2
    fp2 = FlightPlan()
    fp2.set_waypoint(label="wp0", time=0, pos=[400, 100, 60], vel=[-10, 0, 0])
    fp2.set_waypoint(label="wp1", time=10, pos=[300, 100, 60], vel=[-10, 0, 0])
    fp2.set_waypoint(label="wp2", time=20, pos=[200, 100, 60], vel=[-10, 0, 0])
    fp2.set_waypoint(label="wp3", time=30, pos=[100, 100, 60], vel=[-10, 0, 0])
    fp2.set_waypoint(label="wp4", time=40, pos=[0, 100, 60], vel=[-10, 0, 0])

    fp1.connect_waypoints()
    fp2.connect_waypoints()

    # Plot
    build_plot()
    add_trace(fp1)
    add_trace(fp2)
    dist_12, times = fp1.compare_to(fp2, time_step)
    add_distance_separation(dist_12, fp1, fp2, times)

    plt.show()