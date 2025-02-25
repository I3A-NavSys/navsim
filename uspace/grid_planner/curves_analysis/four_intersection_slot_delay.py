import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

import matplotlib.pyplot as plt
from matplotlib.text import Annotation
import numpy as np
import mplcursors
from uspace.flight_plan.flight_plan import FlightPlan

time_step = 0.01
is_3d = False

def build_plot():
    global is_3d
    
    # Build figures
    pos_fig = plt.figure("FOUR_INTERSECTION")
    dist_sepraration_fig = plt.figure("DISTANCE_SEPARATION")

    # Build plots
    if is_3d:       xyz_pos_plot = pos_fig.add_subplot(6, 5, (1, 23), projection="3d")
    else:           xyz_pos_plot = pos_fig.add_subplot(6, 5, (1, 23))
    x_pos_time_plot = pos_fig.add_subplot(6, 5, (4, 10))
    y_pos_time_plot = pos_fig.add_subplot(6, 5, (14, 20))
    z_pos_time_plot = pos_fig.add_subplot(6, 5, (24, 30))
    
    fp1_fp2_plot = dist_sepraration_fig.add_subplot(3, 2, (1, 1))
    fp1_fp3_plot = dist_sepraration_fig.add_subplot(3, 2, (2, 2))
    fp1_fp4_plot = dist_sepraration_fig.add_subplot(3, 2, (3, 3))
    fp2_fp3_plot = dist_sepraration_fig.add_subplot(3, 2, (4, 4))
    fp2_fp4_plot = dist_sepraration_fig.add_subplot(3, 2, (5, 5))
    fp3_fp4_plot = dist_sepraration_fig.add_subplot(3, 2, (6, 6))
    dist_sepraration_plots = [fp1_fp2_plot, fp1_fp3_plot, fp1_fp4_plot, fp2_fp3_plot, fp2_fp4_plot, fp3_fp4_plot]
    
    # Indicate axes' name
    xyz_pos_plot.set_xlabel("x [m]")
    xyz_pos_plot.set_ylabel("y [m]")
    if is_3d:       xyz_pos_plot.set_zlabel("z [m]")
    x_pos_time_plot.set_ylabel("x [m]")
    y_pos_time_plot.set_ylabel("y [m]")
    z_pos_time_plot.set_ylabel("z [m]")
    z_pos_time_plot.set_xlabel("time [s]")

    fp2_fp4_plot.set_xlabel("Time [cs]")
    fp3_fp4_plot.set_xlabel("Time [cs]")
    

    # Set title
    xyz_pos_plot.set_title("Position 3D")
    x_pos_time_plot.set_title("Pos vs Time")
    
    fp1_fp2_plot.set_title("FP1 VS FP2")
    fp1_fp3_plot.set_title("FP1 VS FP3")
    fp1_fp4_plot.set_title("FP1 VS FP4")
    fp2_fp3_plot.set_title("FP2 VS FP3")
    fp2_fp4_plot.set_title("FP2 VS FP4")
    fp3_fp4_plot.set_title("FP3 VS FP4")

    # Set grid to True
    xyz_pos_plot.grid(True)
    x_pos_time_plot.grid(True)
    y_pos_time_plot.grid(True)
    z_pos_time_plot.grid(True)
    
    for i, plot in enumerate(dist_sepraration_plots):
        plot.set_ylabel("Distance [m]")
        plot.grid(True)

def add_trace(fp, label):
    global is_3d
    
    # Figure settings
    color = [0, 0.7, 1]

    # Get the trace
    tr = fp.trace(time_step)
    tr_t = tr[:, 0]
    tr_x = tr[:, 1]
    tr_y = tr[:, 2]
    tr_z = tr[:, 3]

    # POSITION 3D
    # Get plot
    pos_fig = plt.figure("FOUR_INTERSECTION")
    subplots = pos_fig.get_axes()
    xyz_pos_plot = subplots[0]
    x_pos_time_plot = subplots[1]
    y_pos_time_plot = subplots[2]
    z_pos_time_plot = subplots[3]

    # Set plot info
    if is_3d:       xyz_pos_plot.plot(tr_x, tr_y, tr_z, linewidth=2, label=label)
    else:       xyz_pos_plot.plot(tr_x, tr_y, linewidth=2, label=label)
    x_pos_time_plot.plot(tr_t, tr_x, linewidth=2)
    y_pos_time_plot.plot(tr_t, tr_y, linewidth=2)
    z_pos_time_plot.plot(tr_t, tr_z, linewidth=2)
    # xyz_pos_plot.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1.25, 1.15), borderaxespad=0.)
    xyz_pos_plot.legend()

    # Get waypoints positions to highlight
    x_pos = []
    y_pos = []
    z_pos = []

    for wp in fp.waypoints:
        x_pos.append(wp.pos[0])
        y_pos.append(wp.pos[1])
        z_pos.append(wp.pos[2])

        # if wp.pos[2] == 60:
        #     dy = 0

        #     if (wp.pos[1] // 100) % 2 == 0: dx = 100
        #     else:                           dx = -100

        # else:
        #     dx = 0

        #     if (wp.pos[0] // 100) % 2 == 0: dy = 100
        #     else:                           dy = -100

        # Highlight waypoints positions
        if is_3d:       xyz_pos_plot.scatter(wp.pos[0], wp.pos[1], wp.pos[2], marker="o", color="blue", s=25)
        else:           xyz_pos_plot.scatter(wp.pos[0], wp.pos[1], marker="o", color="blue", s=25)
        x_pos_time_plot.scatter(wp.t, wp.pos[0], marker="o", color="blue", s=25)
        y_pos_time_plot.scatter(wp.t, wp.pos[1], marker="o", color="blue", s=25)
        z_pos_time_plot.scatter(wp.t, wp.pos[2], marker="o", color="blue", s=25)
        # xyz_pos_plot.arrow(wp.pos[0], wp.pos[1], dx, dy, head_width=10, head_length=15, linewidth=0.5, linestyle=(0, (5, 10)),
        #                         length_includes_head=True, zorder=2)

def adjust_limits():
    global is_3d

    pos_fig = plt.figure("FOUR_INTERSECTION")
    subplots = pos_fig.get_axes()
    xyz_pos_plot = subplots[0]
    x_pos_time_plot = subplots[1]
    y_pos_time_plot = subplots[2]

    # Update limits to maintain scale in all axes
    if is_3d:
        x_lim = max(np.abs(xyz_pos_plot.get_xlim3d()))
        y_Lim = max(np.abs(xyz_pos_plot.get_ylim3d()))
        z_lim = max(np.abs(xyz_pos_plot.get_zlim3d()))
        max_lim = max(x_lim, y_Lim, z_lim)

        xyz_pos_plot.set_xlim3d(-max_lim, max_lim)
        xyz_pos_plot.set_ylim3d(-max_lim, max_lim)
        xyz_pos_plot.set_zlim3d(-max_lim, max_lim)

    else:
        x_lim = max(np.abs(xyz_pos_plot.get_xlim()))
        y_Lim = max(np.abs(xyz_pos_plot.get_ylim()))
        max_lim = max(x_lim, y_Lim)

        xyz_pos_plot.set_xlim(-max_lim, max_lim)
        xyz_pos_plot.set_ylim(-max_lim, max_lim)

    xLim = x_pos_time_plot.get_ylim()
    yLim = y_pos_time_plot.get_ylim()
    xRange = xLim[1] - xLim[0]
    yRange = yLim[1] - yLim[0]
    
    maxRange = max(xRange, yRange)
    addition = maxRange / 2

    xMidValue = (xLim[1] + xLim[0]) / 2
    yMidValue = (yLim[1] + yLim[0]) / 2

    x_pos_time_plot.set_ylim(xMidValue - addition, xMidValue + addition)
    y_pos_time_plot.set_ylim(yMidValue - addition, yMidValue + addition)

def add_distance_separation(i, dist_sep, fp1, fp2, times):
    pos_fig = plt.figure("FOUR_INTERSECTION")
    subplots = pos_fig.get_axes()
    xyz_pos_plot = subplots[0]
    x_pos_time_plot = subplots[1]
    y_pos_time_plot = subplots[2]
    z_pos_time_plot = subplots[3]
    pos_fig = plt.figure("DISTANCE_SEPARATION")
    subplots = pos_fig.get_axes()
    dist_plot = subplots[i]

    dist_plot.plot(times, dist_sep, linewidth=2)

    # Highlight minimum distance
    min_dist = np.argmin(dist_sep)
    min_dist_x = times[min_dist]
    min_dist_y = dist_sep[min_dist]

    dist_plot.scatter(min_dist_x, min_dist_y, marker="o", color="blue", s=25, zorder=3)

    dist_plot.annotate(round(min_dist_y, 2), (min_dist_x, min_dist_y), textcoords="offset points", xytext=(0,10), ha='center')

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

    if is_3d:
        xyz_pos_plot.scatter(tr1_x, tr1_y, tr1_z, marker="o", color="black", s=20, zorder=3)
        xyz_pos_plot.scatter(tr2_x, tr2_y, tr2_z, marker="o", color="black", s=20, zorder=3)
        xyz_pos_plot.plot([tr1_x, tr2_x], [tr1_y, tr2_y], [tr1_z, tr2_z], color="black", linestyle="--", zorder=3)
    
    else:
        xyz_pos_plot.scatter(tr1_x, tr1_y, marker="o", color="black", s=20, zorder=3)
        xyz_pos_plot.scatter(tr2_x, tr2_y, marker="o", color="black", s=20, zorder=3)
        xyz_pos_plot.plot([tr1_x, tr2_x], [tr1_y, tr2_y], color="black", linestyle="--", zorder=3)

    x_pos_time_plot.scatter(min_dist_x, tr1_x, marker="o", color="black", s=20, zorder=3)
    y_pos_time_plot.scatter(min_dist_x, tr1_y, marker="o", color="black", s=20, zorder=3)
    z_pos_time_plot.scatter(min_dist_x, tr1_z, marker="o", color="black", s=20, zorder=3)
    x_pos_time_plot.scatter(min_dist_x, tr2_x, marker="o", color="black", s=20, zorder=3)
    y_pos_time_plot.scatter(min_dist_x, tr2_y, marker="o", color="black", s=20, zorder=3)
    z_pos_time_plot.scatter(min_dist_x, tr2_z, marker="o", color="black", s=20, zorder=3)

    x_pos_time_plot.plot([min_dist_x, min_dist_x], [tr1_x, tr2_x], color="black", linestyle="--", zorder=3)
    y_pos_time_plot.plot([min_dist_x, min_dist_x], [tr1_y, tr2_y], color="black", linestyle="--", zorder=3)
    z_pos_time_plot.plot([min_dist_x, min_dist_x], [tr1_z, tr2_z], color="black", linestyle="--", zorder=3)
    

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
    fp2.set_waypoint(label="wp0", time=10, pos=[400, 100, 60], vel=[-10, 0, 0])
    fp2.set_waypoint(label="wp1", time=20, pos=[300, 100, 60], vel=[-10, 0, 0])
    # fp2.set_waypoint(label="wp2", time=30, pos=[200, 100, 60], vel=[-10, 0, 0])
    # fp2.set_waypoint(label="wp3", time=40, pos=[150, 50, 100], vel=[0, -10, 0])
    fp2.set_waypoint(label="wp4", time=50, pos=[150, -50, 100], vel=[0, -10, 0])
    fp2.set_waypoint(label="wp5", time=60, pos=[150, -150, 100], vel=[0, -10, 0])

    # Flightplan 3
    fp3 = FlightPlan()
    fp3.set_waypoint(label="wp0", time=10, pos=[150, 250, 100], vel=[0, -10, 0])
    fp3.set_waypoint(label="wp1", time=20, pos=[150, 150, 100], vel=[0, -10, 0])
    # fp3.set_waypoint(label="wp2", time=30, pos=[150, 50, 100], vel=[0, -10, 0])
    # fp3.set_waypoint(label="wp3", time=40, pos=[200, 0, 60], vel=[10, 0, 0])
    fp3.set_waypoint(label="wp4", time=50, pos=[300, 0, 60], vel=[10, 0, 0])
    fp3.set_waypoint(label="wp5", time=60, pos=[400, 0, 60], vel=[10, 0, 0])

    # Flightplan 4
    fp4 = FlightPlan()
    fp4.set_waypoint(label="wp0", time=10, pos=[250, -150, 100], vel=[0, 10, 0])
    fp4.set_waypoint(label="wp1", time=20, pos=[250, -50, 100], vel=[0, 10, 0])
    # fp4.set_waypoint(label="wp2", time=30, pos=[250, 50, 100], vel=[0, 10, 0])
    # fp4.set_waypoint(label="wp3", time=40, pos=[200, 100, 60], vel=[-10, 0, 0])
    fp4.set_waypoint(label="wp4", time=50, pos=[100, 100, 60], vel=[-10, 0, 0])
    fp4.set_waypoint(label="wp5", time=60, pos=[0, 100, 60], vel=[-10, 0, 0])

    fp1.connect_waypoints()
    fp2.connect_waypoints()
    fp3.connect_waypoints()
    fp4.connect_waypoints()

    # Plot
    build_plot()

    add_trace(fp1, label="FP1")
    add_trace(fp2, label="FP2")
    add_trace(fp3, label="FP3")
    add_trace(fp4, label="FP4")
    adjust_limits()

    dist_12, times = fp1.compare_to(fp2, time_step)
    dist_13, times = fp1.compare_to(fp3, time_step)
    dist_14, times = fp1.compare_to(fp4, time_step)

    distances = [(0, dist_12, fp1, fp2, times), 
                 (1, dist_13, fp1, fp3, times), 
                 (2, dist_14, fp1, fp4, times)]
    
    for dist in distances:
        add_distance_separation(dist[0], dist[1], dist[2], dist[3], dist[4])

    plt.show()