import sys
import os
import matplotlib.pyplot as plt
import numpy as np
# from uspace.flight_plan.flight_plan import FlightPlan

file_path = os.path.dirname(__file__)
project_root_path = os.path.abspath(os.path.join(file_path, '../../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

project_root_path = project_root_path.replace("\\", "/")

time_step = 0.01
is_3d = False
colors = ["cornflowerblue", "darkorange", "mediumseagreen", "red", "violet", "palevioletred", "slateblue", "peru", "tan"]
amount_uavs = 5

def get_exported_data():
    waypoints_traces = []
    fp_traces = []
    tracked_info_traces = []
    filename = "giro90_descenso_"

    for i in range(amount_uavs):
        waypoints_path = project_root_path + "/sims/exported_data" + f"/{filename}UAV{i}_waypoints.csv"
        fp_path = project_root_path + "/sims/exported_data" + f"/{filename}UAV{i}_flightplan.csv"
        tracked_info_path = project_root_path + "/sims/exported_data" + f"/{filename}UAV{i}_tracked_info.csv"

        waypoints_traces.append(np.loadtxt(waypoints_path, delimiter=",", dtype=float))
        fp_traces.append(np.loadtxt(fp_path, delimiter=",", dtype=float))
        tracked_info_traces.append(np.loadtxt(tracked_info_path, delimiter=",", dtype=float))

    # Round tracked_info to seconds (no decimals)
    for trace in tracked_info_traces:
        for i in range(len(trace)):
            trace[i, 0] = round(trace[i, 0], 0)        

    return waypoints_traces, fp_traces, tracked_info_traces

def build_plot(pos_fig_name, sep_fig_name):
    global is_3d
    
    # Build figures
    pos_fig = plt.figure(pos_fig_name)
    dist_sepraration_fig = plt.figure(sep_fig_name)

    # Build plots
    if is_3d:       xyz_pos_plot = pos_fig.add_subplot(6, 5, (1, 28), projection="3d")
    else:           xyz_pos_plot = pos_fig.add_subplot(6, 5, (1, 28))
    x_pos_time_plot = pos_fig.add_subplot(6, 5, (4, 10))
    y_pos_time_plot = pos_fig.add_subplot(6, 5, (14, 20))
    z_pos_time_plot = pos_fig.add_subplot(6, 5, (24, 30))
    
    dist_sep_plot = dist_sepraration_fig.add_subplot()
    
    # Indicate axes' name
    xyz_pos_plot.set_xlabel("x [m]")
    xyz_pos_plot.set_ylabel("y [m]")
    if is_3d:       xyz_pos_plot.set_zlabel("z [m]")
    else:           xyz_pos_plot.set_ylabel("y [m]")
    x_pos_time_plot.set_ylabel("x [m]")
    y_pos_time_plot.set_ylabel("y [m]")
    z_pos_time_plot.set_ylabel("z [m]")
    z_pos_time_plot.set_xlabel("time [s]")

    dist_sep_plot.set_xlabel("Time [s]")
    dist_sep_plot.set_ylabel("Distance [m]")
    
    # Set title
    xyz_pos_plot.set_title("Position 3D")
    x_pos_time_plot.set_title("Pos vs Time")
    
    dist_sep_plot.set_title("Distance separation")

    # Set grid to True
    xyz_pos_plot.grid(True)
    x_pos_time_plot.grid(True)
    y_pos_time_plot.grid(True)
    z_pos_time_plot.grid(True)
    
    dist_sep_plot.grid(True)

def add_trace(tr, label, pos_fig_name, color):
    global is_3d
    
    tr_t = tr[:, 0]
    tr_x = tr[:, 1]
    tr_y = tr[:, 2]
    tr_z = tr[:, 3]

    # POSITION 3D
    # Get plot
    pos_fig = plt.figure(pos_fig_name)
    subplots = pos_fig.get_axes()
    xyz_pos_plot = subplots[0]
    x_pos_time_plot = subplots[1]
    y_pos_time_plot = subplots[2]
    z_pos_time_plot = subplots[3]

    # Set plot info
    if is_3d:       xyz_pos_plot.plot(tr_x, tr_y, tr_z, linewidth=2, label=label, color=color)
    else:       xyz_pos_plot.plot(tr_x, tr_y, linewidth=2, label=label, color=color)
    x_pos_time_plot.plot(tr_t, tr_x, linewidth=2, color=color)
    y_pos_time_plot.plot(tr_t, tr_y, linewidth=2, color=color)
    z_pos_time_plot.plot(tr_t, tr_z, linewidth=2, color=color)
    xyz_pos_plot.legend()          

def hightlight_wps(tr, pos_fig_name):
    dot_size = 10
    tr_t = tr[:, 0]
    tr_x = tr[:, 1]
    tr_y = tr[:, 2]
    tr_z = tr[:, 3]

    pos_fig = plt.figure(pos_fig_name)
    subplots = pos_fig.get_axes()
    xyz_pos_plot = subplots[0]
    x_pos_time_plot = subplots[1]
    y_pos_time_plot = subplots[2]
    z_pos_time_plot = subplots[3]

    # Highlight waypoints positions
    if is_3d:       xyz_pos_plot.scatter(tr_x, tr_y, tr_z, marker="o", color="blue", s=dot_size, zorder=3)
    else:           xyz_pos_plot.scatter(tr_x, tr_y, marker="o", color="blue", s=dot_size, zorder=3)
    x_pos_time_plot.scatter(tr_t, tr_x, marker="o", color="blue", s=dot_size, zorder=3)
    y_pos_time_plot.scatter(tr_t, tr_y, marker="o", color="blue", s=dot_size, zorder=3)
    z_pos_time_plot.scatter(tr_t, tr_z, marker="o", color="blue", s=dot_size, zorder=3)

def adjust_limits(pos_fig_name):
    global is_3d

    pos_fig = plt.figure(pos_fig_name)
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

def add_distance_separation(dist_sep, tr1, tr2, times, label, pos_fig_name, sep_fig_name, color):
    pos_fig = plt.figure(pos_fig_name)
    subplots = pos_fig.get_axes()
    xyz_pos_plot = subplots[0]
    x_pos_time_plot = subplots[1]
    y_pos_time_plot = subplots[2]
    z_pos_time_plot = subplots[3]
    pos_fig = plt.figure(sep_fig_name)
    subplots = pos_fig.get_axes()
    dist_plot = subplots[0]

    dist_plot.plot(times, dist_sep, linewidth=2, label=label, color=color)
    dist_plot.legend()

    # Update limits
    dist_plot.set_ylim((0, dist_plot.get_ylim()[1]))

    # Highlight minimum distance
    min_dist = np.argmin(dist_sep)
    min_dist_x = times[min_dist]
    min_dist_y = dist_sep[min_dist]

    dist_plot.scatter(min_dist_x, min_dist_y, marker="o", color="blue", s=25, zorder=3)

    dist_plot.annotate(round(min_dist_y, 2), (min_dist_x, min_dist_y), textcoords="offset points", xytext=(0,10), ha='center')

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
        xyz_pos_plot.scatter(tr1_x, tr1_y, tr1_z, marker="o", color="black", s=20, zorder=4)
        xyz_pos_plot.scatter(tr2_x, tr2_y, tr2_z, marker="o", color="black", s=20, zorder=4)
        xyz_pos_plot.plot([tr1_x, tr2_x], [tr1_y, tr2_y], [tr1_z, tr2_z], color="black", linestyle="--", zorder=4)
    
    else:
        xyz_pos_plot.scatter(tr1_x, tr1_y, marker="o", color="black", s=20, zorder=4)
        xyz_pos_plot.scatter(tr2_x, tr2_y, marker="o", color="black", s=20, zorder=4)
        xyz_pos_plot.plot([tr1_x, tr2_x], [tr1_y, tr2_y], color="black", linestyle="--", zorder=4)

    x_pos_time_plot.scatter(min_dist_x, tr1_x, marker="o", color="black", s=20, zorder=4)
    y_pos_time_plot.scatter(min_dist_x, tr1_y, marker="o", color="black", s=20, zorder=4)
    z_pos_time_plot.scatter(min_dist_x, tr1_z, marker="o", color="black", s=20, zorder=4)
    x_pos_time_plot.scatter(min_dist_x, tr2_x, marker="o", color="black", s=20, zorder=4)
    y_pos_time_plot.scatter(min_dist_x, tr2_y, marker="o", color="black", s=20, zorder=4)
    z_pos_time_plot.scatter(min_dist_x, tr2_z, marker="o", color="black", s=20, zorder=4)

    x_pos_time_plot.plot([min_dist_x, min_dist_x], [tr1_x, tr2_x], color="black", linestyle="--", zorder=4)
    y_pos_time_plot.plot([min_dist_x, min_dist_x], [tr1_y, tr2_y], color="black", linestyle="--", zorder=4)
    z_pos_time_plot.plot([min_dist_x, min_dist_x], [tr1_z, tr2_z], color="black", linestyle="--", zorder=4)

def compare_traces(tr1, tr2):
    global time_step

    decimals = len(str(time_step).split(".")[1])

    trace_1_times = np.round(tr1[:, 0], decimals)
    trace_2_times = np.round(tr2[:, 0], decimals)

    init_trace_1 = [[0]]
    init_trace_2 = [[0]]
    end_trace_1 = [[len(trace_1_times) - 1]]
    end_trace_2 = [[len(trace_2_times) - 1]]

    if trace_1_times[0] < trace_2_times[0]:     init_trace_1 = np.where(trace_1_times == trace_2_times[0])
    else:                                       init_trace_2 = np.where(trace_2_times == trace_1_times[0])

    if trace_1_times[-1] < trace_2_times[-1]:   end_trace_2 = np.where(trace_2_times == trace_1_times[-1])
    else:                                       end_trace_1 = np.where(trace_1_times == trace_2_times[-1])

    distance_separation = np.abs(tr1[init_trace_1[0][0]:end_trace_1[0][0], 1:4] - 
                                    tr2[init_trace_2[0][0]:end_trace_2[0][0], 1:4])
    
    distances = [np.linalg.norm(dist) for dist in distance_separation]

    return distances, trace_1_times[init_trace_1[0][0]:end_trace_1[0][0]]

if __name__ == "__main__":
    waypoints_traces, fp_traces, tracked_info_traces = get_exported_data()

    build_plot(pos_fig_name="POSICIÓN TEÓRICA", sep_fig_name="SEPARACIÓN TEÓRICA")
    build_plot(pos_fig_name="POSICIÓN REAL", sep_fig_name="SEPARACIÓN REAL")

    for i in range(len(fp_traces)):
        add_trace(fp_traces[i], f"UAV{i}", pos_fig_name="POSICIÓN TEÓRICA", color=colors[i])
        hightlight_wps(waypoints_traces[i], "POSICIÓN TEÓRICA")
        hightlight_wps(waypoints_traces[i], "POSICIÓN REAL")
        add_trace(tracked_info_traces[i], f"UAV{i}", pos_fig_name="POSICIÓN REAL", color=colors[i])

    adjust_limits(pos_fig_name="POSICIÓN TEÓRICA")
    adjust_limits(pos_fig_name="POSICIÓN REAL")

    # Theoretical traces
    dist_01, times_01 = compare_traces(fp_traces[0], fp_traces[1])
    dist_21, times_21 = compare_traces(fp_traces[2], fp_traces[1])
    dist_31, times_31 = compare_traces(fp_traces[3], fp_traces[1])
    dist_41, times_41 = compare_traces(fp_traces[4], fp_traces[1])
    dist_30, times_30 = compare_traces(fp_traces[3], fp_traces[0])
    dist_40, times_40 = compare_traces(fp_traces[4], fp_traces[0])
    dist_32, times_32 = compare_traces(fp_traces[3], fp_traces[2])
    dist_42, times_42 = compare_traces(fp_traces[4], fp_traces[2])

    distances = [(dist_01, fp_traces[0], fp_traces[1], times_01, "UAV1 vs UAV0", colors[0]),
                 (dist_21, fp_traces[2], fp_traces[1], times_21, "UAV1 vs UAV2", colors[2]),
                 (dist_31, fp_traces[3], fp_traces[1], times_31, "UAV1 vs UAV3", colors[3]),
                 (dist_41, fp_traces[4], fp_traces[1], times_41, "UAV1 vs UAV4", colors[4]),
                 (dist_30, fp_traces[3], fp_traces[0], times_30, "UAV0 vs UAV3", colors[5]),
                 (dist_40, fp_traces[4], fp_traces[0], times_40, "UAV0 vs UAV4", colors[6]),
                 (dist_32, fp_traces[3], fp_traces[2], times_32, "UAV2 vs UAV3", colors[7]),
                 (dist_42, fp_traces[4], fp_traces[2], times_42, "UAV2 vs UAV4", colors[8])]
    
    for dist in distances:
        add_distance_separation(dist[0], dist[1], dist[2], dist[3], dist[4], 
                                pos_fig_name="POSICIÓN TEÓRICA", sep_fig_name="SEPARACIÓN TEÓRICA", color=dist[5])

    # Real traces
    dist_01, times_01 = compare_traces(tracked_info_traces[0], tracked_info_traces[1])
    dist_21, times_21 = compare_traces(tracked_info_traces[2], tracked_info_traces[1])
    dist_31, times_31 = compare_traces(tracked_info_traces[3], tracked_info_traces[1])
    dist_41, times_41 = compare_traces(tracked_info_traces[4], tracked_info_traces[1])
    dist_30, times_30 = compare_traces(tracked_info_traces[3], tracked_info_traces[0])
    dist_40, times_40 = compare_traces(tracked_info_traces[4], tracked_info_traces[0])
    dist_32, times_32 = compare_traces(tracked_info_traces[3], tracked_info_traces[2])
    dist_42, times_42 = compare_traces(tracked_info_traces[4], tracked_info_traces[2])

    distances = [(dist_01, tracked_info_traces[0], tracked_info_traces[1], times_01, "UAV1 vs UAV0", colors[0]),
                 (dist_21, tracked_info_traces[2], tracked_info_traces[1], times_21, "UAV1 vs UAV2", colors[2]),
                 (dist_31, tracked_info_traces[3], tracked_info_traces[1], times_31, "UAV1 vs UAV3", colors[3]),
                 (dist_41, tracked_info_traces[4], tracked_info_traces[1], times_41, "UAV1 vs UAV4", colors[4]),
                 (dist_30, tracked_info_traces[3], tracked_info_traces[0], times_30, "UAV0 vs UAV3", colors[5]),
                 (dist_40, tracked_info_traces[4], tracked_info_traces[0], times_40, "UAV0 vs UAV4", colors[6]),
                 (dist_32, tracked_info_traces[3], tracked_info_traces[2], times_32, "UAV2 vs UAV3", colors[7]),
                 (dist_42, tracked_info_traces[4], tracked_info_traces[2], times_42, "UAV2 vs UAV4", colors[8])]
    
    for dist in distances:
        add_distance_separation(dist[0], dist[1], dist[2], dist[3], dist[4], 
                                pos_fig_name="POSICIÓN REAL", sep_fig_name="SEPARACIÓN REAL", color=dist[5])

    plt.show()