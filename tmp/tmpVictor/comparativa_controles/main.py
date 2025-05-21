import numpy as np
import matplotlib.pyplot as plt
import os
import sys

root_navsim_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.append(root_navsim_path)

saving_path = os.path.join(root_navsim_path, "sims", "figures", "UAV_control_comparison")

# img_id = "UAV_0_1"
img_id = "UAV_3_1"

ia_path = f"{root_navsim_path}/sims/exported_data/{img_id}_IA_control.csv"
matrix_path = f"{root_navsim_path}/sims/exported_data/{img_id}_modern_control.csv"

ia_data = np.loadtxt(ia_path, delimiter=",", dtype=float)
matrix_data = np.loadtxt(matrix_path, delimiter=",", dtype=float)

# FIGURES & PLOTS
fig_pos_error = plt.figure("Position error")
plot_pos_error = fig_pos_error.add_subplot()

fig_x_acc = plt.figure("X acceleration")
plot_x_acc = fig_x_acc.add_subplot()

fig_y_acc = plt.figure("Y acceleration")
plot_y_acc = fig_y_acc.add_subplot()

fig_z_acc = plt.figure("Z acceleration")
plot_z_acc = fig_z_acc.add_subplot()

fig_yaw_acc = plt.figure("Yaw acceleration")
plot_yaw_acc = fig_yaw_acc.add_subplot()

fig_roll = plt.figure("Roll")
plot_roll = fig_roll.add_subplot()

fig_pitch = plt.figure("Pitch")
plot_pitch = fig_pitch.add_subplot()

fig_servo_time = plt.figure("Time")
plot_servo_time = fig_servo_time.add_subplot()

fig_cpu_usage = plt.figure("CPU usage")
plot_cpu_usage = fig_cpu_usage.add_subplot()

fig_mem_usage = plt.figure("Memory usage")
plot_mem_usage = fig_mem_usage.add_subplot()

fig_gpu_usage = plt.figure("GPU usage")
plot_gpu_usage = fig_gpu_usage.add_subplot()

# CONFIGURING PLOTS
plot_pos_error.set_title("Position error")
plot_pos_error.set_xlabel("Time (s)")
plot_pos_error.set_ylabel("Error (m)")
plot_pos_error.grid(True)
plot_pos_error.plot(ia_data[:, 0], ia_data[:, 1], linewidth=2, color="blue", label="IA")
plot_pos_error.plot(matrix_data[:, 0], matrix_data[:, 1], linewidth=2, color="red", label="Matrix")
plot_pos_error.legend()

plot_x_acc.set_title("X acceleration")
plot_x_acc.set_xlabel("Time (s)")
plot_x_acc.set_ylabel("Acceleration (m/s^2)")
plot_x_acc.grid(True)
plot_x_acc.plot(ia_data[:, 0], ia_data[:, 2], linewidth=2, color="blue", label="IA")
plot_x_acc.plot(matrix_data[:, 0], matrix_data[:, 2], linewidth=2, color="red", label="Matrix")
plot_x_acc.legend()

plot_y_acc.set_title("Y acceleration")
plot_y_acc.set_xlabel("Time (s)")
plot_y_acc.set_ylabel("Acceleration (m/s^2)")
plot_y_acc.grid(True)
plot_y_acc.plot(ia_data[:, 0], ia_data[:, 3], linewidth=2, color="blue", label="IA")
plot_y_acc.plot(matrix_data[:, 0], matrix_data[:, 3], linewidth=2, color="red", label="Matrix")
plot_y_acc.legend()

plot_z_acc.set_title("Z acceleration")
plot_z_acc.set_xlabel("Time (s)")
plot_z_acc.set_ylabel("Acceleration (m/s^2)")
plot_z_acc.grid(True)
plot_z_acc.plot(ia_data[:, 0], ia_data[:, 4], linewidth=2, color="blue", label="IA")
plot_z_acc.plot(matrix_data[:, 0], matrix_data[:, 4], linewidth=2, color="red", label="Matrix")
plot_z_acc.legend()

plot_yaw_acc.set_title("Yaw acceleration")
plot_yaw_acc.set_xlabel("Time (s)")
plot_yaw_acc.set_ylabel("Acceleration (rad/s^2)")
plot_yaw_acc.grid(True)
plot_yaw_acc.plot(ia_data[:, 0], ia_data[:, 5], linewidth=2, color="blue", label="IA")
plot_yaw_acc.plot(matrix_data[:, 0], matrix_data[:, 5], linewidth=2, color="red", label="Matrix")
plot_yaw_acc.legend()

plot_roll.set_title("Roll")
plot_roll.set_xlabel("Time (s)")
plot_roll.set_ylabel("Roll (rad)")
plot_roll.grid(True)
plot_roll.plot(ia_data[:, 0], ia_data[:, 6], linewidth=2, color="blue", label="IA")
plot_roll.plot(matrix_data[:, 0], matrix_data[:, 6], linewidth=2, color="red", label="Matrix")
plot_roll.legend()

plot_pitch.set_title("Pitch")
plot_pitch.set_xlabel("Time (s)")
plot_pitch.set_ylabel("Pitch (rad)")
plot_pitch.grid(True)
plot_pitch.plot(ia_data[:, 0], ia_data[:, 7], linewidth=2, color="blue", label="IA")
plot_pitch.plot(matrix_data[:, 0], matrix_data[:, 7], linewidth=2, color="red", label="Matrix")
plot_pitch.legend()

plot_servo_time.set_title("Servo function time")
plot_servo_time.set_xlabel("Time (s)")
plot_servo_time.set_ylabel("Function time (s)")
plot_servo_time.grid(True)
plot_servo_time.plot(ia_data[:, 0], ia_data[:, 8], linewidth=2, color="blue", label="IA")
plot_servo_time.plot(matrix_data[:, 0], matrix_data[:, 8], linewidth=2, color="red", label="Matrix")
plot_servo_time.legend()

plot_cpu_usage.set_title("CPU usage")
plot_cpu_usage.set_xlabel("Time (s)")
plot_cpu_usage.set_ylabel("CPU usage increment (%)")
plot_cpu_usage.grid(True)
plot_cpu_usage.plot(ia_data[:, 0], ia_data[:, 9], linewidth=2, color="blue", label="IA")
plot_cpu_usage.plot(matrix_data[:, 0], matrix_data[:, 9], linewidth=2, color="red", label="Matrix")
plot_cpu_usage.legend()

plot_mem_usage.set_title("Memory usage")
plot_mem_usage.set_xlabel("Time (s)")
plot_mem_usage.set_ylabel("Memory usage increment (%)")
plot_mem_usage.grid(True)
plot_mem_usage.plot(ia_data[:, 0], ia_data[:, 10], linewidth=2, color="blue", label="IA")
plot_mem_usage.plot(matrix_data[:, 0], matrix_data[:, 10], linewidth=2, color="red", label="Matrix")
plot_mem_usage.legend()

plot_gpu_usage.set_title("GPU usage")
plot_gpu_usage.set_xlabel("Time (s)")
plot_gpu_usage.set_ylabel("GPU usage increment (%)")
plot_gpu_usage.grid(True)
plot_gpu_usage.plot(ia_data[:, 0], ia_data[:, 11], linewidth=2, color="blue", label="IA")
plot_gpu_usage.plot(matrix_data[:, 0], matrix_data[:, 11], linewidth=2, color="red", label="Matrix")
plot_gpu_usage.legend()

# SHOWING PLOTS
plt.show(block=False)

# SAVE PLOTS
save = input("Do you want to save the plots? (y/n): ")
if save.lower() == "y":
    fig_pos_error.savefig(fname=f"{saving_path}/{img_id}_pos_error.svg")
    fig_x_acc.savefig(fname=f"{saving_path}/{img_id}_x_acc.svg")
    fig_y_acc.savefig(fname=f"{saving_path}/{img_id}_y_acc.svg")
    fig_z_acc.savefig(fname=f"{saving_path}/{img_id}_z_acc.svg")
    fig_yaw_acc.savefig(fname=f"{saving_path}/{img_id}_yaw_acc.svg")
    fig_roll.savefig(fname=f"{saving_path}/{img_id}_roll.svg")
    fig_pitch.savefig(fname=f"{saving_path}/{img_id}_pitch.svg")
    fig_servo_time.savefig(fname=f"{saving_path}/{img_id}_servo_time.svg")
    fig_cpu_usage.savefig(fname=f"{saving_path}/{img_id}_cpu_usage.svg")
    fig_mem_usage.savefig(fname=f"{saving_path}/{img_id}_mem_usage.svg")
    fig_gpu_usage.savefig(fname=f"{saving_path}/{img_id}_gpu_usage.svg")
    
    print("Figures saved.")