import time
import sys
import os
import random
import matplotlib.pyplot as plt
import numpy as np

#borrar consola
# os.system('cls')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

##############################################################################

from uspace.grid_planner.grid_planner import GridPlanner


gp = GridPlanner()

random.seed(1)
routes_amount = 1000
start_grid = -1000
end_grid = 1000
init_pos = []
end_pos = []
init_times = []
end_times = []

for r in range(routes_amount):
    i = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)
    j = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)

    i2 = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)
    j2 = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)

    while i == i2 and j == j2:
        i2 = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)
        j2 = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)

    init_pos.append((i, j))
    end_pos.append((i2, j2))

    init_times.append(r)
    end_times.append(r+60)

# Define grid parameters
x = np.arange(start_grid-150, end_grid+201, 100)
y = np.arange(start_grid-200, end_grid+201, 100)


def plot():
    # Create figure and axis
    fig = plt.figure("Computation time test")
    ax = fig.add_subplot()
    ax.set_title("Routes")

    x_sticks = np.arange(start_grid-200, end_grid+200, 100)
    y_sticks = np.arange(start_grid-200, end_grid+200, 100)

    ax.set_xticks(x_sticks)  # Ensure grid spacing is 100 units
    ax.set_yticks(y_sticks)
    ax.set_xticklabels(x_sticks // 100)
    ax.set_yticklabels(y_sticks // 100)

    ax.grid(which="both", color="lightgrey", linestyle="-", linewidth=0.3, zorder=1)

    # Plot points
    for xi in x:
        for yi in y:
            ax.scatter(xi, yi, color="red", s=10, zorder=4)
            ax.scatter(yi, xi, color="orange", s=10, zorder=4)

    # Draw horizontal and vertical lines with arrows
    for xi in x:
        for yi in y:
            # Horizontal lines
            if xi < end_grid+300:  # Avoid drawing beyond the grid
                dx = 100 if yi % 200 == 0 else -100  # Even Y goes right, odd Y goes left
                ax.arrow(xi, yi, dx, 0, head_width=10, head_length=15, linewidth=0.5,
                        fc='red', ec='red', linestyle=(0, (5, 10)), length_includes_head=True, zorder=2)
            
            # Vertical lines
            if yi < end_grid+300:  # Avoid drawing beyond the grid
                dy = 100 if (xi // 100) % 2 == 0 else -100  # Even X goes up, odd X goes down
                ax.arrow(xi-50, yi-50, 0, dy, head_width=10, head_length=15, linewidth=0.5,
                        fc='gold', ec='gold', linestyle=(0, (5, 10)), length_includes_head=True, zorder=2)

    # Set axis limits and labels
    ax.set_xlim(start_grid-205, end_grid+205)
    ax.set_ylim(start_grid-205, end_grid+205)
    ax.set_aspect('equal')
    ax.set_xlabel('Position X (m)')
    ax.set_ylabel('Position Y (m)')

    return ax

def get_routes(ax):
    best_route_times = []
    complete_window_times = []
    start_time = time.time()
    total_time_computing = 0
    for r in range(routes_amount):
        route, comp_params = gp.get_best_route(0, init_pos[r], end_pos[r], init_times[r], end_times[r], 
                                               are_new_restrictions=False)
        time_computing = round(comp_params[2], 5)
        total_time_computing += time_computing
        complete_window_times.append(time_computing)

        if route is not None:
            elapsed_time = round(comp_params[0], 5)
            explored_nodes = comp_params[1]
            best_route_times.append(elapsed_time)

            gp.reserve_nodes(route)

            # X = []
            # Y = []

            # for node in route:
            #     X.append(node.i * 100 + 50 if node.L == "X" else node.i * 100)
            #     Y.append(node.j * 100 + 50 if node.L == "Y" else node.j * 100)

            # line = ax.plot(X, Y, zorder=3, label=f"R{r}: TC-{time_computing}, RT-{elapsed_time}, EN-{explored_nodes}")
            # sc = ax.scatter(X[0], Y[0], color="springgreen")
            # ax.scatter(X[-1], Y[-1], color="lightcoral")
            # ax.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1.25, 1.15), borderaxespad=0.)

        else:
            print(f"Route {r} could not be computed")
            print(f"Time computing: {round(time_computing, 5)}")

    final_time = time.time()
    print(f"Sum of route computing times: {total_time_computing}")
    print(f"Function time: {final_time - start_time}")

    print("BRT:")
    print(best_route_times)
    print(f"Mean: {np.mean(best_route_times)}")
    print()
    print("CWT:")
    print(complete_window_times)
    print(f"Mean: {np.mean(complete_window_times)}")

    gp.clear_grid()

    return best_route_times, complete_window_times

def plot_times(best_route_times, complete_window_times):
    fig = plt.figure("TIMES")
    best_route_time_plot = fig.add_subplot(2, 2, (1, 3))

    best_route_time_plot.set_xlabel("Route")
    best_route_time_plot.set_ylabel("Time [s]")

    best_route_time_plot.plot(best_route_times, color="lightcoral")
    best_route_time_plot.plot(complete_window_times, color="cornflowerblue")

if __name__ == "__main__":
    ax = plot()

    plt.grid(True)
    best_route_times, complete_window_times = get_routes(ax)
    plot_times(best_route_times, complete_window_times)
    plt.show()