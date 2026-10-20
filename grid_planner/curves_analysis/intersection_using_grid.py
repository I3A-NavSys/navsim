import sys
import os
import random
import matplotlib.pyplot as plt
import numpy as np

#borrar consola
os.system('cls')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

##############################################################################

from grid_planner.grid_planner import GridPlanner


gp = GridPlanner()

random.seed(1)
start_grid = -200
end_grid = 200
grid_nodes = 5
debug = True

# Define grid parameters
x = np.arange(start_grid-150, end_grid+201, 100)
y = np.arange(start_grid-200, end_grid+201, 100)

def build_plot():
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(grid_nodes, grid_nodes))
    ax.set_title("TEST")

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

    plt.grid(True)

    return ax

def get_best_route(ax, option, init_pos, end_pos, init_time, end_time):
    print("###################################")
    print(f"COMPUTING BEST ROUTE")
    print()

    best_route, _ = gp.get_best_route(option=option, init_pos=init_pos, end_pos=end_pos, init_time=init_time, end_time=end_time)
    gp.print_route(best_route)

    if best_route is not None:
        gp.reserve_nodes(best_route)

        if not debug:
            X = []
            Y = []

            for node in best_route:
                X.append(node.i * 100 + 50 if node.L == "X" else node.i * 100)
                Y.append(node.j * 100 + 50 if node.L == "Y" else node.j * 100)

            line = ax.plot(X, Y, zorder=6, label=f"Best route {(best_route[0].s, best_route[-1].s, len(best_route))}")
            sc = ax.scatter(X[0], Y[0], zorder=6)
            ax.scatter(X[-1], Y[-1], color=sc.get_facecolor()[0], zorder=6)
            ax.legend()

if __name__ == "__main__":
    gp.clear_grid()
    ax = build_plot()

    gp.debug = debug
    gp.debug_figure = ax
    # fp1
    get_best_route(ax, option=0, init_pos=(-1, 0), end_pos=(3, 2), init_time=1, end_time=1)
    # fp4
    get_best_route(ax, option=0, init_pos=(0, -2), end_pos=(-1, 1), init_time=0, end_time=0)

    plt.show()